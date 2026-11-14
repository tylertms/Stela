//! The Metal device's pipeline store: `stela/store/pipeline.bin`, a file `stela`
//! preallocates beside the translation packs.
//!
//! The file is a journal (`stela_formats::journal`) of the driver's pipeline
//! cache and the compiler's NIR: an update writes only into space the newest
//! commit does not use and then commits, so a write cut by a power loss, a
//! panic or a forced stop leaves the previous content whole. Before boot
//! services end, [`read`] reads the newest whole commit from the store's
//! volume (the first with `store.cfg`). [`install`] hands its content to the
//! Metal device, which seeds its compiled pipelines' cache from it when it is
//! its compiler build's and GPU's.
//!
//! While the guest runs, [`step`] writes what the device added (new NIR and
//! newer cache snapshots) through the store's staged partition, a piece at a
//! time: the processor that steps only queues commands and takes results,
//! and processors step the disk in bounded steps. When the guest restarts or
//! powers off, [`write`] finishes that work and writes what is left. A file
//! of an older layout reads as empty and is replaced by the first write.
//! Every failure leaves the boot running: its pipelines compile again.
use super::store_file::{Direct, Held, Queued, Staged};
use super::translations;
use crate::{firmware, handoff::Region, output, paging::PAGE};
use alloc::{format, string::String, vec, vec::Vec};
use stela_drivers::gpu::metal::native::pipeline_store::{self, Key, MAX_BYTES};
use stela_formats::journal::{self, Blob, Change, Commit, DATA_START, Execution, SLOT_BYTES};
use stela_formats::payload::pieces::Extent;
use uefi::{
    boot,
    proto::media::{
        file::{File as _, FileInfo, FileMode},
        fs::SimpleFileSystem,
    },
};

/// The handoff file holding the file's size, its newest commit and the
/// content that commit names.
pub const FILE: &str = "pipelines";
/// The file's 8.3 name in the store's directory.
const NAME: &str = "pipeline.bin";
/// The placement record at the start of the handoff file: magic and the
/// file's size; the commit's slot ([`SLOT_BYTES`], zeros for none) and the
/// device's store follow.
const PLACEMENT_MAGIC: [u8; 8] = *b"STLGPLC2";
const PLACEMENT_BYTES: usize = 16;
/// The journal's domain, and the kinds of its blobs: the driver's pipeline
/// cache, which a newer one replaces, and pieces of NIR entries, appended.
const DOMAIN: [u8; 8] = *b"PIPELINE";
const CACHE: u32 = 1;
const NIR: u32 = 2;
/// NIR pieces past which an update rewrites the NIR as one.
const NIR_PIECES: usize = 64;
/// The bytes one write command carries, and the most a step queues.
const PIECE_BYTES: u64 = 1 << 20;
/// How long after an update the next may start: new pipelines come in
/// bursts, and each update of the cache rewrites it whole (tens of MB on a
/// USB stick), so a crash loses at most this much of them.
const QUIET_MILLIS: u64 = 30_000;

/// The pipelines the store's volume holds, as the handoff file's pages:
/// run before boot services end. `None` without a store or its file.
pub fn read() -> Option<Region> {
    for handle in boot::find_handles::<SimpleFileSystem>().ok()? {
        let Ok(mut volume) = boot::open_protocol_exclusive::<SimpleFileSystem>(handle) else {
            continue;
        };
        let Ok(mut root) = volume.open_volume() else {
            continue;
        };
        if translations::bound(&mut root).is_none() {
            continue;
        }
        let path = format!("{}\\{NAME}", translations::DIRECTORY);
        let Some(mut file) = translations::open(&mut root, &path, FileMode::Read) else {
            output::arguments(format_args!(
                "PIPELINES no {path}: compiled pipelines compile every boot; `stela repair` \
                 prepares the file\n"
            ));
            return None;
        };
        let size = file.get_boxed_info::<FileInfo>().ok()?.file_size();
        let mut slots = vec![0; DATA_START as usize];
        let found =
            if size >= DATA_START && translations::read_exact(&mut file, &mut slots).is_some() {
                journal::candidates(&slots, DOMAIN, size)
                    .into_iter()
                    .find_map(|commit| {
                        let blobs = journal::read_entries(&commit, |offset, into| {
                            file.set_position(offset).ok()?;
                            translations::read_exact(&mut file, into)
                        })?;
                        Some((commit, blobs))
                    })
            } else {
                None
            };
        let (slot, store) = match &found {
            Some((commit, blobs)) => (commit.encode(), linear(commit, blobs)),
            None => (vec![0; SLOT_BYTES as usize], Vec::new()),
        };
        let bytes = PLACEMENT_BYTES + slot.len() + store.len();
        let pages = bytes.div_ceil(PAGE);
        let physical = firmware::pages(pages).ok()?;
        // SAFETY: firmware returned this exclusive allocation of `bytes` or
        // more. Initialize it before taking a mutable slice.
        let region = unsafe {
            let start = physical as *mut u8;
            start.write_bytes(0, bytes);
            core::slice::from_raw_parts_mut(start, bytes)
        };
        region[..8].copy_from_slice(&PLACEMENT_MAGIC);
        region[8..16].copy_from_slice(&size.to_le_bytes());
        region[PLACEMENT_BYTES..PLACEMENT_BYTES + slot.len()].copy_from_slice(&slot);
        region[PLACEMENT_BYTES + slot.len()..].copy_from_slice(&store);
        match &found {
            Some((commit, blobs)) => output::arguments(format_args!(
                "PIPELINES {path} file={size} stored={} generation={} blobs={}\n",
                store.len(),
                commit.generation,
                blobs.len()
            )),
            None => output::arguments(format_args!(
                "PIPELINES {path} file={size} stored=0 (no whole commit)\n"
            )),
        }
        return Some(Region {
            physical: physical as u64,
            bytes: bytes as u64,
        });
    }
    None
}

/// The device's store layout of `commit`'s `blobs`: header, cache, NIR.
fn linear(commit: &Commit, blobs: &[journal::Found]) -> Vec<u8> {
    let part = |kind| -> Vec<u8> {
        blobs
            .iter()
            .filter(|(blob, _)| *blob == kind)
            .flat_map(|(_, bytes)| bytes.iter().copied())
            .collect()
    };
    let (cache, nir) = (part(CACHE), part(NIR));
    let mut store = pipeline_store::header_with_nir(&Key(commit.key), &cache, &nir).to_vec();
    store.extend_from_slice(&cache);
    store.extend_from_slice(&nir);
    if store.len() as u64 > MAX_BYTES {
        return Vec::new();
    }
    store
}

/// The writer's state: the file, its newest commit, and the update being
/// written.
struct Writer {
    file_bytes: u64,
    committed: Option<Commit>,
    /// The file's device extents, found while the drivers may be called.
    extents: Option<Vec<Extent>>,
    running: Option<Running>,
    /// Host counter value before which no update starts.
    quiet_until: u64,
    /// A write failed: the store is not written again this boot.
    failed: bool,
}

struct Running {
    execution: Execution<'static>,
    update: pipeline_store::Update,
    /// The staged commands of the write in flight.
    queued: Queued,
}

static WRITER: Held<Writer> = Held::new();

fn try_writer<R>(f: impl FnOnce(&mut Writer) -> R) -> Option<R> {
    WRITER.try_with(f)
}

fn writer<R>(f: impl FnOnce(&mut Writer) -> R) -> Option<R> {
    WRITER.with(f)
}

/// Give the Metal device the stored pipelines of the handoff file
/// `contents`, before any GPU opens, and keep its commit for the writes.
pub fn install(contents: &'static [u8]) {
    let head = PLACEMENT_BYTES + SLOT_BYTES as usize;
    if contents.len() < head || contents[..8] != PLACEMENT_MAGIC {
        return;
    }
    let size = u64::from_le_bytes(contents[8..16].try_into().expect("a size"));
    let slot = &contents[PLACEMENT_BYTES..head];
    let committed = (0..2).find_map(|index| Commit::decode(slot, index, DOMAIN, size));
    pipeline_store::provide(
        &contents[head..],
        stela_mesa::COMPILER_SHA256.as_bytes(),
        size.saturating_sub(DATA_START),
    );
    let writer = Writer {
        file_bytes: size,
        committed,
        extents: None,
        running: None,
        quiet_until: 0,
        failed: false,
    };
    WRITER.holding(true, |slot| *slot = Some(writer));
}

/// Find the file's device extents, once the store's partition is kept and
/// while its drivers may still be called directly: the writes while the
/// guest runs only queue.
pub fn prepare() {
    let extents = translations::with_store_file(NAME, |_, extents| extents.to_vec());
    writer(|writer| match extents {
        Ok(extents) => writer.extents = Some(extents),
        Err(error) => {
            output::arguments(format_args!("PIPELINES written only at a stop: {error}\n"));
        }
    });
}

/// The journal change that writes `update` after `committed`.
fn change(update: &pipeline_store::Update, committed: Option<&Commit>) -> Change<'static> {
    let mut change = Change::default();
    if let Some(cache) = &update.cache {
        change.replace.push(CACHE);
        // Shared with the store's snapshot, not copied.
        change.blobs.push(Blob::new(CACHE, cache.clone()));
    }
    if !update.nir.is_empty() {
        let pieces = committed
            .filter(|commit| commit.key == update.key.0)
            .map_or(0, |commit| {
                commit
                    .entries
                    .iter()
                    .filter(|entry| entry.kind == NIR)
                    .count()
            });
        if pieces + 1 > NIR_PIECES {
            change.replace.push(NIR);
            change
                .blobs
                .push(Blob::new(NIR, pipeline_store::all_nir(&update.key)));
        } else {
            change.blobs.push(Blob::new(NIR, update.nir.clone()));
        }
    }
    change
}

impl Writer {
    /// The next update to write, planned, unless one runs or nothing is new.
    fn start(&mut self) -> bool {
        if self.running.is_some() || self.failed {
            return self.running.is_some();
        }
        let Some(update) = pipeline_store::update() else {
            return false;
        };
        let plan = journal::plan(
            self.committed.as_ref(),
            DOMAIN,
            update.key.0,
            self.file_bytes,
            change(&update, self.committed.as_ref()),
        );
        if !plan.dropped.is_empty() {
            output::arguments(format_args!(
                "PIPELINES full: {} of {} pieces not kept in a file of {} bytes\n",
                plan.dropped.len(),
                plan.blobs.len(),
                self.file_bytes
            ));
        }
        self.running = Some(Running {
            execution: Execution::new(plan),
            update,
            queued: Queued::default(),
        });
        true
    }

    /// The update finished: its commit is the file's newest.
    fn finish(&mut self) {
        if let Some(running) = self.running.take() {
            if let Some(last) = running.execution.last() {
                self.committed = Some(last);
            }
            pipeline_store::written(&running.update);
        }
    }

    fn fail(&mut self, error: &str) {
        output::arguments(format_args!("PIPELINES not written: {error}\n"));
        self.running = None;
        self.failed = true;
    }
}

/// While the guest runs, from a processor that holds no device lock: start
/// writing what the device added, or queue the next piece of the write in
/// flight and take finished ones. Never waits for the disk.
pub fn step() {
    let now = super::host::fresh_counter();
    try_writer(|writer| {
        if writer.running.is_none() && now < writer.quiet_until {
            return;
        }
        let Some(extents) = writer.extents.clone() else {
            return;
        };
        if !writer.start() {
            writer.quiet_until = now + super::host::COUNTER_HZ / 1000 * QUIET_MILLIS;
            return;
        }
        let running = writer.running.as_mut().expect("started");
        let advanced = translations::lend_staged_partition(|device| {
            let mut target = Staged {
                device,
                extents: &extents,
                queued: &mut running.queued,
            };
            running.execution.advance(&mut target, PIECE_BYTES)
        });
        match advanced {
            None => {}
            Some(Ok(false)) => {}
            Some(Ok(true)) => {
                writer.finish();
                writer.quiet_until = now + super::host::COUNTER_HZ / 1000 * QUIET_MILLIS;
            }
            Some(Err(error)) => writer.fail(&error),
        }
    });
}

/// At a quiescent point (the guest restarts or powers off), before the
/// translation pack is written: finish the write in flight and write what
/// is left, waiting for the disk.
pub fn write() {
    let started = output::now();
    let written = writer(|writer| {
        let mut bytes = 0u64;
        loop {
            if !writer.start() {
                return Ok(bytes);
            }
            let running = writer.running.as_mut().expect("started");
            bytes += running
                .execution
                .plan()
                .blobs
                .iter()
                .map(Blob::length)
                .sum::<u64>();
            let done = translations::with_store_file(NAME, |device, extents| {
                let mut target = Direct { device, extents };
                while !running.execution.advance(&mut target, PIECE_BYTES)? {}
                Ok::<(), String>(())
            })
            .and_then(|done| done);
            match done {
                Ok(()) => writer.finish(),
                Err(error) => {
                    writer.fail(&error);
                    return Err(());
                }
            }
        }
    });
    if let Some(Ok(bytes)) = written
        && bytes != 0
    {
        output::arguments(format_args!(
            "PIPELINES written bytes={bytes} ms={}\n",
            output::millis_since(started)
        ));
    }
}
