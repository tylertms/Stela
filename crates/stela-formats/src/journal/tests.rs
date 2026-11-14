use super::*;
use alloc::vec;

const DOMAIN: [u8; 8] = *b"TESTSTOR";
const KEY: [u8; 32] = [7; 32];
const CACHE: u32 = 1;
const LOG: u32 = 2;

/// A file in memory whose writes may be cut after a number of bytes, as a
/// power loss cuts a disk's: the cut write lands only partly.
struct File {
    bytes: Vec<u8>,
    /// Bytes still written before the cut; `None` for no cut.
    left: Option<u64>,
    cut: bool,
}

impl File {
    fn new(size: u64) -> Self {
        Self {
            bytes: vec![0; size as usize],
            left: None,
            cut: false,
        }
    }
}

impl Target for File {
    type Error = ();
    fn write_at(&mut self, offset: u64, bytes: &[u8]) -> Result<bool, ()> {
        let take = match self.left {
            Some(left) => (left as usize).min(bytes.len()),
            None => bytes.len(),
        };
        let at = offset as usize;
        self.bytes[at..at + take].copy_from_slice(&bytes[..take]);
        if let Some(left) = &mut self.left {
            *left -= take as u64;
            if take < bytes.len() {
                self.cut = true;
                return Err(());
            }
        }
        Ok(true)
    }
    fn flush(&mut self) -> Result<bool, ()> {
        if self.cut { Err(()) } else { Ok(true) }
    }
}

/// What a reader finds: the newest commit whose blobs read back whole.
fn content(file: &[u8]) -> Option<(Commit, Vec<Found>)> {
    let size = file.len() as u64;
    candidates(&file[..DATA_START as usize], DOMAIN, size)
        .into_iter()
        .find_map(|commit| {
            let blobs = read_entries(&commit, |offset, into| {
                let at = offset as usize;
                into.copy_from_slice(file.get(at..at + into.len())?);
                Some(())
            })?;
            Some((commit, blobs))
        })
}

fn run(file: &mut File, plan: Plan) -> Result<Option<Commit>, ()> {
    let mut execution = Execution::new(plan);
    while !execution.advance(file, 8192)? {}
    Ok(execution.last())
}

/// All of `blob`'s bytes.
fn bytes_of(blob: &Blob) -> Vec<u8> {
    let mut bytes = vec![0; blob.length() as usize];
    blob.source.read_at(0, &mut bytes);
    bytes
}

/// Blobs to write, from `(kind, bytes)` pairs.
macro_rules! blobs {
    ($(($kind:expr, $bytes:expr)),* $(,)?) => {
        vec![$(Blob::new($kind, $bytes)),*]
    };
}

fn sorted(mut blobs: Vec<Found>) -> Vec<Found> {
    blobs.sort();
    blobs
}

fn blob(fill: u8, length: usize) -> Vec<u8> {
    (0..length)
        .map(|index| fill.wrapping_add(index as u8))
        .collect()
}

#[test]
fn a_commit_round_trips_and_a_torn_or_misplaced_one_reads_as_none() {
    let commit = Commit {
        domain: DOMAIN,
        generation: 3,
        key: KEY,
        entries: vec![Entry {
            kind: LOG,
            offset: DATA_START,
            length: 10,
            digest: [1; 32],
        }],
    };
    let slot = commit.encode();
    assert_eq!(
        Commit::decode(&slot, 1, DOMAIN, 1 << 20),
        Some(commit.clone())
    );
    assert_eq!(Commit::decode(&slot, 0, DOMAIN, 1 << 20), None);
    assert_eq!(Commit::decode(&slot, 1, *b"OTHERSTO", 1 << 20), None);
    let mut torn = slot.clone();
    torn[100] ^= 1;
    assert_eq!(Commit::decode(&torn, 1, DOMAIN, 1 << 20), None);
    // A blob past the file's end.
    assert_eq!(Commit::decode(&slot, 1, DOMAIN, DATA_START + 100), None);
}

#[test]
fn overlapping_or_unaligned_blobs_read_as_none() {
    let at = |offset| Entry {
        kind: LOG,
        offset,
        length: 5000,
        digest: [0; 32],
    };
    for entries in [
        vec![at(DATA_START), at(DATA_START + ALIGN)],
        vec![at(DATA_START + 1)],
        vec![at(0)],
    ] {
        let commit = Commit {
            domain: DOMAIN,
            generation: 2,
            key: KEY,
            entries,
        };
        assert_eq!(Commit::decode(&commit.encode(), 0, DOMAIN, 1 << 20), None);
    }
}

#[test]
fn appends_keep_what_was_committed_and_replacements_take_its_place() {
    let mut file = File::new(DATA_START + 64 * ALIGN);
    let first = Change {
        replace: vec![CACHE],
        remove: Vec::new(),
        blobs: blobs![(CACHE, blob(1, 20_000)), (LOG, blob(2, 5000))],
    };
    let size = file.bytes.len() as u64;
    let last = run(&mut file, plan(None, DOMAIN, KEY, size, first)).unwrap();
    let (commit, blobs) = content(&file.bytes).unwrap();
    assert_eq!(Some(commit.clone()), last);
    assert_eq!(
        sorted(blobs),
        vec![(CACHE, blob(1, 20_000)), (LOG, blob(2, 5000))]
    );
    let second = Change {
        replace: vec![CACHE],
        remove: Vec::new(),
        blobs: blobs![(CACHE, blob(3, 30_000)), (LOG, blob(4, 100))],
    };
    run(&mut file, plan(Some(&commit), DOMAIN, KEY, size, second)).unwrap();
    let (_, blobs) = content(&file.bytes).unwrap();
    assert_eq!(
        sorted(blobs),
        vec![
            (CACHE, blob(3, 30_000)),
            (LOG, blob(2, 5000)),
            (LOG, blob(4, 100))
        ]
    );
}

#[test]
fn another_keys_content_is_not_kept() {
    let mut file = File::new(DATA_START + 16 * ALIGN);
    let size = file.bytes.len() as u64;
    let change = Change {
        replace: vec![],
        remove: Vec::new(),
        blobs: blobs![(LOG, blob(1, 9000))],
    };
    let commit = run(&mut file, plan(None, DOMAIN, KEY, size, change.clone())).unwrap();
    run(
        &mut file,
        plan(commit.as_ref(), DOMAIN, [8; 32], size, change),
    )
    .unwrap();
    let (commit, blobs) = content(&file.bytes).unwrap();
    assert_eq!(commit.key, [8; 32]);
    assert_eq!(blobs, vec![(LOG, blob(1, 9000))]);
}

#[test]
fn a_replacement_with_no_room_beside_the_old_goes_where_it_was() {
    // Room for 10 blocks: the old cache takes 6, its replacement 6.
    let mut file = File::new(DATA_START + 10 * ALIGN);
    let size = file.bytes.len() as u64;
    let old = Change {
        replace: vec![CACHE],
        remove: Vec::new(),
        blobs: blobs![(CACHE, blob(1, 6 * ALIGN as usize))],
    };
    let commit = run(&mut file, plan(None, DOMAIN, KEY, size, old)).unwrap();
    let new = Change {
        replace: vec![CACHE],
        remove: Vec::new(),
        blobs: blobs![(CACHE, blob(2, 6 * ALIGN as usize))],
    };
    let plan = plan(commit.as_ref(), DOMAIN, KEY, size, new);
    assert!(plan.dropped.is_empty());
    let commits = plan
        .steps
        .iter()
        .filter(|step| matches!(step, Step::Commit(_)))
        .count();
    assert_eq!(commits, 2);
    run(&mut file, plan).unwrap();
    assert_eq!(
        content(&file.bytes).unwrap().1,
        vec![(CACHE, blob(2, 6 * ALIGN as usize))]
    );
}

#[test]
fn a_replacement_that_can_never_fit_keeps_the_old_and_an_append_without_room_is_dropped() {
    let mut file = File::new(DATA_START + 4 * ALIGN);
    let size = file.bytes.len() as u64;
    let old = Change {
        replace: vec![CACHE],
        remove: Vec::new(),
        blobs: blobs![(CACHE, blob(1, 3 * ALIGN as usize))],
    };
    let commit = run(&mut file, plan(None, DOMAIN, KEY, size, old)).unwrap();
    let new = Change {
        replace: vec![CACHE],
        remove: Vec::new(),
        blobs: blobs![
            (CACHE, blob(2, 5 * ALIGN as usize)),
            (LOG, blob(3, 2 * ALIGN as usize)),
        ],
    };
    let plan = plan(commit.as_ref(), DOMAIN, KEY, size, new);
    assert_eq!(plan.dropped, vec![0, 1]);
    assert!(plan.steps.is_empty());
    assert_eq!(
        content(&file.bytes).unwrap().1,
        vec![(CACHE, blob(1, 3 * ALIGN as usize))]
    );
}

/// Every cut of every update leaves the old content, an intermediate
/// commit's or the new: never nothing and never a mix.
#[test]
fn a_cut_anywhere_leaves_the_old_or_the_new_content_whole() {
    let size = DATA_START + 12 * ALIGN;
    let setups: [(Change, Change); 3] = [
        // Appended beside the old.
        (
            Change {
                replace: vec![CACHE],
                remove: Vec::new(),
                blobs: blobs![(CACHE, blob(1, 3 * ALIGN as usize)), (LOG, blob(2, 1000))],
            },
            Change {
                replace: vec![CACHE],
                remove: Vec::new(),
                blobs: blobs![
                    (CACHE, blob(3, 4 * ALIGN as usize)),
                    (LOG, blob(4, 2 * ALIGN as usize)),
                ],
            },
        ),
        // Replaced where the old was, after an intermediate commit.
        (
            Change {
                replace: vec![CACHE],
                remove: Vec::new(),
                blobs: blobs![(CACHE, blob(5, 7 * ALIGN as usize))],
            },
            Change {
                replace: vec![CACHE],
                remove: Vec::new(),
                blobs: blobs![(CACHE, blob(6, 8 * ALIGN as usize)), (LOG, blob(7, 100))],
            },
        ),
        // Into a blank file.
        (
            Change::default(),
            Change {
                replace: vec![],
                remove: Vec::new(),
                blobs: blobs![(LOG, blob(8, 5 * ALIGN as usize))],
            },
        ),
    ];
    for (before, after) in setups {
        let mut base = File::new(size);
        let committed = run(&mut base, plan(None, DOMAIN, KEY, size, before)).unwrap();
        let old = content(&base.bytes).map(|(_, blobs)| sorted(blobs));
        let full = plan(committed.as_ref(), DOMAIN, KEY, size, after.clone());
        // Each commit's content is a state the reader may find.
        let mut whole: Vec<Option<Vec<Found>>> = vec![old.clone()];
        // A new entry's digest comes as its blob streams: the plan's data
        // steps name the blob at each offset.
        let placed = |offset: u64| {
            full.steps.iter().find_map(|step| match *step {
                Step::Data { blob, offset: at } if at == offset => Some(blob),
                _ => None,
            })
        };
        for step in &full.steps {
            if let Step::Commit(commit) = step {
                let blobs = commit
                    .entries
                    .iter()
                    .map(|entry| {
                        let from_plan = placed(entry.offset)
                            .map(|blob| (full.blobs[blob].kind, bytes_of(&full.blobs[blob])));
                        from_plan.unwrap_or_else(|| {
                            old.as_ref()
                                .unwrap()
                                .iter()
                                .find(|(kind, bytes)| {
                                    *kind == entry.kind && Sha256::digest(bytes)[..] == entry.digest
                                })
                                .cloned()
                                .unwrap()
                        })
                    })
                    .collect();
                whole.push(Some(sorted(blobs)));
            }
        }
        let total: u64 = full
            .steps
            .iter()
            .map(|step| match step {
                Step::Data { blob, .. } => aligned(full.blobs[*blob].length()),
                Step::Commit(_) => SLOT_BYTES,
                Step::Flush => 0,
            })
            .sum();
        let mut cut = 0;
        while cut <= total {
            let mut file = File {
                bytes: base.bytes.clone(),
                left: Some(cut),
                cut: false,
            };
            let plan = plan(committed.as_ref(), DOMAIN, KEY, size, after.clone());
            let _ = run(&mut file, plan);
            let found = content(&file.bytes).map(|(_, blobs)| sorted(blobs));
            assert!(
                whole.contains(&found),
                "a cut after {cut} bytes left another content"
            );
            cut += 512;
        }
        assert!(cut > total);
    }
}

#[test]
fn a_queued_write_is_made_again_until_it_completes() {
    struct Slow {
        file: File,
        calls: u32,
    }
    impl Target for Slow {
        type Error = ();
        fn write_at(&mut self, offset: u64, bytes: &[u8]) -> Result<bool, ()> {
            self.calls += 1;
            if self.calls % 2 == 1 {
                return Ok(false);
            }
            self.file.write_at(offset, bytes)
        }
        fn flush(&mut self) -> Result<bool, ()> {
            Ok(true)
        }
    }
    let size = DATA_START + 8 * ALIGN;
    let mut slow = Slow {
        file: File::new(size),
        calls: 0,
    };
    let change = Change {
        replace: vec![],
        remove: Vec::new(),
        blobs: blobs![(LOG, blob(9, 3 * ALIGN as usize + 5))],
    };
    let mut execution = Execution::new(plan(None, DOMAIN, KEY, size, change));
    let mut rounds = 0;
    while !execution.advance(&mut slow, ALIGN).unwrap() {
        rounds += 1;
        assert!(rounds < 100);
    }
    assert_eq!(
        content(&slow.file.bytes).unwrap().1,
        vec![(LOG, blob(9, 3 * ALIGN as usize + 5))]
    );
}

#[test]
fn file_ranges_map_onto_the_extents_in_order() {
    let extents = [
        Extent {
            offset: 8192,
            bytes: 4096,
        },
        Extent {
            offset: 65536,
            bytes: 8192,
        },
    ];
    assert_eq!(
        device_ranges(&extents, 512, 2048, 4096),
        Some(vec![(20, 2048, 0), (128, 2048, 2048)])
    );
    assert_eq!(
        device_ranges(&extents, 512, 0, 12288).map(|ranges| ranges.len()),
        Some(2)
    );
    // Past the file, or off a block.
    assert_eq!(device_ranges(&extents, 512, 8192, 8192), None);
    assert_eq!(device_ranges(&extents, 512, 100, 512), None);
}

#[test]
fn a_streamed_blob_is_read_a_piece_at_a_time_and_its_digest_committed() {
    use core::sync::atomic::{AtomicUsize, Ordering};
    /// A source that counts the most bytes any one read asked for.
    struct Counted {
        bytes: Vec<u8>,
        largest: AtomicUsize,
    }
    impl Source for Counted {
        fn length(&self) -> u64 {
            self.bytes.len() as u64
        }
        fn read_at(&self, offset: u64, into: &mut [u8]) {
            self.largest.fetch_max(into.len(), Ordering::Relaxed);
            let at = offset as usize;
            into.copy_from_slice(&self.bytes[at..at + into.len()]);
        }
    }
    let size = DATA_START + 64 * ALIGN;
    let mut file = File::new(size);
    let bytes = blob(5, 40 * ALIGN as usize + 123);
    let source = Arc::new(Counted {
        bytes: bytes.clone(),
        largest: AtomicUsize::new(0),
    });
    let change = Change {
        replace: vec![],
        remove: Vec::new(),
        blobs: vec![Blob {
            kind: LOG,
            source: source.clone(),
        }],
    };
    let mut execution = Execution::new(plan(None, DOMAIN, KEY, size, change));
    while !execution.advance(&mut file, 2 * ALIGN).unwrap() {}
    assert!(source.largest.load(Ordering::Relaxed) <= 2 * ALIGN as usize);
    let last = execution.last().unwrap();
    assert_eq!(last.entries[0].digest[..], Sha256::digest(&bytes)[..]);
    let (commit, found) = content(&file.bytes).unwrap();
    assert_eq!(commit, last);
    assert_eq!(found, vec![(LOG, bytes)]);
}

#[test]
fn blobs_replaced_by_offset_leave_and_stay_when_the_new_ones_do_not_fit() {
    let size = DATA_START + 16 * ALIGN;
    let mut file = File::new(size);
    let change = Change {
        replace: vec![],
        remove: vec![],
        blobs: blobs![
            (LOG, blob(1, 100)),
            (LOG, blob(2, 200)),
            (LOG, blob(3, 300))
        ],
    };
    let commit = run(&mut file, plan(None, DOMAIN, KEY, size, change))
        .unwrap()
        .unwrap();
    let offsets: Vec<u64> = commit.entries.iter().map(|entry| entry.offset).collect();
    // The last two leave for one blob of both.
    let merged = Change {
        replace: vec![],
        remove: offsets[1..].to_vec(),
        blobs: blobs![(LOG, blob(4, 500))],
    };
    let planned = plan(Some(&commit), DOMAIN, KEY, size, merged);
    assert!(planned.dropped.is_empty());
    let commit = run(&mut file, planned).unwrap().unwrap();
    let (_, blobs) = content(&file.bytes).unwrap();
    assert_eq!(
        sorted(blobs),
        sorted(vec![(LOG, blob(1, 100)), (LOG, blob(4, 500))])
    );
    // A blob that fits nowhere leaves the ones it would replace.
    let offsets: Vec<u64> = commit.entries.iter().map(|entry| entry.offset).collect();
    let huge = Change {
        replace: vec![],
        remove: offsets.clone(),
        blobs: blobs![(LOG, blob(5, 20 * ALIGN as usize))],
    };
    let planned = plan(Some(&commit), DOMAIN, KEY, size, huge);
    assert_eq!(planned.dropped, [0]);
    assert_eq!(planned.last.as_ref(), Some(&commit));
}

#[test]
fn the_largest_free_run_takes_a_blob_beside_every_committed_one() {
    let size = DATA_START + 16 * ALIGN;
    let mut file = File::new(size);
    assert_eq!(largest_free(None, DOMAIN, KEY, size), 16 * ALIGN);
    // Blocks 0-2 and 3-4 taken: 11 free after them.
    let change = Change {
        replace: vec![],
        remove: Vec::new(),
        blobs: blobs![(LOG, blob(1, 3 * ALIGN as usize - 5)), (LOG, blob(2, 5000))],
    };
    let commit = run(&mut file, plan(None, DOMAIN, KEY, size, change)).unwrap();
    let free = largest_free(commit.as_ref(), DOMAIN, KEY, size);
    assert_eq!(free, 11 * ALIGN);
    // Another key's blobs leave the whole area free.
    assert_eq!(
        largest_free(commit.as_ref(), DOMAIN, [8; 32], size),
        16 * ALIGN
    );
    // A replacement of that size lands in one commit, which keeps the old
    // blobs until it is whole.
    let replacement = Change {
        replace: vec![LOG],
        remove: Vec::new(),
        blobs: blobs![(LOG, blob(3, free as usize))],
    };
    let planned = plan(commit.as_ref(), DOMAIN, KEY, size, replacement);
    assert!(planned.dropped.is_empty());
    let commits = planned
        .steps
        .iter()
        .filter(|step| matches!(step, Step::Commit(_)))
        .count();
    assert_eq!(commits, 1);
    run(&mut file, planned).unwrap();
    let (_, blobs) = content(&file.bytes).unwrap();
    assert_eq!(blobs, vec![(LOG, blob(3, free as usize))]);
}

#[test]
fn a_shared_blob_is_read_without_a_copy_of_its_own() {
    let shared = Arc::new(vec![1u8, 2, 3, 4]);
    let blob = Blob::new(CACHE, shared.clone());
    assert_eq!(blob.length(), 4);
    assert_eq!(
        Arc::strong_count(&shared),
        2,
        "the blob holds the same bytes"
    );
    let mut into = [0; 2];
    blob.source.read_at(1, &mut into);
    assert_eq!(into, [2, 3]);
}
