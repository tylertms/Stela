extern crate std;
mod polling;
use super::shared::WINDOW_DEPTH;
use super::*;
use alloc::rc::Rc;
use core::cell::Cell;

/// A queued RAM disk of `depth` tags whose commands complete once
/// released, oldest first, recording each command it was given and
/// counting the reads that reach it directly.
struct Delayed {
    disk: RamDisk,
    depth: usize,
    started: VecDeque<(u32, Command)>,
    done: BTreeMap<u32, Finished>,
    next: u32,
    released: Rc<Cell<usize>>,
    reads: Rc<Cell<usize>>,
    given: Rc<core::cell::RefCell<Vec<Command>>>,
    last_read_address: Rc<Cell<usize>>,
    /// Whether a finish collects every completion ([`BlockDevice::finished`]),
    /// and the finishes that reached the device.
    together: bool,
    finishes: Rc<Cell<usize>>,
    /// The commits that reached the device ([`BlockDevice::commit`]).
    commits: Rc<Cell<usize>>,
}

type Probes = (
    Rc<Cell<usize>>,
    Rc<Cell<usize>>,
    Rc<core::cell::RefCell<Vec<Command>>>,
);

impl Delayed {
    fn new(disk: RamDisk, depth: usize) -> (Self, Probes) {
        let probes: Probes = Default::default();
        let delayed = Self {
            disk,
            depth,
            started: VecDeque::new(),
            done: BTreeMap::new(),
            next: 0,
            released: probes.0.clone(),
            reads: probes.1.clone(),
            given: probes.2.clone(),
            last_read_address: Default::default(),
            together: false,
            finishes: Default::default(),
            commits: Default::default(),
        };
        (delayed, probes)
    }

    fn complete(&mut self) {
        while self.released.get() > 0 {
            let Some((tag, command)) = self.started.pop_front() else {
                return;
            };
            self.released.set(self.released.get() - 1);
            let result = self.disk.execute(command);
            if let Ok(Some(data)) = &result {
                self.last_read_address.set(data.as_ptr() as usize);
            }
            self.done.insert(tag, result);
        }
    }
}

impl BlockDevice for Delayed {
    fn identity(&self) -> &Identity {
        &self.disk.identity
    }
    fn read(&mut self, lba: u64, buffer: &mut [u8]) -> Result<(), BlockError> {
        self.reads.set(self.reads.get() + 1);
        self.disk.read(lba, buffer)
    }
    fn write(&mut self, lba: u64, data: &[u8]) -> Result<(), BlockError> {
        self.disk.write(lba, data)
    }
    fn flush_cache(&mut self) -> Result<(), BlockError> {
        self.disk.flush_cache()
    }
    fn write_fua(&mut self, lba: u64, data: &[u8]) -> Result<(), BlockError> {
        self.disk.write_fua(lba, data)
    }
    fn depth(&self) -> usize {
        self.depth
    }
    fn room(&self) -> bool {
        self.started.len() < self.depth
    }
    fn start(&mut self, command: Command) -> Result<u32, BlockError> {
        assert!(self.room(), "a command past the device's depth");
        self.given.borrow_mut().push(command.clone());
        let tag = self.next;
        self.next += 1;
        self.started.push_back((tag, command));
        Ok(tag)
    }
    fn commit(&mut self) {
        self.commits.set(self.commits.get() + 1);
    }
    fn finish(&mut self, tag: u32) -> Option<Finished> {
        self.finishes.set(self.finishes.get() + 1);
        self.complete();
        self.done.remove(&tag)
    }
    fn reaps_together(&self) -> bool {
        self.together
    }
    fn finished(&mut self, tag: u32) -> Option<Finished> {
        self.together.then(|| self.done.remove(&tag)).flatten()
    }
}

fn write(lba: u64, byte: u8) -> Command {
    Command::Write {
        lba,
        data: vec![byte; 512],
        fua: false,
    }
}

#[test]
fn callback_profile_is_default_off_and_empty_advance_adds_no_clock_read() {
    let _clock = crate::test_clock::virtualize();
    let disk = Shared::new(Box::new(RamDisk::new("profile", 8, 512)));
    let mut held = disk.lock();
    let before = crate::Clock::ticks();
    let single_read = crate::Clock::ticks().wrapping_sub(before);
    let before = crate::Clock::ticks();
    held.advance();
    assert_eq!(crate::Clock::ticks().wrapping_sub(before), single_read);
    assert_eq!(held.stats().callbacks, None);

    disk.set_callback_profile(true);
    let before = crate::Clock::ticks();
    held.advance();
    assert_eq!(crate::Clock::ticks().wrapping_sub(before), 3 * single_read);
    let recorded = held.stats().callbacks.unwrap();
    assert_eq!(recorded.advance.calls, 1);
    assert_eq!(recorded.advance.maximum_ticks, single_read);
    assert_eq!((recorded.advance.starts, recorded.advance.finishes), (0, 0));
    assert_eq!(
        disk.callback_stats(),
        None,
        "a held owner is never waited on"
    );

    disk.set_callback_profile(false);
    let before = crate::Clock::ticks();
    held.advance();
    assert_eq!(crate::Clock::ticks().wrapping_sub(before), single_read);
    assert_eq!(held.stats().callbacks, Some(recorded));
    drop(held);
    assert_eq!(disk.callback_stats(), None, "disabled sampling stays off");
    disk.set_callback_profile(true);
    let before = crate::Clock::ticks();
    assert_eq!(disk.callback_stats(), Some(recorded));
    assert_eq!(crate::Clock::ticks().wrapping_sub(before), single_read);
}

#[test]
fn callback_profile_times_returned_direct_errors_without_changing_them() {
    let _clock = crate::test_clock::virtualize();
    let disk = Shared::new(Box::new(RamDisk::new("profile", 8, 512)));
    disk.set_callback_profile(true);
    let mut held = disk.lock();
    let result: Result<(), BlockError> =
        held.direct((Direction::Write, 512), crate::Clock::ticks(), |_| {
            Err(BlockError::Device { status: 7 })
        });
    assert_eq!(result, Err(BlockError::Device { status: 7 }));
    let recorded = held.stats().callbacks.unwrap();
    assert_eq!((recorded.direct.calls, recorded.advance.calls), (1, 0));
    assert!(recorded.direct.elapsed_ticks > 0);
    assert_eq!(recorded.direct.maximum_ticks, recorded.direct.elapsed_ticks);
}

#[test]
fn callback_profile_counts_queued_work_per_advance() {
    let _clock = crate::test_clock::virtualize();
    let (delayed, (released, _, _)) = Delayed::new(RamDisk::new("profile", 64, 512), 1);
    let disk = Shared::new(Box::new(delayed));
    let mut window = disk.window(0, 64).unwrap();
    let first = window.start(write(1, 1)).unwrap();
    let second = window.start(write(2, 2)).unwrap();
    disk.set_callback_profile(true);
    released.set(1);
    assert_eq!(window.finish(first), Some(Ok(None)));
    let recorded = disk.stats().callbacks.unwrap();
    assert_eq!(
        (
            recorded.advance.calls,
            recorded.advance.starts,
            recorded.advance.finishes
        ),
        (1, 1, 1)
    );
    assert_eq!(
        (
            recorded.advance.maximum_starts,
            recorded.advance.maximum_finishes
        ),
        (1, 1)
    );
    assert_eq!(recorded.direct.calls, 0);
    disk.set_callback_profile(false);
    released.set(1);
    assert_eq!(window.finish(second), Some(Ok(None)));
    assert_eq!(disk.stats().callbacks, Some(recorded));
}

#[test]
fn windows_queue_commands_and_whichever_looks_moves_the_queue_on() {
    let (delayed, (released, _, given)) = Delayed::new(RamDisk::new("q", 64, 512), 1);
    let disk = Shared::new(Box::new(delayed));
    let (mut a, mut b) = (disk.window(0, 32).unwrap(), disk.window(32, 32).unwrap());
    let first = a.start(write(1, 1)).unwrap();
    let read = b.start(Command::Read { lba: 1, bytes: 512 }).unwrap();
    assert_eq!((a.finish(first), b.finish(read)), (None, None));
    // B's look collects A's write and starts B's read.
    released.set(1);
    assert_eq!(b.finish(read), None);
    assert_eq!(a.finish(first), Some(Ok(None)));
    released.set(1);
    assert_eq!(b.finish(read), Some(Ok(Some(vec![0; 512]))));
    assert_eq!(given.borrow().len(), 2);
    // A command outside its window refuses before it queues.
    let outside = Command::Read {
        lba: 32,
        bytes: 512,
    };
    assert!(a.start(outside).is_err());
}

#[test]
fn a_single_queued_read_returns_the_devices_original_buffer() {
    let data: Vec<u8> = (0..4096).map(|byte| (byte % 251) as u8).collect();
    let mut ram = RamDisk::new("single", 16, 512);
    ram.write(1, &data).unwrap();
    let (delayed, (released, _, _)) = Delayed::new(ram, 1);
    let address = delayed.last_read_address.clone();
    let disk = Shared::new(Box::new(delayed));
    let mut window = disk.whole();
    let mut read = Submission::new(
        Request::Read {
            lba: 1,
            bytes: data.len(),
        },
        window.identity(),
    )
    .unwrap();
    assert_eq!(read.poll(&mut window), None);
    released.set(1);
    let result = read.poll(&mut window).unwrap().unwrap().unwrap();
    assert_eq!(result, data);
    assert_eq!(result.as_ptr() as usize, address.get());
    assert_eq!(read.poll(&mut window), Some(Ok(None)));
}

#[test]
fn bulk_submission_respects_window_capacity() {
    let blocks = 2 * WINDOW_DEPTH + 3;
    let data: Vec<u8> = (0..blocks * 512).map(|byte| (byte / 512) as u8).collect();
    let mut ram = RamDisk::new("bounded", blocks as u64, 512);
    ram.identity.features.max_transfer = 512;
    ram.write(0, &data).unwrap();
    let (delayed, (released, _, given)) = Delayed::new(ram, 1);
    let disk = Shared::new(Box::new(delayed));
    let mut window = disk.whole();
    let mut submission = Submission::new(
        Request::Read {
            lba: 0,
            bytes: data.len(),
        },
        window.identity(),
    )
    .unwrap();
    assert_eq!(submission.poll(&mut window), None);
    assert!(!window.room());
    assert_eq!(given.borrow().len(), 1);
    assert_eq!(
        window.start(Command::Read { lba: 0, bytes: 512 }),
        Err(BlockError::Protocol("the disk window queue is full"))
    );
    released.set(usize::MAX);
    let mut completed = None;
    for _ in 0..blocks.div_ceil(WINDOW_DEPTH) {
        if let Some(result) = submission.poll(&mut window) {
            completed = Some(result);
            break;
        }
    }
    assert_eq!(completed, Some(Ok(Some(data))));
    assert!(window.room());
    assert_eq!(given.borrow().len(), blocks);
}

#[test]
fn directional_transfer_limits_split_and_merge_only_reads_at_the_larger_bound() {
    let mut ram = RamDisk::new("directional", 16, 512);
    ram.identity.features.max_transfer = 512;
    ram.identity.features.max_read_transfer = NonZeroUsize::new(1024);
    let read = Plan::new(
        Request::Read {
            lba: 0,
            bytes: 1536,
        },
        ram.identity(),
    )
    .unwrap();
    assert_eq!(
        read.collect::<Vec<_>>(),
        [
            (
                0,
                Command::Read {
                    lba: 0,
                    bytes: 1024
                }
            ),
            (1024, Command::Read { lba: 2, bytes: 512 }),
        ]
    );
    let write_plan = Plan::new(
        Request::Write {
            lba: 0,
            data: vec![3; 1024],
            durable: false,
        },
        ram.identity(),
    )
    .unwrap();
    assert_eq!(write_plan.count(), 2);

    let (delayed, (released, _, given)) = Delayed::new(ram, 1);
    let disk = Shared::new(Box::new(delayed));
    let mut window = disk.whole();
    let commands = [
        write(0, 1),
        write(1, 2),
        write(2, 3),
        Command::Read { lba: 1, bytes: 512 },
        Command::Read { lba: 2, bytes: 512 },
    ];
    let tags = commands.map(|command| window.start(command).unwrap());
    released.set(usize::MAX);
    for tag in &tags[..3] {
        assert_eq!(window.finish(*tag), Some(Ok(None)));
    }
    assert_eq!(window.finish(tags[3]), Some(Ok(Some(vec![2; 512]))));
    assert_eq!(window.finish(tags[4]), Some(Ok(Some(vec![3; 512]))));
    assert_eq!(
        *given.borrow(),
        [
            write(0, 1),
            write(1, 2),
            write(2, 3),
            Command::Read {
                lba: 1,
                bytes: 1024
            },
        ]
    );
}

#[test]
fn adjoining_waiting_commands_reach_the_device_as_one() {
    let (delayed, (released, _, given)) = Delayed::new(RamDisk::new("m", 64, 512), 1);
    let disk = Shared::new(Box::new(delayed));
    let mut window = disk.whole();
    let tags: Vec<u32> = [write(0, 1), write(1, 2), write(2, 3), write(9, 4)]
        .into_iter()
        .map(|command| window.start(command).unwrap())
        .collect();
    let reads: Vec<u32> = [(1, 1024), (3, 512)]
        .into_iter()
        .map(|(lba, bytes)| window.start(Command::Read { lba, bytes }).unwrap())
        .collect();
    released.set(usize::MAX);
    for tag in tags {
        assert_eq!(window.finish(tag), Some(Ok(None)));
    }
    let mut data = vec![2; 512];
    data.extend_from_slice(&[3; 512]);
    assert_eq!(window.finish(reads[0]), Some(Ok(Some(data))));
    assert_eq!(window.finish(reads[1]), Some(Ok(Some(vec![0; 512]))));
    // The first write was alone in flight; the next two joined, the
    // fourth does not adjoin them, and the reads joined.
    let given = given.borrow();
    let shapes: Vec<(bool, u64, usize)> = given
        .iter()
        .map(|command| match command {
            Command::Read { lba, bytes } => (true, *lba, *bytes),
            Command::Write { lba, data, .. } => (false, *lba, data.len()),
            _ => unreachable!(),
        })
        .collect();
    assert_eq!(
        shapes,
        [
            (false, 0, 512),
            (false, 1, 1024),
            (false, 9, 512),
            (true, 1, 1536)
        ]
    );
    // Accounted as Linux's diskstats: a command each, merges apart.
    let stats = disk.stats();
    let [read, write, ..] = stats.directions;
    assert_eq!((read.ios, read.merges, read.bytes), (1, 1, 1536));
    assert_eq!((write.ios, write.merges, write.bytes), (3, 1, 2048));
    assert_eq!(stats.deepest, 1);
}

fn discard(lba: u64, blocks: u64) -> Request {
    Request::Discard {
        extents: vec![Extent { lba, blocks }],
    }
}

fn cached(fua: bool, max_transfer: usize) -> Identity {
    let mut identity = RamDisk::new("p", 64, 512).identity;
    identity.features = Features {
        write_cache: true,
        fua,
        max_transfer,
        ..Features::default()
    };
    identity
}

#[test]
fn the_block_layer_sends_only_the_flushes_a_device_needs() {
    let durable = || Request::Write {
        lba: 2,
        data: vec![7; 1024],
        durable: true,
    };
    let commands = |request: Request, identity: &Identity| {
        let plan = request.plan(identity).unwrap();
        plan.into_iter()
            .map(|(_, command)| command)
            .collect::<Vec<_>>()
    };
    let fua = Command::Write {
        lba: 2,
        data: vec![7; 1024],
        fua: true,
    };
    let plain = Command::Write {
        lba: 2,
        data: vec![7; 1024],
        fua: false,
    };
    // FUA where the device has it, else a flush after; nothing extra
    // without a write cache.
    assert_eq!(commands(durable(), &cached(true, 0)), [fua]);
    assert_eq!(
        commands(durable(), &cached(false, 0)),
        [plain.clone(), Command::Flush]
    );
    let uncached = RamDisk::new("p", 64, 512).identity;
    assert_eq!(commands(durable(), &uncached), [plain]);
    assert!(commands(Request::Flush, &uncached).is_empty());
    assert_eq!(
        commands(Request::Flush, &cached(false, 0)),
        [Command::Flush]
    );
    // Transfers split at the device's limit; Write Zeroes without the
    // command writes zeros; discard without it refuses.
    let split = Request::Read {
        lba: 0,
        bytes: 2048,
    }
    .plan(&cached(true, 1024))
    .unwrap();
    assert_eq!(
        split,
        [
            (
                0,
                Command::Read {
                    lba: 0,
                    bytes: 1024
                }
            ),
            (
                1024,
                Command::Read {
                    lba: 2,
                    bytes: 1024
                }
            )
        ]
    );
    let zeroes = Request::WriteZeroes {
        lba: 4,
        blocks: 3,
        unmap: true,
    };
    assert_eq!(
        commands(zeroes, &cached(false, 1024)),
        [
            Command::Write {
                lba: 4,
                data: vec![0; 1024],
                fua: false
            },
            Command::Write {
                lba: 6,
                data: vec![0; 512],
                fua: false
            }
        ]
    );
    let discard = discard(0, 1);
    assert!(discard.plan(&uncached).is_err());
}

#[test]
fn a_submission_runs_its_writes_together_and_its_flush_after_them() {
    let mut ram = RamDisk::new("s", 64, 512);
    ram.identity = cached(false, 512);
    let (mut delayed, (released, _, given)) = Delayed::new(ram, 4);
    let request = Request::Write {
        lba: 0,
        data: vec![5; 1536],
        durable: true,
    };
    let mut submission = Submission::new(request, delayed.identity()).unwrap();
    assert_eq!(submission.poll(&mut delayed), None);
    assert_eq!(given.borrow().len(), 3, "the three pieces run together");
    released.set(2);
    assert_eq!(submission.poll(&mut delayed), None);
    assert_eq!(given.borrow().len(), 3, "the flush waits for every piece");
    released.set(1);
    assert_eq!(submission.poll(&mut delayed), None);
    assert_eq!(given.borrow().last(), Some(&Command::Flush));
    released.set(1);
    assert_eq!(submission.poll(&mut delayed), Some(Ok(None)));
    assert_eq!(delayed.disk.flushes, 1);
    assert_eq!(&delayed.disk.bytes[..1536], &[5; 1536]);
}

#[test]
fn range_plans_keep_chunk_sizes_and_unmap_permissions() {
    let mut identity = cached(false, 1024);
    identity.features.discard = Some(DiscardLimits::single(2));
    identity.features.write_zeroes = Some(WriteZeroes {
        blocks: 3,
        unmap: false,
    });
    let commands = |request: Request, identity: &Identity| {
        Plan::new(request, identity)
            .unwrap()
            .map(|(_, command)| command)
            .collect::<Vec<_>>()
    };
    assert_eq!(
        commands(discard(4, 7), &identity),
        [(4, 2), (6, 2), (8, 2), (10, 1)].map(|(lba, blocks)| Command::discard(lba, blocks))
    );
    for allowed in [false, true] {
        identity.features.write_zeroes.as_mut().unwrap().unmap = allowed;
        for requested in [false, true] {
            assert_eq!(
                commands(
                    Request::WriteZeroes {
                        lba: 4,
                        blocks: 7,
                        unmap: requested,
                    },
                    &identity,
                ),
                [(4, 3), (7, 3), (10, 1)].map(|(lba, blocks)| Command::WriteZeroes {
                    lba,
                    blocks,
                    unmap: allowed && requested,
                })
            );
        }
    }
    identity.features.discard = Some(DiscardLimits::single(0));
    assert_eq!(commands(discard(4, 7), &identity), [Command::discard(4, 7)]);
    // Ranges fill each command in order, cut at the limit on a range.
    identity.features.discard = Some(DiscardLimits {
        blocks: 3,
        ranges: 2,
    });
    let extents = |ranges: &[(u64, u64)]| {
        ranges
            .iter()
            .map(|&(lba, blocks)| Extent { lba, blocks })
            .collect::<Vec<_>>()
    };
    assert_eq!(
        commands(
            Request::Discard {
                extents: extents(&[(0, 5), (9, 0), (20, 1), (30, 1)]),
            },
            &identity,
        ),
        [
            Command::Discard {
                extents: extents(&[(0, 3), (3, 2)]),
            },
            Command::Discard {
                extents: extents(&[(20, 1), (30, 1)]),
            },
        ]
    );
    identity.features.write_zeroes.as_mut().unwrap().blocks = 0;
    assert_eq!(
        commands(
            Request::WriteZeroes {
                lba: 4,
                blocks: 7,
                unmap: true
            },
            &identity,
        ),
        [Command::WriteZeroes {
            lba: 4,
            blocks: 7,
            unmap: true
        }]
    );
}

#[test]
fn batch_preflight_rejects_later_invalid_requests() {
    let mut identity = cached(false, 512);
    identity.features.discard = Some(DiscardLimits::single(2));
    for invalid in [
        Request::Write {
            lba: 0,
            data: vec![0; 1],
            durable: false,
        },
        discard(63, 2),
        Request::WriteZeroes {
            lba: 1,
            blocks: u64::MAX,
            unmap: false,
        },
        Request::Read { lba: 0, bytes: 512 },
    ] {
        assert!(
            Submission::batch(
                vec![
                    Request::Write {
                        lba: 0,
                        data: vec![5; 512],
                        durable: true
                    },
                    invalid
                ],
                &identity,
            )
            .is_err()
        );
    }
    identity.read_only = true;
    assert!(matches!(
        Submission::new(
            Request::WriteZeroes {
                lba: 0,
                blocks: 1,
                unmap: false
            },
            &identity
        ),
        Err(BlockError::ReadOnly)
    ));
    identity.read_only = false;
    identity.features.discard = None;
    assert!(matches!(
        Submission::new(discard(0, 1), &identity),
        Err(BlockError::Unsupported(_))
    ));
}

#[test]
fn durable_write_flush_waits_across_window_refills() {
    let blocks = WINDOW_DEPTH + 5;
    let mut ram = RamDisk::new("flush", blocks as u64, 512);
    ram.identity.features = cached(false, 512).features;
    let (delayed, (released, _, given)) = Delayed::new(ram, 2);
    let mut window = Shared::new(Box::new(delayed)).whole();
    let data = vec![0x5a; blocks * 512];
    let mut submission = Submission::new(
        Request::Write {
            lba: 0,
            data: data.clone(),
            durable: true,
        },
        window.identity(),
    )
    .unwrap();
    assert_eq!(submission.poll(&mut window), None);
    assert!(!window.room());
    released.set(usize::MAX);
    assert_eq!(submission.poll(&mut window), None);
    assert!(!given.borrow().contains(&Command::Flush));
    assert_eq!(submission.poll(&mut window), None);
    assert_eq!(given.borrow().last(), Some(&Command::Flush));
    assert_eq!(submission.poll(&mut window), Some(Ok(None)));
    assert_eq!(
        given
            .borrow()
            .iter()
            .filter(|cmd| **cmd == Command::Flush)
            .count(),
        1
    );
    let mut read = vec![0; data.len()];
    window.read(0, &mut read).unwrap();
    assert_eq!(read, data);
}

#[test]
fn streamed_zero_fallback_preserves_surrounding_blocks() {
    let mut ram = RamDisk::new("zero", 8, 512);
    ram.bytes.fill(0x5a);
    ram.identity.features.max_transfer = 1024;
    assert_eq!(
        (Request::WriteZeroes {
            lba: 1,
            blocks: 5,
            unmap: true
        })
        .run(&mut ram),
        Ok(None)
    );
    assert_eq!(&ram.bytes[..512], &[0x5a; 512]);
    assert!(ram.bytes[512..6 * 512].iter().all(|&byte| byte == 0));
    assert!(ram.bytes[6 * 512..].iter().all(|&byte| byte == 0x5a));
    assert_eq!(
        Request::Write {
            lba: 0,
            data: vec![],
            durable: true
        }
        .plan(&cached(false, 512))
        .unwrap(),
        [
            (
                0,
                Command::Write {
                    lba: 0,
                    data: vec![],
                    fua: false
                }
            ),
            (0, Command::Flush),
        ]
    );
}

#[test]
fn a_sequential_reader_finds_the_chunks_read_ahead_in_memory() {
    let mut ram = RamDisk::new("r", 16, 512);
    for (index, byte) in ram.bytes.iter_mut().enumerate() {
        *byte = (index / 512) as u8;
    }
    let (delayed, (released, reads, _)) = Delayed::new(ram, 2);
    let ahead = Ahead::new(ReadAhead::new(Box::new(delayed), 1024, 2));
    let mut reader = ahead.clone();
    let mut block = [0; 512];
    reader.read(0, &mut block).unwrap();
    assert_eq!((block[0], reads.get()), (0, 1), "a miss reads directly");
    released.set(usize::MAX);
    for _ in 0..3 {
        ahead.pump();
    }
    let mut two = [0; 1024];
    reader.read(1, &mut two).unwrap();
    assert_eq!((two[0], two[512], reads.get()), (1, 2, 1), "found ahead");
    reader.read(3, &mut block).unwrap();
    assert_eq!((block[0], reads.get()), (3, 1));
}

/// Owners on different processors at once each reach the disk whole:
/// one guest's storage on a PE, another's store on the boot processor.
#[test]
fn windows_on_different_threads_take_turns_with_the_disk() {
    let disk = Shared::new(Box::new(RamDisk::new("s", 64, 512)));
    std::thread::scope(|scope| {
        for half in 0..2u8 {
            let mut window = disk.window(u64::from(half) * 32, 32).unwrap();
            scope.spawn(move || {
                for round in 0..200u8 {
                    let data = vec![half ^ round; 32 * 512];
                    window.write(0, &data).unwrap();
                    let mut read = vec![0; data.len()];
                    window.read(0, &mut read).unwrap();
                    assert_eq!(read, data);
                }
            });
        }
    });
}

#[test]
fn transfers_must_be_whole_blocks_inside_the_device() {
    let disk = RamDisk::new("s", 8, 512);
    assert_eq!(disk.identity.check(0, 4096), Ok(8));
    assert_eq!(disk.identity.check(7, 512), Ok(1));
    assert_eq!(
        disk.identity.check(7, 1024),
        Err(BlockError::OutOfRange {
            lba: 7,
            blocks: 2,
            capacity: 8
        })
    );
    assert_eq!(
        disk.identity.check(u64::MAX, 512),
        Err(BlockError::OutOfRange {
            lba: u64::MAX,
            blocks: 1,
            capacity: 8
        })
    );
    assert_eq!(
        disk.identity.check(0, 100),
        Err(BlockError::Misaligned {
            bytes: 100,
            block_size: 512
        })
    );
    let mut read_only = disk.identity.clone();
    read_only.read_only = true;
    assert_eq!(read_only.check_write(0, 512), Err(BlockError::ReadOnly));
}

#[test]
fn windows_share_one_disk_and_stay_inside_their_blocks() {
    let shared = Shared::new(Box::new(RamDisk::new("s", 8, 512)));
    let mut low = shared.window(0, 4).unwrap();
    let mut high = shared.window(4, 4).unwrap();
    assert!(shared.window(6, 3).is_err());
    assert_eq!(high.identity().blocks, 4);
    high.write(0, &[7; 512]).unwrap();
    let mut block = [0; 512];
    low.read(3, &mut block).unwrap();
    assert_eq!(block, [0; 512]);
    assert!(low.read(4, &mut block).is_err());
    let mut whole = shared.window(0, 8).unwrap();
    whole.read(4, &mut block).unwrap();
    assert_eq!(block, [7; 512]);
    low.flush().unwrap();
}

#[test]
fn runs_join_scattered_blocks_into_one_device() {
    let shared = Shared::new(Box::new(RamDisk::new("s", 16, 512)));
    assert!(shared.runs(vec![(0, 2), (15, 2)]).is_err());
    let mut runs = shared.runs(vec![(10, 2), (3, 1), (6, 3)]).unwrap();
    assert_eq!(runs.identity().blocks, 6);
    let data: Vec<u8> = (0..6 * 512).map(|i| (i / 512) as u8 + 1).collect();
    runs.write(0, &data).unwrap();
    assert!(runs.write(5, &[0; 1024]).is_err());
    let mut whole = shared.whole();
    let mut block = [0; 512];
    for (lba, value) in [(10, 1), (11, 2), (3, 3), (6, 4), (7, 5), (8, 6), (9, 0)] {
        whole.read(lba, &mut block).unwrap();
        assert_eq!(block, [value; 512], "disk block {lba}");
    }
    let mut back = vec![0; 3 * 512];
    runs.read(1, &mut back).unwrap();
    assert_eq!(back, data[512..4 * 512]);
    runs.flush().unwrap();
}

/// A log file's runs on a disk that queues commands: a submission's store
/// starts one command per run it reaches and returns before the disk runs
/// them, where runs took no queued command and every store ran its writes
/// where it was asked, waiting for the disk and the commands queued ahead.
#[test]
fn runs_queue_each_piece_on_the_disk_and_never_run_one_where_asked() {
    let (delayed, (released, _, given)) = Delayed::new(RamDisk::new("q", 16, 512), 4);
    let shared = Shared::new(Box::new(delayed));
    let mut runs = shared.runs(vec![(10, 2), (3, 1), (6, 3)]).unwrap();
    assert!(runs.depth() > 0 && runs.room());
    // Another owner's command is queued ahead.
    let mut other = shared.window(0, 3).unwrap();
    let ahead = other.start(write(0, 9)).unwrap();
    let data: Vec<u8> = (0..4 * 512).map(|i| (i / 512) as u8 + 1).collect();
    let mut store = Submission::new(
        Request::Write {
            lba: 1,
            data: data.clone(),
            durable: false,
        },
        runs.identity(),
    )
    .unwrap();
    assert_eq!(store.poll(&mut runs), None);
    // Its three pieces reached the disk's queue behind the other's write,
    // and nothing ran.
    let lbas: Vec<u64> = given
        .borrow()
        .iter()
        .map(|command| match command {
            Command::Write { lba, .. } => *lba,
            _ => panic!("only writes"),
        })
        .collect();
    assert_eq!(lbas, [0, 11, 3, 6]);
    assert_eq!(store.poll(&mut runs), None);
    released.set(4);
    assert_eq!(store.poll(&mut runs), Some(Ok(None)));
    assert_eq!(other.finish(ahead), Some(Ok(None)));
    // A queued read across the runs assembles its pieces in order.
    let mut read = Submission::new(
        Request::Read {
            lba: 0,
            bytes: 6 * 512,
        },
        runs.identity(),
    )
    .unwrap();
    assert_eq!(read.poll(&mut runs), None);
    released.set(3);
    let back = read.poll(&mut runs).unwrap().unwrap().unwrap();
    assert_eq!(&back[..512], &[0; 512]);
    assert_eq!(&back[512..5 * 512], &data[..]);
    assert_eq!(&back[5 * 512..], &[0; 512]);
    // A command past the runs refuses before any piece queues.
    let queued = given.borrow().len();
    assert!(runs.start(write(6, 1)).is_err());
    assert_eq!(given.borrow().len(), queued);
}

#[test]
fn a_volatile_disk_reads_zeros_until_written_and_refuses_past_its_limit() {
    assert!(Volatile::new("ram", 4096, 0).is_err());
    let chunk = Volatile::CHUNK;
    let mut disk = Volatile::new("ram", 1 << 30, 2 * chunk).unwrap();
    assert_eq!(disk.identity().blocks, (1 << 30) / 4096);
    assert_eq!(disk.identity().transport, Transport::Ram);
    let mut block = vec![1; 4096];
    disk.read(1000, &mut block).unwrap();
    assert!(block.iter().all(|&b| b == 0));
    // Zeros take no memory; a write spanning two chunks takes both.
    disk.write(0, &vec![0; 4 * chunk]).unwrap();
    assert!(disk.chunks.is_empty());
    let span: Vec<u8> = (0..8192).map(|i| (i % 251) as u8 + 1).collect();
    let lba = (chunk / 4096 - 1) as u64;
    disk.write(lba, &span).unwrap();
    assert_eq!(disk.chunks.len(), 2);
    let mut back = vec![0; 8192];
    disk.read(lba, &mut back).unwrap();
    assert_eq!(back, span);
    // A third chunk is past the limit: the write refuses and changes
    // nothing, while chunks already taken still accept writes.
    let far = (8 * chunk / 4096) as u64;
    assert!(disk.write(far, &[9; 4096]).is_err());
    disk.read(far, &mut block).unwrap();
    assert!(block.iter().all(|&b| b == 0));
    disk.write(0, &[7; 4096]).unwrap();
    disk.read(0, &mut block).unwrap();
    assert!(block.iter().all(|&b| b == 7));
    assert!(disk.read(disk.identity().blocks, &mut block).is_err());
    disk.flush().unwrap();
}

#[test]
fn identify_strings_lose_padding_and_terminators() {
    assert_eq!(ascii(b"  S4EVNX0R123   "), "S4EVNX0R123");
    assert_eq!(ascii(b"QEMU\0junk"), "QEMU");
    assert_eq!(ascii(b"a\x01b"), "a?b");
}

#[test]
fn only_a_write_zeroes_that_stores_zeros_counts_as_written() {
    let zeroes = |unmap| Command::WriteZeroes {
        lba: 0,
        blocks: 8,
        unmap,
    };
    assert_eq!(zeroes(false).accounted(512), (Direction::Write, 4096));
    assert_eq!(zeroes(true).accounted(512), (Direction::Discard, 4096));
}

#[test]
fn a_start_with_room_reaches_no_completion_and_one_finish_reaps_them_all() {
    let (mut delayed, (released, _, _)) = Delayed::new(RamDisk::new("reap", 64, 512), 8);
    delayed.together = true;
    let finishes = delayed.finishes.clone();
    let disk = Shared::new(Box::new(delayed));
    let (mut a, mut b) = (disk.window(0, 32).unwrap(), disk.window(32, 32).unwrap());
    let tickets: Vec<u32> = (0..3).map(|lba| a.start(write(lba, 1)).unwrap()).collect();
    let other = b.start(write(8, 2)).unwrap();
    // Starting with room looks at no completion.
    assert_eq!(finishes.get(), 0);
    released.set(4);
    // One look collects all four: the device is reached once.
    assert_eq!(a.finish(tickets[2]), Some(Ok(None)));
    assert_eq!(finishes.get(), 1);
    // What it collected needs no device.
    assert_eq!(a.finished(tickets[0]), Some(Ok(None)));
    assert_eq!(b.finished(other), Some(Ok(None)));
    assert_eq!(a.finish(tickets[1]), Some(Ok(None)));
    assert_eq!(finishes.get(), 1);
    assert_eq!(a.finished(tickets[1]), None);
}

#[test]
fn a_disk_that_reaps_one_at_a_time_is_asked_for_each() {
    let (delayed, (released, _, _)) = Delayed::new(RamDisk::new("each", 64, 512), 8);
    let finishes = delayed.finishes.clone();
    let disk = Shared::new(Box::new(delayed));
    let mut window = disk.whole();
    let tickets: Vec<u32> = (0..3)
        .map(|lba| window.start(write(lba, 1)).unwrap())
        .collect();
    released.set(3);
    assert_eq!(window.finish(tickets[0]), Some(Ok(None)));
    // Without `reaps_together` every running command was asked.
    assert_eq!(finishes.get(), 3);
    assert_eq!(window.finished(tickets[1]), Some(Ok(None)));
    assert_eq!(window.finished(tickets[2]), Some(Ok(None)));
}

/// A window's starts with more to follow reach the device in order and tell
/// it once, at the commit; a plain start and a look tell it at once.
#[test]
fn a_window_tells_its_device_of_a_batch_once() {
    let (delayed, (released, _, given)) = Delayed::new(RamDisk::new("batch", 8, 512), 4);
    let commits = delayed.commits.clone();
    let mut window = Shared::new(Box::new(delayed)).whole();
    let first = window.start_more(write(0, 1)).unwrap();
    let second = window.start_more(write(1, 2)).unwrap();
    assert_eq!(given.borrow().as_slice(), &[write(0, 1), write(1, 2)]);
    assert_eq!(commits.get(), 0);
    window.commit();
    assert_eq!(commits.get(), 1);
    window.start(write(2, 3)).unwrap();
    assert_eq!(commits.get(), 2);
    released.set(3);
    assert_eq!(window.finish(first), Some(Ok(None)));
    assert_eq!(window.finish(second), Some(Ok(None)));
}

/// A merged read's first request keeps the device's buffer, cut to its
/// part; each later request gets its own part, in order.
#[test]
fn a_merged_reads_first_part_keeps_the_devices_buffer() {
    let mut ram = RamDisk::new("merged", 16, 512);
    let data: Vec<u8> = (0..1536).map(|byte| (byte % 251) as u8).collect();
    ram.write(4, &data).unwrap();
    let (delayed, (released, _, given)) = Delayed::new(ram, 1);
    let address = delayed.last_read_address.clone();
    let mut window = Shared::new(Box::new(delayed)).whole();
    // The first command holds the device; the three reads wait and join.
    let first = window.start(write(0, 1)).unwrap();
    let reads: Vec<u32> = [(4, 512), (5, 512), (6, 512)]
        .into_iter()
        .map(|(lba, bytes)| window.start(Command::Read { lba, bytes }).unwrap())
        .collect();
    released.set(usize::MAX);
    assert_eq!(window.finish(first), Some(Ok(None)));
    let parts: Vec<Vec<u8>> = reads
        .iter()
        .map(|&read| window.finish(read).unwrap().unwrap().unwrap())
        .collect();
    assert_eq!(
        given.borrow().last(),
        Some(&Command::Read {
            lba: 4,
            bytes: 1536
        })
    );
    for (index, part) in parts.iter().enumerate() {
        assert_eq!(part.as_slice(), &data[index * 512..][..512]);
    }
    assert_eq!(parts[0].as_ptr() as usize, address.get());
}

/// While another owner holds the disk, which may itself wait long for a
/// lock its device needs, a window's looks come back later instead of
/// waiting: no result, no room, a possible completion, and a commit that
/// the next call taking the disk makes.
#[test]
fn a_window_never_waits_to_look_at_a_disk_another_owner_holds() {
    let (delayed, (released, _, _)) = Delayed::new(RamDisk::new("held", 8, 512), 4);
    let commits = delayed.commits.clone();
    let disk = Shared::new(Box::new(delayed));
    let mut window = disk.whole();
    let tag = window.start_more(write(0, 1)).unwrap();
    released.set(1);
    {
        let _held = disk.lock();
        assert!(disk.try_lock().is_none());
        window.commit();
        assert_eq!(commits.get(), 0);
        assert!(!window.room());
        assert!(window.posted());
        assert_eq!(window.finished(tag), None);
        assert_eq!(window.finish(tag), None);
    }
    assert!(window.room());
    assert_eq!(window.finish(tag), Some(Ok(None)));
    assert_eq!(commits.get(), 1);
    // A plain start tells the device of a put-off commit too.
    let tag = window.start_more(write(1, 2)).unwrap();
    {
        let _held = disk.lock();
        window.commit();
    }
    window.start(write(2, 3)).unwrap();
    assert_eq!(commits.get(), 2);
    released.set(2);
    assert_eq!(window.finish(tag), Some(Ok(None)));
}

/// Another owner may take the disk between a window's `room` and its
/// start: the start waits for it instead of refusing, as its caller found
/// room (a NOR store failed on this, and the machine stopped).
#[test]
fn a_window_start_waits_for_an_owner_that_took_the_disk_after_room() {
    extern crate std;
    use std::{sync::mpsc, thread, time::Duration};
    let disk = Shared::new(Box::new(RamDisk::new("raced", 8, 512)));
    let mut window = disk.whole();
    assert!(window.room());
    let (taken, wait) = mpsc::channel();
    let other = disk.clone();
    let owner = thread::spawn(move || {
        let _held = other.lock();
        taken.send(()).unwrap();
        thread::sleep(Duration::from_millis(50));
    });
    wait.recv().unwrap();
    assert!(!window.room());
    let tag = window.start(write(0, 1)).unwrap();
    owner.join().unwrap();
    let mut result = None;
    for _ in 0..1000 {
        result = window.finish(tag);
        if result.is_some() {
            break;
        }
    }
    assert_eq!(result, Some(Ok(None)));
}

#[test]
fn a_rotation_rate_reads_as_ata_word_217_and_scsi_vpd_b1_encode_it() {
    assert_eq!(Rotation::from_nominal(0), Rotation::Unreported);
    assert_eq!(Rotation::from_nominal(1), Rotation::None);
    // 0002h-0400h and FFFFh are reserved.
    for reserved in [2, 0x0400, 0xffff] {
        assert_eq!(Rotation::from_nominal(reserved), Rotation::Unreported);
    }
    assert_eq!(Rotation::from_nominal(0x0401), Rotation::Rpm(0x0401));
    assert_eq!(Rotation::from_nominal(5400), Rotation::Rpm(5400));
    assert_eq!(Rotation::from_nominal(0xfffe), Rotation::Rpm(0xfffe));
    assert_eq!(alloc::format!("{}", Rotation::Rpm(7200)), "7200rpm");
    assert_eq!(alloc::format!("{}", Rotation::None), "none");
    assert_eq!(alloc::format!("{}", Rotation::default()), "unreported");
}
