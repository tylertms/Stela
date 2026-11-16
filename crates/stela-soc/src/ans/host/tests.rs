use super::*;
use alloc::{rc::Rc, vec, vec::Vec};
use core::cell::RefCell;
use stela_drivers::block::{Command, DiscardLimits, RamDisk, WriteZeroes};

fn runs(discards: &Discards) -> Vec<(u64, u64)> {
    discards
        .waiting
        .iter()
        .map(|(&start, &end)| (start, end))
        .collect()
}

#[test]
fn waiting_runs_merge_what_touches_and_keep_the_rest() {
    let mut discards = Discards::default();
    discards.add(10, 20);
    discards.add(30, 40);
    discards.add(50, 60);
    assert_eq!(runs(&discards), [(10, 20), (30, 40), (50, 60)]);
    // Adjacent on both sides, then swallowing runs whole.
    discards.add(20, 30);
    assert_eq!(runs(&discards), [(10, 40), (50, 60)]);
    discards.add(5, 70);
    assert_eq!(runs(&discards), [(5, 70)]);
    discards.add(6, 7);
    assert_eq!(runs(&discards), [(5, 70)]);
}

#[test]
fn a_write_takes_its_blocks_back_from_every_run_it_touches() {
    let mut discards = Discards::default();
    discards.add(10, 20);
    discards.add(30, 40);
    discards.add(50, 60);
    // Inside a run, which splits, then across the ends of two and a whole one.
    assert!(!discards.written(14, 16));
    assert_eq!(runs(&discards), [(10, 14), (16, 20), (30, 40), (50, 60)]);
    assert!(!discards.written(18, 55));
    assert_eq!(runs(&discards), [(10, 14), (16, 18), (55, 60)]);
    assert!(!discards.written(0, 100));
    assert!(runs(&discards).is_empty());
}

#[test]
fn batches_take_the_first_runs_a_batch_at_a_time() {
    let mut discards = Discards::default();
    for start in [0, 10, 20] {
        discards.add(start, start + 2);
    }
    let lbas = |extents: Vec<Extent>| extents.iter().map(|e| e.lba).collect::<Vec<_>>();
    assert_eq!(lbas(discards.batch(2)), [0, 10]);
    assert_eq!(lbas(discards.batch(2)), [20]);
    assert!(discards.batch(2).is_empty());
}

/// What the test sees of a queued disk: the commands started, whose tags
/// are their indices, and the tags it has completed.
#[derive(Clone, Default)]
struct Log(Rc<RefCell<(Vec<Command>, Vec<u32>)>>);

impl Log {
    fn started(&self) -> Vec<Command> {
        self.0.borrow().0.clone()
    }
}

struct Queued {
    disk: RamDisk,
    log: Log,
}

impl BlockDevice for Queued {
    fn identity(&self) -> &Identity {
        self.disk.identity()
    }

    fn read(&mut self, lba: u64, buffer: &mut [u8]) -> Result<(), BlockError> {
        self.disk.read(lba, buffer)
    }

    fn write(&mut self, lba: u64, data: &[u8]) -> Result<(), BlockError> {
        self.disk.write(lba, data)
    }

    fn depth(&self) -> usize {
        8
    }

    fn room(&self) -> bool {
        true
    }

    fn start(&mut self, command: Command) -> Result<u32, BlockError> {
        let mut log = self.log.0.borrow_mut();
        log.0.push(command);
        Ok(log.0.len() as u32 - 1)
    }

    fn finish(&mut self, tag: u32) -> Option<Finished> {
        self.log.0.borrow().1.contains(&tag).then_some(Ok(None))
    }
}

fn queued(ranges: usize, unmap: bool) -> (HostDisk, Log) {
    let mut disk = RamDisk::new("", 64, 4096);
    disk.identity.features.discard = Some(DiscardLimits { blocks: 0, ranges });
    disk.identity.features.write_zeroes = Some(WriteZeroes { blocks: 64, unmap });
    disk.identity.features.write_cache = true;
    let log = Log::default();
    let device = Queued {
        disk,
        log: log.clone(),
    };
    (HostDisk::new(Box::new(device)), log)
}

fn write(lba: u64) -> Request {
    Request::Write {
        lba,
        data: vec![7; 4096],
        durable: false,
    }
}

fn release(log: &Log, tag: u32) {
    log.0.borrow_mut().1.push(tag);
}

#[test]
fn deallocated_blocks_reach_the_disk_in_one_command_in_the_background() {
    let (mut disk, log) = queued(256, false);
    disk.discard(0, 4);
    disk.discard(8, 4);
    assert!(disk.discarding());
    assert!(disk.poll(0).is_empty());
    assert_eq!(
        log.started(),
        [Command::Discard {
            extents: vec![Extent { lba: 0, blocks: 4 }, Extent { lba: 8, blocks: 4 }],
        }]
    );
    release(&log, 0);
    assert!(disk.poll(0).is_empty());
    assert!(!disk.discarding());
}

#[test]
fn a_write_keeps_its_blocks_from_a_later_batch() {
    let (mut disk, log) = queued(256, false);
    disk.discard(0, 8);
    assert!(disk.submit(1, write(2), 0).is_none());
    release(&log, 0);
    assert_eq!(disk.poll(0).len(), 1);
    assert_eq!(log.started().len(), 1, "no batch while transfers run");
    assert!(disk.poll(DISCARD_QUIET).is_empty());
    assert_eq!(
        log.started().last().unwrap(),
        &Command::Discard {
            extents: vec![Extent { lba: 0, blocks: 2 }, Extent { lba: 3, blocks: 5 }],
        }
    );
}

#[test]
fn a_write_waits_out_the_batch_it_overlaps() {
    let (mut disk, log) = queued(256, false);
    disk.discard(0, 8);
    disk.poll(0);
    // The batch runs: a write beside it goes, one inside it waits.
    assert!(disk.submit(1, write(9), 0).is_none());
    assert!(disk.submit(2, write(2), 0).is_none());
    assert_eq!(log.started().len(), 2);
    assert_eq!(disk.discards.held.len(), 1);
    release(&log, 0);
    assert!(disk.poll(0).is_empty());
    assert_eq!(log.started().len(), 3, "the held write started");
    release(&log, 1);
    release(&log, 2);
    let mut keys: Vec<u32> = disk.poll(0).iter().map(|(key, _)| *key).collect();
    keys.sort_unstable();
    assert_eq!(keys, [1, 2]);
}

#[test]
fn a_flush_waits_for_discards_only_where_deallocated_blocks_stay_zeros() {
    for zeros in [true, false] {
        let (mut disk, log) = queued(256, zeros);
        disk.discard(0, 8);
        assert!(disk.submit(1, Request::Flush, 0).is_none());
        assert_eq!(disk.discards.held.len(), usize::from(zeros));
        if zeros {
            disk.poll(0);
            release(&log, 0);
            assert!(disk.poll(0).is_empty());
            assert_eq!(log.started().last(), Some(&Command::Flush));
        }
    }
}

#[test]
fn a_disk_that_refuses_a_discard_hears_no_more() {
    let mut disk = RamDisk::new("", 4, 4096);
    disk.identity.features.discard = Some(DiscardLimits::single(0));
    disk.identity.read_only = true;
    let mut disk = HostDisk::new(Box::new(disk));
    disk.discard(0, 2);
    assert!(disk.poll(0).is_empty());
    assert!(!disk.discarding());
    disk.discard(0, 2);
    assert!(!disk.discarding());
    assert!(disk.diagnostic(0).contains("refused=Some"));
}

#[test]
fn discards_wait_for_a_pause_in_guest_transfers() {
    let (mut disk, log) = queued(256, false);
    assert!(disk.submit(1, write(40), 100).is_none());
    disk.discard(0, 8);
    release(&log, 0);
    assert_eq!(disk.poll(200).len(), 1);
    // The transfer completed at 200: the batch waits for the pause, and the
    // model looks again when it ends.
    assert!(disk.poll(200 + DISCARD_QUIET - 1).is_empty());
    assert_eq!(log.started().len(), 1);
    assert_eq!(disk.next_look(300), 200 + DISCARD_QUIET);
    // A read meanwhile restarts the pause.
    let read = Request::Read {
        lba: 50,
        bytes: 4096,
    };
    assert!(disk.submit(2, read, 1_000).is_none());
    release(&log, 1);
    assert_eq!(disk.poll(1_000).len(), 1);
    assert!(disk.poll(200 + DISCARD_QUIET).is_empty());
    assert_eq!(log.started().len(), 2);
    assert!(disk.poll(1_000 + DISCARD_QUIET).is_empty());
    assert!(matches!(
        log.started().last(),
        Some(Command::Discard { .. })
    ));
}

#[test]
fn a_flush_that_waits_for_discards_sends_them_without_a_pause() {
    let (mut disk, log) = queued(256, true);
    assert!(disk.submit(1, write(40), 0).is_none());
    disk.discard(0, 8);
    release(&log, 0);
    assert_eq!(disk.poll(10).len(), 1);
    assert!(disk.submit(2, Request::Flush, 20).is_none());
    assert!(disk.poll(20).is_empty());
    assert!(matches!(
        log.started().last(),
        Some(Command::Discard { .. })
    ));
}

/// In the background a batch is one command, so a read that comes while it
/// runs waits for that one only, and the next waits for the next pause.
#[test]
fn a_background_batch_is_one_command_a_read_waits_behind() {
    let (mut disk, log) = queued(2, false);
    for start in [0, 4, 8, 12, 16] {
        disk.discard(start, 2);
    }
    assert!(disk.poll(0).is_empty());
    assert_eq!(
        log.started(),
        [Command::Discard {
            extents: vec![Extent { lba: 0, blocks: 2 }, Extent { lba: 4, blocks: 2 }],
        }]
    );
    // A read meanwhile goes beside it; no more discards start.
    let read = Request::Read {
        lba: 50,
        bytes: 4096,
    };
    assert!(disk.submit(1, read, 10).is_none());
    release(&log, 0);
    assert!(disk.poll(10).is_empty());
    assert_eq!(log.started().len(), 2);
    release(&log, 1);
    assert_eq!(disk.poll(20).len(), 1);
    assert!(disk.poll(20 + DISCARD_QUIET - 1).is_empty());
    assert_eq!(log.started().len(), 2);
    // After the pause, one command again.
    assert!(disk.poll(20 + DISCARD_QUIET).is_empty());
    assert_eq!(
        log.started().last(),
        Some(&Command::Discard {
            extents: vec![Extent { lba: 8, blocks: 2 }, Extent { lba: 12, blocks: 2 }],
        })
    );
    assert_eq!(log.started().len(), 3);
}

/// A flush that waits for discards sends several commands together.
#[test]
fn a_waiting_flush_sends_discards_several_commands_at_once() {
    let (mut disk, log) = queued(1, true);
    for start in [0, 4, 8] {
        disk.discard(start, 2);
    }
    assert!(disk.submit(1, Request::Flush, 0).is_none());
    assert!(disk.poll(0).is_empty());
    let discards = log
        .started()
        .iter()
        .filter(|command| matches!(command, Command::Discard { .. }))
        .count();
    assert_eq!(discards, 3);
}

/// A shutdown's discards go several commands together with no pause in
/// transfers, where the background sends one command after each pause.
#[test]
fn hurried_discards_go_at_once_several_commands_together() {
    let (mut disk, log) = queued(1, false);
    assert!(disk.submit(1, write(40), 100).is_none());
    for start in [0, 4, 8] {
        disk.discard(start, 2);
    }
    release(&log, 0);
    assert_eq!(disk.poll(200).len(), 1);
    assert_eq!(log.started().len(), 1);
    disk.hurry_discards(true);
    assert!(disk.poll(201).is_empty());
    let discards = log
        .started()
        .iter()
        .filter(|command| matches!(command, Command::Discard { .. }))
        .count();
    assert_eq!(discards, 3);
}

/// Giving the waiting discards up leaves the batch on the disk to complete
/// and nothing after it.
#[test]
fn abandoned_discards_never_reach_the_disk() {
    let (mut disk, log) = queued(1, false);
    for start in [0, 4, 8] {
        disk.discard(start, 2);
    }
    assert!(disk.poll(0).is_empty());
    assert_eq!(log.started().len(), 1);
    assert_eq!(disk.abandon_discards(), 2);
    assert!(disk.discarding());
    release(&log, 0);
    assert!(disk.poll(DISCARD_QUIET).is_empty());
    assert!(!disk.discarding());
    assert_eq!(log.started().len(), 1);
    assert_eq!(disk.abandon_discards(), 0);
}

fn discard_tags(log: &Log) -> Vec<u32> {
    log.started()
        .iter()
        .enumerate()
        .filter(|(_, command)| matches!(command, Command::Discard { .. }))
        .map(|(tag, _)| tag as u32)
        .collect()
}

/// A waiting flush keeps [`DISCARD_COMMANDS`] on the disk: each one that
/// completes makes room for the next at once, whatever the others do, and
/// the flush goes only once none remains.
#[test]
fn a_waiting_flush_refills_each_discard_slot_as_it_completes() {
    let (mut disk, log) = queued(1, true);
    let runs = DISCARD_COMMANDS + 4;
    for run in 0..runs as u64 {
        disk.discard(run * 2, 1);
    }
    assert!(disk.submit(1, Request::Flush, 0).is_none());
    assert!(disk.poll(0).is_empty());
    assert_eq!(discard_tags(&log).len(), DISCARD_COMMANDS);
    // The last started completes first; the first stays on the disk.
    release(&log, DISCARD_COMMANDS as u32 - 1);
    assert!(disk.poll(1).is_empty());
    assert_eq!(discard_tags(&log).len(), DISCARD_COMMANDS + 1);
    assert!(!log.started().contains(&Command::Flush));
    for tag in discard_tags(&log) {
        release(&log, tag);
    }
    assert!(disk.poll(2).is_empty());
    for tag in discard_tags(&log) {
        release(&log, tag);
    }
    assert!(disk.poll(3).is_empty());
    assert_eq!(discard_tags(&log).len(), runs);
    assert_eq!(log.started().last(), Some(&Command::Flush));
    assert!(!disk.discarding());
}

/// Urgent batches go beside guest transfers, and a write waits out
/// whichever running batch it overlaps, not the others.
#[test]
fn a_write_waits_only_for_the_urgent_batch_it_overlaps() {
    let (mut disk, log) = queued(1, true);
    assert!(disk.submit(1, write(40), 0).is_none());
    for start in [0, 4, 8] {
        disk.discard(start, 2);
    }
    assert!(disk.submit(2, Request::Flush, 0).is_none());
    assert!(disk.poll(0).is_empty());
    assert_eq!(discard_tags(&log), [1, 2, 3], "beside the running write");
    // A write into the batch at 4 waits; one beside every batch goes.
    assert!(disk.submit(3, write(5), 1).is_none());
    assert!(disk.submit(4, write(20), 1).is_none());
    assert_eq!(disk.discards.held.len(), 2, "the flush and the write at 5");
    assert_eq!(log.started().len(), 5);
    release(&log, 1);
    release(&log, 3);
    assert!(disk.poll(2).is_empty());
    assert_eq!(log.started().len(), 5, "the batch at 4 still runs");
    release(&log, 2);
    disk.poll(3);
    // Both held transfers go once the last batch completed, in the order
    // they came: the flush, then the write.
    let after: Vec<Command> = log.started()[5..].to_vec();
    assert!(matches!(
        after.as_slice(),
        [Command::Flush, Command::Write { lba: 5, .. }]
    ));
}
