//! Stack geometry and the per-executor partition of the code reservation
//! and shadow pool. Each executor owns its pool share and a small private
//! code ring; the rest of the code reservation is one window every PE's
//! engine shares.
use crate::Result;
use crate::topology::MAX_CORES;
use alloc::vec::Vec;
use core::ops::Range;
use stela_contracts::refuse;

use stela_contracts::bytes::HOST_PAGE_BYTES as PAGE;
const MIB: u64 = 1024 * 1024;

pub const STACK_BYTES: u64 = 2 * 1024 * 1024;
pub const STACK_STRIDE: u64 = STACK_BYTES + PAGE;
pub const INVOCATION_STACK_BYTES: u64 = 128 * 1024;
pub const CODE_INSTALL_ALIGNMENT: u64 = 16;

/// Initialized code extents of one private arena part, stored by exclusive
/// end. Installs start at the aligned end of the preceding one, so retaining
/// that start as well would duplicate information.
#[derive(Default)]
pub struct CodeExtents(Vec<u32>);

impl CodeExtents {
    /// Reserve one ledger entry, initialize its code, then record the extent.
    /// A refusal leaves both the ledger and its code unchanged.
    pub fn append(
        &mut self,
        part_bytes: u64,
        code_bytes: usize,
        initialize: impl FnOnce(u64),
    ) -> Option<Range<u64>> {
        let previous = u64::from(self.0.last().copied().unwrap_or(0));
        let start = previous.next_multiple_of(CODE_INSTALL_ALIGNMENT);
        let end = start.checked_add(code_bytes as u64)?;
        if code_bytes == 0 || end > part_bytes {
            return None;
        }
        let finish = u32::try_from(end).ok()?;
        self.0.try_reserve(1).ok()?;
        initialize(start);
        self.0.push(finish);
        Some(start..end)
    }

    /// Whether one install owns the whole part-relative interval. Alignment
    /// gaps and spans across separately initialized installs are refused.
    pub fn contains(&self, range: Range<u64>) -> bool {
        if range.is_empty() {
            return false;
        }
        let index = self.0.partition_point(|&end| u64::from(end) <= range.start);
        self.0.get(index).is_some_and(|&end| {
            let previous = if index == 0 { 0 } else { self.0[index - 1] };
            let start = u64::from(previous).next_multiple_of(CODE_INSTALL_ALIGNMENT);
            start <= range.start && range.end <= u64::from(end)
        })
    }

    pub fn is_empty(&self) -> bool {
        self.0.is_empty()
    }

    /// Forget a part's code when the ring reuses it, keeping the ledger's
    /// capacity ready for new code.
    pub fn clear(&mut self) {
        self.0.clear();
    }
}

pub fn invocation_stacks(
    cores: usize,
) -> Result<crate::guarded_stacks::Layout, crate::guarded_stacks::Error> {
    crate::guarded_stacks::Layout::new(INVOCATION_STACK_BYTES, cores)
}

/// The BSP's work pool holds as many stacks as its executor can.
pub const TASK_STACKS: usize = stela_driver_base::platform::work::MAX_CONTINUATIONS;
/// General workers on ordinary stacks, for native driver probes.
pub const ORDINARY_GENERAL_TASKS: usize = 4;
/// General workers on Linux task stacks: every General stack but the ordinary
/// ones. Each Linux kthread, ioctl worker and driver thread holds one.
pub const KERNEL_GENERAL_TASKS: usize =
    TASK_STACKS - stela_driver_base::platform::work::ROLE_STACKS - ORDINARY_GENERAL_TASKS;
/// An ordinary task's Rust stack. Native driver probes run on one, through
/// controller attach paths whose frames alone pass 128 KiB.
pub const ORDINARY_TASK_STACK_BYTES: u64 = 1024 * 1024;
pub const TASK_WINDOW_BYTES: u64 = 32 * 1024 * 1024;
/// A Linux task's stack and its alignment: Linux's THREAD_SIZE
/// (`stela-linux`'s `compat/asm/thread_info.h`), 64 KiB, which amdgpu's
/// probe on a Navi 48 needs past 16 KiB with the host's paths a printk
/// reaches.
pub const LINUX_TASK_STACK_BYTES: u64 = 64 * 1024;
/// The BSP's work pool. A Linux task runs only on a stack of exactly its
/// configured size and alignment, so the service and the kernel General
/// continuations have that shape; the other roles and the last
/// [`ORDINARY_GENERAL_TASKS`] keep generic stacks.
pub fn task_stacks(
    base: u64,
) -> Result<crate::guarded_stacks::MixedLayout, crate::guarded_stacks::Error> {
    use crate::guarded_stacks::{Error, Geometry, MixedLayout};
    let kernel = Geometry::new(LINUX_TASK_STACK_BYTES, LINUX_TASK_STACK_BYTES)?;
    let ordinary = Geometry::new(ORDINARY_TASK_STACK_BYTES, PAGE)?;
    let geometries = core::array::from_fn::<_, TASK_STACKS, _>(|index| {
        let roles = stela_driver_base::platform::work::ROLE_STACKS;
        if index == 2 || (roles..roles + KERNEL_GENERAL_TASKS).contains(&index) {
            kernel
        } else {
            ordinary
        }
    });
    let layout = MixedLayout::new(base, &geometries)?;
    if layout.virtual_bytes() > TASK_WINDOW_BYTES {
        return Err(Error::Overflow);
    }
    Ok(layout)
}

#[derive(Debug, PartialEq, Eq)]
pub struct Partition {
    /// The executor's own part of the code reservation, for what its host
    /// installs itself (context leaves, scalable leaves, shared-memory
    /// native runs); a ring of [`PRIVATE_PARTS`].
    pub code_offset: u64,
    pub code_bytes: u64,
    /// The board's code window, the same for every executor: compiled
    /// blocks, which every PE runs (`Host::code_window`).
    pub window_offset: u64,
    pub window_bytes: u64,
    pub pool_offset: u64,
    pub pool_bytes: u64,
}

/// What an executor that runs no PE keeps of the shadow pool.
const PARKED_POOL: u64 = 4 * 1024 * 1024;

/// The frames a shadow pool keeps for its roots: the host root and every
/// root set's views (`stela_compiler::x86_64::table_switch`).
pub const SHADOW_PERMANENT_FRAMES: u64 = (1 + stela_compiler::x86_64::table_switch::SETS
    * stela_compiler::x86_64::table_switch::SET_ROOTS)
    as u64;

/// Each executor's own code: a few hundred leaves and native-run
/// artifacts of at most a block each, in four parts of 512 KiB. Blocks go
/// to the board's window.
pub const PRIVATE_CODE: u64 = 2 * MIB;
pub const PRIVATE_PARTS: u32 = 4;
/// The board's code window holds the guest's working set of code once for
/// every PE: free RAM / 16, 64 MiB to 2 GiB, in 16 MiB steps. The block
/// table grows with it (one entry per 256 bytes, 128 MiB at 1 GiB and
/// above), and each PE's ITLB to 16,384 entries from 64 MiB.
///
/// The set does not shrink with the PEs: fewer PEs run the same macOS. A
/// 16 GiB PC's desktop session on 8 PEs filled a 768 MiB window (1.6 million
/// blocks) 14 times in 10 minutes: its ring took 6 to 10 of 32 parts every
/// 10 s, and half of its installs were of blocks it had held before, 86% of
/// them back within one turn of the ring. A window twice the size keeps a
/// block for two of those turns, so most of them never leave.
const WINDOW_SHARE: u64 = 16;
/// The window per runner where the guest would not keep its room beside the
/// host's whole share ([`crate::host_memory::sizes`]). A sign-in on 8
/// PEs with 512 MiB (64 MiB per runner) compiled 45% of its installs again
/// within one turn of the ring, the code lock waited 3.5 s per 10 s, and
/// each part the ring took cost about 0.75 s of PE time. With 704 MiB (88
/// MiB per runner) the ring evicted 31-51 thousand blocks a heartbeat and
/// the lock waited 0.6-0.9 s; with 976 MiB, 22 thousand and 0.5 s. Each
/// window byte also takes a heap byte for its block records, so 96 MiB per
/// runner, between the two, gives the guest twice the 32 MiB it takes off
/// each runner's window.
pub const WINDOW_PER_RUNNER: u64 = 96 * MIB;
/// The window per runner when [`WINDOW_PER_RUNNER`] would leave the guest
/// under its floor ([`crate::host_memory::sizes`] falls back to it).
pub const SMALL_WINDOW_PER_RUNNER: u64 = 64 * MIB;
const MIN_WINDOW: u64 = 64 * MIB;
pub const MAX_WINDOW: u64 = 2048 * MIB;
pub const CODE_ALIGNMENT: u64 = 16 * MIB;
pub const MAX_CODE_PARTS: u32 = 32;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CodeReservation {
    /// Every executor's [`PRIVATE_CODE`], then the board's window.
    pub bytes: u64,
    /// The parts of each executor's private ring.
    pub parts: u32,
    pub window: u64,
}

/// The aligned bytes in an install/eviction part. Reduced rings cover
/// their whole private arena; a legacy ring can leave a rounding tail.
pub fn code_part_bytes(bytes: u64, parts: u32) -> Result<u64> {
    if parts == MAX_CODE_PARTS {
        // Legacy partition callers can leave an unused sub-alignment tail.
        // Retain their established aligned unit calculation.
        return Ok(bytes / u64::from(parts) / CODE_INSTALL_ALIGNMENT * CODE_INSTALL_ALIGNMENT);
    }
    if !(2..MAX_CODE_PARTS).contains(&parts)
        || bytes == 0
        || !bytes.is_multiple_of(u64::from(parts) * CODE_INSTALL_ALIGNMENT)
    {
        return Err(refuse!("invalid native code ring geometry"));
    }
    Ok(bytes / u64::from(parts))
}

/// The executable window holds the largest reservation the loader can make.
pub const CODE_WINDOW_BYTES: u64 = MAX_CORES as u64 * 512 * MIB;
const _: () = assert!(MAX_CORES as u64 * PRIVATE_CODE + MAX_WINDOW <= CODE_WINDOW_BYTES);

/// The code and pool bytes the executors past the `runners` of `cores` keep.
/// The loader adds them to the runners' own shares.
pub const fn parked_reserve(cores: usize, runners: usize) -> (u64, u64) {
    let parked = cores.saturating_sub(runners) as u64;
    (parked * PRIVATE_CODE, parked * PARKED_POOL)
}

/// The firmware code allocation: each of `cores` executors' private part
/// and the board's window, sized from `free` host bytes and the `runners`
/// that run PEs at [`WINDOW_PER_RUNNER`] each, checked before any firmware
/// pages are reserved: what a host whose guest lacks its room for the
/// host's whole share gets ([`crate::host_memory::sizes`]).
pub fn code_reservation(free: u64, cores: usize, runners: usize) -> Result<CodeReservation> {
    code_reservation_within(free, cores, runners, runners as u64 * WINDOW_PER_RUNNER)
}

/// The window the host's share of `free` bytes gives, whatever its runners:
/// the most [`code_reservation_within`] gives.
pub fn window_share(free: u64) -> u64 {
    (free / WINDOW_SHARE).clamp(MIN_WINDOW, MAX_WINDOW) / CODE_ALIGNMENT * CODE_ALIGNMENT
}

/// [`code_reservation`] with a window of the host's share of `free` bytes
/// and at most `most` bytes, within the window's bounds.
pub fn code_reservation_within(
    free: u64,
    cores: usize,
    runners: usize,
    most: u64,
) -> Result<CodeReservation> {
    if cores == 0 || cores > MAX_CORES || runners == 0 || runners > cores {
        return Err(refuse!("invalid native executor count"));
    }
    let window = (free / WINDOW_SHARE)
        .min(most)
        .clamp(MIN_WINDOW, MAX_WINDOW)
        / CODE_ALIGNMENT
        * CODE_ALIGNMENT;
    Ok(CodeReservation {
        bytes: cores as u64 * PRIVATE_CODE + window,
        parts: PRIVATE_PARTS,
        window,
    })
}

/// Divide only whole frames; unused tail bytes remain owned by the
/// allocation and cannot be lent to a second executor. Every executor
/// keeps [`PRIVATE_CODE`] of `code`, in slot order, and the rest is the
/// board's window. The first `runners` executors run PEs and share alike
/// all of `pool` but the others' [`PARKED_POOL`].
pub fn partition(
    cores: usize,
    runners: usize,
    slot: usize,
    code: u64,
    pool: u64,
) -> Result<Partition> {
    if cores == 0 || cores > MAX_CORES || slot >= cores || runners == 0 || runners > cores {
        return Err(refuse!("invalid native executor slot"));
    }
    let private = cores as u64 * PRIVATE_CODE;
    let Some(window_bytes) = code.checked_sub(private) else {
        return Err(refuse!("native executor resources too small"));
    };
    let parked = (cores - runners) as u64;
    // Each shadow pool needs its permanent root frames and at least one
    // frame in each half.
    let pool_share = pool.saturating_sub(parked * PARKED_POOL) / runners as u64 / PAGE * PAGE;
    let (pool_bytes, pool_offset) = match slot.checked_sub(runners) {
        None => (pool_share, slot as u64 * pool_share),
        Some(behind) => (
            PARKED_POOL,
            runners as u64 * pool_share + behind as u64 * PARKED_POOL,
        ),
    };
    if pool_bytes < (SHADOW_PERMANENT_FRAMES + 2) * PAGE {
        return Err(refuse!("native executor resources too small"));
    }
    Ok(Partition {
        code_offset: slot as u64 * PRIVATE_CODE,
        code_bytes: PRIVATE_CODE,
        window_offset: private,
        window_bytes,
        pool_offset,
        pool_bytes,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_window_grows_with_the_host_and_its_runners_between_bounds() {
        for (free, cores, runners, window) in [
            (0, 1, 1, 64),
            (6 * 1024, 2, 2, 192),
            (7936, 8, 8, 496),
            (12 * 1024, 8, 8, 768),
            (16 * 1024, 8, 8, 768),
            (32 * 1024, 8, 8, 768),
            (32 * 1024, 32, 32, 2048),
            (128 * 1024, 64, 64, 2048),
            (128 * 1024, 64, 8, 768),
        ] {
            let code = code_reservation(free * MIB, cores, runners).unwrap();
            assert_eq!(
                code,
                CodeReservation {
                    bytes: cores as u64 * PRIVATE_CODE + window * MIB,
                    parts: PRIVATE_PARTS,
                    window: window * MIB,
                },
                "{free} {cores} {runners}"
            );
        }
        for (cores, runners) in [(0, 0), (1, 0), (2, 3), (MAX_CORES + 1, 1)] {
            assert!(code_reservation(0, cores, runners).is_err());
        }
    }

    #[test]
    fn the_hosts_share_of_window_does_not_depend_on_its_runners() {
        for (free, window) in [
            (0, 64),
            (6 * 1024, 384),
            (7936, 496),
            (16 * 1024, 1024),
            (32 * 1024, 2048),
            (128 * 1024, 2048),
        ] {
            assert_eq!(window_share(free * MIB), window * MIB, "{free}");
            for (cores, runners) in [(1, 1), (2, 2), (8, 8), (64, 8)] {
                let code = code_reservation_within(free * MIB, cores, runners, u64::MAX).unwrap();
                assert_eq!(
                    code,
                    CodeReservation {
                        bytes: cores as u64 * PRIVATE_CODE + window * MIB,
                        parts: PRIVATE_PARTS,
                        window: window * MIB,
                    },
                    "{free} {cores} {runners}"
                );
            }
        }
        // A bound under the share holds, within the window's own bounds.
        let within = |most: u64| {
            code_reservation_within(32 * 1024 * MIB, 8, 8, most)
                .unwrap()
                .window
        };
        assert_eq!(within(800 * MIB), 800 * MIB);
        assert_eq!(within(0), MIN_WINDOW);
        assert!(code_reservation_within(0, 2, 3, u64::MAX).is_err());
    }

    #[test]
    fn every_executor_keeps_a_private_ring_beside_one_shared_window() {
        for cores in 1..=MAX_CORES {
            for runners in 1..=cores {
                for free in [0, 8 * 1024 * MIB, 64 * 1024 * MIB, u64::MAX] {
                    let code = code_reservation(free, cores, runners).unwrap();
                    assert!(code.bytes <= CODE_WINDOW_BYTES);
                    assert!(code.bytes.is_multiple_of(2 * MIB));
                    // Room for every runner's permanent roots at the most cores.
                    let pool = MAX_CORES as u64 * 2 * MIB + parked_reserve(cores, runners).1;
                    let mut end = 0;
                    for slot in 0..cores {
                        let part = partition(cores, runners, slot, code.bytes, pool).unwrap();
                        assert_eq!(part.code_offset, end);
                        assert_eq!(part.code_bytes, PRIVATE_CODE);
                        end += part.code_bytes;
                        assert_eq!(part.window_offset, cores as u64 * PRIVATE_CODE);
                        assert_eq!(part.window_bytes, code.window);
                        assert_eq!(code_part_bytes(part.code_bytes, code.parts), Ok(512 * 1024));
                    }
                    assert_eq!(end + code.window, code.bytes);
                }
            }
        }
        assert!(partition(4, 4, 0, 4 * PRIVATE_CODE - 1, 64 * MIB).is_err());
        assert!(partition(4, 4, 4, 64 * MIB, 64 * MIB).is_err());
    }

    #[test]
    fn installed_code_extents_cover_only_individual_initialized_ranges() {
        let mut ledger = CodeExtents::default();
        let mut exact = Vec::new();
        for bytes in [13, 16, 2, 9, 25, 1] {
            let start = exact.last().map_or(0, |range: &Range<u64>| {
                range.end.next_multiple_of(CODE_INSTALL_ALIGNMENT)
            });
            let range = ledger
                .append(128, bytes, |at| assert_eq!(at, start))
                .unwrap();
            assert_eq!(range, start..start + bytes as u64);
            exact.push(range);
        }
        for start in 0..=128 {
            for end in start..=129 {
                let expected = start < end
                    && exact
                        .iter()
                        .any(|extent| extent.start <= start && end <= extent.end);
                assert_eq!(ledger.contains(start..end), expected, "{start}..{end}");
            }
        }
        ledger.clear();
        assert!(ledger.is_empty());
        assert!(!ledger.contains(0..1));
        assert_eq!(ledger.append(64, 7, |_| {}), Some(0..7));
    }

    #[test]
    fn refused_code_installs_leave_the_end_and_backing_untouched() {
        let mut ledger = CodeExtents::default();
        assert_eq!(ledger.append(32, 13, |_| {}), Some(0..13));
        assert_eq!(ledger.append(32, 0, |_| panic!("empty code")), None);
        assert_eq!(ledger.append(32, 17, |_| panic!("over the part")), None);
        assert_eq!(
            ledger.append(u64::MAX, u32::MAX as usize, |_| panic!("over the ledger")),
            None
        );
        assert_eq!(ledger.append(32, 16, |_| {}), Some(16..32));
        assert!(!ledger.contains(13..16));
        assert!(ledger.contains(16..32));
    }

    #[test]
    fn task_plan_keeps_exact_payload_padding_guards_and_classes() {
        let layout = task_stacks(0x8_0000_0000).unwrap();
        let (kernel, ordinary) = (
            KERNEL_GENERAL_TASKS + 1,
            TASK_STACKS - KERNEL_GENERAL_TASKS - 1,
        );
        assert_eq!(layout.count(), TASK_STACKS);
        assert_eq!(
            layout.physical_bytes(),
            kernel as u64 * LINUX_TASK_STACK_BYTES + ordinary as u64 * ORDINARY_TASK_STACK_BYTES
        );
        assert_eq!(
            layout.raw_allocation_bytes(),
            layout.physical_bytes() + LINUX_TASK_STACK_BYTES - 4096
        );
        let mut counts = (0, 0);
        for index in 0..TASK_STACKS {
            let slot = layout.slot(index).unwrap();
            let bytes = slot.usable.end - slot.usable.start;
            if bytes == LINUX_TASK_STACK_BYTES {
                counts.0 += 1;
                assert_eq!(slot.usable.start % (LINUX_TASK_STACK_BYTES), 0);
            } else {
                assert_eq!(bytes, ORDINARY_TASK_STACK_BYTES);
                counts.1 += 1;
            }
            assert_eq!(slot.lower_guard.end, slot.usable.start);
            assert_eq!(slot.upper_guard.start, slot.usable.end);
        }
        assert_eq!(counts, (kernel, ordinary));
        assert!(layout.virtual_bytes() < TASK_WINDOW_BYTES);
    }
    #[test]
    fn private_pools_have_no_overlap_or_fractional_frames() {
        for (cores, runners) in
            (1..=MAX_CORES).flat_map(|cores| [(cores, cores), (cores, 1), (cores, cores.min(8))])
        {
            let (_, pool_reserve) = parked_reserve(cores, runners);
            let code = cores as u64 * PRIVATE_CODE + 64 * MIB;
            let pool = MAX_CORES as u64 * 2 * MIB + PAGE - 1 + pool_reserve;
            let mut previous_pool = 0;
            for slot in 0..cores {
                let part = partition(cores, runners, slot, code, pool).unwrap();
                assert_eq!(part.pool_offset, previous_pool);
                assert_eq!(part.pool_bytes % PAGE, 0);
                previous_pool += part.pool_bytes;
            }
            assert!(previous_pool <= pool);
        }
    }

    /// Serialized, the BSP runs every PE and takes all of the pool but the
    /// workers' reserves; every executor keeps the same private code.
    #[test]
    fn a_serialized_bsp_takes_all_but_the_workers_reserves() {
        let (code, pool) = (512 * MIB, 1024 * MIB);
        let bsp = partition(8, 1, 0, code, pool).unwrap();
        assert_eq!(bsp.code_bytes, PRIVATE_CODE);
        assert_eq!(bsp.window_bytes, code - 8 * PRIVATE_CODE);
        assert_eq!(bsp.pool_bytes, pool - 7 * PARKED_POOL);
        let last = partition(8, 1, 7, code, pool).unwrap();
        assert_eq!(last.code_offset + last.code_bytes, last.window_offset);
        assert_eq!(last.pool_offset + last.pool_bytes, pool);
    }

    /// A host with more cores than the guest has PEs parks the rest, which
    /// take no share of the pool.
    #[test]
    fn cores_past_the_guests_pes_keep_only_a_reserve() {
        let (code, pool) = (64 * PRIVATE_CODE + 256 * MIB, 56 * PARKED_POOL + (8 << 28));
        let share = 1 << 28;
        let first = partition(64, 8, 7, code, pool).unwrap();
        assert_eq!(first.pool_bytes, share);
        let parked = partition(64, 8, 8, code, pool).unwrap();
        assert_eq!(
            (parked.code_bytes, parked.pool_bytes),
            (PRIVATE_CODE, PARKED_POOL)
        );
        assert_eq!(parked.code_offset, 8 * PRIVATE_CODE);
        assert_eq!(parked.pool_offset, 8 * share);
        assert_eq!(parked.window_bytes, 256 * MIB);
    }

    #[test]
    fn unavailable_slots_and_insufficient_private_roots_refuse() {
        let code = 2 * PRIVATE_CODE;
        for (cores, runners, slot) in [
            (0, 1, 0),
            (MAX_CORES + 1, 1, 0),
            (2, 2, 2),
            (2, 0, 0),
            (2, 3, 0),
        ] {
            assert!(partition(cores, runners, slot, code, 1024 * PAGE).is_err());
        }
        assert!(partition(2, 2, 0, code - 1, 1024 * PAGE).is_err());
        assert!(partition(2, 2, 0, code, 2 * (SHADOW_PERMANENT_FRAMES + 2) * PAGE - 1).is_err());
        assert!(partition(2, 2, 0, code, 2 * (SHADOW_PERMANENT_FRAMES + 2) * PAGE).is_ok());
        assert!(partition(2, 1, 0, code, 2 * (SHADOW_PERMANENT_FRAMES + 2) * PAGE).is_err());
    }
}
