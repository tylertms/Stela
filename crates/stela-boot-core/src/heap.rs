//! Boot heap: two-level segregated fit (TLSF). Every block starts with a
//! boundary tag naming its size and the block before it, so deallocation
//! coalesces both neighbours without a walk, and free blocks are kept in bins
//! by size, 16 per power of two, whose bitmaps find the smallest fitting one
//! in constant time.
//!
//! The system engine allocates and frees small blocks constantly. Blocks of up
//! to 4 KiB with at most 16-byte alignment are therefore rounded up to a
//! power-of-two size class and, once freed, kept on that class's LIFO list
//! for the next allocation of the class instead of returning to the bins.
//! When the bins cannot serve a request, the class lists return to them, so
//! blocks of one size never starve a request of another.
//!
//! Every PE on a core of its own allocates at once, so each core the engine
//! runs on keeps a small cache of free blocks per class ([`Heap::tag_cpu`]),
//! which it takes from and gives back to without the lock, moving them to
//! and from the class lists [`BATCH`] at a time under it. The cache and the
//! counts only that core writes lie in cache lines of their own ([`Core`]):
//! counts packed beside other cores' would move their line between the
//! cores on every allocation and free.
//!
//! Small blocks that live for different lengths of time leave holes, and a
//! long run can hold a fifth of the heap free with no extent of a few MiB, so
//! the engine keeps the heap's end out of the main bins ([`Heap::reserve_tail`]):
//! a large request the main bins cannot serve takes it. Linux keeps
//! `ZONE_MOVABLE` and CMA apart from unmovable allocations for the same reason.
use core::{
    alloc::{GlobalAlloc, Layout},
    cell::UnsafeCell,
    marker::PhantomData,
    ops::Range,
    ptr,
    sync::atomic::{AtomicBool, AtomicIsize, AtomicU64, AtomicUsize, Ordering},
};
// Explicit, so the NVIDIA LUT fixture can compile this file by path.
#[path = "heap/failure.rs"]
mod failure;
#[path = "heap/tally.rs"]
pub mod tally;

/// Called with the carved bytes each time they first pass another
/// [`PEAK_STEP`] above [`PEAK_WATCH`], under the heap's lock: it must not
/// allocate. The boot records the allocating stack, so a transient that
/// raises the high-water mark names its code.
static PEAK_HOOK: AtomicUsize = AtomicUsize::new(0);
/// The carved bytes the peak hook next fires at.
static PEAK_NEXT: AtomicUsize = AtomicUsize::new(PEAK_WATCH);
const PEAK_WATCH: usize = 1280 << 20;
const PEAK_STEP: usize = 64 << 20;

/// Sets the hook [`PEAK_HOOK`] calls.
pub fn set_peak_hook(hook: fn(usize)) {
    PEAK_HOOK.store(hook as usize, Ordering::Release);
}

fn passed_peak(carved: usize) {
    let next = PEAK_NEXT.load(Ordering::Relaxed);
    if carved < next {
        return;
    }
    PEAK_NEXT.store(carved - carved % PEAK_STEP + PEAK_STEP, Ordering::Relaxed);
    let hook = PEAK_HOOK.load(Ordering::Acquire);
    if hook != 0 {
        // SAFETY: only `set_peak_hook` stores a nonzero value, a `fn(usize)`.
        let hook: fn(usize) = unsafe { core::mem::transmute(hook) };
        hook(carved);
    }
}

/// Called with the payload address and bytes of each free-list allocation
/// of at least [`TRACKED_BYTES`] when it is made (`true`) and when it is
/// freed (`false`), outside the heap's lock: it must neither allocate nor
/// wait. The boot keeps their allocating stacks, so each large block on the
/// heap names its code.
static LARGE_HOOK: AtomicUsize = AtomicUsize::new(0);
/// The least allocation [`LARGE_HOOK`] hears of: a few hundred live at most,
/// rare enough that the hook costs nothing measurable.
pub const TRACKED_BYTES: usize = 4 << 20;

/// Sets the hook [`LARGE_HOOK`] calls.
pub fn set_large_hook(hook: fn(usize, usize, bool)) {
    LARGE_HOOK.store(hook as usize, Ordering::Release);
}

fn tracked(pointer: *mut u8, bytes: usize, allocated: bool) {
    if bytes < TRACKED_BYTES || pointer.is_null() {
        return;
    }
    let hook = LARGE_HOOK.load(Ordering::Acquire);
    if hook != 0 {
        // SAFETY: only `set_large_hook` stores a nonzero value, a
        // `fn(usize, usize, bool)`.
        let hook: fn(usize, usize, bool) = unsafe { core::mem::transmute(hook) };
        hook(pointer as usize, bytes, allocated);
    }
}

const PAGE_BYTES: usize = 4096;
/// A block's boundary tag, at its start: the address of the block before it
/// and its own size, a multiple of [`GRANULE`] whose low bits hold [`FREE`]
/// and [`PREVIOUS_FREE`]. A payload follows it.
#[repr(C)]
struct Header {
    previous: usize,
    size: usize,
}
/// A free block's links in its bin, after its header.
#[repr(C)]
struct Links {
    next: *mut Header,
    previous: *mut Header,
}
/// Block sizes and payload addresses are multiples of this.
const GRANULE: usize = 16;
const HEADER_BYTES: usize = core::mem::size_of::<Header>();
/// The smallest block: a header and its links. A remainder smaller than
/// this stays in the allocation beside it.
const MIN_BLOCK: usize = HEADER_BYTES + core::mem::size_of::<Links>();
const FREE: usize = 1;
const PREVIOUS_FREE: usize = 2;
const FLAGS: usize = FREE | PREVIOUS_FREE;
/// The owners a live block can be counted under ([`Heap::set_owner`]):
/// owner 0 counts nothing, the rest of the heap, and the others count each
/// block they allocate until its free. A block's owner lies in the low bits
/// of its header's `previous`, which a granule-aligned address leaves
/// clear and which only the lock's holder reads, masked.
pub const OWNERS: usize = GRANULE;
const OWNER_BITS: usize = OWNERS - 1;
/// Bins per power of two.
const SUB_BITS: u32 = 4;
const SUBS: usize = 1 << SUB_BITS;
/// The smallest request the reserve serves while the class lists hold
/// blocks: what a growing table asks for, which has no smaller form.
const LARGE_BYTES: usize = 1 << 20;
/// Blocks smaller than this share the first level, a bin per granule.
const LINEAR: usize = GRANULE << SUB_BITS;
/// First levels: the linear one, then one per power of two from [`LINEAR`].
const LEVELS: usize = (usize::BITS - LINEAR.trailing_zeros() + 1) as usize;

/// The bin of a free block of `size` bytes.
fn bin(size: usize) -> (usize, usize) {
    if size < LINEAR {
        return (0, size / GRANULE);
    }
    let top = usize::BITS - 1 - size.leading_zeros();
    (
        (top + 1 - LINEAR.trailing_zeros()) as usize,
        (size >> (top - SUB_BITS)) & (SUBS - 1),
    )
}

/// `size` rounded up to the next bin boundary, so that every block of its
/// bin holds it.
fn bin_round(size: usize) -> Option<usize> {
    if size < LINEAR {
        return Some(size);
    }
    let step = (1 << (usize::BITS - 1 - size.leading_zeros() - SUB_BITS)) - 1;
    size.checked_add(step).map(|size| size & !step)
}

/// The size of `block`.
///
/// # Safety
/// `block` is a block header of the heap, read with its lock held.
unsafe fn size(block: *mut Header) -> usize {
    // SAFETY: the caller's live header.
    unsafe { (*block).size & !FLAGS }
}

/// The address of the block before `block`, its owner bits masked.
///
/// # Safety
/// `block` is a block header of the heap, read with its lock held.
unsafe fn previous(block: *mut Header) -> usize {
    // SAFETY: the caller's header; its owner's core may change the owner
    // bits meanwhile, only atomically.
    unsafe { previous_word(block).load(Ordering::Relaxed) & !OWNER_BITS }
}

/// `block`'s `previous` word, which the lock's holder and the core that
/// owns the block share: the address bits are the holder's, the owner bits
/// the owning core's, and each changes only its own, atomically.
///
/// # Safety
/// `block` is a block header of the heap.
unsafe fn previous_word<'a>(block: *mut Header) -> &'a AtomicUsize {
    // SAFETY: the header's first word, aligned for an atomic access.
    unsafe { AtomicUsize::from_ptr(&raw mut (*block).previous) }
}

/// Set the address bits of `block`'s `previous` to `address`, keeping its
/// owner bits.
///
/// # Safety
/// The lock is held and `block` is a block header of the heap.
unsafe fn set_previous(block: *mut Header, address: usize) {
    // SAFETY: the caller's header.
    let word = unsafe { previous_word(block) };
    let old = word.load(Ordering::Relaxed);
    word.fetch_xor((old ^ address) & !OWNER_BITS, Ordering::Relaxed);
}

/// The owner of the live block whose payload is `pointer`.
///
/// # Safety
/// `pointer` is a live allocation of the heap.
unsafe fn owner_of(pointer: *mut u8) -> usize {
    // SAFETY: the allocation's header precedes its payload.
    unsafe { previous_word(pointer.sub(HEADER_BYTES).cast()).load(Ordering::Relaxed) & OWNER_BITS }
}

/// Name `owner` the owner of the live block whose payload is `pointer`.
///
/// # Safety
/// `pointer` is a live allocation of the heap that the caller owns.
unsafe fn set_owner_of(pointer: *mut u8, owner: usize) {
    // SAFETY: the allocation's header precedes its payload.
    let word = unsafe { previous_word(pointer.sub(HEADER_BYTES).cast()) };
    let old = word.load(Ordering::Relaxed);
    if old & OWNER_BITS != owner {
        word.fetch_xor((old ^ owner) & OWNER_BITS, Ordering::Relaxed);
    }
}

/// A free `block`'s links.
///
/// # Safety
/// `block` is a block of at least [`MIN_BLOCK`] bytes.
unsafe fn links(block: *mut Header) -> *mut Links {
    // SAFETY: the links lie inside the caller's block.
    unsafe { block.cast::<u8>().add(HEADER_BYTES).cast() }
}

/// The free blocks of the bin whose first block is `head`.
///
/// # Safety
/// The lock is held while the iterator is used.
unsafe fn bin_blocks(head: *mut Header) -> impl Iterator<Item = *mut Header> {
    core::iter::successors((!head.is_null()).then_some(head), |&block| {
        // SAFETY: a binned free block, read with the lock held.
        let next = unsafe { (*links(block)).next };
        (!next.is_null()).then_some(next)
    })
}
/// A freed size-class block, linked through its first word.
struct Small {
    next: *mut Small,
}
const SMALLEST: usize = 16;
const CLASSES: usize = 9;
const SMALL_ALIGNMENT: usize = 16;
/// Cores with a cache of their own; later ones take the lock every time.
const CPUS: usize = 64;
/// Free blocks a core keeps per class, and moves to or from the class list
/// at once.
const CACHED: usize = 64;
const BATCH: usize = 32;
/// Where a core's GS base keeps what names its cache: `TAG` and the cache's
/// index, which [`Heap::tag_cpu`] writes.
pub const CORE_RECORD: usize = 8;
const TAG: u32 = 0x5a7e_0000;

/// One core's free blocks by class.
struct Cache {
    blocks: [[*mut Small; CACHED]; CLASSES],
    counts: [usize; CLASSES],
}

impl Cache {
    const EMPTY: Self = Self {
        blocks: [[ptr::null_mut(); CACHED]; CLASSES],
        counts: [0; CLASSES],
    };
}

/// What one core alone writes, apart from every other core's lines.
#[repr(C, align(64))]
struct Core {
    cache: UnsafeCell<Cache>,
    /// The core's bytes allocated from its cache less those freed to it,
    /// which [`State::live`] does not count.
    cached: AtomicIsize,
    /// The core's bytes allocated less those it freed, by every route:
    /// what its own work holds, which other cores' allocations leave alone.
    own: AtomicIsize,
    /// [`Self::own`]'s size-class blocks by class.
    own_classes: [AtomicIsize; CLASSES],
    /// The owner the core's allocations are counted under.
    owner: AtomicUsize,
    /// The core's bytes allocated less those it freed, by the owner each
    /// block was allocated under.
    owned: [AtomicIsize; OWNERS],
}

impl Core {
    const fn new() -> Self {
        Self {
            cache: UnsafeCell::new(Cache::EMPTY),
            cached: AtomicIsize::new(0),
            own: AtomicIsize::new(0),
            own_classes: [const { AtomicIsize::new(0) }; CLASSES],
            owner: AtomicUsize::new(0),
            owned: [const { AtomicIsize::new(0) }; OWNERS],
        }
    }
}
/// Size buckets of the free list's live allocations: from 4 KiB, each a
/// power of two, the last holding everything from 128 MiB.
pub const LARGE_BUCKETS: usize = 16;

/// The [`LARGE_BUCKETS`] bucket of a free-list allocation of `bytes`.
fn large_bucket(bytes: usize) -> usize {
    ((usize::BITS - 1 - bytes.max(1).leading_zeros()) as usize)
        .saturating_sub(12)
        .min(LARGE_BUCKETS - 1)
}

/// The size class of `layout`, or `None` for the free list.
fn class(layout: Layout) -> Option<usize> {
    let bytes = layout.size().max(SMALLEST).next_power_of_two();
    (layout.align() <= SMALL_ALIGNMENT && bytes <= SMALLEST << (CLASSES - 1))
        .then(|| (bytes / SMALLEST).trailing_zeros() as usize)
}
/// Free blocks by size: the nonempty levels, each level's nonempty bins, and
/// the bins' blocks.
struct Bins {
    levels: u64,
    bins: [u16; LEVELS],
    heads: [[*mut Header; SUBS]; LEVELS],
}

impl Bins {
    const EMPTY: Self = Self {
        levels: 0,
        bins: [0; LEVELS],
        heads: [[ptr::null_mut(); SUBS]; LEVELS],
    };

    /// A free block `fits` accepts: the first of the smallest bin all of
    /// whose blocks hold `need` bytes, else one of `need`'s own bin.
    ///
    /// # Safety
    /// The lock is held.
    unsafe fn find(&self, need: usize, fits: impl Fn(*mut Header) -> bool) -> *mut Header {
        if let Some((level, sub)) = bin_round(need).map(bin)
            && level < LEVELS
        {
            let subs = u32::from(self.bins[level]) & (u32::MAX << sub);
            let found = if subs != 0 {
                Some((level, subs))
            } else {
                let above = u64::MAX.checked_shl(level as u32 + 1).unwrap_or(0);
                let levels = self.levels & above;
                (levels != 0).then(|| {
                    let level = levels.trailing_zeros() as usize;
                    (level, u32::from(self.bins[level]))
                })
            };
            if let Some((level, subs)) = found {
                let block = self.heads[level][subs.trailing_zeros() as usize];
                if fits(block) {
                    return block;
                }
            }
        }
        // Rounding passed over the blocks of `need`'s own bin that hold it.
        let (level, sub) = bin(need);
        if level >= LEVELS {
            return ptr::null_mut();
        }
        // SAFETY: the caller's lock.
        unsafe { bin_blocks(self.heads[level][sub]) }
            .find(|&block| fits(block))
            .unwrap_or(ptr::null_mut())
    }

    /// The largest free block's size: it is in the highest nonempty bin.
    ///
    /// # Safety
    /// The lock is held.
    unsafe fn largest(&self) -> usize {
        if self.levels == 0 {
            return 0;
        }
        let level = 63 - self.levels.leading_zeros() as usize;
        let sub = 15 - self.bins[level].leading_zeros() as usize;
        // SAFETY: the caller's lock.
        unsafe { bin_blocks(self.heads[level][sub]) }
            // SAFETY: binned free blocks, read with the lock held.
            .map(|block| unsafe { size(block) })
            .max()
            .unwrap_or(0)
    }
}

/// The bins of the blocks below [`State::floor`].
const MAIN: usize = 0;
/// The bins of the reserve's blocks.
const RESERVE: usize = 1;

struct State {
    /// The heap's extent.
    start: usize,
    end: usize,
    /// Where the reserve starts: the heap's end without one. No block
    /// crosses it, so the block there may follow a free one.
    floor: usize,
    pools: [Bins; 2],
    /// Free blocks in the bins.
    extents: usize,
    small: [*mut Small; CLASSES],
    ready: bool,
    total: usize,
    /// Bytes allocated and not freed, size-class blocks by their class.
    live: usize,
    /// Bytes carved from the bins and not released: live blocks and the
    /// size-class blocks kept for reuse.
    carved: usize,
    /// The part of `carved` in the reserve.
    carved_reserve: usize,
    peak: usize,
    /// The most carved since [`Heap::mark`].
    marked_peak: usize,
    /// Allocations the bins served.
    carves: u64,
    /// Live allocations the bins served: count and bytes by [`large_bucket`].
    large: [(u64, usize); LARGE_BUCKETS],
}

impl State {
    /// The bins of `block`.
    fn pool(&mut self, block: *mut Header) -> &mut Bins {
        &mut self.pools[usize::from(block as usize >= self.floor)]
    }

    /// Link free `block` into its bin.
    ///
    /// # Safety
    /// The lock is held and `block` is a free block in no bin.
    unsafe fn insert(&mut self, block: *mut Header) {
        // SAFETY: the caller's free block, which holds its links.
        let (level, sub) = bin(unsafe { size(block) });
        let pool = self.pool(block);
        let head = pool.heads[level][sub];
        // SAFETY: as above; `head`, when not null, is a binned free block.
        unsafe {
            links(block).write(Links {
                next: head,
                previous: ptr::null_mut(),
            });
            if !head.is_null() {
                (*links(head)).previous = block;
            }
        }
        pool.heads[level][sub] = block;
        pool.bins[level] |= 1 << sub;
        pool.levels |= 1 << level;
        self.extents += 1;
    }

    /// Unlink free `block` from its bin.
    ///
    /// # Safety
    /// The lock is held and `block` is in its bin.
    unsafe fn remove(&mut self, block: *mut Header) {
        // SAFETY: the caller's binned free block and its binned neighbours.
        unsafe {
            let (level, sub) = bin(size(block));
            let Links { next, previous } = links(block).read();
            if !next.is_null() {
                (*links(next)).previous = previous;
            }
            if previous.is_null() {
                let pool = self.pool(block);
                pool.heads[level][sub] = next;
                if next.is_null() {
                    pool.bins[level] &= !(1 << sub);
                    if pool.bins[level] == 0 {
                        pool.levels &= !(1 << level);
                    }
                }
            } else {
                (*links(previous)).next = next;
            }
        }
        self.extents -= 1;
    }

    /// Tell the block after `block`, if any, where `block` starts and
    /// whether it is free.
    ///
    /// # Safety
    /// The lock is held and `block`'s header is current.
    unsafe fn tell_following(&mut self, block: *mut Header) {
        // SAFETY: the caller's block; the one after it, inside the heap, is
        // a block whose header this updates.
        unsafe {
            let following = block as usize + size(block);
            if following < self.end {
                let following = following as *mut Header;
                let free = if (*block).size & FREE != 0 {
                    PREVIOUS_FREE
                } else {
                    0
                };
                set_previous(following, block as usize);
                (*following).size = (*following).size & !PREVIOUS_FREE | free;
            }
        }
    }

    /// Raise the high-water marks to the carved bytes.
    fn note_carved(&mut self) {
        self.marked_peak = self.marked_peak.max(self.carved);
        if self.carved > self.peak {
            self.peak = self.carved;
            passed_peak(self.carved);
        }
    }

    fn count_large(&mut self, bytes: usize, allocated: bool) {
        let (count, total) = &mut self.large[large_bucket(bytes)];
        if allocated {
            *count += 1;
            *total += bytes;
        } else {
            *count -= 1;
            *total -= bytes;
        }
    }
}

/// How full the heap is, in bytes.
#[derive(Clone, Copy, Debug)]
pub struct Usage {
    pub total: usize,
    pub live: usize,
    pub carved: usize,
    pub peak: usize,
    /// The largest free block: the most one allocation can get.
    pub largest: usize,
    /// Free blocks in the bins.
    pub extents: usize,
    /// Allocations the bins served.
    pub carves: u64,
    /// Live allocations the bins served, as count and bytes by power
    /// of two from 4 KiB ([`LARGE_BUCKETS`]); the rest of `live` is
    /// size-class blocks.
    pub large: [(u64, usize); LARGE_BUCKETS],
}
pub struct Heap {
    held: AtomicBool,
    failure_reported: AtomicBool,
    /// Allocations that found no memory, the first of them reported; a
    /// fallible caller such as `try_reserve` survives the rest.
    failures: AtomicU64,
    /// Where the first failed allocation is reported; it must not allocate.
    failure_sink: fn(core::fmt::Arguments<'_>),
    /// The first failed allocation's bytes and alignment, for a report that
    /// can no longer ask the heap ([`Heap::first_failure`]).
    failed: [AtomicUsize; 2],
    state: UnsafeCell<State>,
    /// Each tagged core's cache and counts.
    cores: [Core; CPUS],
    /// Cores tagged so far, and whether any is.
    tagged: AtomicUsize,
    any_tagged: AtomicBool,
    /// `State::live`, `State::total` and the main bins' bytes and carved
    /// bytes as the lock last left them, for [`Heap::low`] to read without
    /// taking it.
    live: AtomicUsize,
    total: AtomicUsize,
    main: AtomicUsize,
    carved: AtomicUsize,
    /// The heap's bytes not carved, the reserve's with the main bins'.
    room: AtomicUsize,
    waits: AtomicU64,
    /// The owner, and the counts by owner, of allocations on cores without
    /// a cache.
    default_owner: AtomicUsize,
    unowned: [AtomicIsize; OWNERS],
}
// SAFETY: all list/header mutations use the real acquire/release exclusion.
// UEFI callers mask local interrupts before acquiring the lock. Critical
// sections cannot migrate/block; NMI and fault handlers never reenter the heap.
unsafe impl Sync for Heap {}
impl Default for Heap {
    fn default() -> Self {
        Self::new()
    }
}

impl Heap {
    /// Report the first failed allocation to `sink`, which must neither
    /// allocate nor wait (the mount's serial port).
    pub const fn reporting_failures(self, sink: fn(core::fmt::Arguments<'_>)) -> Self {
        Self {
            failure_sink: sink,
            ..self
        }
    }

    /// The bytes and alignment of the first allocation that found no
    /// memory, once one has.
    pub fn first_failure(&self) -> Option<(usize, usize)> {
        let align = self.failed[1].load(Ordering::Acquire);
        (align != 0).then(|| (self.failed[0].load(Ordering::Relaxed), align))
    }

    pub const fn new() -> Self {
        Self {
            held: AtomicBool::new(false),
            failure_reported: AtomicBool::new(false),
            failures: AtomicU64::new(0),
            failure_sink: |_| {},
            failed: [const { AtomicUsize::new(0) }; 2],
            cores: [const { Core::new() }; CPUS],
            tagged: AtomicUsize::new(0),
            any_tagged: AtomicBool::new(false),
            live: AtomicUsize::new(0),
            total: AtomicUsize::new(0),
            main: AtomicUsize::new(0),
            carved: AtomicUsize::new(0),
            room: AtomicUsize::new(0),
            waits: AtomicU64::new(0),
            default_owner: AtomicUsize::new(0),
            unowned: [const { AtomicIsize::new(0) }; OWNERS],
            state: UnsafeCell::new(State {
                start: 0,
                end: 0,
                floor: 0,
                pools: [Bins::EMPTY; 2],
                extents: 0,
                small: [ptr::null_mut(); CLASSES],
                ready: false,
                total: 0,
                live: 0,
                carved: 0,
                carved_reserve: 0,
                peak: 0,
                marked_peak: 0,
                carves: 0,
                large: [(0, 0); LARGE_BUCKETS],
            }),
        }
    }
    fn lock(&self) -> Guard<'_> {
        // Save before spinning: an ISR must not wait for its interrupted
        // context to release this same lock. No state access crosses the asm.
        #[cfg(target_os = "uefi")]
        let flags: usize;
        #[cfg(target_os = "uefi")]
        // SAFETY: the UEFI allocator executes at CPL0 on this CPU; save IF
        // before masking reentry, with no state borrow spanning restoration.
        unsafe {
            core::arch::asm!("pushfq", "pop {}", "cli", out(reg) flags);
        }
        if self
            .held
            .compare_exchange(false, true, Ordering::Acquire, Ordering::Relaxed)
            .is_err()
        {
            self.waits.fetch_add(1, Ordering::Relaxed);
            while self
                .held
                .compare_exchange_weak(false, true, Ordering::Acquire, Ordering::Relaxed)
                .is_err()
            {
                core::hint::spin_loop();
            }
        }
        Guard {
            heap: self,
            #[cfg(target_os = "uefi")]
            flags,
            same_cpu: PhantomData,
        }
    }
    fn try_lock(&self) -> Option<Guard<'_>> {
        // Mask before the single attempt, as for the blocking owner. An ISR
        // cannot safely wait on its interrupted context's allocator lock.
        #[cfg(target_os = "uefi")]
        let flags: usize;
        #[cfg(target_os = "uefi")]
        // SAFETY: the UEFI allocator executes at CPL0 on this CPU; save IF
        // before masking reentry, with no state borrow spanning restoration.
        unsafe {
            core::arch::asm!("pushfq", "pop {}", "cli", out(reg) flags);
        }
        if self
            .held
            .compare_exchange(false, true, Ordering::Acquire, Ordering::Relaxed)
            .is_err()
        {
            #[cfg(target_os = "uefi")]
            // SAFETY: no guard/state access was acquired. Restore this CPU's
            // exact incoming flags, including an already-clear interrupt flag.
            unsafe {
                core::arch::asm!("push {}", "popfq", in(reg) flags);
            }
            return None;
        }
        Some(Guard {
            heap: self,
            #[cfg(target_os = "uefi")]
            flags,
            same_cpu: PhantomData,
        })
    }
    /// # Safety
    /// The entire aligned extent is live RW+NX boot-private Normal-WB RAM, has
    /// no aliases/references/other owners, and remains reserved for the boot.
    /// Called exactly once before any Rust allocation with reentry excluded.
    pub unsafe fn initialize(&self, base: *mut u8, bytes: usize) {
        let _guard = self.lock();
        // SAFETY: this live acquire/release guard excludes every metadata
        // writer and masks local reentry until this state borrow ends.
        let state = unsafe { &mut *self.state.get() };
        assert!(!state.ready && !base.is_null());
        let start = aligned(base as usize, GRANULE).expect("a heap below the address space's end");
        let end = (base as usize + bytes) & !(GRANULE - 1);
        assert!(end >= start + MIN_BLOCK);
        let block = start as *mut Header;
        // SAFETY: initialize's exclusive live extent holds this first block,
        // aligned to a granule and at least the smallest block.
        unsafe {
            block.write(Header {
                previous: 0,
                size: (end - start) | FREE,
            });
        }
        state.start = start;
        state.end = end;
        state.floor = end;
        // SAFETY: the guard and the one free block just written.
        unsafe { state.insert(block) };
        state.ready = true;
        state.total = end - start;
    }

    /// Keep the last `bytes` of an unused heap out of the main bins, for large
    /// requests they cannot serve and for what they run out of.
    ///
    /// # Panics
    /// When the heap has been used or holds no more than `bytes`.
    pub fn reserve_tail(&self, bytes: usize) {
        let _guard = self.lock();
        // SAFETY: this live acquire/release guard excludes every metadata
        // writer and masks local reentry until this state borrow ends.
        let state = unsafe { &mut *self.state.get() };
        let block = state.start as *mut Header;
        let floor = state.end.saturating_sub(bytes) & !(GRANULE - 1);
        assert!(state.ready && state.carved == 0 && bytes >= MIN_BLOCK);
        assert!(floor >= state.start + MIN_BLOCK);
        // SAFETY: the guard; an unused heap is its one free block, which the
        // new header splits at a granule with a block's room on each side.
        unsafe {
            state.remove(block);
            (*block).size = (floor - state.start) | FREE;
            state.floor = floor;
            state.insert(block);
            let reserve = floor as *mut Header;
            reserve.write(Header {
                previous: block as usize,
                size: (state.end - floor) | FREE | PREVIOUS_FREE,
            });
            state.insert(reserve);
        }
    }

    /// Stop managing the heap's end beyond `keep` more bytes of its last
    /// free block, and return those whole pages for their owner to reuse.
    /// `None` when the last block is allocated or holds no more than `keep`.
    pub fn release_tail(&self, keep: usize) -> Option<Range<usize>> {
        let _guard = self.lock();
        // SAFETY: this live acquire/release guard excludes every metadata
        // writer and masks local reentry until this state borrow ends.
        let state = unsafe { &mut *self.state.get() };
        if !state.ready {
            return None;
        }
        let mut block = state.start as *mut Header;
        // SAFETY: the guard; the blocks cover the heap, each named by its
        // predecessor's size, and the last ends at `state.end`.
        unsafe {
            while block as usize + size(block) < state.end {
                block = (block as usize + size(block)) as *mut Header;
            }
            if (*block).size & FREE == 0 {
                return None;
            }
            let end = aligned(block as usize + keep.max(MIN_BLOCK), PAGE_BYTES)?;
            if end >= state.end {
                return None;
            }
            let released = end..state.end;
            state.remove(block);
            (*block).size = (end - block as usize) | (*block).size & (FREE | PREVIOUS_FREE);
            state.end = end;
            state.floor = state.floor.min(end);
            state.total -= released.len();
            state.insert(block);
            Some(released)
        }
    }

    /// Attempt ordinary heap allocation without waiting for the heap lock.
    /// Zero-sized requests, an uninitialized heap, contention and exhaustion
    /// return NULL without changing heap bytes or accounting. Success uses the
    /// ordinary size-class/alignment/header rules and exact-layout deallocation.
    /// This makes one lock attempt; a successful free-list search is finite,
    /// but is not promised to take constant time.
    ///
    /// # Safety
    /// The caller preserves the initialized heap's RAM/mapping/owner
    /// lifetime and exclusively owns a successful payload until its one matching
    /// `GlobalAlloc::dealloc` on this same heap with this exact layout. NMI/fault
    /// handlers do not reenter the heap. No physical/page/DMA permission follows.
    #[cfg_attr(
        not(test),
        allow(
            dead_code,
            reason = "reserved nonblocking entry for the retained Linux GPU heap registration"
        )
    )]
    pub unsafe fn try_allocate(&self, layout: Layout) -> *mut u8 {
        if layout.size() == 0 {
            return ptr::null_mut();
        }
        let Some(guard) = self.try_lock() else {
            return ptr::null_mut();
        };
        // SAFETY: this guard exclusively protects the heap state. The
        // shared body checks initialization before accessing backing storage.
        let block = unsafe { allocate_locked(&mut *self.state.get(), layout, class(layout)) };
        drop(guard);
        if !block.is_null() {
            let bytes = class(layout).map_or(layout.size(), |class| SMALLEST << class);
            // SAFETY: the new allocation, which only this call holds.
            unsafe { self.claim(self.cpu(), block, bytes) };
        }
        if class(layout).is_none() {
            tracked(block, layout.size(), true);
        }
        block
    }

    /// Attempt allocation as in `try_allocate`, then zero the entire payload
    /// before returning it. Zeroing owns only the successful payload and takes
    /// place after releasing the metadata guard/restoring the incoming IRQ state.
    ///
    /// # Safety
    /// The complete `try_allocate` ownership and exclusion contract applies.
    #[cfg_attr(
        not(test),
        allow(
            dead_code,
            reason = "zeroed counterpart of the reserved Linux GPU heap entry"
        )
    )]
    pub unsafe fn try_allocate_zeroed(&self, layout: Layout) -> *mut u8 {
        // SAFETY: the caller keeps the same heap, mapping and exact layout.
        let block = unsafe { self.try_allocate(layout) };
        if !block.is_null() {
            // SAFETY: successful allocation gives this full payload exclusively.
            unsafe { block.write_bytes(0, layout.size()) };
        }
        block
    }

    /// Select whole, zeroed pages inside a CPU interval from this same heap.
    /// No reservation or CPU-to-physical relationship is inferred here.
    /// Invalid geometry and exhaustion change no free-list bytes/accounting.
    /// Returned pointers use ordinary `GlobalAlloc::dealloc` and their original
    /// layout; page alignment excludes the small-object size-class route.
    ///
    /// # Safety
    /// The caller obeys the initialized heap's allocation/exclusion contract,
    /// keeps its RAM/mapping/root epoch, and owns the returned payload
    /// exclusively until exact-layout deallocation. `allowed` constrains payload
    /// bytes only; the canonical allocation header remains private heap metadata.
    pub unsafe fn alloc_pages_zeroed_in(&self, layout: Layout, allowed: Range<usize>) -> *mut u8 {
        if layout.size() == 0
            || !layout.size().is_multiple_of(PAGE_BYTES)
            || layout.align() < PAGE_BYTES
            || allowed
                .end
                .checked_sub(allowed.start)
                .is_none_or(|bytes| bytes < layout.size())
        {
            return ptr::null_mut();
        }
        let guard = self.lock();
        // SAFETY: acquire/release exclusion, as for ordinary allocation.
        let state = unsafe { &mut *self.state.get() };
        if !state.ready {
            return ptr::null_mut();
        }
        // SAFETY: lock held; selection changes nothing until a complete fit.
        let block = unsafe { carve_within(state, layout.size(), layout.align(), allowed) };
        if !block.is_null() {
            state.live += layout.size();
            state.count_large(layout.size(), true);
        }
        // No metadata/state borrow survives zeroing or a possible IRQ restore.
        drop(guard);
        if !block.is_null() {
            // SAFETY: the selected full payload is exclusively owned and live.
            unsafe { block.write_bytes(0, layout.size()) };
            // SAFETY: the new allocation, which only this call holds.
            unsafe { self.claim(self.cpu(), block, layout.size()) };
            tracked(block, layout.size(), true);
        }
        block
    }

    /// Whether more than three quarters of the heap is allocated, as the
    /// lock and each core's cache last left it, or so nearly all of the main
    /// bins' bytes are out of them that a large request may find no block:
    /// freed small blocks wait in their classes, and what they free between
    /// live ones is holes.
    pub fn low(&self) -> bool {
        let total = self.total.load(Ordering::Relaxed);
        self.live() > total / 4 * 3
            || self.carved.load(Ordering::Relaxed) > self.main.load(Ordering::Relaxed) / 16 * 15
    }

    /// The heap's bytes, as the lock last left them.
    pub fn total(&self) -> usize {
        self.total.load(Ordering::Relaxed)
    }

    /// The bytes no block was carved from, the reserve's with the main
    /// bins', as the lock last left them: what a large request can surely
    /// still find, short of the holes between live blocks and the freed
    /// small blocks their classes keep. A cache that may grow asks it
    /// without taking the lock.
    pub fn room(&self) -> usize {
        self.room.load(Ordering::Relaxed)
    }

    /// Bytes allocated and not freed: the class lists' and the caches'.
    fn live(&self) -> usize {
        self.live
            .load(Ordering::Relaxed)
            .saturating_add_signed(self.cached_live())
    }

    /// Bytes the caches allocated less those freed to them.
    fn cached_live(&self) -> isize {
        (0..self.tagged.load(Ordering::Relaxed).min(CPUS))
            .map(|cpu| self.cores[cpu].cached.load(Ordering::Relaxed))
            .sum()
    }

    /// Give the calling core a cache of its own, which every later
    /// allocation on it uses: once per core, with `record` at
    /// [`CORE_RECORD`] from its GS base. A core with a tagged one finds its
    /// cache there in a load, where an instruction that names the core
    /// costs tens of cycles on every allocation and free.
    ///
    /// # Safety
    /// `record` lies at [`CORE_RECORD`] from the calling core's GS base, for
    /// as long as it runs, and every core allocates only once its GS base
    /// is set, after the first call.
    #[cfg(target_os = "uefi")]
    pub unsafe fn tag_cpu(&self, record: *mut u32) {
        let cpu = self.tagged.fetch_add(1, Ordering::Relaxed);
        if cpu >= CPUS {
            return;
        }
        // SAFETY: as the caller guarantees.
        unsafe { record.write(TAG | cpu as u32) };
        self.any_tagged.store(true, Ordering::Release);
    }

    /// The calling core's cache, if it has one.
    fn cpu(&self) -> Option<usize> {
        if !self.any_tagged.load(Ordering::Relaxed) {
            return None;
        }
        #[cfg(not(target_os = "uefi"))]
        let aux = 0;
        #[cfg(target_os = "uefi")]
        let aux: u32;
        // SAFETY: as `tag_cpu` requires, this core's GS base is set.
        #[cfg(target_os = "uefi")]
        unsafe {
            core::arch::asm!("mov {:e}, gs:[{}]", out(reg) aux, const CORE_RECORD,
                options(nostack, preserves_flags, readonly));
        }
        (aux & !0xffff == TAG)
            .then_some((aux & 0xffff) as usize)
            .filter(|&cpu| cpu < CPUS)
    }

    /// A block of `class` from core `cpu`'s cache, refilled under the lock
    /// when empty; null when the heap has none.
    ///
    /// # Safety
    /// On core `cpu`, with interrupts disabled.
    unsafe fn take_cached(&self, cpu: usize, class: usize) -> *mut u8 {
        // SAFETY: only this core reaches its cache, and nothing interrupts.
        let cache = unsafe { &mut *self.cores[cpu].cache.get() };
        if cache.counts[class] == 0 {
            let _guard = self.lock();
            // SAFETY: the guard excludes all free-list writers, while
            // CPU-local cache ownership is already exclusive to this core.
            let state = unsafe { &mut *self.state.get() };
            while cache.counts[class] < BATCH {
                let head = state.small[class];
                let block = if head.is_null() {
                    // SAFETY: the initialized heap is exclusively guarded;
                    // this class supplies its nonzero extent and alignment.
                    unsafe { carve_draining(state, SMALLEST << class, SMALL_ALIGNMENT) }
                } else {
                    // SAFETY: the guarded class head is a complete free Small
                    // node previously published by matching deallocation.
                    state.small[class] = unsafe { (*head).next };
                    head.cast()
                };
                if block.is_null() {
                    break;
                }
                cache.blocks[class][cache.counts[class]] = block.cast();
                cache.counts[class] += 1;
            }
        }
        if cache.counts[class] == 0 {
            return ptr::null_mut();
        }
        cache.counts[class] -= 1;
        self.count_cached(cpu, (SMALLEST << class) as isize);
        cache.blocks[class][cache.counts[class]].cast()
    }

    /// Give `block` of `class` to core `cpu`'s cache, moving a batch to the
    /// class list under the lock when full.
    ///
    /// # Safety
    /// On core `cpu`, with interrupts disabled; `block` is a freed block of
    /// `class`.
    unsafe fn give_cached(&self, cpu: usize, class: usize, block: *mut u8) {
        // SAFETY: only this core reaches its cache, and nothing interrupts.
        let cache = unsafe { &mut *self.cores[cpu].cache.get() };
        if cache.counts[class] == CACHED {
            let _guard = self.lock();
            // SAFETY: the guard excludes all free-list writers, while
            // CPU-local cache ownership is already exclusive to this core.
            let state = unsafe { &mut *self.state.get() };
            for _ in 0..BATCH {
                cache.counts[class] -= 1;
                let freed = cache.blocks[class][cache.counts[class]];
                // SAFETY: a free block of this class that only the cache held.
                unsafe {
                    freed.write(Small {
                        next: state.small[class],
                    })
                };
                state.small[class] = freed;
            }
        }
        cache.blocks[class][cache.counts[class]] = block.cast();
        cache.counts[class] += 1;
        self.count_cached(cpu, -((SMALLEST << class) as isize));
    }

    /// Count the calling core's later allocations under `owner`, below
    /// [`OWNERS`] (0: under none), until the next call; the owner before.
    /// A core without a cache sets the owner of all such cores.
    pub fn set_owner(&self, owner: usize) -> usize {
        let owner = owner & OWNER_BITS;
        match self.cpu() {
            Some(cpu) => {
                let current = &self.cores[cpu].owner;
                let before = current.load(Ordering::Relaxed);
                current.store(owner, Ordering::Relaxed);
                before
            }
            None => self.default_owner.swap(owner, Ordering::Relaxed),
        }
    }

    /// The live bytes each owner allocated, owner 0 none: the rest of
    /// [`Usage::live`].
    pub fn owners(&self) -> [isize; OWNERS] {
        let cores = self.tagged.load(Ordering::Relaxed).min(CPUS);
        core::array::from_fn(|owner| {
            self.unowned[owner].load(Ordering::Relaxed)
                + self.cores[..cores]
                    .iter()
                    .map(|core| core.owned[owner].load(Ordering::Relaxed))
                    .sum::<isize>()
        })
    }

    /// Add `bytes` to `owner`'s count on core `cpu`, which only it writes,
    /// or on the shared count without one.
    fn count_owned(&self, cpu: Option<usize>, owner: usize, bytes: isize) {
        if owner == 0 {
            return;
        }
        match cpu {
            Some(cpu) => {
                let count = &self.cores[cpu].owned[owner];
                count.store(count.load(Ordering::Relaxed) + bytes, Ordering::Relaxed);
            }
            None => {
                self.unowned[owner].fetch_add(bytes, Ordering::Relaxed);
            }
        }
    }

    /// Name the calling core's owner the owner of new block `pointer` of
    /// `bytes` and count them under it.
    ///
    /// # Safety
    /// `pointer` is a live allocation of the heap that the caller owns.
    unsafe fn claim(&self, cpu: Option<usize>, pointer: *mut u8, bytes: usize) {
        let owner = match cpu {
            Some(cpu) => self.cores[cpu].owner.load(Ordering::Relaxed),
            None => self.default_owner.load(Ordering::Relaxed),
        };
        // SAFETY: the caller's live allocation.
        unsafe { set_owner_of(pointer, owner) };
        self.count_owned(cpu, owner, bytes as isize);
    }

    /// Take `bytes` of block `pointer`, about to be freed, off its owner's
    /// count.
    ///
    /// # Safety
    /// `pointer` is a live allocation of the heap that the caller owns.
    unsafe fn disclaim(&self, cpu: Option<usize>, pointer: *mut u8, bytes: usize) {
        // SAFETY: the caller's live allocation.
        let owner = unsafe { owner_of(pointer) };
        self.count_owned(cpu, owner, -(bytes as isize));
    }

    /// Add `bytes` to core `cpu`'s count, which only it writes.
    fn count_cached(&self, cpu: usize, bytes: isize) {
        let count = &self.cores[cpu].cached;
        count.store(count.load(Ordering::Relaxed) + bytes, Ordering::Relaxed);
    }

    /// Ordinary allocations that found no memory so far; only the first is
    /// reported in full.
    pub fn failures(&self) -> u64 {
        self.failures.load(Ordering::Relaxed)
    }

    /// Acquisitions of the heap's lock that found it held: with every PE
    /// on a core of its own, the cores' allocations meet there.
    pub fn waits(&self) -> u64 {
        self.waits.load(Ordering::Relaxed)
    }

    /// The calling core's bytes allocated less those it freed, once it is
    /// tagged.
    pub fn own(&self) -> Option<isize> {
        self.cpu()
            .map(|cpu| self.cores[cpu].own.load(Ordering::Relaxed))
    }

    /// Each tagged core's bytes allocated less those it freed: a block one
    /// core allocates and another frees moves between their counts.
    pub fn owned(&self) -> impl Iterator<Item = isize> + '_ {
        let cores = self.tagged.load(Ordering::Relaxed).min(CPUS);
        self.cores[..cores]
            .iter()
            .map(|core| core.own.load(Ordering::Relaxed))
    }

    /// Add `bytes` to the calling core's own count, which only it writes.
    fn count_own(&self, cpu: usize, layout: Layout, class: Option<usize>, allocated: bool) {
        let bytes = class.map_or(layout.size(), |class| SMALLEST << class) as isize;
        let bytes = if allocated { bytes } else { -bytes };
        let core = &self.cores[cpu];
        let count = &core.own;
        count.store(count.load(Ordering::Relaxed) + bytes, Ordering::Relaxed);
        if let Some(class) = class {
            let count = &core.own_classes[class];
            count.store(count.load(Ordering::Relaxed) + bytes, Ordering::Relaxed);
        }
    }

    /// Live size-class bytes by class (16 bytes doubling to 4 KiB), as the
    /// tagged cores counted them: allocations before tagging are not in it.
    pub fn class_live(&self) -> [isize; CLASSES] {
        let cores = self.tagged.load(Ordering::Relaxed).min(CPUS);
        core::array::from_fn(|class| {
            self.cores[..cores]
                .iter()
                .map(|core| core.own_classes[class].load(Ordering::Relaxed))
                .sum()
        })
    }

    /// Count the whole page frames that can individually serve a one-page
    /// constrained request at the time of the snapshot. This walks the free
    /// bins under the heap lock without draining small-object caches.
    /// Adjacent frames may need an intervening allocation header when
    /// allocated separately, so this is not a reservation or allocation count.
    pub fn available_page_frames_in(&self, allowed: Range<usize>) -> usize {
        let _guard = self.lock();
        // SAFETY: the guard keeps the free blocks and their bins stable.
        let state = unsafe { &*self.state.get() };
        let (first_level, _) = bin(PAGE_BYTES + HEADER_BYTES);
        state
            .pools
            .iter()
            .flat_map(|pool| pool.heads[first_level..].iter().flatten())
            // SAFETY: the guard keeps every binned link and header live.
            .flat_map(|&head| unsafe { bin_blocks(head) })
            .filter_map(|block| {
                // SAFETY: the guard keeps this binned free block stable.
                let first =
                    unsafe { fit(block, PAGE_BYTES, PAGE_BYTES, allowed.start, allowed.end) }?;
                // SAFETY: the same guarded free block as above.
                let end = (block.addr() + unsafe { size(block) }).min(allowed.end);
                Some((end - first) / PAGE_BYTES)
            })
            .sum()
    }

    /// Start a new high-water mark of the carved bytes, for
    /// [`Self::marked_peak`]: the bytes carved now.
    pub fn mark(&self) -> usize {
        let _guard = self.lock();
        // SAFETY: the guard excludes every other state access.
        let state = unsafe { &mut *self.state.get() };
        state.marked_peak = state.carved;
        state.carved
    }

    /// The most bytes carved at once since the last [`Self::mark`].
    pub fn marked_peak(&self) -> usize {
        let _guard = self.lock();
        // SAFETY: the guard excludes metadata writers.
        unsafe { (*self.state.get()).marked_peak }
    }

    /// The heap's usage, walking the free list for its largest extent.
    pub fn usage(&self) -> Usage {
        let _guard = self.lock();
        // SAFETY: the guard excludes metadata writers throughout; no state
        // reference escapes it.
        let state = unsafe { &*self.state.get() };
        self.usage_locked(state)
    }

    // The caller keeps the metadata guard; CPU cache counters remain
    // relaxed estimates even while this free-list metadata is exclusively read.
    fn usage_locked(&self, state: &State) -> Usage {
        let largest = state
            .pools
            .iter()
            // SAFETY: the caller's guard.
            .map(|pool| unsafe { pool.largest() })
            .max()
            .unwrap_or(0);
        let live = state.live.saturating_add_signed(self.cached_live());
        Usage {
            total: state.total,
            live,
            carved: state.carved,
            peak: state.peak,
            largest,
            extents: state.extents,
            carves: state.carves,
            large: state.large,
        }
    }
}
struct Guard<'a> {
    heap: &'a Heap,
    #[cfg(target_os = "uefi")]
    flags: usize,
    // A saved interrupt state belongs to the acquiring CPU, even though the
    // underlying heap may be shared. Do not make this guard Send or Sync.
    same_cpu: PhantomData<*mut ()>,
}
impl Drop for Guard<'_> {
    fn drop(&mut self) {
        // SAFETY: the lock is still held; no other borrow of the state lives.
        let state = unsafe { &*self.heap.state.get() };
        self.heap.live.store(state.live, Ordering::Relaxed);
        self.heap.total.store(state.total, Ordering::Relaxed);
        self.heap
            .main
            .store(state.floor - state.start, Ordering::Relaxed);
        self.heap
            .carved
            .store(state.carved - state.carved_reserve, Ordering::Relaxed);
        self.heap
            .room
            .store(state.total - state.carved, Ordering::Relaxed);
        self.heap.held.store(false, Ordering::Release);
        #[cfg(target_os = "uefi")]
        // SAFETY: this CPU's saved flags; the guard cannot migrate and no
        // lock/state access spans restoration. An incoming clear IF stays clear.
        unsafe {
            core::arch::asm!("push {}", "popfq", in(reg) self.flags);
        }
    }
}
/// Disable interrupts, returning RFLAGS before.
fn interrupts_off() -> usize {
    #[cfg(not(target_os = "uefi"))]
    let flags = 0;
    #[cfg(target_os = "uefi")]
    let flags: usize;
    // SAFETY: CPL0; changes only IF.
    #[cfg(target_os = "uefi")]
    unsafe {
        core::arch::asm!("pushfq", "pop {}", "cli", out(reg) flags)
    };
    flags
}

/// Restore RFLAGS' interrupt flag as [`interrupts_off`] found it.
fn restore_interrupts(flags: usize) {
    // SAFETY: this core's flags from `interrupts_off`.
    #[cfg(target_os = "uefi")]
    unsafe {
        core::arch::asm!("push {}", "popfq", in(reg) flags)
    };
    #[cfg(not(target_os = "uefi"))]
    let _ = flags;
}

fn aligned(value: usize, alignment: usize) -> Option<usize> {
    value
        .checked_add(alignment - 1)
        .map(|v| v & !(alignment - 1))
}
/// Ordinary and attempted allocation share the original locked algorithm.
///
/// # Safety
/// The heap's guard exclusively owns `state`. Initialization is checked
/// before backing access; `class` must be the size class of `layout`.
unsafe fn allocate_locked(state: &mut State, layout: Layout, class: Option<usize>) -> *mut u8 {
    if !state.ready {
        return ptr::null_mut();
    }
    let Some(class) = class else {
        // SAFETY: this helper keeps the initialized state guard and
        // GlobalAlloc's layout supplies the payload size and alignment.
        let block = unsafe { carve_draining(state, layout.size(), layout.align()) };
        if !block.is_null() {
            state.live += layout.size();
            state.count_large(layout.size(), true);
        }
        return block;
    };
    let head = state.small[class];
    let block = if head.is_null() {
        // SAFETY: guarded initialized state and a validated nonzero class.
        unsafe { carve_draining(state, SMALLEST << class, SMALL_ALIGNMENT) }
    } else {
        // SAFETY: a block of this class that `dealloc` linked and
        // nothing else owns.
        state.small[class] = unsafe { (*head).next };
        head.cast()
    };
    if !block.is_null() {
        state.live += SMALLEST << class;
    }
    block
}

// SAFETY: callers follow the GlobalAlloc contract. The free-list split returns
// a disjoint aligned extent, and its header records the original bytes until
// the single matching deallocation.
unsafe impl GlobalAlloc for Heap {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        let class = class(layout);
        let cpu = self.cpu();
        let block = if let (Some(class), Some(cpu)) = (class, cpu) {
            let flags = interrupts_off();
            // SAFETY: on this core, with interrupts disabled.
            let block = unsafe { self.take_cached(cpu, class) };
            restore_interrupts(flags);
            block
        } else {
            let _guard = self.lock();
            // SAFETY: the guard excludes all free-list writers, while
            // CPU-local cache ownership is already exclusive to this core.
            let state = unsafe { &mut *self.state.get() };
            // SAFETY: the guard owns this state for the entire operation.
            unsafe { allocate_locked(state, layout, class) }
        };
        if block.is_null() {
            self.failures.fetch_add(1, Ordering::Relaxed);
            failure::report(self, layout);
        } else {
            if let Some(cpu) = cpu {
                self.count_own(cpu, layout, class, true);
            }
            let bytes = class.map_or(layout.size(), |class| SMALLEST << class);
            // SAFETY: the new allocation, which only this call holds.
            unsafe { self.claim(cpu, block, bytes) };
            if class.is_none() {
                tracked(block, layout.size(), true);
            }
        }
        block
    }
    unsafe fn dealloc(&self, pointer: *mut u8, layout: Layout) {
        let class = class(layout);
        let cpu = self.cpu();
        let bytes = class.map_or(layout.size(), |class| SMALLEST << class);
        // SAFETY: the caller's live allocation, freed below.
        unsafe { self.disclaim(cpu, pointer, bytes) };
        if let Some(cpu) = cpu {
            self.count_own(cpu, layout, class, false);
            if let Some(class) = class {
                let flags = interrupts_off();
                // SAFETY: on this core, with interrupts disabled; the caller's
                // block of this class, which it no longer accesses.
                unsafe { self.give_cached(cpu, class, pointer) };
                restore_interrupts(flags);
                return;
            }
        }
        if class.is_none() {
            tracked(pointer, layout.size(), false);
        }
        let _guard = self.lock();
        // SAFETY: this live acquire/release guard excludes every metadata
        // writer and masks local reentry until this state borrow ends.
        let state = unsafe { &mut *self.state.get() };
        if let Some(class) = class {
            state.live -= SMALLEST << class;
            let block = pointer.cast::<Small>();
            // SAFETY: the caller's block of this class, at least 16 bytes
            // and 16-byte aligned, which it no longer accesses.
            unsafe {
                block.write(Small {
                    next: state.small[class],
                })
            };
            state.small[class] = block;
        } else {
            state.live -= layout.size();
            state.count_large(layout.size(), false);
            // SAFETY: GlobalAlloc requires this live allocation and
            // layout; its last owner has ended all payload access, lock held.
            unsafe { release(state, pointer) };
        }
    }

    unsafe fn realloc(&self, pointer: *mut u8, layout: Layout, new_size: usize) -> *mut u8 {
        // SAFETY: GlobalAlloc's realloc contract guarantees that this size
        // rounded to the original alignment fits in isize.
        let new_layout = unsafe { Layout::from_size_align_unchecked(new_size, layout.align()) };
        if layout.size() == new_size {
            return pointer;
        }
        let (old_class, new_class) = (class(layout), class(new_layout));
        if old_class.is_some() && old_class == new_class {
            // The live block already has the full size-class capacity, and
            // this resize leaves all class accounting and ownership intact.
            return pointer;
        }
        if old_class.is_none() && new_class.is_none() {
            let resized = {
                let _guard = self.lock();
                // SAFETY: the guard excludes other accesses to the state.
                let state = unsafe { &mut *self.state.get() };
                // SAFETY: the old layout identifies a live free-list block
                // exclusively owned by the caller, and the lock is held.
                let resized = unsafe { resize_large(state, pointer, new_size) };
                if resized {
                    state.live = state.live - layout.size() + new_size;
                    state.count_large(layout.size(), false);
                    state.count_large(new_size, true);
                }
                resized
            };
            if resized {
                let cpu = self.cpu();
                if let Some(cpu) = cpu {
                    self.count_own(cpu, layout, None, false);
                    self.count_own(cpu, new_layout, None, true);
                }
                // SAFETY: the caller's live allocation, resized in place.
                let owner = unsafe { owner_of(pointer) };
                self.count_owned(cpu, owner, new_size as isize - layout.size() as isize);
                tracked(pointer, layout.size(), false);
                tracked(pointer, new_size, true);
                return pointer;
            }
        }
        // SAFETY: both layouts meet GlobalAlloc's realloc contract. A new
        // successful allocation is disjoint, and only then can the old one
        // be freed. On failure the old owner keeps its block.
        unsafe {
            let next = self.alloc(new_layout);
            if !next.is_null() {
                ptr::copy_nonoverlapping(pointer, next, layout.size().min(new_size));
                self.dealloc(pointer, layout);
            }
            next
        }
    }
}

/// Resize a free-list allocation without moving its payload, using or
/// returning the following free space in the same pool.
///
/// # Safety
/// The heap's lock is held. `pointer` identifies a live free-list block,
/// exclusively owned by the caller, and `new_size` is nonzero.
unsafe fn resize_large(state: &mut State, pointer: *mut u8, new_size: usize) -> bool {
    let Some(wanted) = aligned(new_size, GRANULE).and_then(|bytes| bytes.checked_add(HEADER_BYTES))
    else {
        return false;
    };
    // SAFETY: the old block and its neighbours have live headers, and the
    // lock protects both the boundary tags and bins. The replacement header
    // lies beyond the retained payload and inside the old or joined block.
    unsafe {
        let block = pointer.sub(HEADER_BYTES).cast::<Header>();
        let old_bytes = size(block);
        if wanted == old_bytes {
            return true;
        }
        let following = block as usize + old_bytes;
        let next = following as *mut Header;
        let join = following < state.end && following != state.floor && (*next).size & FREE != 0;
        let available = if join {
            old_bytes + size(next)
        } else {
            old_bytes
        };
        if wanted > available {
            return false;
        }
        let new_bytes = if available - wanted >= MIN_BLOCK {
            wanted
        } else {
            available
        };
        if join {
            state.remove(next);
        }
        (*block).size = new_bytes | ((*block).size & PREVIOUS_FREE);
        if new_bytes < available {
            let rest = (block as usize + new_bytes) as *mut Header;
            rest.write(Header {
                previous: block as usize,
                size: (available - new_bytes) | FREE,
            });
            state.tell_following(rest);
            state.insert(rest);
        } else {
            state.tell_following(block);
        }
        state.carved = state.carved - old_bytes + new_bytes;
        if block as usize >= state.floor {
            state.carved_reserve = state.carved_reserve - old_bytes + new_bytes;
        }
        state.note_carved();
    }
    true
}

/// Where a payload of `payload` bytes aligned to `alignment` fits in free
/// `block`, starting at or after `from` and ending by `limit`: its address,
/// after a gap before its header that is empty or large enough to stay free.
///
/// # Safety
/// The lock is held and `block` is a free block.
unsafe fn fit(
    block: *mut Header,
    payload: usize,
    alignment: usize,
    from: usize,
    limit: usize,
) -> Option<usize> {
    let base = block as usize;
    // SAFETY: the caller's free block.
    let end = (base + unsafe { size(block) }).min(limit);
    let mut user = aligned(base.checked_add(HEADER_BYTES)?.max(from), alignment)?;
    let gap = user - HEADER_BYTES - base;
    if gap != 0 && gap < MIN_BLOCK {
        user = aligned(user.checked_add(MIN_BLOCK - gap)?, alignment)?;
    }
    (user.checked_add(payload)? <= end).then_some(user)
}

/// Allocate the payload at `user` of `payload` bytes from free `block`,
/// which is out of its bin: a gap before its header and a remainder after
/// it that can hold a block become free blocks; a smaller remainder stays
/// in the allocation.
///
/// # Safety
/// The lock is held, `block` is a free block in no bin and `fit` placed
/// `user` in it.
unsafe fn take(state: &mut State, block: *mut Header, user: usize, payload: usize) -> *mut u8 {
    let base = block as usize;
    let head = user - HEADER_BYTES;
    // SAFETY: the caller's free block, unbinned; every header written lies
    // inside it, and each new block is at least the smallest one.
    unsafe {
        let end = base + size(block);
        let before = (*block).size & PREVIOUS_FREE;
        let (previous, previous_free) = if head > base {
            (*block).size = (head - base) | FREE | before;
            state.insert(block);
            (base, PREVIOUS_FREE)
        } else {
            (previous(block), before)
        };
        let mut used_end = user + payload;
        if end - used_end < MIN_BLOCK {
            used_end = end;
        }
        let used = head as *mut Header;
        used.write(Header {
            previous,
            size: (used_end - head) | previous_free,
        });
        if used_end < end {
            let rest = used_end as *mut Header;
            rest.write(Header {
                previous: head,
                size: (end - used_end) | FREE,
            });
            state.tell_following(rest);
            state.insert(rest);
        } else {
            state.tell_following(used);
        }
        state.carved += used_end - head;
        if head >= state.floor {
            state.carved_reserve += used_end - head;
        }
    }
    state.note_carved();
    user as *mut u8
}

/// `size` bytes aligned to `alignment` from `pool`'s bins.
///
/// # Safety
/// The heap's lock is held and `state` is initialized.
unsafe fn carve(state: &mut State, pool: usize, size: usize, alignment: usize) -> *mut u8 {
    let alignment = alignment.max(GRANULE);
    let Some(payload) = aligned(size.max(1), GRANULE) else {
        return ptr::null_mut();
    };
    // A larger alignment may need a gap before the header that can hold a
    // block of its own.
    let extra = if alignment > GRANULE {
        alignment + MIN_BLOCK
    } else {
        0
    };
    let Some(need) = payload
        .checked_add(HEADER_BYTES)
        .and_then(|need| need.checked_add(extra))
    else {
        return ptr::null_mut();
    };
    state.carves += 1;
    // SAFETY: binned free blocks, with the caller's guard held.
    let fits = |block| unsafe { fit(block, payload, alignment, 0, usize::MAX) };
    // SAFETY: the caller's guard.
    let block = unsafe { state.pools[pool].find(need, |block| fits(block).is_some()) };
    let Some(user) = (!block.is_null()).then(|| fits(block)).flatten() else {
        return ptr::null_mut();
    };
    // SAFETY: the guard; `block` is binned and `fit` placed `user` in it.
    unsafe {
        state.remove(block);
        take(state, block, user, payload)
    }
}

/// [`carve`] from the main bins, then from the reserve for a large request,
/// giving the class lists back to the bins and trying both when that fails.
///
/// # Safety
/// The heap's lock is held and `state` is initialized.
unsafe fn carve_draining(state: &mut State, size: usize, alignment: usize) -> *mut u8 {
    // SAFETY: the caller's guard and initialized state.
    let mut block = unsafe { carve(state, MAIN, size, alignment) };
    if block.is_null() && size >= LARGE_BYTES {
        // SAFETY: as above.
        block = unsafe { carve(state, RESERVE, size, alignment) };
    }
    if !block.is_null() {
        return block;
    }
    for class in 0..CLASSES {
        let mut head = core::mem::replace(&mut state.small[class], ptr::null_mut());
        while !head.is_null() {
            // SAFETY: a freed block `carve` made for this class, which only
            // this list owned.
            unsafe {
                let next = (*head).next;
                release(state, head.cast());
                head = next;
            }
        }
    }
    // SAFETY: as above.
    let block = unsafe { carve(state, MAIN, size, alignment) };
    if !block.is_null() {
        return block;
    }
    // SAFETY: as above.
    unsafe { carve(state, RESERVE, size, alignment) }
}

/// Page selection inside a CPU interval: the lowest payload address that
/// fits, over every free block. Selection changes nothing until it fits.
///
/// # Safety
/// Heap lock held; nonzero page-sized payload and at least page alignment.
unsafe fn carve_within(
    state: &mut State,
    size: usize,
    alignment: usize,
    allowed: Range<usize>,
) -> *mut u8 {
    let chosen = state
        .pools
        .iter()
        .flat_map(|pool| pool.heads.iter().flatten())
        // SAFETY: the caller's guard.
        .flat_map(|&head| unsafe { bin_blocks(head) })
        .filter_map(|block| {
            // SAFETY: a binned free block, read with the guard held.
            unsafe { fit(block, size, alignment, allowed.start, allowed.end) }
                .map(|user| (user, block))
        })
        .min_by_key(|&(user, _)| user);
    let Some((user, block)) = chosen else {
        return ptr::null_mut();
    };
    // SAFETY: the guard; `block` is binned and `fit` placed `user` in it.
    unsafe {
        state.remove(block);
        take(state, block, user, size)
    }
}

/// Return the block of an allocation `carve` made to the bins, coalescing
/// it with free neighbours.
///
/// # Safety
/// The heap's lock is held, and `pointer` came from `carve` and is no longer
/// accessed.
unsafe fn release(state: &mut State, pointer: *mut u8) {
    // SAFETY: the allocation's header precedes its payload; its neighbours'
    // headers are current, and free ones are binned.
    unsafe {
        let mut block = pointer.sub(HEADER_BYTES).cast::<Header>();
        let mut bytes = size(block);
        state.carved -= bytes;
        if block as usize >= state.floor {
            state.carved_reserve -= bytes;
        }
        let following = block as usize + bytes;
        if following < state.end
            && following != state.floor
            && (*(following as *mut Header)).size & FREE != 0
        {
            let following = following as *mut Header;
            state.remove(following);
            bytes += size(following);
        }
        if (*block).size & PREVIOUS_FREE != 0 && block as usize != state.floor {
            let previous = previous(block) as *mut Header;
            state.remove(previous);
            bytes += size(previous);
            block = previous;
        }
        // A free block's predecessor is free only across the floor: it would
        // have merged.
        (*block).size = bytes | FREE | (*block).size & PREVIOUS_FREE;
        state.tell_following(block);
        state.insert(block);
    }
}

// Explicit for the same reason as `failure`.
#[cfg(test)]
#[path = "heap/tests.rs"]
mod tests;
