//! What a host shadow of the current context's translations records beside
//! its page tables, so that a TLBI drops only what it reaches
//! ([`super::tlbi::Reach`]) instead of the whole shadow.
//!
//! A shadow maps host pages of [`PAGE`] bytes from guest leaves of any
//! size, a window of each leaf per fill ([`window`]). A TLBI by address
//! removes the leaf that held the address, whose extent the guest's tables
//! no longer tell after the change it follows. Each half of the address
//! space therefore records the leaves its fills came from: the size of the
//! largest leaf of at most [`SMALL`] bytes, whose aligned extent around an
//! address holds every smaller leaf at that address too, and each larger
//! leaf by its own extent, because a few large blocks (a physical aperture)
//! must not make every TLBI by page drop megabytes. The lower halves of
//! contexts the shadow switched away from it may keep too ([`Kept`]).

use super::tlbi::{Everywhere, Reach};
use alloc::vec::Vec;

/// The host page a shadow maps.
pub const PAGE: u64 = 1 << 12;
/// Leaves up to this size are recorded by their largest size alone.
pub const SMALL: u64 = 1 << 21;
/// The large leaves one half records; past them its extents are unknown
/// and a TLBI by address drops the whole half.
const LARGE_LEAVES: usize = 512;

/// The first and last byte of the `size`-byte leaf, a power of two of at
/// least a page, holding `address`.
pub fn leaf(address: u64, size: u64) -> (u64, u64) {
    let size = size.max(PAGE).next_power_of_two();
    let first = address & !(size - 1);
    (first, first | (size - 1))
}

/// The bytes one fill at `address` maps from its `size`-byte leaf: the
/// leaf's part of the aligned `window` around the address.
pub fn window(address: u64, size: u64, window: u64) -> (u64, u64) {
    let (first, last) = leaf(address, size);
    let (start, end) = leaf(address, window);
    (first.max(start), last.min(end))
}

/// The first byte of each other `size`-byte leaf in the aligned `window`
/// around `address`, nearest first and the following one before the
/// preceding at each distance: the leaves a fill may map beside its own,
/// each after its own walk. None when the leaf fills the window.
pub fn around(address: u64, size: u64, window: u64) -> impl Iterator<Item = u64> {
    let size = size.max(PAGE).next_power_of_two();
    let (start, end) = leaf(address, window);
    let own = address & !(size - 1);
    let leaves = if size > end - start {
        0
    } else {
        (end - start + 1) / size
    };
    (1..leaves).flat_map(move |distance| {
        let after = own.checked_add(distance * size).filter(|&a| a <= end);
        let before = own.checked_sub(distance * size).filter(|&b| b >= start);
        after.into_iter().chain(before)
    })
}

/// The leaves one half of a shadow was filled from since it was last
/// emptied, and the global ones apart, each by its own extent: a TLBI by VA
/// of another ASID reaches only those of a kept half ([`Kept::invalidate`]),
/// and a process's lower half holds few (XNU's commpage).
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct Extents {
    every: Leaves,
    /// The global leaves, all in `large`: past [`LARGE_LEAVES`] of them
    /// their visits fall back to every leaf's.
    global: Leaves,
    /// A non-global translation was filled.
    private: bool,
}

/// Leaves recorded by their extents.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
struct Leaves {
    /// log2 of the largest leaf of at most [`SMALL`] bytes, 0 for none.
    small: u32,
    /// The larger leaves' first and last bytes, sorted and distinct.
    large: Vec<(u64, u64)>,
    /// More than [`LARGE_LEAVES`] large leaves were filled.
    overflow: bool,
}

impl Leaves {
    const fn new() -> Self {
        Self {
            small: 0,
            large: Vec::new(),
            overflow: false,
        }
    }

    fn clear(&mut self) {
        self.small = 0;
        self.large.clear();
        self.overflow = false;
    }

    /// Record the leaf from `first` to `last`, by its size alone if it is
    /// of at most `small` bytes.
    fn record(&mut self, first: u64, last: u64, small: u64) {
        let bytes = last - first + 1;
        if bytes <= small {
            self.small = self.small.max(bytes.trailing_zeros());
            return;
        }
        if let Err(at) = self.large.binary_search(&(first, last)) {
            if self.large.len() == LARGE_LEAVES {
                self.overflow = true;
            } else {
                self.large.insert(at, (first, last));
            }
        }
    }

    /// [`Extents::visit_ranges`] over these leaves.
    fn visit(&mut self, reach: Reach, bounds: (u64, u64), mut visit: impl FnMut(u64, u64)) -> bool {
        let (first, last) = match reach {
            Reach::Nothing => return true,
            Reach::All => return false,
            Reach::Containing(address) => (address, address),
            Reach::Overlapping { first, last } => (first, last),
        };
        let (low, high) = bounds;
        if last < low || first > high || first > last {
            return true;
        }
        if self.overflow {
            return false;
        }
        let (first, last) = (first.max(low), last.min(high));
        if self.small != 0 {
            let mask = (1u64 << self.small) - 1;
            visit((first & !mask).max(low), (last | mask).min(high));
        }
        self.large.retain(|&(start, end)| {
            let overlaps = start <= last && first <= end;
            if overlaps {
                visit(start.max(low), end.min(high));
            }
            !overlaps
        });
        true
    }
}

impl Extents {
    pub const fn new() -> Self {
        Self {
            every: Leaves::new(),
            global: Leaves::new(),
            private: false,
        }
    }

    /// The half was emptied.
    pub fn clear(&mut self) {
        self.every.clear();
        self.global.clear();
        self.private = false;
    }

    /// A fill at `address` from a leaf of `size` bytes, global or not.
    pub fn record(&mut self, address: u64, size: u64, global: bool) {
        self.private |= !global;
        let (first, last) = leaf(address, size);
        self.every.record(first, last, SMALL);
        if global {
            self.global.record(first, last, 0);
        }
    }

    /// The half holds a non-global translation, which another ASID would
    /// not use.
    pub fn private(&self) -> bool {
        self.private
    }

    /// Visit the ranges within `bounds` (this half's addresses) that hold
    /// every page this half maps from an entry `reach` removes. Forget the
    /// large leaves among them as the caller unmaps them. Return `false`
    /// without visiting a range when the whole half must go.
    pub fn visit_ranges(
        &mut self,
        reach: Reach,
        bounds: (u64, u64),
        visit: impl FnMut(u64, u64),
    ) -> bool {
        self.every.visit(reach, bounds, visit)
    }

    /// [`Self::visit_ranges`] for the pages this half maps from global
    /// entries `reach` removes: what a TLBI by VA of another ASID removes.
    /// A global leaf visited here stays among every leaf's, which a later
    /// TLBI of the half's own ASID visits again.
    pub fn visit_global_ranges(
        &mut self,
        reach: Reach,
        bounds: (u64, u64),
        visit: impl FnMut(u64, u64),
    ) -> bool {
        if self.global.overflow {
            return self.every.visit(reach, bounds, visit);
        }
        self.global.visit(reach, bounds, visit)
    }
}

/// The lower halves of the contexts a shadow switched away from that it
/// keeps besides the current one, at most [`KEPT`]: XNU switches between a
/// few processes' TTBR0s and the kernel's empty one thousands of times a
/// second during a restore, and a switch back to one finds its half again
/// instead of filling it anew.
///
/// A half is kept as the root entries that held it, detached from their
/// roots, with the leaves it was filled from. Its tables stay in the
/// shadow's frames until the half is emptied, which drops every kept half
/// ([`Kept::clear`]). A TLBI reaches the kept halves through what it
/// removes from every context ([`Everywhere`]), as the current half
/// through [`Reach`].
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct Kept {
    halves: Vec<KeptHalf>,
    /// Switches so far, which order the halves for replacement.
    switches: u64,
    /// Emptied entry lists whose storage the next switches reuse
    /// ([`Kept::buffer`]), at most [`SPARE_BUFFERS`]: a switch allocates
    /// none once the halves are full.
    spare: Vec<Vec<(u64, usize, u64)>>,
}

/// The emptied entry lists [`Kept`] keeps for reuse at most.
const SPARE_BUFFERS: usize = 2;

/// The contexts whose lower halves [`Kept`] holds at most. A replaced half
/// frees no frames (they return only when the lower half is emptied), and
/// refilling a context that comes back allocates its tables again, so a
/// replacement costs frames as well as fills: a desktop session rotates
/// among dozens of processes, which eight halves missed half the time.
pub const KEPT: usize = 32;

/// One context's kept lower half.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct KeptHalf {
    /// What selects the context: TTBR0 and the TCR fields of the lower half.
    pub key: (u64, u64),
    /// Each nonzero root entry of the half: root, entry index and value.
    pub entries: Vec<(u64, usize, u64)>,
    pub extents: Extents,
    used: u64,
}

impl Kept {
    pub const fn new() -> Self {
        Self {
            halves: Vec::new(),
            switches: 0,
            spare: Vec::new(),
        }
    }

    /// An empty list for the entries a switch detaches, with the storage of
    /// one an earlier switch gave back when there is one.
    pub fn buffer(&mut self) -> Vec<(u64, usize, u64)> {
        self.spare.pop().unwrap_or_default()
    }

    /// Give back the storage of an entry list a switch is done with.
    pub fn recycle(&mut self, mut entries: Vec<(u64, usize, u64)>) {
        if self.spare.len() < SPARE_BUFFERS && entries.capacity() != 0 {
            entries.clear();
            self.spare.push(entries);
        }
    }

    /// Keep the half of context `key` that a switch detached, replacing
    /// the least recently kept one when full, and take the half of `to`
    /// if one is kept.
    pub fn switch(
        &mut self,
        key: (u64, u64),
        entries: Vec<(u64, usize, u64)>,
        extents: Extents,
        to: (u64, u64),
    ) -> Option<KeptHalf> {
        self.switches += 1;
        self.halves.retain(|half| half.key != key);
        // Take the incoming half before making room for the outgoing one:
        // the half we need can itself be the least recently kept.
        let found = self
            .halves
            .iter()
            .position(|half| half.key == to)
            .map(|index| self.halves.swap_remove(index));
        if entries.is_empty() {
            self.recycle(entries);
            return found;
        }
        let outgoing = KeptHalf {
            key,
            entries,
            extents,
            used: self.switches,
        };
        if key == to {
            return Some(outgoing);
        }
        if self.halves.len() == KEPT
            && let Some(oldest) = (0..self.halves.len()).min_by_key(|&i| self.halves[i].used)
        {
            let evicted = self.halves.swap_remove(oldest);
            self.recycle(evicted.entries);
        }
        self.halves.push(outgoing);
        found
    }

    /// Drop every kept half, returning how many there were.
    pub fn clear(&mut self) -> usize {
        let halves = self.halves.len();
        self.halves.clear();
        halves
    }

    pub fn is_empty(&self) -> bool {
        self.halves.is_empty()
    }

    /// Apply a TLBI that removes `everywhere` to the kept halves within
    /// `bounds`, whose contexts share the TCR `tcr` outside the lower
    /// half's own fields: `unmap(value, first, last)` unmaps a range under
    /// the root entry `value`, whose top-level entries each map
    /// `1 << entry_bits` bytes. A TLBI by VA of an ASID reaches the global
    /// pages of a half that may hold no entry of that ASID
    /// ([`same_asid`]), and every page at its addresses of one that may.
    /// A half whose extents are unknown for the TLBI's addresses, or that a
    /// TLBI of non-global or all entries reaches, is dropped. Returns the
    /// halves dropped.
    pub fn invalidate(
        &mut self,
        everywhere: Everywhere,
        tcr: u64,
        bounds: (u64, u64),
        entry_bits: u32,
        mut unmap: impl FnMut(u64, u64, u64),
    ) -> usize {
        let (reach, asid) = match everywhere {
            Everywhere::Addresses(reach) => (reach, None),
            Everywhere::AsidAddresses(reach, asid) => (reach, Some(asid)),
            Everywhere::NonGlobal(_) | Everywhere::All => (Reach::All, None),
        };
        let before = self.halves.len();
        self.halves.retain_mut(|half| {
            let entries = &half.entries;
            let visit = |first: u64, last: u64| {
                for &(_, entry, value) in entries {
                    let start = (entry as u64) << entry_bits;
                    let end = start | ((1 << entry_bits) - 1);
                    if first <= end && start <= last {
                        unmap(value, first.max(start), last.min(end));
                    }
                }
            };
            let other = asid
                .is_some_and(|asid| half.extents.private() && !same_asid(half.key.0, tcr, asid));
            if other {
                half.extents.visit_global_ranges(reach, bounds, visit)
            } else {
                half.extents.visit_ranges(reach, bounds, visit)
            }
        });
        before - self.halves.len()
    }

    /// Apply a TLBI ASIDE1 of `asid` to the kept halves, whose contexts
    /// share the TCR `tcr` (a change of TCR outside the lower half's own
    /// fields empties the shadow): drop only a half that may hold a
    /// non-global entry of `asid` ([`same_asid`]), and only if it holds a
    /// non-global entry at all ([`Extents::private`]). ASIDE1 removes the
    /// non-global entries of its ASID alone (Arm `TLBIMatch`): another
    /// ASID's half, and a half of global entries alone, stay. Returns the
    /// halves dropped.
    pub fn drop_asid(&mut self, asid: u16, tcr: u64) -> usize {
        let before = self.halves.len();
        self.halves
            .retain(|half| !(half.extents.private() && same_asid(half.key.0, tcr, asid)));
        before - self.halves.len()
    }
}

/// A TCR for [`Kept::drop_asid`] under which every ASID may match: TCR.A1
/// set, so TTBR0 does not name the ASID.
pub const UNKNOWN_ASIDS: u64 = 1 << 22;

/// Whether the non-global entries a walk through TTBR0 value `ttbr0` made
/// under TCR `tcr` may be of ASID `asid`: with TCR.A1 = 0 their ASID is
/// TTBR0[63:48], compared on 16 bits with TCR.AS = 1 and on the low 8 bits
/// otherwise (the TLBI's high 8 bits are then ignored); with TCR.A1 = 1 it
/// is TTBR1's, which `ttbr0` does not tell, so any ASID may match.
pub fn same_asid(ttbr0: u64, tcr: u64, asid: u16) -> bool {
    const TCR_A1: u64 = 1 << 22;
    const TCR_AS: u64 = 1 << 36;
    if tcr & TCR_A1 != 0 {
        return true;
    }
    let mask = if tcr & TCR_AS != 0 { 0xffff } else { 0xff };
    ((ttbr0 >> 48) as u16 ^ asid) & mask == 0
}

#[cfg(test)]
mod tests {
    use super::*;

    const LOWER: (u64, u64) = (0, (1 << 48) - 1);
    const UPPER: (u64, u64) = (0xffff_0000_0000_0000, u64::MAX);

    impl Extents {
        fn ranges(&mut self, reach: Reach, bounds: (u64, u64)) -> Option<Vec<(u64, u64)>> {
            let mut ranges = Vec::new();
            self.visit_ranges(reach, bounds, |first, last| ranges.push((first, last)))
                .then_some(ranges)
        }
    }

    #[test]
    fn leaves_and_windows_are_aligned_extents() {
        assert_eq!(leaf(0x1_2345, 1 << 14), (0x1_0000, 0x1_3fff));
        assert_eq!(leaf(0x1_2345, 0), (0x1_2000, 0x1_2fff));
        assert_eq!(leaf(u64::MAX, 1 << 14), (0xffff_ffff_ffff_c000, u64::MAX));
        // A 16 KiB page fills whole within a 64 KiB window; a 32 MiB block
        // fills the window alone.
        assert_eq!(window(0x1_2345, 1 << 14, 1 << 16), (0x1_0000, 0x1_3fff));
        assert_eq!(window(0x1_2345, 1 << 25, 1 << 16), (0x1_0000, 0x1_ffff));
        assert_eq!(window(0x1_2345, 1 << 12, 1 << 16), (0x1_2000, 0x1_2fff));
    }

    #[test]
    fn small_leaves_widen_the_address_and_large_ones_are_named() {
        let mut extents = Extents::new();
        assert_eq!(
            extents.ranges(Reach::Containing(0x8000), LOWER),
            Some(Vec::new())
        );
        extents.record(0x4_1000, 1 << 14, true);
        extents.record(0x10_0000, 1 << 12, true);
        assert!(!extents.private());
        assert_eq!(
            extents.ranges(Reach::Containing(0x8_5000), LOWER),
            Some(alloc::vec![(0x8_4000, 0x8_7fff)])
        );
        let block = 0xffff_fe00_0200_0000;
        extents.record(block + 0x12_3456, 1 << 25, true);
        extents.record(block + 0x45_6789, 1 << 25, false);
        assert!(extents.private());
        assert_eq!(
            extents.every.large,
            alloc::vec![(block, block + (1 << 25) - 1)]
        );
        // A page of the block drops its 16 KiB neighbourhood and the block,
        // which is then forgotten.
        assert_eq!(
            extents.ranges(Reach::Containing(block + 0x8000), UPPER),
            Some(alloc::vec![
                (block + 0x8000, block + 0xbfff),
                (block, block + (1 << 25) - 1)
            ])
        );
        assert!(extents.every.large.is_empty());
        // Ranges clamp to the half; other halves and inverted ranges reach
        // nothing; All drops the half.
        assert_eq!(
            extents.ranges(
                Reach::Overlapping {
                    first: 0x1_0000,
                    last: u64::MAX
                },
                LOWER
            ),
            Some(alloc::vec![(0x1_0000, LOWER.1)])
        );
        assert_eq!(
            extents.ranges(Reach::Containing(block), LOWER),
            Some(Vec::new())
        );
        assert_eq!(extents.ranges(Reach::All, LOWER), None);
        extents.clear();
        assert_eq!(extents, Extents::new());
    }

    #[test]
    fn too_many_large_leaves_make_any_address_drop_the_half() {
        let mut extents = Extents::new();
        for block in 0..=LARGE_LEAVES as u64 {
            extents.record(block << 25, 1 << 25, true);
        }
        assert_eq!(extents.ranges(Reach::Containing(0x4000), LOWER), None);
        assert_eq!(
            extents.ranges(Reach::Containing(0xffff_ffff_0000_0000), LOWER),
            Some(Vec::new())
        );
    }

    /// Over random fills of random leaves, every page filled from a leaf a
    /// reach overlaps lies in the returned ranges.
    #[test]
    fn the_ranges_hold_every_page_filled_from_a_reached_leaf() {
        let mut seed = 0x2545_f491_4f6c_dd1d_u64;
        let mut next = move || {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            seed
        };
        let sizes = [12, 14, 16, 21, 25, 30];
        for _ in 0..2000 {
            let mut extents = Extents::new();
            let mut fills = Vec::new();
            for _ in 0..1 + next() % 40 {
                let address = next() & 0x3_ffff_ffff;
                let size = 1u64 << sizes[next() as usize % sizes.len()];
                extents.record(address, size, true);
                fills.push((leaf(address, size), window(address, size, 1 << 16)));
            }
            let first = next() & 0x3_ffff_ffff;
            let reach = if next() & 1 == 0 {
                Reach::Containing(first)
            } else {
                Reach::Overlapping {
                    first,
                    last: first + (next() & 0xf_ffff),
                }
            };
            let (start, end) = match reach {
                Reach::Containing(address) => (address, address),
                Reach::Overlapping { first, last } => (first, last),
                _ => unreachable!(),
            };
            let ranges = extents.ranges(reach, LOWER).unwrap();
            for ((leaf_first, leaf_last), (mapped_first, mapped_last)) in fills {
                if leaf_first > end || start > leaf_last {
                    continue;
                }
                let held = |page: u64| ranges.iter().any(|&(a, b)| a <= page && page <= b);
                assert!(
                    held(mapped_first) && held(mapped_last),
                    "{reach:?} {ranges:?}"
                );
            }
        }
    }

    #[test]
    fn kept_halves_return_on_a_switch_back_and_follow_tlbis() {
        const ROOT: u64 = 0x1000;
        let a = (0x0001_0000_8000_0000, 0x10);
        let b = (0x0002_0000_9000_0000, 0x10);
        let mut kept = Kept::new();
        let mut extents = Extents::new();
        extents.record(0x1_0000_4000, 1 << 14, false);
        // A first switch keeps A's half and finds no B.
        let half = kept.switch(a, alloc::vec![(ROOT, 0, 0xa003)], extents.clone(), b);
        assert_eq!(half, None);
        // An empty half is not kept; the switch back finds A's.
        let half = kept.switch(b, Vec::new(), Extents::new(), a).unwrap();
        assert_eq!(half.key, a);
        assert_eq!(half.entries, [(ROOT, 0, 0xa003)]);
        assert_eq!(half.extents, extents);
        assert!(kept.is_empty());
        // A TLBI by address unmaps its page's neighbourhood under each
        // kept entry it overlaps, and keeps the half.
        kept.switch(a, half.entries, half.extents, b);
        let mut unmapped = Vec::new();
        let reach = Reach::Containing(0x1_0000_5000);
        let dropped = kept.invalidate(
            Everywhere::Addresses(reach),
            0,
            LOWER,
            39,
            |value, first, last| unmapped.push((value, first, last)),
        );
        assert_eq!(dropped, 0);
        assert_eq!(unmapped, [(0xa003, 0x1_0000_4000, 0x1_0000_7fff)]);
        // An upper-half address reaches nothing of it.
        unmapped.clear();
        let upper = Everywhere::Addresses(Reach::Containing(0xffff_fe00_0000_0000));
        assert_eq!(
            kept.invalidate(upper, 0, LOWER, 39, |v, f, l| unmapped.push((v, f, l))),
            0
        );
        assert!(unmapped.is_empty());
        // A TLBI of an ASID's or of every entry drops the kept halves.
        assert_eq!(
            kept.invalidate(Everywhere::NonGlobal(1), 0, LOWER, 39, |_, _, _| {}),
            1
        );
        assert!(kept.is_empty());
    }

    /// TLBI ASIDE1 drops only the kept halves that hold a non-global entry
    /// of its ASID; with TCR.A1 = 1 the ASID is unknown and any private
    /// half goes.
    #[test]
    fn an_asid_tlbi_drops_only_its_asids_private_kept_halves() {
        const TCR_A1: u64 = 1 << 22;
        const TCR_AS: u64 = 1 << 36;
        let key = |asid: u64| (asid << 48 | 0x8000_0000, 0x10);
        let mut private = Extents::new();
        private.record(0x1_0000_4000, 1 << 14, false);
        let mut global = Extents::new();
        global.record(0x1_8000_0000, 1 << 14, true);
        let fill = |kept: &mut Kept| {
            for (n, extents) in [(5, &private), (6, &private), (7, &global)] {
                kept.switch(key(n), alloc::vec![(0x1000, 0, n)], extents.clone(), key(0));
            }
        };
        let held = |kept: &Kept| {
            let mut keys: Vec<u64> = kept.halves.iter().map(|half| half.key.0 >> 48).collect();
            keys.sort_unstable();
            keys
        };
        let mut kept = Kept::new();
        fill(&mut kept);
        assert_eq!(kept.drop_asid(5, 0), 1);
        assert_eq!(held(&kept), [6, 7]);
        // A global-only half stays for its own ASID too.
        assert_eq!(kept.drop_asid(7, 0), 0);
        // 8-bit ASIDs ignore the TLBI's high byte; 16-bit ones do not.
        assert_eq!(kept.drop_asid(0x106, TCR_AS), 0);
        assert_eq!(kept.drop_asid(0x106, 0), 1);
        assert_eq!(held(&kept), [7]);
        // TTBR1 names the ASID: every private half may hold it.
        let mut kept = Kept::new();
        fill(&mut kept);
        assert_eq!(kept.drop_asid(9, TCR_A1), 2);
        assert_eq!(held(&kept), [7]);
    }

    /// A TLBI by VA of an ASID unmaps only the global pages of a kept half
    /// of another ASID, and every page at its address of a half of its own
    /// ASID, of a half without non-global entries and, with TCR.A1 = 1 or
    /// 8-bit ASIDs that agree, of a half that may be of it.
    #[test]
    fn a_tlbi_by_va_of_an_asid_unmaps_only_global_pages_of_other_asids_halves() {
        const TCR_A1: u64 = 1 << 22;
        const TCR_AS: u64 = 1 << 36;
        // A process's page at 0x1_0000_4000 and a global 16 KiB page (a
        // commpage) at 0xf_ffff_c000; a 32 MiB global block at 2 GiB.
        let key = |asid: u64| (asid << 48 | 0x8000_0000, 0x10);
        let mut process = Extents::new();
        process.record(0x1_0000_4000, 1 << 14, false);
        process.record(0xf_ffff_c000, 1 << 14, true);
        process.record(0x8000_0000, 1 << 25, true);
        let mut shared = Extents::new();
        shared.record(0x2_0000_0000, 1 << 14, true);
        let fill = || {
            let mut kept = Kept::new();
            for (asid, extents) in [(5, &process), (6, &process), (7, &shared)] {
                kept.switch(
                    key(asid),
                    alloc::vec![(0x1000, 0, asid)],
                    extents.clone(),
                    key(0),
                );
            }
            kept
        };
        let unmapped = |kept: &mut Kept, everywhere, tcr| {
            let mut unmapped = Vec::new();
            let dropped = kept.invalidate(everywhere, tcr, LOWER, 39, |value, first, last| {
                unmapped.push((value, first, last));
            });
            assert_eq!(dropped, 0);
            unmapped.sort_unstable();
            unmapped
        };
        let page = Reach::Containing(0x1_0000_5000);
        // ASID 5's page: its half's neighbourhood, and the global-only
        // half's; ASID 6's half holds no global page there.
        let mut kept = fill();
        assert_eq!(
            unmapped(&mut kept, Everywhere::AsidAddresses(page, 5), 0),
            [
                (5, 0x1_0000_4000, 0x1_0000_7fff),
                (7, 0x1_0000_4000, 0x1_0000_7fff)
            ]
        );
        // The commpage's address reaches every half.
        let commpage = Reach::Containing(0xf_ffff_d000);
        assert_eq!(
            unmapped(&mut kept, Everywhere::AsidAddresses(commpage, 5), 0)
                .iter()
                .map(|&(value, ..)| value)
                .collect::<Vec<_>>(),
            [5, 6, 7]
        );
        // A page of the global block reaches the block alone in another
        // ASID's half, which forgets it from its global leaves, and the
        // page's neighbourhood besides in the others.
        let block = Reach::Containing(0x8000_4000);
        assert_eq!(
            unmapped(&mut kept, Everywhere::AsidAddresses(block, 5), 0),
            [
                (5, 0x8000_0000, 0x8000_0000 + (1 << 25) - 1),
                (5, 0x8000_4000, 0x8000_7fff),
                (6, 0x8000_0000, 0x8000_0000 + (1 << 25) - 1),
                (7, 0x8000_4000, 0x8000_7fff),
            ]
        );
        assert!(
            unmapped(&mut kept, Everywhere::AsidAddresses(block, 5), 0)
                .iter()
                .all(|&(value, ..)| value != 6)
        );
        // Without an ASID, every half's neighbourhood goes.
        assert_eq!(
            unmapped(&mut fill(), Everywhere::Addresses(page), 0).len(),
            3
        );
        // 8-bit ASIDs ignore the TLBI's high byte; 16-bit ones do not.
        assert_eq!(
            unmapped(&mut fill(), Everywhere::AsidAddresses(page, 0x106), 0).len(),
            2
        );
        assert_eq!(
            unmapped(&mut fill(), Everywhere::AsidAddresses(page, 0x106), TCR_AS).len(),
            1
        );
        // TTBR1 names the ASID: every half may hold it.
        assert_eq!(
            unmapped(&mut fill(), Everywhere::AsidAddresses(page, 9), TCR_A1).len(),
            3
        );
        // Past the global leaves it records, a half of another ASID visits
        // every leaf's ranges again.
        let mut many = process.clone();
        for n in 0..=LARGE_LEAVES as u64 {
            many.record(0x10_0000_0000 + (n << 14), 1 << 14, true);
        }
        let mut kept = Kept::new();
        kept.switch(key(6), alloc::vec![(0x1000, 0, 6)], many, key(0));
        assert_eq!(
            unmapped(&mut kept, Everywhere::AsidAddresses(page, 5), 0),
            [(6, 0x1_0000_4000, 0x1_0000_7fff)]
        );
    }

    #[test]
    fn kept_halves_replace_the_least_recently_kept() {
        let mut kept = Kept::new();
        let key = |n: u64| (n << 12, 0);
        for n in 0..=KEPT as u64 {
            kept.switch(
                key(n),
                alloc::vec![(0x1000, 0, n + 1)],
                Extents::new(),
                key(n + 1),
            );
        }
        // The first is gone, the rest remain.
        assert_eq!(
            kept.switch(key(99), Vec::new(), Extents::new(), key(0)),
            None
        );
        for n in 1..=KEPT as u64 {
            let half = kept
                .switch(key(99), Vec::new(), Extents::new(), key(n))
                .unwrap();
            assert_eq!(half.entries, [(0x1000, 0, n + 1)]);
        }
        kept.switch(key(1), alloc::vec![(0x1000, 0, 2)], Extents::new(), key(2));
        assert_eq!(kept.clear(), 1);
        assert!(kept.is_empty());
    }

    #[test]
    fn a_fill_maps_the_other_leaves_of_its_window_nearest_first() {
        let leaves = |address, size| around(address, size, 1 << 16).collect::<Vec<_>>();
        // A 16 KiB leaf in a 64 KiB window: the three others.
        assert_eq!(leaves(0x1_4123, 1 << 14), [0x1_8000, 0x1_0000, 0x1_c000]);
        assert_eq!(leaves(0x1_0000, 1 << 14), [0x1_4000, 0x1_8000, 0x1_c000]);
        assert_eq!(leaves(0x1_fff8, 1 << 14), [0x1_8000, 0x1_4000, 0x1_0000]);
        // 4 KiB leaves: fifteen, all within the window and distinct.
        let small = leaves(0x2_3456, 1 << 12);
        assert_eq!(small.len(), 15);
        assert!(small.iter().all(|&a| (0x2_0000..0x3_0000).contains(&a)));
        assert!(!small.contains(&0x2_3000));
        // A leaf as large as the window or larger has no neighbours in it.
        assert!(leaves(0x5_0000, 1 << 16).is_empty());
        assert!(leaves(0x20_0000, 1 << 21).is_empty());
        // The top of the address space does not wrap.
        assert_eq!(
            leaves(u64::MAX, 1 << 14),
            [
                0xffff_ffff_ffff_8000,
                0xffff_ffff_ffff_4000,
                0xffff_ffff_ffff_0000
            ]
        );
    }

    #[test]
    fn a_rotation_of_many_contexts_finds_every_half() {
        // Two dozen processes taking turns, as a desktop session's do,
        // each find their half again after the first round.
        let mut kept = Kept::new();
        let key = |n: u64| (n << 12, 0);
        const CONTEXTS: u64 = 24;
        const { assert!(CONTEXTS as usize <= KEPT) };
        let mut found = 0;
        for turn in 0..CONTEXTS * 4 {
            let (from, to) = (turn % CONTEXTS, (turn + 1) % CONTEXTS);
            let half = kept.switch(
                key(from),
                alloc::vec![(0x1000, 0, from + 1)],
                Extents::new(),
                key(to),
            );
            if let Some(half) = half {
                assert_eq!(half.entries, [(0x1000, 0, to + 1)]);
                found += 1;
            }
        }
        assert_eq!(found, CONTEXTS * 4 - (CONTEXTS - 1));
    }

    #[test]
    fn switching_to_the_oldest_kept_half_reuses_it_when_full() {
        let mut kept = Kept::new();
        let key = |n: u64| (n << 12, 0);
        for from in 0..KEPT as u64 {
            assert!(
                kept.switch(
                    key(from),
                    alloc::vec![(0x1000, 0, from + 1)],
                    Extents::new(),
                    key(from + 1),
                )
                .is_none()
            );
        }
        for from in (KEPT as u64..).take(50).map(|n| n % (KEPT as u64 + 1)) {
            let to = (from + 1) % (KEPT as u64 + 1);
            let half = kept
                .switch(
                    key(from),
                    alloc::vec![(0x1000, 0, from + 1)],
                    Extents::new(),
                    key(to),
                )
                .unwrap();
            assert_eq!(half.entries, [(0x1000, 0, to + 1)]);
        }
    }

    /// Entry lists given back, empty ones a switch did not keep and the
    /// evicted ones, are handed out again with their storage, emptied, so a
    /// steady stream of switches stops allocating.
    #[test]
    fn switches_reuse_the_storage_of_entry_lists_given_back() {
        let mut kept = Kept::new();
        let key = |n: u64| (n << 12, 0);
        let mut empty = kept.buffer();
        assert_eq!(empty.capacity(), 0, "nothing to reuse yet");
        empty.reserve(4);
        let storage = empty.as_ptr();
        assert_eq!(kept.switch(key(0), empty, Extents::new(), key(1)), None);
        let reused = kept.buffer();
        assert!(reused.is_empty());
        assert_eq!(reused.as_ptr(), storage, "the empty list's storage");
        kept.recycle(reused);

        // Fill the halves, then one more evicts the oldest: its list
        // comes back for the next switch.
        for n in 0..KEPT as u64 {
            let mut entries = kept.buffer();
            entries.push((0x1000, 0, n + 1));
            kept.switch(key(n), entries, Extents::new(), key(99));
        }
        let mut entries = kept.buffer();
        entries.push((0x1000, 0, 100));
        kept.switch(key(KEPT as u64), entries, Extents::new(), key(99));
        let evicted = kept.buffer();
        assert!(evicted.is_empty() && evicted.capacity() != 0);
        // At most `SPARE_BUFFERS` are kept.
        for _ in 0..4 {
            kept.recycle(alloc::vec![(0, 0, 0)]);
        }
        assert_eq!(kept.spare.len(), SPARE_BUFFERS);
    }
}
