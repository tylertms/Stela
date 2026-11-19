//! Guest V registers kept in host XMM registers within one block.
//!
//! A system-mode block ([`super::block`]) gives the AdvSIMD operand layer
//! ([`super::simd::read_vector`], [`super::simd::commit`],
//! [`super::simd::vector_source`]) a pool of host XMM registers that no
//! recipe names: XMM8 to XMM15. Recipes keep their fixed scratch registers
//! XMM0 to XMM7. A guest vector gets a home in the pool when a recipe
//! commits a result to it or takes it as a source operand; later reads take
//! it from there, and commits write there and mark it dirty, instead of
//! storing to `GuestState::v` and loading it back at once.
//!
//! The state block is made current ([`VectorCache::flush`]) wherever
//! anything else may read it: before every instruction that can leave the
//! block (a conditional branch's side exit, the terminal and its direct
//! exits, a self-loop's back edge) and before every instruction the cache
//! does not know (which also forgets every home, since it may write
//! `GuestState::v` itself or use any XMM register). Guest accesses and
//! checks keep the dirty homes: each fault site records which V register
//! only which home holds ([`DirtyHomes`]), and the fault's settlement
//! takes those from the XMM8-15 the host saved at the fault
//! ([`crate::block::RecoveryBinding::vector`]). So a fault's settlement,
//! an exit, a helper and an interrupt taken at a block boundary all find
//! the exact architectural V state; an exit or a helper call with a dirty
//! home is refused ([`super::asm::Asm::refuse_dirty_homes`]).
//!
//! The pool is the legacy-SSE register file's upper half on every x86-64
//! host: SSE2 recipes take a home directly as a source operand, which
//! XMM16 to XMM31 (AVX-512VL, EVEX only) cannot be.
//!
//! Those sixteen are a second bank on AVX-512VL hosts, for values recipes
//! derive and use again later in the block (a prepared by-element operand,
//! lane constants): [`Code::derived`]. EVEX forms read them; nothing
//! stores them; a value goes when its source vector is written, when the
//! bank needs its register, and wherever the cache forgets its homes, so
//! no fault site, exit or helper depends on one.
use super::asm::{Code, Gpr, Mem};

/// The first register of the derived bank.
pub(crate) const FIRST_DERIVED: u8 = 16;
/// The derived bank's registers.
const DERIVED_HOSTS: usize = 16;

/// A value recipes derive from guest vectors or from nothing, which the
/// derived bank may keep for later instructions of the block.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct Derived {
    /// What the value is, in the deriving recipe's own terms.
    pub(crate) key: u32,
    /// The guest vector it is computed from, if any.
    pub(crate) source: Option<u8>,
}

/// The first host register of the pool.
pub(crate) const FIRST_HOST: u8 = 8;
/// The pool's registers.
pub(crate) const HOSTS: usize = 8;
const NONE: u8 = u8::MAX;

/// The guest vectors whose only current value is a home, with that home:
/// what a fault site records so that recovery reads them from the saved
/// XMM8-15 ([`crate::block::RecoveryBinding::vector`]).
pub(crate) type DirtyHomes = [Option<(u8, u8)>; HOSTS];

/// Where each guest vector lives in this block, and which homes hold a
/// value the state block lacks.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct VectorCache {
    /// Whether the code being emitted may keep vectors in the pool.
    active: bool,
    /// The pool register of each guest vector, or `NONE`.
    home: [u8; 32],
    /// The guest vector each pool register holds, or `NONE`.
    owner: [u8; HOSTS],
    /// Guest vectors whose home is newer than `GuestState::v`.
    dirty: u32,
    /// The last use of each pool register, for the least recently used spill.
    used: [u32; HOSTS],
    clock: u32,
    /// Whether commits go to the state: the next instruction stores or
    /// forgets every home anyway, so a home would cost a second move.
    write_through: bool,
    /// The home a recipe is writing as its destination
    /// ([`Code::begin_vector_write`]), which the encoder lets it write, or
    /// `NONE`.
    writing: u8,
    /// The derived value each bank register holds.
    derived: [Option<Derived>; DERIVED_HOSTS],
    /// The last use of each bank register.
    derived_used: [u32; DERIVED_HOSTS],
    /// The bank register a recipe is filling, or `NONE`.
    deriving: u8,
    /// Whether that home's old value is gone: a destination written whole,
    /// whose vector no recipe may read until the write ends.
    fresh: bool,
    /// The pool registers that have an owner and the bank registers that
    /// hold a derived value, by register number: what [`Self::occupied`]
    /// reports for every encoder the block takes.
    held: u32,
}

impl Default for VectorCache {
    fn default() -> Self {
        Self {
            active: false,
            home: [NONE; 32],
            owner: [NONE; HOSTS],
            dirty: 0,
            used: [0; HOSTS],
            clock: 0,
            write_through: false,
            writing: NONE,
            derived: [None; DERIVED_HOSTS],
            derived_used: [0; DERIVED_HOSTS],
            deriving: NONE,
            fresh: false,
            held: 0,
        }
    }
}

/// `GuestState::v[vector]`, `[rdi+disp32]`.
fn slot(vector: u8) -> Mem {
    let offset = core::mem::offset_of!(crate::GuestState, v) + usize::from(vector) * 16;
    Mem::base_disp32(Gpr::Rdi, offset as i32)
}

impl VectorCache {
    pub(crate) const fn active(&self) -> bool {
        self.active
    }
    /// Whether any home holds a value the state block lacks.
    pub(crate) const fn has_dirty(&self) -> bool {
        self.dirty != 0
    }
    /// The dirty homes, for a fault site here.
    pub(crate) fn dirty_homes(&self) -> DirtyHomes {
        let mut homes = [None; HOSTS];
        if self.dirty == 0 {
            return homes;
        }
        for (index, owner) in self.owner.iter().enumerate() {
            if *owner != NONE && self.is_dirty(*owner) {
                homes[index] = Some((*owner, FIRST_HOST + index as u8));
            }
        }
        homes
    }
    /// The pool registers that hold a guest vector: no recipe may write one,
    /// except the destination home it is writing.
    pub(crate) fn occupied(&self) -> u32 {
        debug_assert_eq!(self.held, self.scan_held(), "the held registers");
        [self.writing, self.deriving]
            .into_iter()
            .filter(|host| *host != NONE)
            .fold(self.held, |mask, host| mask & !(1 << host))
    }
    /// The registers `held` records, from the pool's owners and the bank's
    /// values.
    fn scan_held(&self) -> u32 {
        let owned = self
            .owner
            .iter()
            .enumerate()
            .filter(|(_, owner)| **owner != NONE)
            .fold(0, |mask, (index, _)| {
                mask | 1 << (usize::from(FIRST_HOST) + index)
            });
        self.derived
            .iter()
            .enumerate()
            .filter(|(_, derived)| derived.is_some())
            .fold(owned, |mask, (index, _)| {
                mask | 1 << (usize::from(FIRST_DERIVED) + index)
            })
    }
    fn touch(&mut self, index: usize) {
        self.clock += 1;
        self.used[index] = self.clock;
    }
    /// The home of `vector`, if it has one.
    pub(crate) fn home(&mut self, vector: u8) -> Option<u8> {
        let host = self.home[usize::from(vector)];
        assert!(
            !(self.fresh && host != NONE && host == self.writing),
            "a destination home is read before its write ends"
        );
        (self.active && host != NONE).then(|| {
            self.touch(usize::from(host - FIRST_HOST));
            host
        })
    }
    /// The home of `vector`, without counting a use.
    pub(crate) fn peek(&self, vector: u8) -> Option<u8> {
        let host = self.home[usize::from(vector)];
        (self.active && host != NONE).then_some(host)
    }
    pub(crate) fn is_dirty(&self, vector: u8) -> bool {
        self.dirty & 1 << vector != 0
    }
    /// Whether a commit goes to the state rather than to a home.
    pub(crate) const fn writes_through(&self) -> bool {
        self.active && self.write_through
    }
}

impl Code {
    /// Keep guest vectors in the pool from here on, or not. Turning the
    /// cache off requires that no home is dirty, and forgets them all.
    pub(crate) fn set_vector_cache(&mut self, active: bool) {
        if !active {
            self.forget_vectors();
        }
        self.vectors.active = active;
    }
    pub(crate) const fn vector_cache(&self) -> &VectorCache {
        &self.vectors
    }
    /// The home of `vector`, if the cache is on and gave it one.
    pub(crate) fn vector_home(&mut self, vector: u8) -> Option<u8> {
        self.vectors.home(vector)
    }
    /// A home for `vector`, which the caller then fills: the free pool
    /// register, or the least recently used one, whose vector is stored
    /// first if dirty.
    pub(crate) fn allocate_vector(&mut self, vector: u8) -> u8 {
        debug_assert!(self.vectors.active);
        if let Some(host) = self.vectors.home(vector) {
            return host;
        }
        let index = match self.vectors.owner.iter().position(|owner| *owner == NONE) {
            Some(free) => free,
            None => {
                let (index, _) = self
                    .vectors
                    .used
                    .iter()
                    .enumerate()
                    .min_by_key(|(_, used)| **used)
                    .expect("a non-empty pool");
                self.spill_vector(self.vectors.owner[index]);
                index
            }
        };
        let host = FIRST_HOST + index as u8;
        self.vectors.owner[index] = vector;
        self.vectors.held |= 1 << host;
        self.vectors.home[usize::from(vector)] = host;
        self.vectors.touch(index);
        host
    }
    /// Let the recipe being emitted write `vector`'s home, which it then
    /// holds whole (`fresh`: the old value is not read through the cache
    /// again before [`Self::end_vector_write`]). Only one at a time.
    pub(crate) fn begin_vector_write(&mut self, vector: u8, fresh: bool) -> u8 {
        assert_eq!(self.vectors.writing, NONE, "one destination home at a time");
        let host = self.allocate_vector(vector);
        self.vectors.writing = host;
        self.vectors.fresh = fresh;
        host
    }
    /// The recipe wrote `vector`'s home: it is newer than the state.
    pub(crate) fn end_vector_write(&mut self, vector: u8) {
        assert_eq!(
            self.vectors.writing,
            self.vectors.home[usize::from(vector)],
            "the destination home being written"
        );
        self.vectors.writing = NONE;
        self.vectors.fresh = false;
        self.mark_vector_dirty(vector);
    }
    /// The derived bank register that holds `derived`, if one does.
    pub(crate) fn derived(&mut self, derived: Derived) -> Option<u8> {
        if !self.vectors.active {
            return None;
        }
        let index = self
            .vectors
            .derived
            .iter()
            .position(|kept| *kept == Some(derived))?;
        self.vectors.clock += 1;
        self.vectors.derived_used[index] = self.vectors.clock;
        Some(FIRST_DERIVED + index as u8)
    }
    /// Keep `derived` in a derived bank register (free, else the least
    /// recently used), which `emit` fills with EVEX forms; returns it.
    pub(crate) fn keep_derived(
        &mut self,
        derived: Derived,
        emit: impl FnOnce(&mut Self, u8),
    ) -> u8 {
        assert!(self.vectors.active, "derived values live with the homes");
        let index = match self.vectors.derived.iter().position(Option::is_none) {
            Some(free) => free,
            None => {
                let (index, _) = self
                    .vectors
                    .derived_used
                    .iter()
                    .enumerate()
                    .min_by_key(|(_, used)| **used)
                    .expect("a non-empty bank");
                index
            }
        };
        let host = FIRST_DERIVED + index as u8;
        self.vectors.derived[index] = Some(derived);
        self.vectors.held |= 1 << host;
        self.vectors.clock += 1;
        self.vectors.derived_used[index] = self.vectors.clock;
        self.vectors.deriving = host;
        emit(self, host);
        self.vectors.deriving = NONE;
        host
    }
    /// [`Self::derived`], or else [`Self::keep_derived`].
    pub(crate) fn derive(&mut self, derived: Derived, emit: impl FnOnce(&mut Self, u8)) -> u8 {
        match self.derived(derived) {
            Some(host) => host,
            None => self.keep_derived(derived, emit),
        }
    }
    /// Forget the derived values computed from `vector`, which changes.
    fn invalidate_derived(&mut self, vector: u8) {
        for (index, kept) in self.vectors.derived.iter_mut().enumerate() {
            if kept.is_some_and(|derived| derived.source == Some(vector)) {
                *kept = None;
                self.vectors.held &= !(1 << (usize::from(FIRST_DERIVED) + index));
            }
        }
    }
    /// Commit to the state from here on, or to homes.
    pub(crate) fn set_vector_write_through(&mut self, through: bool) {
        self.vectors.write_through = through;
    }
    /// Give up `vector`'s home without storing it: the caller writes the
    /// whole register to the state next.
    pub(crate) fn discard_vector(&mut self, vector: u8) {
        self.invalidate_derived(vector);
        let host = self.vectors.home[usize::from(vector)];
        if host != NONE {
            self.vectors.dirty &= !(1 << vector);
            self.vectors.home[usize::from(vector)] = NONE;
            self.vectors.owner[usize::from(host - FIRST_HOST)] = NONE;
            self.vectors.held &= !(1 << host);
        }
    }
    /// Store `vector` if its home is dirty, keeping the home: for a
    /// recipe that then reads part of its slot.
    pub(crate) fn write_back_vector(&mut self, vector: u8) {
        if self.vectors.home[usize::from(vector)] != NONE && self.vectors.is_dirty(vector) {
            self.store_vector(vector);
        }
    }
    /// `vector`'s home holds a value the state block lacks.
    pub(crate) fn mark_vector_dirty(&mut self, vector: u8) {
        debug_assert_ne!(self.vectors.home[usize::from(vector)], NONE);
        self.invalidate_derived(vector);
        self.vectors.dirty |= 1 << vector;
    }
    fn store_vector(&mut self, vector: u8) {
        let host = self.vectors.home[usize::from(vector)];
        // GuestState has only 8-byte alignment.
        self.asm_unguarded().movdqu_store(slot(vector), host);
        self.vectors.dirty &= !(1 << vector);
    }
    /// Store `vector` if dirty and give up its home, for a write of its
    /// slot: its derived values go too.
    pub(crate) fn evict_vector(&mut self, vector: u8) {
        self.invalidate_derived(vector);
        self.spill_vector(vector);
    }
    /// Store `vector` if dirty and give up its home; its value, and so its
    /// derived values, stay.
    fn spill_vector(&mut self, vector: u8) {
        let host = self.vectors.home[usize::from(vector)];
        if host == NONE {
            return;
        }
        if self.vectors.is_dirty(vector) {
            self.store_vector(vector);
        }
        self.vectors.home[usize::from(vector)] = NONE;
        self.vectors.owner[usize::from(host - FIRST_HOST)] = NONE;
        self.vectors.held &= !(1 << host);
    }
    /// Store every dirty home: the state block is then current, and the
    /// homes stay valid copies.
    pub(crate) fn flush_vectors(&mut self) {
        assert_eq!(
            self.vectors.writing, NONE,
            "a destination home is written before the cache stores"
        );
        while self.vectors.dirty != 0 {
            self.store_vector(self.vectors.dirty.trailing_zeros() as u8);
        }
    }
    /// Give up every home; none may be dirty.
    pub(crate) fn forget_vectors(&mut self) {
        assert!(
            !self.vectors.has_dirty(),
            "a dirty V register is stored before the cache forgets it"
        );
        let active = self.vectors.active;
        self.vectors = VectorCache {
            active,
            write_through: self.vectors.write_through,
            clock: self.vectors.clock,
            ..VectorCache::default()
        };
    }
    /// [`Self::flush_vectors`] and [`Self::forget_vectors`].
    pub(crate) fn settle_vectors(&mut self) {
        self.flush_vectors();
        self.forget_vectors();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn homes_fill_the_pool_and_spill_the_least_recently_used_dirty_vector() {
        let mut code = Code::new();
        code.set_vector_cache(true);
        for vector in 0..HOSTS as u8 {
            let host = code.allocate_vector(vector);
            assert_eq!(host, FIRST_HOST + vector);
            code.mark_vector_dirty(vector);
        }
        assert!(code.is_empty());
        // V0 is used again, so V1 is the least recently used.
        assert_eq!(code.vector_home(0), Some(FIRST_HOST));
        let host = code.allocate_vector(20);
        assert_eq!(host, FIRST_HOST + 1);
        // movdqu [rdi+0x118],xmm9: V1 stored before its home is reused.
        assert_eq!(&code[..], [0xf3, 0x44, 0x0f, 0x7f, 0x8f, 0x18, 0x01, 0, 0]);
        assert_eq!(code.vector_home(1), None);
        assert!(!code.vector_cache().is_dirty(1));
        let stored = code.len();
        code.flush_vectors();
        // The seven other dirty vectors, once each; V20 is clean.
        assert_eq!(code.instructions(), 8);
        assert!(!code.vector_cache().has_dirty());
        code.flush_vectors();
        assert!(code.len() > stored);
        assert_eq!(code.instructions(), 8);
        code.forget_vectors();
        assert_eq!(code.vector_cache().occupied(), 0);
        assert_eq!(code.vector_home(0), None);
    }

    #[test]
    #[should_panic(expected = "a dirty V register is stored")]
    fn forgetting_a_dirty_home_is_refused() {
        let mut code = Code::new();
        code.set_vector_cache(true);
        code.allocate_vector(3);
        code.mark_vector_dirty(3);
        code.forget_vectors();
    }
}
