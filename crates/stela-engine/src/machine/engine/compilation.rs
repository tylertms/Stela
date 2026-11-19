//! Compiling, installing and patching blocks, and the exits native code takes
//! to the engine.

use super::*;

/// The board's arena part holding `code`, or [`board::NO_PART`].
fn shared_part(code: &board::Code, at: NativeCode) -> u8 {
    code.arena
        .as_ref()
        .and_then(|arena| arena.part_of(at.0))
        .map_or(board::NO_PART, |part| part as u8)
}

/// Installs of a block in a row whose part a retirement chose before they
/// published: each needs another retirement of the part it just took, so
/// more than a few mean the ring is broken.
const REDONE_INSTALLS: u32 = 64;

/// Put `target`, block `index`'s entry for a fetch at `pc`, in slot `slot`
/// of the block table, under the board's lock (`cache`), if the block is
/// still the indexed one: a retirement or an IC IVAU takes a block out of
/// the index and its entries out of the table in one hold, so an entry put
/// in after would outlive its block. Whether the table took it.
fn insert_target(
    code: &board::Code,
    cache: &mut board::Cache,
    pc: u64,
    index: usize,
    slot: usize,
    target: chain::LookupEntry,
) -> bool {
    let block = &code.blocks[index];
    if cache
        .index
        .get(&code.blocks, pc, block.physical, block.variant)
        != Some(index)
    {
        cache.targets_raced += 1;
        return false;
    }
    cache.targets.insert(slot, target);
    cache.targets_inserted += 1;
    true
}

impl<H: Host, D: Devices> Machine<H, D> {
    pub(super) fn fetch_block(&mut self, pc: u64, single: bool) -> Result<Fetched, Stop> {
        let context = self.current_variant(single, false);
        let el = self.cpu.pstate.el | u8::from(self.cpu.pstate.guarded) << 4;
        let contexts = self.contexts(el, context);
        self.chain.set_contexts(contexts);
        if !self.dropped_refills.is_empty() || !self.refills.is_empty() {
            let refilling = self.host.counter();
            if !self.dropped_refills.is_empty() {
                self.refill_dropped(contexts);
            }
            if !self.refills.is_empty() {
                self.refill_pages(el, context);
            }
            self.statistics.refill_ticks += self.host.counter().wrapping_sub(refilling);
        }
        // A fetch elsewhere came between the exit waiting for a block and
        // its target: an exception, an interrupt or another PE's run.
        if self
            .direct_exit
            .is_some_and(|(_, _, target, _)| target != pc || single)
        {
            self.direct_exit = None;
        }
        // What a lookup dispatcher would enter, through the translation the
        // ITLB keeps current, with the probe's BTYPE check: the entry's
        // targets are the BTYPEs its block's first instruction accepts in
        // a guarded page, or all of them in another, for the key's page
        // guard and SCTLR_ELx.BT ([`Self::remember_target`]). An entry that
        // admits a nonzero BTYPE spares the branch target check its
        // translation; one that refuses it leaves the check, and its
        // Branch Target exception, to the walk.
        self.landing_admitted = false;
        let btype = self.cpu.pstate.btype;
        let path = if single {
            FetchPath::Single
        } else if let Some(key) = self.current_key(pc) {
            let table = &self.code.table;
            let target = table.find(table.slot(pc, key), pc, key);
            if target.pc != pc || target.key != key {
                FetchPath::TableMiss
            } else if target.targets & (1 << btype) == 0 {
                FetchPath::BtypeRefused
            } else {
                let path = if btype == 0 {
                    FetchPath::Fast
                } else {
                    self.landing_admitted = true;
                    FetchPath::FastBtype
                };
                self.statistics.fetch_paths[path as usize] += 1;
                self.statistics.fetches[0] += 1;
                self.link_direct_exit(pc);
                self.entered(target.block as usize);
                return Ok(Fetched::Block(target.block as usize));
            }
        } else {
            FetchPath::ItlbMiss
        };
        self.statistics.fetch_paths[path as usize] += 1;
        let generation = self.translation_generation;
        let walking = self.host.counter();
        let walked = self.fetch_physical(pc);
        self.statistics.walk_ticks += self.host.counter().wrapping_sub(walking);
        let physical = match walked {
            Ok(physical) => physical,
            Err(Ok(exception)) => return Ok(Fetched::Exception(exception)),
            Err(Err(stop)) => return Err(stop),
        };
        // The walk may have forgotten contexts (the upper half's first
        // non-global fetch): the page is recorded, and dispatchers look
        // up, in their successors.
        if self.translation_generation != generation {
            let contexts = self.contexts(el, context);
            self.chain.set_contexts(contexts);
        }
        let current = Variant {
            guarded_page: self.fetch_guarded(pc),
            ..context
        };
        let variant = self.page_variant(current, physical);
        // The block another context, address space or fetch of the frame
        // recorded: what a TLBI ALL or a process switch leaves.
        if !single && let Some(id) = self.known_variant(variant) {
            let key = self.block_key(physical, id);
            self.translate_leaf(pc, key, physical, current);
            let table = &self.code.table;
            let target = table.find(table.slot(pc, key), pc, key);
            // Every release of a block forgets its entries; the block is
            // checked anyway, on this path of a fresh translation.
            let named = (target.pc == pc && target.key == key)
                .then(|| self.code.blocks.installed(target.block as usize))
                .flatten()
                .is_some_and(|block| {
                    (block.pc, block.physical, block.variant) == (pc, physical, id)
                });
            if named {
                self.statistics.fetches[1] += 1;
                self.link_direct_exit(pc);
                self.entered(target.block as usize);
                return Ok(Fetched::Block(target.block as usize));
            }
        }
        let slot = (pc / INSTRUCTION) as usize % JUMP_CACHE;
        if let Some(entry) = self.jump[slot]
            && entry.pc == pc
            && entry.physical == physical
            && entry.variant == variant
            && entry.generation == self.jump_generation()
            && self.code.blocks.record(entry.block).incarnation() == entry.incarnation
        {
            self.statistics.fetches[2] += 1;
            self.remember_or_link(pc, entry.block);
            self.entered(entry.block);
            return Ok(Fetched::Block(entry.block));
        }
        let indexed = self
            .known_variant(variant)
            .and_then(|id| self.code.find(pc, physical, id));
        self.statistics.fetches[if indexed.is_some() { 3 } else { 4 }] += 1;
        let index = match indexed {
            Some(index) => index,
            None => {
                // The ring may choose a part to go at this install.
                self.flush_entered();
                self.reclaim_for_memory();
                self.publishing_target = (!single).then_some(pc);
                let (compiled, mail) = self.compile_aside(pc, physical, variant);
                self.publishing_target = None;
                let compiled = compiled?;
                // What came meanwhile may undo this fetch's translation or
                // the code it compiled; the block stays for the next one.
                if mail {
                    self.read_mail()?;
                    return self.fetch_block(pc, single);
                }
                match compiled {
                    Fetched::Block(index) => index,
                    other => return Ok(other),
                }
            }
        };
        self.jump[slot] = Some(JumpEntry {
            pc,
            physical,
            variant,
            generation: self.jump_generation(),
            block: index,
            incarnation: self.code.blocks.record(index).incarnation(),
        });
        if !single {
            let key = self.block_key(physical, self.code.blocks[index].variant);
            self.translate_leaf(pc, key, physical, current);
        }
        self.remember_or_link(pc, index);
        self.entered(index);
        Ok(Fetched::Block(index))
    }

    /// The block key of the frame holding `physical` for blocks of
    /// `variant` fetched in the PE's current state
    /// ([`chain::block_key`]): the frame, the variant, and in a guarded page
    /// the SCTLR_ELx.BT bit of the current EL that decides which BTYPEs a
    /// PACIASP or PACIBSP accepts ([`fetch::Landing::targets`]).
    pub(super) fn block_key(&self, physical: u64, variant: VariantId) -> u64 {
        const SCTLR_BT0: u64 = 1 << 35;
        const SCTLR_BT1: u64 = 1 << 36;
        let guarded = self.variants[variant.0 as usize].guarded_page;
        let forbid = if self.cpu.pstate.el == 0 {
            SCTLR_BT0
        } else {
            SCTLR_BT1
        };
        let bit = guarded && self.cpu.regime_sctlr() & forbid != 0;
        let state = (variant.0 + 1) << 1 | u32::from(bit);
        assert!(state < chain::KEY_STATES, "a bounded set of variants");
        chain::block_key(physical, state)
    }

    /// The block key the ITLB holds for the page of `pc` in the current
    /// context of its half ([`Self::translate_page`]), or for a lower-half
    /// page in the upper half's current context, which holds the pages
    /// whose walk ended in a global leaf ([`Self::itlb_context`]): what a
    /// lookup dispatcher's translation finds (`chain::translate`).
    pub(super) fn current_key(&self, pc: u64) -> Option<u64> {
        let half = chain::half(pc);
        let contexts = self.chain.contexts();
        let page = pc & !(FETCH_PAGE - 1);
        let held = |context: u64| {
            let held = self
                .itlb
                .find(self.itlb.page_slot(pc, context), page, context);
            (held.pc == page && held.key == context).then_some(held.entry)
        };
        held(contexts[half]).or_else(|| if half == 0 { held(contexts[1]) } else { None })
    }

    /// The context an ITLB entry of the page of `pc` takes: its half's
    /// current one, but for a lower-half page whose walk ended in a global
    /// leaf (`shared`) the upper half's. A global entry matches every ASID
    /// (Arm `TLBIMatch`), so a TLB serves it to every address space of the
    /// regime: the upper half's context is keyed by the same translation
    /// inputs, EL and variant as the lower half's but TTBR0
    /// ([`Self::half_inputs`]), so every process of a PE shares one entry
    /// of each shared-cache page instead of taking its own. Every TLBI by
    /// VA reaches the entry by its PC, with or without an ASID
    /// ([`Self::forget_lookups`], whose global mark it carries); an ASID or
    /// non-global TLBI leaves it, as it leaves global entries; a TLBI of
    /// everything forgets both halves' contexts.
    fn itlb_context(&self, pc: u64, shared: bool) -> u64 {
        let half = chain::half(pc);
        let contexts = self.chain.contexts();
        if half == 0 && shared {
            contexts[1]
        } else {
            contexts[half]
        }
    }

    /// Record that the page of `pc`, which a fetch just translated for
    /// execution in the current context of its half, has the block key
    /// `key` there: lookup dispatchers then translate its PCs without the
    /// engine until a TLBI or a control change forgets the page or the
    /// context. The entry of a forgotten context of the same salt, which
    /// takes the same slots, is this one's older self: it takes its place
    /// and keeps its neighbour. Counts the pages the ITLB lacked in the
    /// context apart from the new keys of pages it held.
    pub(super) fn translate_page(&mut self, pc: u64, key: u64) {
        // A page the fetch cache no longer holds is marked global, which
        // only makes more TLBIs reach it, and kept in its own context.
        let walk = self.cached_fetch(pc);
        let global = walk.is_none_or(|walk| walk.global);
        let shared = walk.is_some_and(|walk| walk.global);
        self.take_page(pc, key, Taken::Fetch, global, shared);
    }

    /// [`Self::translate_page`] for the fetch at `pc` of the frame
    /// `physical` in `current` (its variant before the page's own:
    /// `page_variant`), then the other pages of its leaf
    /// ([`Self::take_leaf_pages`]).
    pub(super) fn translate_leaf(&mut self, pc: u64, key: u64, physical: u64, current: Variant) {
        self.translate_page(pc, key);
        if let Some(walk) = self.cached_fetch(pc) {
            self.take_leaf_pages(pc, physical, walk.size, current, walk.global);
        }
    }

    /// Give the ITLB the other 4 KiB pages of the [`LEAF_FILL`] granule
    /// around `pc` that the leaf of `size` bytes holding `pc` maps, whose
    /// page is in the frame of `physical`, as a fetch of each would take
    /// them: the walk that translated `pc` translated them all, through
    /// the same descriptors (the same output block, permissions, guarded
    /// and global bits), so a TLB may cache each at once. Pages the host
    /// refuses to fetch, whose blocks' variant has no number yet, or that
    /// the ITLB holds already are left to their own fetch. A TLBI by VA of
    /// any of them reaches all: the fetch recorded the whole leaf
    /// (`fetched_through`). `current` is the fetch's variant before each
    /// page's own (`page_variant`).
    fn take_leaf_pages(
        &mut self,
        pc: u64,
        physical: u64,
        size: u64,
        current: Variant,
        global: bool,
    ) {
        let span = size.min(LEAF_FILL);
        if span <= FETCH_PAGE {
            return;
        }
        let page = pc & !(FETCH_PAGE - 1);
        let frame = physical & !(FETCH_PAGE - 1);
        let first = pc & !(span - 1);
        for other in (first..first + span).step_by(FETCH_PAGE as usize) {
            if other == page || self.current_key(other).is_some() {
                continue;
            }
            if !self.host.fetch_permitted(other) {
                continue;
            }
            let other_frame = frame.wrapping_add(other).wrapping_sub(page);
            let variant = self.page_variant(current, other_frame);
            let Some(id) = self.known_variant(variant) else {
                continue;
            };
            let key = self.block_key(other_frame, id);
            self.take_page(other, key, Taken::Leaf, global, global);
        }
    }

    /// [`Self::translate_page`], counting a page the ITLB lacked by how it
    /// was `taken` ([`Taken`]). `global` marks a page whose walk ended in
    /// a global leaf ([`chain::LookupEntry::global_page`]), or may have;
    /// `shared` puts a lower-half page whose walk did in the context every
    /// address space shares ([`Self::itlb_context`]).
    fn take_page(&mut self, pc: u64, key: u64, taken: Taken, global: bool, shared: bool) {
        debug_assert!(global || !shared, "only a global page is shared");
        let half = chain::half(pc);
        let context = self.itlb_context(pc, shared);
        let page = pc & !(FETCH_PAGE - 1);
        let slot = self.itlb.page_slot(pc, context);
        let entry = match global {
            true => chain::LookupEntry::global_page(page, context, key),
            false => chain::LookupEntry::page(page, context, key),
        };
        let held = self.itlb.find(slot, page, context);
        if held == entry {
            return;
        }
        let held_page = held.pc == page && held.key == context;
        if !held_page {
            self.log_fill(page, context);
            if shared && half == 0 {
                self.statistics.itlb_sources[ItlbSource::SharedFills as usize] += 1;
            }
        }
        match taken {
            Taken::Refill if !held_page => {
                self.statistics.itlb_sources[ItlbSource::Refilled as usize] += 1;
            }
            Taken::Leaf if !held_page => {
                self.statistics.itlb_sources[ItlbSource::LeafFills as usize] += 1;
            }
            Taken::Refill | Taken::Leaf => {}
            Taken::Fetch if held_page => self.statistics.itlb[1] += 1,
            Taken::Fetch => {
                self.statistics.itlb[0] += 1;
                let miss = self.itlb_miss(slot, page, context);
                self.statistics.itlb_misses[miss as usize] += 1;
                self.statistics.itlb_sources[ItlbSource::FillsLower as usize + half] += 1;
                if self.note_itlb_seen(half, page, context) && miss == ItlbMiss::Cold {
                    self.statistics.itlb_sources[ItlbSource::ColdSeen as usize] += 1;
                }
            }
        }
        let resident = self.itlb.entry(slot);
        if resident.pc == page && resident.key != context && !self.live_context(resident.key) {
            self.itlb.set(slot, entry);
        } else {
            // A context no lookup reaches any more gives its place first
            // (`insert_over`): a dead resident leaves rather than a live
            // neighbour.
            let contexts = self.chain.contexts();
            let spaces = &self.spaces;
            let dead = |key: u64| {
                !contexts.contains(&key) && !spaces.iter().any(|s| s.live(key).is_some())
            };
            // What the resident's move takes the place of, unless it is the
            // page's own older entry, or the dead resident itself leaves.
            let lost = self.itlb.entry(slot ^ 1);
            if resident.key != 0
                && (resident.pc, resident.key) != (page, context)
                && !dead(resident.key)
                && lost.key != 0
                && (lost.pc, lost.key) != (page, context)
                && !dead(lost.key)
            {
                self.statistics.itlb_drops[0] += 1;
                note_itlb_ghost(&mut self.itlb_ghosts, &lost, ItlbMiss::Evicted);
            }
            self.itlb.insert_over(slot, entry, &dead);
        }
    }

    /// Why the ITLB lacked the page `page` in `context`, whose
    /// slot is `slot` ([`ItlbMiss`]): what [`note_itlb_ghost`] noted of
    /// the page in the context, or of it in a forgotten context of the
    /// same salt, or an entry of such a context in the page's set.
    fn itlb_miss(&self, slot: usize, page: u64, context: u64) -> ItlbMiss {
        let salt = context % chain::LOOKUP_SALTS as u64;
        let forgotten = |number: u64| {
            number != context
                && number % chain::LOOKUP_SALTS as u64 == salt
                && !self.live_context(number)
        };
        if let Some(&(noted, number)) = self.itlb_ghosts.get(itlb_ghost_slot(page, context))
            && noted & !ITLB_GHOST_CAUSE == page
        {
            if number == context {
                return if noted & ITLB_GHOST_CAUSE == ItlbMiss::Forgotten as u64 {
                    ItlbMiss::Forgotten
                } else {
                    ItlbMiss::Evicted
                };
            }
            if forgotten(number) {
                return ItlbMiss::Context;
            }
        }
        let older = [slot, slot ^ 1].into_iter().any(|at| {
            let held = self.itlb.entry(at);
            held.pc == page && held.key != 0 && forgotten(held.key)
        });
        if older {
            ItlbMiss::Context
        } else {
            ItlbMiss::Cold
        }
    }

    /// Link the direct exit that waited for the block `index` the fetch at
    /// `pc` found ([`Self::link_direct_exit`]), or, when there is none or
    /// it cannot link, [`Self::remember_target`]. A block only linked
    /// exits enter needs no lookup entry: the link is the page's, and
    /// enters it in every address space.
    ///
    /// An exit from another page links through a cell, which starts with
    /// the block table's entry of its target: the entry goes in first.
    fn remember_or_link(&mut self, pc: u64, index: usize) {
        let from_another_page = self.direct_exit.is_some_and(|(block, _, target, _)| {
            target == pc
                && self
                    .code
                    .blocks
                    .installed(block)
                    .is_some_and(|block| (block.pc ^ pc) & !(FETCH_PAGE - 1) != 0)
        });
        if from_another_page {
            self.remember_target(pc, index);
            self.link_direct_exit(pc);
        } else if !self.link_direct_exit(pc) {
            self.remember_target(pc, index);
        }
    }

    /// Record that a fetch at `pc` runs block `index` while the page of
    /// `pc` translates to the block's frame in the fetch state its key
    /// names ([`Self::block_key`]), for lookup dispatchers: every address
    /// space and context that maps the frame there finds it. Not for
    /// single-instruction blocks, which debug checks precede, nor trace
    /// points, which the engine records. Whether the table took a new
    /// entry: one it holds already, in either place of its set, stays where
    /// it is, and keeps its neighbour.
    pub(super) fn remember_target(&mut self, pc: u64, index: usize) -> bool {
        let Some((slot, target)) = self.target_entry(pc, index) else {
            return false;
        };
        let block = &self.code.blocks[index];
        let (physical, variant_id) = (block.physical, block.variant);
        // Only an indexed block takes an entry: one an IC IVAU forgot (that
        // a PE's jump cache still names until it reads the post) or one
        // retired since it was found would be reachable again, and links
        // out of it are cut by no detach ([`Self::detach`] visits the
        // indexed blocks of a target's page). The finder answers without
        // the lock; the lock checks again before the insert.
        let code = &self.code;
        match code.finder.find(&code.blocks, pc, physical, variant_id) {
            Some(Some(found)) if found == index => {}
            Some(_) => {
                board::Events::count(&code.events.targets_unindexed);
                return false;
            }
            None => {}
        }
        if !code.blocks.record(index).remember() {
            board::Events::count(&code.events.targets_deferred);
            return false;
        }
        let mut cache = code.cache_at(board::LockSite::Target);
        insert_target(code, &mut cache, pc, index, slot, target)
    }

    /// The block table slot and entry a fetch at `pc` of block `index`
    /// would add for lookup dispatchers ([`Self::remember_target`]), or
    /// none when the block takes none or the table holds it already.
    fn target_entry(&self, pc: u64, index: usize) -> Option<(usize, chain::LookupEntry)> {
        let block = &self.code.blocks[index];
        let variant = &self.variants[block.variant.0 as usize];
        if variant.single || self.traces.binary_search(&pc).is_ok() {
            return None;
        }
        let targets = match variant.guarded_page {
            true => block.landing.targets(&self.cpu) | 1,
            false => 0b1111,
        };
        let key = self.block_key(block.physical, block.variant);
        let slot = self.code.table.slot(pc, key);
        let target = chain::LookupEntry {
            pc,
            key,
            entry: block.code.0 as u64,
            block: index as u32,
            targets,
        };
        (self.code.table.find(slot, pc, key) != target).then_some((slot, target))
    }

    /// Whether lookups may still run in context `number`, of either half:
    /// one no TLBI or control change forgot. Context numbers are never
    /// reused, in either half, and a lower-half page's entry may be in an
    /// upper-half context ([`Self::itlb_context`]).
    fn live_context(&self, number: u64) -> bool {
        self.spaces
            .iter()
            .any(|spaces| spaces.live(number).is_some())
    }

    /// Remember that the ITLB took `page` in `context` ([`Self::fill_log`]).
    fn log_fill(&mut self, page: u64, context: u64) {
        if self.fill_log.len() < FILL_LOG {
            self.fill_log.push((page, context));
        } else {
            self.fill_log[self.fill_next] = (page, context);
        }
        self.fill_next = (self.fill_next + 1) % FILL_LOG;
    }

    /// Before the contexts of `half` go: keep, by key, the pages each took
    /// last ([`Self::fill_log`]), for the context made anew for its key to
    /// walk again ([`Self::refill_pages`]). Lists kept from before for keys
    /// of `half` no context took since give way.
    pub(super) fn refill_after(&mut self, half: usize) {
        self.pending_refills.retain(|&(held, _), _| held != half);
        let spaces = &self.spaces[half];
        let other = &self.spaces[1 - half];
        let pending = &mut self.pending_refills;
        // A context's fills are of its half's pages, and for the upper half
        // also of the lower half's global ones (`itlb_context`).
        self.fill_log.retain(|&(page, context)| {
            if let Some(space) = spaces.live(context) {
                let pages = pending.entry((half, space.key)).or_default();
                if pages.len() < REFILLS_PER_CONTEXT {
                    pages.push(page);
                }
                return false;
            }
            other.live(context).is_some() || chain::half(page) != half
        });
        self.fill_next = self.fill_log.len() % FILL_LOG;
        for pages in self.pending_refills.values_mut() {
            pages.sort_unstable();
            pages.dedup();
        }
    }

    /// Keep the page `page` a TLBI by VA dropped of the live context
    /// `context` for the next fetch in that context to walk again
    /// ([`Self::refill_dropped`]): a TLBI that changed a mapping is mostly
    /// followed by a fetch of the page through its new one. At most
    /// [`REFILLS_PER_CONTEXT`] pages of [`DROPPED_CONTEXTS`] contexts; the
    /// lists of contexts that went give way.
    pub(super) fn keep_dropped(
        dropped: &mut BTreeMap<u64, Vec<u64>>,
        live: impl Fn(u64) -> bool,
        context: u64,
        page: u64,
    ) {
        if !dropped.contains_key(&context) && dropped.len() >= DROPPED_CONTEXTS {
            dropped.retain(|&number, _| live(number));
            if dropped.len() >= DROPPED_CONTEXTS {
                return;
            }
        }
        let pages = dropped.entry(context).or_default();
        if pages.len() < REFILLS_PER_CONTEXT && !pages.contains(&page) {
            pages.push(page);
        }
    }

    /// Queue for [`Self::refill_pages`] the pages TLBIs by VA dropped of the
    /// contexts the fetch now runs in (`contexts`, by half): each walks in
    /// the translation that dropped it, now changed or not. Context numbers
    /// are never reused, so a list of a context that went matches no fetch.
    fn refill_dropped(&mut self, contexts: [u64; 2]) {
        for context in contexts {
            if let Some(pages) = self.dropped_refills.remove(&context) {
                self.refills.extend(pages);
            }
        }
    }

    /// Walk again, in the contexts the fetch now runs in, the pages their
    /// forgotten predecessors of the same key took last
    /// ([`Self::refills`]), and give the ITLB those that translate: what a
    /// fetch of each would record, without an exit for each. A TLB may
    /// take any translation a walk gives at any time; a page whose walk
    /// faults or that the host refuses is left to its fetch. `el` (with
    /// the guarded level) and `context` are what the fetch runs in. A walk
    /// that forgets contexts ends it, and the fetch takes its contexts
    /// anew.
    pub(super) fn refill_pages(&mut self, el: u8, context: Variant) {
        let generation = self.translation_generation;
        for pc in core::mem::take(&mut self.refills) {
            // Taken already, with its leaf's other pages.
            if self.current_key(pc).is_some() {
                continue;
            }
            let Some(walk) = self.quiet_fetch_physical(pc) else {
                continue;
            };
            if self.translation_generation != generation {
                let contexts = self.contexts(el, context);
                self.chain.set_contexts(contexts);
                return;
            }
            let current = Variant {
                guarded_page: walk.guarded,
                ..context
            };
            let variant = self.page_variant(current, walk.physical);
            if let Some(id) = self.known_variant(variant) {
                let key = self.block_key(walk.physical, id);
                self.take_page(pc, key, Taken::Refill, walk.global, walk.global);
            }
            self.take_leaf_pages(pc, walk.physical, walk.size, current, walk.global);
        }
    }

    /// Note in [`Self::itlb_seen`] that the ITLB took the page `page` of
    /// `half` in `context`, by its address space (TTBR0's value in the
    /// lower half): whether it had before, in whatever context.
    fn note_itlb_seen(&mut self, half: usize, page: u64, context: u64) -> bool {
        let table = self.spaces[half]
            .live(context)
            .map_or(0, |space| space.key.ttbr0);
        // A quarter full, it starts over: false answers stay under 25%.
        if self.itlb_seen.is_empty() || self.itlb_seen_bits >= ITLB_SEEN_BITS / 4 {
            self.itlb_seen.clear();
            self.itlb_seen.resize(ITLB_SEEN_BITS / 64, 0);
            self.itlb_seen_bits = 0;
        }
        let mixed =
            ((page / FETCH_PAGE) ^ table.rotate_left(23)).wrapping_mul(0x9e37_79b9_7f4a_7c15);
        let bit = (mixed >> (64 - ITLB_SEEN_BITS.trailing_zeros())) as usize;
        let word = &mut self.itlb_seen[bit / 64];
        let seen = *word >> (bit % 64) & 1 != 0;
        *word |= 1 << (bit % 64);
        self.itlb_seen_bits += usize::from(!seen);
        seen
    }

    /// Compile the block at `pc`, unless its first instruction is a
    /// trapped FP/SIMD instruction or one whose access checks the engine
    /// applies.
    pub(super) fn compile(
        &mut self,
        pc: u64,
        physical: u64,
        variant: Variant,
    ) -> Result<Fetched, Stop> {
        // Before its bytes are read: an IC IVAU of the page posted after
        // a write this read may miss reaches this PE then.
        if let Some(hub) = &self.parallel {
            hub.compiling(physical);
        }
        let window = source_window(pc, variant);
        if window > FIRST_WINDOW_BYTES
            && let Some(fetched) = self.compile_window(pc, physical, variant, FIRST_WINDOW_BYTES)?
        {
            return Ok(fetched);
        }
        Ok(self
            .compile_window(pc, physical, variant, window)?
            .expect("a block of the whole window"))
    }

    /// [`Self::compile`] from the first `available` bytes of the block's
    /// source window, or nothing when the block may run past them (it
    /// takes them all, or they are refused) and they are not the whole
    /// window.
    pub(super) fn compile_window(
        &mut self,
        pc: u64,
        physical: u64,
        variant: Variant,
        available: usize,
    ) -> Result<Option<Fetched>, Stop> {
        let whole = available == source_window(pc, variant);
        let page = pc & !(FETCH_PAGE - 1);
        let offset = (pc - page) as usize;
        let mut staging = [0; MAX_BLOCK_BYTES as usize];
        let source = &mut staging[..available];
        if !self.host.read_ram(physical, source) {
            // The fetch takes the external abort the fabric raises outside
            // memory (IFSC 0b010000).
            self.degrade(self.requirement(RequirementKind::Fetch { physical }));
            return Ok(Some(Fetched::Exception(self.external_abort_exception(
                [Class::InstructionAbortLower, Class::InstructionAbortSame],
                SYNCHRONOUS_EXTERNAL_ABORT,
                pc,
            ))));
        }
        // A block from elsewhere is used only when its bytes are among
        // those read, and so validated against the stored page.
        let within =
            |compiled: &stela_compiler::block::CompiledBlock| compiled.source_bytes <= available;
        // Blocks end before a device site, which no entry context names:
        // the store neither gives nor takes its page's blocks.
        let sited = !variant.single && self.page_has_device_sites(physical);
        let reading = self.host.counter();
        let stored = match variant.single || sited {
            true => None,
            false => self.stored_page(pc, physical, source),
        };
        self.statistics.page_ticks += self.host.counter().wrapping_sub(reading);
        let preparation = self.preparation(variant);
        preparation.admit(source);
        if sited && self.device_site(physical) {
            return self
                .install_device_stub(pc, physical, variant, source)
                .map(Some);
        }
        let context = stored
            .is_some()
            .then(|| self.stored_context(variant, &preparation, page));
        // Compiled at another PC of the same page content before: its
        // stored form is kept once compiled here ([`super::rebase`]).
        let mut keep = false;
        if let (Some((key, record)), Some(context)) = (&stored, &context) {
            // The machine's buffers take the entry's relocated code: an
            // install from the store allocates nothing.
            let mut relocated = core::mem::take(&mut self.relocated);
            let entry = record
                .as_ref()
                .and_then(|record| record.find(offset as u16, context));
            let found = entry
                .as_ref()
                .filter(|entry| entry.relocate(page, source, &mut relocated));
            if record.is_some() && found.is_none() {
                self.statistics.record_misses[usize::from(entry.is_some())] += 1;
            }
            if let Some(entry) = found {
                if usize::from(entry.source_bytes) > available {
                    self.relocated = relocated;
                    return Ok(None);
                }
                self.statistics.stored += 1;
                self.count_store_source(StoreSource::Pack);
                let origin = self.stored_origin(physical, page, *key, *context, &preparation);
                let installed = self
                    .install(
                        pc,
                        physical,
                        variant,
                        source,
                        recovery::Template::stored(entry, &relocated, origin),
                    )
                    .map(Some);
                self.relocated = relocated;
                return installed;
            }
            self.relocated = relocated;
            let looking = self.host.counter();
            if let Some(compiled) = self
                .host
                .translations()
                .and_then(|translations| translations.prepared(key, pc, context))
            {
                if !within(compiled.block()) {
                    return Ok(None);
                }
                let matches = compiled.matches(key, pc, context, source);
                if !matches && let Some(translations) = self.host.translations() {
                    translations.note(super::StoreEvent::SharedRejected);
                }
                if matches {
                    self.statistics.shared += 1;
                    self.count_store_source(StoreSource::Shared);
                    self.statistics.shared_ticks += self.host.counter().wrapping_sub(looking);
                    return self
                        .install(
                            pc,
                            physical,
                            variant,
                            source,
                            recovery::Template::compiled(compiled.emission.clone()),
                        )
                        .map(Some);
                }
            }
            // The same page content, compiled at another address of this
            // boot: its kept entry, rebased to this page.
            // A block the ring took away and the guest runs again: its
            // form is kept, and its next return relocated from it.
            let again = self.ring_took(pc, physical, variant);
            match self
                .code
                .rebases
                .look(key, offset as u16, context, pc, again)
            {
                super::rebase::Found::Entry(entry) => {
                    if entry.input.len() > available && !whole {
                        return Ok(None);
                    }
                    let mut relocated = core::mem::take(&mut self.relocated);
                    if entry.relocate(page, source, &mut relocated) {
                        if usize::from(entry.source_bytes) > available {
                            self.relocated = relocated;
                            return Ok(None);
                        }
                        self.code.rebases.hit();
                        self.count_store_source(StoreSource::Rebased);
                        let origin = Arc::new(recovery::StoredOrigin {
                            key: *key,
                            context: *context,
                            page,
                            preparation: preparation.clone(),
                            kept: Some(entry.clone()),
                        });
                        let installed = self
                            .install(
                                pc,
                                physical,
                                variant,
                                source,
                                recovery::Template::stored(&entry, &relocated, origin),
                            )
                            .map(Some);
                        self.relocated = relocated;
                        return installed;
                    }
                    self.relocated = relocated;
                }
                super::rebase::Found::Keep => keep = true,
                super::rebase::Found::Compile => {}
            }
        }
        let started = self.host.counter();
        // No block runs into a host breakpoint or trace point: one ends
        // before it.
        let limit = self
            .breakpoints
            .iter()
            .chain(&self.traces)
            .filter(|&&stop| stop > pc && stop < pc + source.len() as u64)
            .min()
            .map_or(source.len(), |&stop| (stop - pc) as usize);
        let limit = match sited {
            true => self
                .before_device_site(physical, source.len())
                .map_or(limit, |site| site.min(limit)),
            false => limit,
        };
        let prepared = preparation.prepare_window(source, pc, limit, whole);
        #[cfg(all(feature = "std", feature = "native-profile"))]
        if let Some(metrics) = &mut self.preparation_metrics {
            let ticks = self.host.counter().wrapping_sub(started);
            metrics.record(prepared.as_ref(), ticks, !whole);
        }
        let prepared = match prepared {
            Some(Err(_)) | None if !whole => return Ok(None),
            prepared => prepared.expect("complete window"),
        };
        let (compiled, input, successors) = match prepared {
            Ok(Prepared::Block {
                compiled,
                input,
                successors,
                ..
            }) => (compiled, input, successors),
            Ok(Prepared::Checked(word)) => return Ok(Some(Fetched::Checked(word))),
            Ok(Prepared::FloatingPointTrap) => {
                return Ok(Some(Fetched::Exception(self.floating_point_exception(pc))));
            }
            Err(refused) => {
                return Err(self.unsupported_word(
                    source,
                    pc,
                    refused.pc,
                    refused.error.to_string(),
                ));
            }
        };
        self.statistics.compiled += 1;
        self.statistics.self_loops += u64::from(compiled.self_loop);
        self.count_store_source(if stored.is_some() {
            StoreSource::Compiled
        } else {
            StoreSource::Unstorable
        });
        self.statistics.compiled_instructions += (compiled.source_bytes as u64) / INSTRUCTION;
        self.statistics.compile_ticks += self.host.counter().wrapping_sub(started);
        let emission = Arc::new(super::stored::Emission::new(
            source[..input].to_vec(),
            compiled,
            preparation,
        ));
        if keep
            && !self.host.memory_low()
            && let (Some((key, _)), Some(context)) = (&stored, &context)
        {
            let entry = super::stored::stored_entry(
                &emission.block,
                &emission.input,
                page,
                pc,
                *context,
                &emission.preparation,
            );
            self.code.rebases.keep(key, offset as u16, context, entry);
        }
        // A store keeps an offer's stored form within its own bound, and its
        // pack writer sheds offers before the heap runs short (writer
        // `govern`): memory low, as the heap rests once
        // the engine's caches have grown, still keeps it, or the pack would
        // miss every block compiled after that. The other PEs take the block
        // either way.
        if let (Some((key, _)), Some(context)) = (stored, context)
            && let low = self.host.memory_low()
            && let Some(translations) = self.host.translations()
        {
            let offer = Offer {
                key,
                page,
                pc,
                context,
                emission: emission.clone(),
                successors,
                quiet: self.cpu.pstate.el == 0,
            };
            translations.offer(offer);
            self.statistics.offered += 1;
            self.statistics.offered_low += u64::from(low);
        }
        self.install(
            pc,
            physical,
            variant,
            source,
            recovery::Template::compiled(emission),
        )
        .map(Some)
    }

    /// Count an install from `source` at the PE's level: EL0, or above.
    fn count_store_source(&mut self, source: StoreSource) {
        let level = usize::from(self.cpu.pstate.el != 0);
        self.statistics.store_sources[level][source as usize] += 1;
    }

    /// How the next fetch compiles its block, in the PE's current state:
    /// for preparing blocks elsewhere as this machine would compile them.
    pub fn current_preparation(&mut self) -> Arc<Preparation> {
        let variant = self.current_variant(false, false);
        self.preparation(variant)
    }

    /// The variant a fetch compiles for in the PE's current state.
    pub(super) fn current_variant(&mut self, single: bool, guarded_page: bool) -> Variant {
        Variant {
            fp: self.floating_point_enabled(),
            checks: self.memory_checks(),
            single,
            scalable: self
                .cpu
                .scalable
                .is_some()
                .then(|| self.cpu.svcr() & 1 != 0),
            system: self.native_system(),
            guarded_page,
            timing: self.cpu.pstate.dit && self.timing_target().is_some(),
            translated: false,
        }
    }

    /// `variant`'s id, added to [`Self::variants`] the first time by
    /// whichever machine of the board meets it first.
    pub(super) fn variant_id(&mut self, variant: Variant) -> VariantId {
        let id = self
            .variants
            .find_or_add(variant, || true)
            .expect("a bounded set of variants");
        VariantId(id as u32)
    }

    /// `variant`'s id, once [`Self::variant_id`] gave it one: for a block
    /// compiled in it, or a context made in it ([`Self::context_salt`]).
    pub(super) fn known_variant(&self, variant: Variant) -> Option<VariantId> {
        self.variants.find(&variant).map(|at| VariantId(at as u32))
    }

    /// What block `index` was compiled for.
    pub(super) fn block_variant(&self, index: usize) -> Variant {
        self.variants[self.code.blocks[index].variant.0 as usize]
    }

    /// How blocks of `variant` are compiled, made once per variant.
    pub(super) fn preparation(&mut self, variant: Variant) -> Arc<Preparation> {
        let variant = Variant {
            guarded_page: false,
            ..variant
        };
        if let Some(preparation) = self.preparations.get(&variant) {
            return preparation.clone();
        }
        let target = match variant.timing {
            true => self
                .timing_target()
                .expect("a timing variant has its target"),
            false => self.target,
        };
        let preparation = Arc::new(Preparation::new(
            self.profile.chip,
            self.admit_table.clone(),
            target,
            variant.fp,
            variant.checks,
            variant.scalable,
            // A single instruction's variant leaves every PAC instruction to
            // the engine, as it installs no thunk for one.
            NativeSystem {
                pointer: variant.system.pointer.filter(|_| !variant.single),
                ..variant.system
            },
        ));
        self.preparations.insert(variant, preparation.clone());
        preparation
    }

    /// Install `compiled`, the block at `pc` of `variant` compiled from the
    /// admitted `source`, and its thunks, and index it.
    pub(super) fn install(
        &mut self,
        pc: u64,
        physical: u64,
        variant: Variant,
        source: &[u8],
        compiled: recovery::Template<'_>,
    ) -> Result<Fetched, Stop> {
        let started = self.host.counter();
        // A retirement may choose the part of an install before it
        // publishes: the install is done again, in a part that stays.
        let mut tries = 0;
        let installed = loop {
            let placed = self.place(pc, physical, variant, source, &compiled);
            self.release_places();
            match placed {
                Ok(Some(fetched)) => break Ok(fetched),
                Err(stop) => break Err(stop),
                Ok(None) => {}
            }
            tries += 1;
            if tries >= REDONE_INSTALLS {
                break Err(self.unsupported(RequirementKind::Native(
                    "every install of a block lost its arena part before it published",
                )));
            }
        };
        self.statistics.install_ticks += self.host.counter().wrapping_sub(started);
        installed
    }

    /// Install `compiled` once: the block, or none when a retirement chose
    /// its part before it published ([`board::Ring::live`]).
    fn place(
        &mut self,
        pc: u64,
        physical: u64,
        variant: Variant,
        source: &[u8],
        compiled: &recovery::Template<'_>,
    ) -> Result<Option<Fetched>, Stop> {
        let single = variant.single;
        if compiled.portable_requires_unavailable_capture() {
            return Err(self.unsupported(RequirementKind::Native(
                "native spill/extended-register invocation capture is not qualified",
            )));
        }
        let source_bytes = compiled.source_bytes();
        let instructions = compiled.instructions();
        if !compiled.matches_source(source)
            || source_bytes < INSTRUCTION as usize
            || source_bytes > source.len()
            || pc.checked_add(source_bytes as u64).is_none()
            || instructions * INSTRUCTION as usize != source_bytes
        {
            return Err(self.unsupported(RequirementKind::Native(
                "native template has no matching admitted source extent",
            )));
        }
        let last = source_bytes - INSTRUCTION as usize;
        let terminal = u32::from_le_bytes(source[last..last + 4].try_into().expect("a word"));
        let code_bytes = compiled.code();
        let id = self.variant_id(variant);
        let stream = self.stream_for(pc, physical, id);
        self.install_stream = stream;
        let code = self.install_code(code_bytes)?;
        // With every slot given, or a heap without room for the records'
        // next chunk, every block goes; a full arena reclaimed its oldest
        // part above, whose slots serve first.
        let (code, slot) = match self.take_slot() {
            Some(slot) => (code, slot),
            None => {
                // The code is installed again after: its place goes, and
                // this PE holds none while the ring moves on.
                self.release_places();
                // A board's ring gives slots back by taking its oldest
                // parts away, which may be the one `code` went to.
                let retired = match self.code.arena.is_some() {
                    true => self.retire_for_slot(),
                    false => None,
                };
                match retired {
                    Some(slot) => {
                        self.install_stream = stream;
                        (self.install_code(code_bytes)?, slot)
                    }
                    None => {
                        self.discard_code(Discard::Table);
                        self.install_stream = stream;
                        let code = self.install_code(code_bytes)?;
                        let slot = self.take_slot().ok_or_else(|| {
                            self.unsupported(RequirementKind::Native("no block slot is left"))
                        })?;
                        (code, slot)
                    }
                }
            }
        };
        // The block's place in the board's arena, its install's first.
        let taken = self.held_places.first().copied();
        let installed_bytes = code_bytes.len();
        let terminal_pc = pc + last as u64;
        let direct_exits = compiled.direct_exits();
        if self.step_timing.is_some() {
            self.profile_block(code, pc, code_bytes, compiled.cold(), direct_exits);
        }
        // Built in the machine's own list; the block's box comes from
        // those evicted blocks left ([`Self::successor_box`]).
        let mut successors = core::mem::take(&mut self.successor_scratch);
        successors.clear();
        // Only an exit whose patch point lies where its address puts it
        // can be linked: any other stays an exit to the engine.
        successors.extend(direct_exits.iter().filter_map(|exit| {
            let slot = chain::link_slot(Some(pc), exit.target, exit.kind);
            chain::exit_ready(code_bytes, exit.offset, code.0 as u64, slot).then(|| {
                Successor::new(
                    code.0 as u64 + exit.offset as u64,
                    exit.target,
                    exit.word(),
                    slot,
                )
            })
        }));
        // RET, BR and BLR end the code with their exit to the target in
        // RAX; the PAC thunk of a RETA*, BRA* or BLRA* has one too. Only
        // side exits follow a terminal's, and of the direct exits only a
        // terminal that has one precedes them: what a thunk completes has
        // none, whatever a side exit just before it leaves unrun.
        let terminal_end = direct_exits
            .iter()
            .map(|exit| exit.offset - chain::link_slot(Some(pc), exit.target, exit.kind))
            .min()
            .unwrap_or(chain::without_failures(code_bytes).len());
        let code_bytes = &code_bytes[..terminal_end];
        let mut dynamic_site = (!single
            && terminal & 0xff9f_fc1f == 0xd61f_0000
            && dynamic_branch(terminal).is_some_and(|(kind, _)| {
                let exit = code_bytes.len().saturating_sub(chain::DYNAMIC_EXIT_BYTES);
                chain::is_dynamic_exit(&code_bytes[exit..], kind)
                    && chain::dynamic_ready(code_bytes, exit, code.0 as u64)
            }))
        .then(|| (code.0 + code_bytes.len() - chain::DYNAMIC_EXIT_BYTES) as u64);
        let first = self.admit(u32::from_le_bytes(source[..4].try_into().expect("a word")));
        // A device site's stub completes only a plain device load natively.
        if !single && self.installing_stub {
            successors.extend(self.complete_device_read_natively(
                code,
                code_bytes,
                terminal_pc,
                terminal,
                physical,
            ));
        } else if !single && !self.device_site(physical) {
            let (successor, dynamic) = self
                .complete_pointer_natively(code, code_bytes, terminal_pc, terminal)
                .unwrap_or_default();
            successors.extend(successor);
            dynamic_site = dynamic_site.or(dynamic);
            let ending = u32::try_from(instructions).unwrap_or(u32::MAX);
            if let Some(exit) =
                self.complete_transition_natively(code, code_bytes, terminal_pc, terminal, ending)
            {
                dynamic_site = dynamic_site.or(Some(exit));
            }
            successors.extend(self.complete_mirrored_read_natively(
                code,
                code_bytes,
                terminal_pc,
                terminal,
                (last / INSTRUCTION as usize) as u32,
            ));
        }
        // The record keeps a bit per successor; any more stay exits.
        successors.truncate(board::MAX_SUCCESSORS);
        // The slot's last block leaves its box for this one.
        if let Some(old) = self.code.blocks.vacate(slot) {
            self.keep_successor_box(old.successors);
        }
        let boxed = self.successor_box(&successors);
        self.successor_scratch = successors;
        let block = Block {
            pc,
            physical,
            code,
            instructions: instructions as u16,
            variant: id,
            successors: boxed,
            dynamic_site: dynamic_site.and_then(NonZeroU64::new),
            landing: fetch::Landing::of(first),
            terminal,
            source_hash: source_hash(&source[..source_bytes]),
            installer: self.current as u8,
            part: shared_part(&self.code, code),
            device_stub: self.installing_stub,
        };
        let shared = Arc::clone(&self.code);
        shared.blocks.fill(slot, block);
        // The slot is this PE's until published: its rows go in first.
        shared
            .blocks
            .recovery()
            .insert(slot, compiled.clone().installed());
        let mut cache = shared.cache_at(board::LockSite::Install);
        self.census_installed(
            &mut cache,
            pc,
            physical,
            variant,
            installed_bytes,
            instructions,
        );
        // Another PE installed the block meanwhile: the first install wins,
        // and this one's code stays unreachable until its part goes.
        if let Some(held) = cache.index.get(&shared.blocks, pc, physical, id) {
            shared.blocks.record(slot).mark_lost();
            // Its rows go: nothing faults in a block never reached.
            shared.blocks.recovery().remove(&slot);
            shared.blocks.retire(slot);
            let (grace, _) = self.grace();
            shared
                .blocks
                .release(alloc::vec![slot as u32], grace.snapshot());
            drop(cache);
            self.statistics.lost_installs += 1;
            board::Events::count(&shared.events.lost);
            return Ok(Some(Fetched::Block(held)));
        }
        // A retirement chose the code's part since its place was taken:
        // it listed the part's blocks without this one, so the block is
        // never published. Its code is reached by nothing (its thunks only
        // from it) and goes with its part, and the install is done again.
        if let (Some(arena), Some(place)) = (&shared.arena, taken)
            && !arena.ring.live(place)
        {
            shared.blocks.record(slot).mark_lost();
            shared.blocks.recovery().remove(&slot);
            shared.blocks.retire(slot);
            let (grace, _) = self.grace();
            shared
                .blocks
                .release(alloc::vec![slot as u32], grace.snapshot());
            drop(cache);
            board::Events::count(&shared.events.redone);
            return Ok(None);
        }
        self.note_churn(&mut cache, slot);
        cache.code_slots.record(code.0, slot, || {
            shared
                .arena
                .as_ref()
                .map(board::Arena::partitions)
                .or_else(|| self.host.code_partitions())
        });
        if let Some(replaced) = cache.index.insert(&shared.blocks, slot) {
            // Only indexed blocks are reached, which eviction relies on.
            self.detach(&mut cache, &[replaced as u32]);
        }
        // The fetch that compiled the block would take the lock again for
        // its block table entry right after; it takes it in this hold,
        // unless a direct exit waits to link it instead
        // ([`Self::remember_or_link`]).
        if self.publishing_target == Some(pc)
            && !self
                .direct_exit
                .is_some_and(|(_, _, target, _)| target == pc)
            && let Some((at, target)) = self.target_entry(pc, slot)
            && shared.blocks.record(slot).remember()
        {
            insert_target(&shared, &mut cache, pc, slot, at, target);
        }
        drop(cache);
        self.mirrors.close_code_page();
        Ok(Some(Fetched::Block(slot)))
    }

    /// Describe block `code` at `pc`, `bytes` long, to the host under step
    /// timing ([`super::Host::profile_code`]): its cold code, its direct
    /// exits with their link slots, the dynamic exit ending its terminal and
    /// the check failures after its exits; the rest is inline.
    fn profile_block(
        &mut self,
        code: NativeCode,
        pc: u64,
        bytes: &[u8],
        cold: (u32, u32),
        direct_exits: &[chain::DirectExit],
    ) {
        use step_timing::CodeRegion::{Cold, Exit, Link};
        let mut spans = Vec::with_capacity(2 * direct_exits.len() + 3);
        spans.push((cold.0, cold.1, Cold));
        let mut terminal_end = chain::without_failures(bytes).len();
        spans.push((terminal_end as u32, bytes.len() as u32, Exit));
        for exit in direct_exits {
            let slot = chain::link_slot(Some(pc), exit.target, exit.kind);
            let start = exit.offset - slot;
            spans.push((start as u32, exit.offset as u32, Link));
            let end = exit.offset + chain::DIRECT_EXIT_BYTES;
            spans.push((exit.offset as u32, end as u32, Exit));
            terminal_end = terminal_end.min(start);
        }
        // RET, BR and BLR end their terminal with an exit to RAX.
        if let Some(start) = terminal_end.checked_sub(chain::DYNAMIC_EXIT_BYTES)
            && chain::is_dynamic_exit_of_any_kind(&bytes[start..terminal_end])
        {
            spans.push((start as u32, terminal_end as u32, Exit));
        }
        let regions = step_timing::block_regions(bytes.len() as u32, &mut spans);
        self.host.profile_code(code, bytes.len(), &regions);
    }

    /// Install `bytes` of a block, reclaiming the arena's oldest part
    /// when it is full.
    fn install_code(&mut self, bytes: &[u8]) -> Result<NativeCode, Stop> {
        match self.install_native(bytes, true) {
            Ok(code) => Ok(code),
            Err(InstallError::Full) => {
                self.reclaim_code();
                self.install_native(bytes, true).map_err(|_| {
                    self.unsupported(RequirementKind::Native("block exceeds the code arena"))
                })
            }
        }
    }

    /// A slot for a new block, if the board has one.
    pub(super) fn take_slot(&self) -> Option<usize> {
        let (grace, skip) = self.grace();
        self.code.blocks.take(grace, skip)
    }

    /// A box holding `successors`: one an evicted block left of that
    /// length when there is one, so the steady install allocates none.
    pub(super) fn successor_box(&mut self, successors: &[Successor]) -> Box<[Successor]> {
        let kept = successors
            .len()
            .checked_sub(1)
            .and_then(|at| self.successor_boxes.get_mut(at))
            .and_then(Vec::pop);
        match kept {
            Some(mut boxed) => {
                boxed.copy_from_slice(successors);
                boxed
            }
            None => successors.into(),
        }
    }

    /// Keep a released block's `successors` box for a block of as many.
    pub(super) fn keep_successor_box(&mut self, successors: Box<[Successor]>) {
        if let Some(kept) = successors
            .len()
            .checked_sub(1)
            .and_then(|at| self.successor_boxes.get_mut(at))
            && kept.len() < KEPT_SUCCESSOR_BOXES
        {
            kept.push(successors);
        }
    }

    /// Count an install of `bytes` of code for `instructions` guest
    /// instructions of the block at `pc`, `physical` and `variant`, and
    /// with step timing on, why it was needed ([`install_census`]). A
    /// shared board keeps one census, under its lock (`cache`), and its
    /// blocks are never elsewhere.
    pub(super) fn census_installed(
        &mut self,
        cache: &mut board::Cache,
        pc: u64,
        physical: u64,
        variant: Variant,
        bytes: usize,
        instructions: usize,
    ) {
        let causes = &mut self.install_causes;
        causes.installs += 1;
        causes.installed_bytes += bytes as u64;
        causes.guest_instructions += instructions as u64;
        if pc >> 63 != 0 {
            causes.upper_bytes += bytes as u64;
            causes.upper_instructions += instructions as u64;
        }
        if self.step_timing.is_none() {
            self.census = None;
            return;
        }
        let identity = install_census::identity(pc, physical, &variant);
        let now = self.host.counter();
        let window = 10 * self.profile.counter_frequency;
        let shared = self.code_hub.is_some();
        let census = if shared {
            &mut cache.census
        } else {
            &mut self.census
        }
        .get_or_insert_with(|| Box::new(install_census::Census::new()));
        let (thrash, first) = census.installed(identity, now, window);
        let elsewhere = !shared
            && self
                .parallel
                .as_ref()
                .is_some_and(|hub| hub.installs.install(identity, self.current));
        let causes = &mut self.install_causes;
        causes.thrash += u64::from(thrash);
        causes.first += u64::from(first);
        causes.elsewhere += u64::from(elsewhere);
    }

    /// Count the eviction of block `index`'s code: its bytes, and with the
    /// census on, the block for a later install of it.
    /// Under the board's lock, `cache`.
    pub(super) fn census_evicted(
        &mut self,
        cache: &mut board::Cache,
        index: usize,
        bytes: usize,
        now: u64,
    ) {
        self.install_causes.evicted_bytes += bytes as u64;
        let shared = self.code_hub.is_some();
        let census = if shared {
            &mut cache.census
        } else {
            &mut self.census
        };
        let Some(census) = census else {
            return;
        };
        let block = &self.code.blocks[index];
        let variant = &self.variants[block.variant.0 as usize];
        let identity = install_census::identity(block.pc, block.physical, variant);
        census.evicted(identity, now);
        if !shared && let Some(hub) = &self.parallel {
            hub.installs.evict(identity, self.current);
        }
    }

    /// What this machine's installs and evictions came to.
    pub fn install_causes(&self) -> InstallCauses {
        self.install_causes
    }

    /// Redirect the PAC exit ending a block's `native` code, installed at
    /// `code`, through a thunk that completes the PAC instruction `word` at
    /// `pc` from the memo when it can (`x86_64::pointer`); the thunk's
    /// direct exit to the next instruction, if it has one.
    pub(super) fn complete_pointer_natively(
        &mut self,
        code: NativeCode,
        native: &[u8],
        pc: u64,
        word: u32,
    ) -> Option<(Option<Successor>, Option<u64>)> {
        let site = native.len().checked_sub(chain::DIRECT_EXIT_BYTES)?;
        let exit = ExitKind::PointerAuthentication;
        if !chain::is_direct_exit(&native[site..], GuestAddress(pc), exit) {
            return None;
        }
        let instruction = block::decode_pointer_authentication(GuestAddress(pc), word).ok()??;
        let thunk = match instruction.operation {
            PointerAuthenticationOperation::Strip { data, destination } => {
                x86_64_pointer::strip::thunk(GuestAddress(pc), destination, strip_place(data))?
            }
            operation => {
                let form = x86_64_pointer::Form::of(operation)?;
                x86_64_pointer::thunk(
                    form,
                    GuestAddress(pc),
                    TAGS,
                    SERVICE as usize as u64,
                    pointer::native_cipher(self.target),
                )
            }
        };
        self.install_stream = self.stream_at(code.0 as u64);
        let installed = self.install_native(&thunk.code, true).ok()?;
        self.profile_code(
            installed,
            thunk.code.len(),
            step_timing::CodeRegion::Pointer,
        );
        let site = code.0 as u64 + site as u64;
        // The block is not reachable yet: any bytes may change.
        let jump = chain::jump(site, installed.0 as u64)?;
        // Its part was chosen meanwhile: the instruction stays an exit.
        if !self.note_reference(site, installed.0 as u64) {
            return None;
        }
        self.patch_native(NativeCode(site as usize), &jump);
        let at = installed.0 as u64;
        let successor = thunk
            .next
            .filter(|&next| chain::exit_ready(&thunk.code, next, at, 0))
            .map(|next| {
                Successor::new(
                    at + next as u64,
                    GuestAddress(pc + INSTRUCTION),
                    ExitKind::Fallthrough.into(),
                    0,
                )
            });
        let dynamic = thunk
            .dynamic
            .filter(|&dynamic| chain::dynamic_ready(&thunk.code, dynamic, at))
            .map(|dynamic| at + dynamic as u64);
        Some((successor, dynamic))
    }

    /// Redirect the System exit ending a block's `native` code, installed at
    /// `code`, through a thunk that completes `word` at `pc`, if it is MRS
    /// of a [`MIRRORED`] register, from its cell in [`Self::mirrors`] while
    /// it is readable (`x86_64::register_read`); the thunk's direct exit to
    /// the next instruction. `before` instructions of the block precede it.
    pub(super) fn complete_mirrored_read_natively(
        &mut self,
        code: NativeCode,
        native: &[u8],
        pc: u64,
        word: u32,
        before: u32,
    ) -> Option<Successor> {
        let site = native.len().checked_sub(chain::DIRECT_EXIT_BYTES)?;
        if !chain::is_direct_exit(&native[site..], GuestAddress(pc), ExitKind::System) {
            return None;
        }
        let masks = register_read::MaskWrite::of(word).filter(|_| self.native_system().exclusive);
        let thunk = if let Some(write) = masks {
            let cell = cell_place(MASK_WRITES);
            register_read::mask_thunk(write, GuestAddress(pc), cell)
        } else if let Some(pan) = pan_write(word).filter(|_| self.host.pan_in_flags()) {
            let cell = cell_place(PAN_WRITES);
            register_read::pan_thunk(GuestAddress(pc), pan, cell)
        } else if let Some(send) = event_instruction(pc, word) {
            if send {
                register_read::send_event_thunk(GuestAddress(pc), EVENTS)
            } else {
                register_read::event_wait_thunk(GuestAddress(pc), EVENT_WAIT)
            }
        } else if data_barrier(pc, word) {
            register_read::barrier_thunk(GuestAddress(pc), BARRIER)
        } else if clear_exclusive(pc, word) {
            register_read::clear_exclusive_thunk(GuestAddress(pc), CLEAR_EXCLUSIVE)
        } else if let Some(rt) = code_page_invalidation(word).filter(|&rt| rt != ZERO_REGISTER) {
            let cell = cell_place(CODE_PAGE);
            register_read::code_page_thunk(GuestAddress(pc), rt, cell)
        } else if let Some(select) = stack_select(word) {
            let cell = cell_place(STACK_SELECT);
            let current = SELECTED;
            let sp0 = Place::new(Anchor::StackZero, 0);
            register_read::stack_select_thunk(GuestAddress(pc), select, cell, current, sp0)
        } else if let Some((index, rt)) = written_register(pc, word) {
            self.stored_write_thunk(GuestAddress(pc), index, rt)
        } else if let Some(rt) = table_base_write(pc, word) {
            let cell = cell_place(TABLE_BASE_KEPT);
            let written = self.register_fixed_bits(SystemRegister::TTBR0_EL1);
            register_read::unchanged_write_thunk(GuestAddress(pc), rt, cell, written)
        } else if let Some((kept, rt)) = permission_write_word(word) {
            let all = register_read::Written {
                keep: u64::MAX,
                set: 0,
            };
            register_read::unchanged_write_thunk(GuestAddress(pc), rt, cell_place(kept), all)
        } else if let Some((index, rt)) = guarded_written_register(pc, word) {
            let cell = cell_place(index);
            let all = register_read::Written {
                keep: u64::MAX,
                set: 0,
            };
            register_read::stored_write_thunk(GuestAddress(pc), rt, cell, all)
        } else if let Some((index, rt)) = mirrored_read(pc, word) {
            let cell = cell_place(index);
            if GUARDED_MIRRORS.contains(&index) {
                register_read::stored_thunk(GuestAddress(pc), rt, cell)
            } else if COUNT_MIRRORS.contains(&index) {
                self.clock.native(self.host.tsc_counter()?);
                let count = Place::new(Anchor::Count, 0);
                register_read::count_thunk(GuestAddress(pc), rt, cell, count)
            } else if index == COUNTER_MIRRORS[0]
                && let Some(tsc) = self.host.tsc_counter()
            {
                self.clock.native(tsc);
                let count = Place::new(Anchor::Count, 0);
                let plain = cell_place(CYCLES_PLAIN);
                register_read::cycle_counter_thunk(GuestAddress(pc), rt, cell, plain, CYCLES, count)
            } else if index == COUNTER_MIRRORS[1] {
                // Its block's instructions before it have retired, and the
                // chain counts the blocks native code left: what the
                // engine's exit here would retire before it reads.
                let plain = cell_place(INSTRUCTIONS_PLAIN);
                register_read::instruction_counter_thunk(
                    GuestAddress(pc),
                    rt,
                    cell,
                    plain,
                    INSTRUCTION_COUNTER,
                    before,
                )?
            } else if index == DAIF_MIRROR && self.native_system().exclusive {
                register_read::mask_read_thunk(GuestAddress(pc), rt, cell)
            } else {
                register_read::thunk(GuestAddress(pc), rt, cell)
            }
        } else if let Some(rt) = interrupt_status_read(word) {
            self.clock.native(self.host.tsc_counter()?);
            let epoch = core::mem::offset_of!(chain::ChainControl, interrupt_epoch);
            let (epoch, count) = (
                Place::new(Anchor::Control, epoch),
                Place::new(Anchor::Count, 0),
            );
            register_read::interrupt_status_thunk(
                GuestAddress(pc),
                rt,
                INTERRUPT_STATUS,
                epoch,
                count,
            )
        } else if let Some((register, rt)) = fast_ipi_write(pc, word) {
            use crate::machine::fast_ipi::Register;
            self.enter_fast_ipi()?;
            let pc = GuestAddress(pc);
            match register {
                Register::RequestLocal => {
                    register_read::fast_ipi_request_thunk(pc, rt, false, FAST_IPI)
                }
                Register::RequestGlobal => {
                    register_read::fast_ipi_request_thunk(pc, rt, true, FAST_IPI)
                }
                Register::Status => register_read::fast_ipi_status_thunk(pc, rt, FAST_IPI),
                Register::Countdown => return None,
            }
        } else if let Some((access, rt)) = timer_word(word) {
            self.clock.native(self.host.tsc_counter()?);
            let access = match access {
                TimerWord::Value => register_read::TimerAccess::WriteValue,
                TimerWord::Compare => register_read::TimerAccess::WriteCompare,
                TimerWord::ControlRead => register_read::TimerAccess::ReadControl,
                TimerWord::ControlWrite => register_read::TimerAccess::WriteControl(
                    self.register_fixed_bits(SystemRegister::CNTHV_CTL_EL2),
                ),
            };
            register_read::timer_thunk(GuestAddress(pc), rt, access, TIMER)
        } else if let Some(access) = mrs_access(pc, word) {
            self.stored_read_thunk(pc, access)?
        } else {
            let access = msr_access(pc, word)?;
            let (index, refused) = self.stored_write_cell(access.encoding())?;
            let cell = cell_place(stored_write_cell(index));
            let (pc, rt) = (GuestAddress(pc), access.rt());
            if let Some((_, bit)) = pstate_field(access.encoding()) {
                let written = register_read::Written { keep: bit, set: 0 };
                register_read::stored_write_thunk(pc, rt, cell, written)
            } else {
                register_read::checked_write_thunk(pc, rt, cell, refused)
            }
        };
        self.attach_thunk(code, site, pc, thunk)
    }

    /// Complete the plain device load `word` of the device site stub at
    /// `pc` and physical `site` natively while the read the engine kept
    /// there holds (`register_read::device_read_thunk`), on a host whose
    /// counter native code reads.
    fn complete_device_read_natively(
        &mut self,
        code: NativeCode,
        native: &[u8],
        pc: u64,
        word: u32,
        site: u64,
    ) -> Option<Successor> {
        let load = register_read::DeviceLoad::of(word)?;
        let at = native.len().checked_sub(chain::DIRECT_EXIT_BYTES)?;
        if !chain::is_direct_exit(&native[at..], GuestAddress(pc), ExitKind::System) {
            return None;
        }
        self.clock.native(self.host.tsc_counter()?);
        let thunk = register_read::device_read_thunk(GuestAddress(pc), site, load, DEVICE_READ);
        self.attach_thunk(code, at, pc, thunk)
    }

    /// Install `thunk` and jump to it from the System exit at `site` bytes
    /// into block code `code`, completing the instruction at `pc`: the
    /// successor its direct exit to the next instruction is, once ready.
    fn attach_thunk(
        &mut self,
        code: NativeCode,
        site: usize,
        pc: u64,
        thunk: x86_64_pointer::Thunk,
    ) -> Option<Successor> {
        self.install_stream = self.stream_at(code.0 as u64);
        let installed = self.install_native(&thunk.code, true).ok()?;
        self.profile_code(installed, thunk.code.len(), step_timing::CodeRegion::Other);
        let site = code.0 as u64 + site as u64;
        let jump = chain::jump(site, installed.0 as u64)?;
        // Its part was chosen meanwhile: the instruction stays an exit.
        if !self.note_reference(site, installed.0 as u64) {
            return None;
        }
        self.patch_native(NativeCode(site as usize), &jump);
        let at = installed.0 as u64;
        thunk
            .next
            .filter(|&next| chain::exit_ready(&thunk.code, next, at, 0))
            .map(|next| {
                Successor::new(
                    at + next as u64,
                    GuestAddress(pc + INSTRUCTION),
                    ExitKind::Fallthrough.into(),
                    0,
                )
            })
    }

    /// The thunk completing MSR of `WRITTEN[index]` from Xt at `pc`.
    pub(super) fn stored_write_thunk(
        &self,
        pc: GuestAddress,
        index: usize,
        rt: u8,
    ) -> x86_64_pointer::Thunk {
        let (_, register) = WRITTEN[index];
        let cell = cell_place(written_cell(index));
        let written = self.register_fixed_bits(register);
        if register == SystemRegister::FPSR {
            let offset = core::mem::offset_of!(GuestState, fpsr);
            return register_read::frame_write_thunk(pc, rt, cell, offset, written);
        }
        if register == SystemRegister::FPCR {
            // The fields this PE holds, as the engine's write keeps them.
            let written = register_read::Written {
                keep: u64::from(super::system::fpcr_fields(&self.cpu.registers)),
                set: 0,
            };
            let offset = core::mem::offset_of!(GuestState, fpcr);
            return register_read::frame_write_thunk(pc, rt, cell, offset, written);
        }
        if register == SystemRegister::SP_EL0 {
            let selected = SELECTED;
            return register_read::selected_write_thunk(pc, rt, cell, selected, written);
        }
        if index == THREAD_POINTER_WRITE {
            return register_read::thread_switch_thunk(pc, rt, cell, &THREAD_SWITCH);
        }
        if pointer_key(register) {
            let words = x86_64_pointer::TAG_WORDS;
            return register_read::key_write_thunk(pc, rt, cell, pointer::TAGS, words);
        }
        register_read::stored_write_thunk(pc, rt, cell, written)
    }

    /// Where a store completes MSR of `encoding` and the bits of a value that
    /// take a write more than that, if one without them takes no more: an
    /// Apple register's storage or a [`PSTATE_FIELDS`] word. PSTATE.DIT
    /// changes code compiled for a timing target, so its writes stay the
    /// engine's there.
    pub(super) fn write_storage(&self, encoding: Encoding) -> Option<(u64, u64)> {
        if let Some((index, _)) = pstate_field(encoding) {
            let timed = index == 0 && self.timing_target().is_some();
            return (!timed).then(|| (self.mirrors.field_storage(index), 0));
        }
        self.cpu.apple_write_storage(encoding)
    }

    /// The bits of `register` a write takes and those it holds.
    pub(super) fn register_fixed_bits(&self, register: SystemRegister) -> register_read::Written {
        let (res0, res1) = (
            self.cpu.registers.res0(register),
            self.cpu.registers.res1(register),
        );
        register_read::Written {
            keep: !(res0 | res1),
            set: res1,
        }
    }

    /// The cell of MRS of `encoding`, made if there is room. Its index is
    /// the board's ([`Self::stored_reads`]): code that names it reads the
    /// same register's cell on every PE.
    fn stored_cell(&mut self, encoding: Encoding) -> Option<usize> {
        let cell = self.stored_reads.find_or_add(encoding, || {
            // A performance counter's reads are stored while it is idle.
            let counter = self.cpu.pmu.as_ref().and_then(|pmu| pmu.counter(encoding));
            storable(encoding)
                || counter.is_some()
                || self.cpu.energy_accumulator(encoding)
                || self.cpu.apple_read_storage(encoding).is_some()
                || self
                    .devices
                    .register_storage(self.current, encoding)
                    .is_some()
        })?;
        Some(MIRRORED.len() + cell)
    }

    /// The index of `encoding` in [`Self::stored_writes`], made if there is
    /// room, and the bits of a value its writes leave to the engine.
    fn stored_write_cell(&mut self, encoding: Encoding) -> Option<(usize, u64)> {
        let (_, refused) = self.write_storage(encoding)?;
        let index = self.stored_writes.find_or_add(encoding, || true)?;
        Some((index, refused))
    }

    /// The thunk completing `access` at `pc` from its stored cell.
    fn stored_read_thunk(
        &mut self,
        pc: u64,
        access: RegisterAccess,
    ) -> Option<x86_64_pointer::Thunk> {
        use registers::encodings::{FPCR, FPSR, PAN};
        let encoding = access.encoding();
        let index = self.stored_cell(encoding)?;
        let cell = cell_place(index);
        let (pc, rt) = (GuestAddress(pc), access.rt());
        if encoding == STACK_ZERO {
            let selected = SELECTED;
            return Some(register_read::selected_stored_thunk(pc, rt, cell, selected));
        }
        if self.cpu.energy_accumulator(encoding) {
            self.clock.native(self.host.tsc_counter()?);
            let count = Place::new(Anchor::Count, 0);
            return Some(register_read::energy_thunk(pc, rt, cell, count));
        }
        Some(match encoding {
            FPCR => {
                register_read::frame_thunk(pc, rt, cell, core::mem::offset_of!(GuestState, fpcr))
            }
            FPSR => {
                register_read::frame_thunk(pc, rt, cell, core::mem::offset_of!(GuestState, fpsr))
            }
            PAN => register_read::pan_read_thunk(pc, rt, cell),
            _ => register_read::stored_thunk(pc, rt, cell),
        })
    }

    /// The target PSTATE.DIT code compiles for
    /// (`TargetCpu::data_independent_timing_target`). Without one,
    /// ID_AA64PFR0_EL1.DIT reads 0 (`identification::Basis::Timing`) and
    /// PSTATE.DIT changes no code. That code's atomics, ordered accesses and
    /// barriers are plain accesses, which only one running PE makes atomic:
    /// a machine whose PEs run concurrently (`enter_parallel`) never selects
    /// it, whatever its target.
    pub(super) fn timing_target(&self) -> Option<TargetCpu> {
        if self.parallel.is_some() || self.attached.is_some() {
            return None;
        }
        self.target.data_independent_timing_target()
    }

    pub(super) fn unsupported_word(
        &self,
        source: &[u8],
        base: u64,
        pc: u64,
        detail: String,
    ) -> Stop {
        let offset = (pc - base) as usize;
        let word = source.get(offset..offset + 4).map_or(0, |bytes| {
            u32::from_le_bytes(bytes.try_into().expect("four bytes"))
        });
        Stop::Unsupported(Requirement {
            pc,
            el: self.cpu.pstate.el,
            link: self.cpu.state.x[30],
            kind: RequirementKind::Instruction { word, detail },
            syndrome: Some(super::super::exception::syndrome(Class::Unknown, 0)),
            address: None,
            written: None,
        })
    }

    /// The word an exit at `pc` executes, counted in [`Self::exit_words`]:
    /// the block's last instruction, as it was compiled, or one it reads.
    pub(super) fn exit_word(&mut self, index: usize, pc: u64) -> Result<u32, Stop> {
        let block = &self.code.blocks[index];
        if pc == block.terminal_pc() {
            let terminal = block.terminal;
            let incarnation = self.code.blocks.record(index).incarnation();
            let pending = &mut self.exit_pending[index % PENDING_EXITS];
            if pending.count != 0
                && (pending.slot, pending.incarnation) == (index as u32, incarnation)
                && pending.count != u32::MAX
            {
                pending.count += 1;
            } else {
                let held = core::mem::replace(
                    pending,
                    PendingExits {
                        slot: index as u32,
                        incarnation,
                        word: terminal,
                        count: 1,
                    },
                );
                self.add_exits(held);
            }
            return Ok(terminal);
        }
        let word = self.word(index, pc)?;
        *self.exit_words.entry(word).or_default() += 1;
        Ok(word)
    }

    /// Add exits this PE kept ([`PendingExits`]) to their block's record,
    /// or by word when the slot no longer holds that block.
    fn add_exits(&mut self, held: PendingExits) {
        if held.count == 0 {
            return;
        }
        let record = self.code.blocks.record(held.slot as usize);
        if record.incarnation() == held.incarnation && record.live() {
            let _ = record
                .taken
                .try_update(Ordering::Relaxed, Ordering::Relaxed, |taken| {
                    (taken != u32::MAX).then(|| taken.saturating_add(held.count))
                });
        } else {
            *self.exit_words.entry(held.word).or_default() += u64::from(held.count);
        }
    }

    /// Add every exit this PE kept to the shared counts.
    pub(super) fn flush_exits(&mut self) {
        for at in 0..PENDING_EXITS {
            let held = core::mem::take(&mut self.exit_pending[at]);
            self.add_exits(held);
        }
    }

    /// Exits to the engine from compiled code (System, FP, PAC and AMX), by
    /// the instruction word that took them: which words to compile inline.
    pub fn exit_words(&self) -> BTreeMap<u32, u64> {
        let mut words = self.exit_words.clone();
        for held in &self.exit_pending {
            if held.count != 0 {
                *words.entry(held.word).or_default() += u64::from(held.count);
            }
        }
        for (slot, block) in self.code.blocks.iter() {
            let record = self.code.blocks.record(slot);
            let taken = record.taken.load(Ordering::Relaxed);
            if taken != 0 && record.live() {
                *words.entry(block.terminal).or_default() += u64::from(taken);
            }
        }
        words
    }

    /// Device accesses by the physical address they reached: which
    /// device registers to serve faster.
    pub fn system_words(&self) -> &BTreeMap<u32, DeviceRegister> {
        &self.system_words
    }

    pub fn device_registers(&self) -> &BTreeMap<u64, DeviceRegister> {
        &self.device_registers
    }

    /// Count a device access at `physical`, the register its native fault's
    /// handling is timed for.
    pub(super) fn count_device_access(&mut self, physical: u64) {
        self.statistics.device_accesses += 1;
        self.device_registers.entry(physical).or_default().accesses += 1;
        self.timed_device = Some(physical);
    }

    /// The word an instruction inside a compiled block executes as: its
    /// original word, or the UDF or NOP of a form the chip lacks. The
    /// block's last is kept as it was compiled; others are read again.
    pub(super) fn word(&mut self, index: usize, pc: u64) -> Result<u32, Stop> {
        let block = &self.code.blocks[index];
        if pc == block.terminal_pc() {
            return Ok(block.terminal);
        }
        let physical = block.physical + (pc - block.pc);
        let mut bytes = [0; 4];
        if !self.host.read_ram(physical, &mut bytes) {
            return Err(self.unsupported(RequirementKind::Fetch { physical }));
        }
        Ok(self.admit(u32::from_le_bytes(bytes)))
    }

    /// The word the profile's chip executes for `word` (`Chip::admit`).
    pub(super) fn admit(&self, word: u32) -> u32 {
        self.admit_table.admit(word)
    }

    /// A System exit of block `index` at `pc` met `word`, which is not
    /// System: the code that ran is not what the block records, or not
    /// what the PE's translation of `pc` names. Reported once per PE, with
    /// what tells which.
    fn report_exit_mismatch(&mut self, index: usize, pc: u64, word: u32) {
        if core::mem::replace(&mut self.exit_mismatch_reported, true) {
            return;
        }
        let block = &self.code.blocks[index];
        let record = self.code.blocks.record(index);
        let part = self.code.arena.as_ref().and_then(|arena| {
            let offset = block.code.0.checked_sub(arena.window.exec)?;
            Some(offset / arena.part_bytes)
        });
        let detail = alloc::format!(
            "System exit at a word that is not System: pe={} slot={index} part={part:?} \
             incarnation={} live={} installer={} stub={} site_here={} block_pc={:#x} \
             physical={:#x} instructions={} terminal={:#x} installed_key={:#x} \
             looked_up_key={:?} word={word:#x}",
            self.current,
            record.incarnation(),
            record.live(),
            block.installer,
            block.device_stub,
            self.device_site(block.physical.wrapping_add(pc.wrapping_sub(block.pc))),
            block.pc,
            block.physical,
            block.instructions,
            block.terminal,
            self.block_key(block.physical, block.variant),
            self.current_key(pc),
        );
        let requirement = self.requirement(RequirementKind::NativePreparation(detail));
        self.degrade(requirement);
    }

    pub(super) fn exit(&mut self, index: usize, next_pc: u64, kind: u64) -> Step {
        let kind = ExitKind::try_from(kind);
        // BTYPE belongs to the block's first instruction until it completes;
        // a branch ending the block gives the next one (Arm `BTypeNext`).
        if next_pc != self.code.blocks[index].pc {
            self.cpu.pstate.btype = 0;
        }
        match kind {
            Ok(
                ExitKind::Fallthrough
                | ExitKind::Direct
                | ExitKind::Indirect
                | ExitKind::Call
                | ExitKind::Return,
            ) => {
                // The block's last instruction is the branch that ends it.
                // What a side exit skipped has come off `retired` already.
                self.cpu.retired = self
                    .cpu
                    .retired
                    .wrapping_add(u64::from(self.code.blocks[index].instructions));
                self.cpu.pstate.btype = self.native_branch_type(index, kind)?;
                self.cpu.pc = next_pc;
                Ok(())
            }
            Ok(ExitKind::System) => {
                self.retire_before(index, next_pc);
                self.cpu.pc = next_pc;
                if self.device_stub(index, next_pc) {
                    return self.complete_device_site(index, next_pc);
                }
                self.statistics.system_exits += 1;
                let word = self.exit_word(index, next_pc)?;
                let operation = self.decode_system_operation(word);
                if operation.is_none() {
                    self.report_exit_mismatch(index, next_pc, word);
                }
                self.completing(next_pc, |machine| {
                    machine.complete_system_instruction(word, operation)
                })
            }
            Ok(ExitKind::FloatingPoint) => {
                self.retire_before(index, next_pc);
                self.cpu.pc = next_pc;
                let word = self.exit_word(index, next_pc)?;
                let Some(Ok(operation)) = FloatingPointOperation::decode(word) else {
                    return Err(self.unsupported(RequirementKind::Native(
                        "numerical exit at a word that is not numerical FP",
                    )));
                };
                match self.complete_floating_point(word, &operation) {
                    Ok(()) => self.completing(next_pc, Self::next),
                    Err(trapped) => self.completing(next_pc, |machine| {
                        machine.take_floating_point_exception(trapped)
                    }),
                }
            }
            Ok(ExitKind::PointerAuthentication) => {
                self.retire_before(index, next_pc);
                self.cpu.pc = next_pc;
                let word = self.exit_word(index, next_pc)?;
                self.completing(next_pc, |machine| machine.pointer_authentication(word))
            }
            Ok(ExitKind::Amx) => {
                self.retire_before(index, next_pc);
                self.cpu.pc = next_pc;
                let word = self.exit_word(index, next_pc)?;
                self.completing(next_pc, |machine| machine.amx(word))
            }
            _ => Err(self.unsupported(RequirementKind::Native("unknown block exit kind"))),
        }
    }
}

/// How [`Machine::take_page`] came to a page: counted as a fetch's ITLB
/// hit or miss, a batch refill ([`Machine::refill_pages`]) or a page of a
/// walked leaf ([`Machine::take_leaf_pages`]).
#[derive(Clone, Copy)]
enum Taken {
    Fetch,
    Refill,
    Leaf,
}

/// The span around a fetched page whose other pages one walk gives the
/// ITLB, when its leaf maps them all ([`Machine::take_leaf_pages`]): a
/// 16 KiB page, the granule XNU maps code with, in 4 KiB ITLB pages.
const LEAF_FILL: u64 = 1 << 14;

/// The most pages one forgotten context leaves to walk again
/// ([`Machine::refill_after`]). At 512 the cap bound: after each local TLBI
/// of everything a PE refilled about 510 pages and then missed about 600
/// more one exit at a time (run 20261115-112116, beat 40: 880 such TLBIs
/// per PE, `refilled` 427-495 K, `context` misses 430-566 K).
const REFILLS_PER_CONTEXT: usize = 2048;
/// The contexts whose pages TLBIs by VA dropped that a PE keeps for refill
/// ([`Machine::keep_dropped`]).
const DROPPED_CONTEXTS: usize = 64;

/// The bits of [`Machine::itlb_seen`]: 64 KiB, which tens of thousands of
/// pages a desktop's processes run fill to a few percent; it starts over
/// at a quarter.
const ITLB_SEEN_BITS: usize = 1 << 19;

/// The entries of [`Machine::itlb_ghosts`]: a direct-mapped table.
const ITLB_GHOSTS: usize = 1 << 12;
/// The low bits of a ghost's page that hold its [`ItlbMiss`] cause.
const ITLB_GHOST_CAUSE: u64 = 3;

/// The ghost slot of `page` in contexts of the salt of `context`: a
/// context and the forgotten ones it took the place of share it.
fn itlb_ghost_slot(page: u64, context: u64) -> usize {
    let salt = context % chain::LOOKUP_SALTS as u64;
    let mixed = ((page / FETCH_PAGE) ^ (salt << 40)).wrapping_mul(0x9e37_79b9_7f4a_7c15);
    (mixed >> (64 - ITLB_GHOSTS.trailing_zeros())) as usize
}

/// Note that the ITLB entry `entry` (a page in a context) went for `cause`,
/// for [`Machine::itlb_miss`] to tell when a fill takes the page again.
pub(super) fn note_itlb_ghost(
    ghosts: &mut Vec<(u64, u64)>,
    entry: &chain::LookupEntry,
    cause: ItlbMiss,
) {
    if ghosts.is_empty() {
        ghosts.resize(ITLB_GHOSTS, (0, 0));
    }
    ghosts[itlb_ghost_slot(entry.pc, entry.key)] = (entry.pc | cause as u64, entry.key);
}

/// The blocks whose exits at their last instruction a PE keeps
/// ([`Machine::exit_word`]), a direct-mapped table by slot.
pub(super) const PENDING_EXITS: usize = 16;

/// Exits at the last instruction of the block in `slot`, while it held the
/// block of `incarnation`, which ends with `word`, not yet added to the
/// block's record. A `count` of 0 is no entry.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub(super) struct PendingExits {
    pub(super) slot: u32,
    pub(super) incarnation: u32,
    pub(super) word: u32,
    pub(super) count: u32,
}
