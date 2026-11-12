use super::*;
use std::{boxed::Box, sync::Barrier, thread, vec::Vec};

#[repr(align(4096))]
struct Ram([u8; 128 * 1024]);

fn backing() -> Box<Ram> {
    Box::new(Ram([0; 128 * 1024]))
}

fn usage_values(heap: &Heap) -> [usize; 5] {
    let u = heap.usage();
    [u.total, u.live, u.carved, u.peak, u.largest]
}

#[test]
fn a_heap_whose_bins_are_nearly_empty_is_low_though_little_is_live() {
    let heap = Heap::new();
    let mut ram = backing();
    // SAFETY: exclusive test backing, kept through every exact free.
    unsafe { heap.initialize(ram.0.as_mut_ptr(), ram.0.len()) };
    let layout = Layout::from_size_align(4096, 16).unwrap();
    // SAFETY: valid layouts; the blocks return to their class, never to the
    // bins, which is what this test measures.
    unsafe {
        let blocks: Vec<*mut u8> = (0..31).map(|_| heap.alloc(layout)).collect();
        assert!(blocks.iter().all(|block| !block.is_null()));
        for block in blocks {
            heap.dealloc(block, layout);
        }
    }
    assert_eq!(heap.usage().live, 0);
    assert!(heap.low());
}

#[test]
fn available_page_frames_respect_headers_bounds_and_live_allocations() {
    let heap = Heap::new();
    let mut ram = backing();
    let start = ram.0.as_mut_ptr().addr();
    let end = start + ram.0.len();
    let whole = start..end;
    assert_eq!(heap.available_page_frames_in(whole.clone()), 0);
    // SAFETY: this test exclusively owns the backing until all releases.
    unsafe { heap.initialize(ram.0.as_mut_ptr(), ram.0.len()) };
    let layout = Layout::from_size_align(PAGE_BYTES, PAGE_BYTES).unwrap();
    let initial = ram.0.len() / PAGE_BYTES - 1;
    assert_eq!(heap.available_page_frames_in(whole.clone()), initial);
    assert_eq!(heap.available_page_frames_in(start..start + PAGE_BYTES), 0);
    assert_eq!(
        heap.available_page_frames_in(start + PAGE_BYTES + 1..start + 3 * PAGE_BYTES),
        1
    );
    // SAFETY: exact live heap and layout; the page is exclusively owned below.
    let page = unsafe { heap.alloc_pages_zeroed_in(layout, whole.clone()) };
    assert_eq!(page.addr(), start + PAGE_BYTES);
    assert_eq!(heap.available_page_frames_in(whole.clone()), initial - 2);
    // SAFETY: return the same allocation once with its original layout.
    unsafe { heap.dealloc(page, layout) };
    assert_eq!(heap.available_page_frames_in(whole.clone()), initial);

    heap.reserve_tail(4 * PAGE_BYTES);
    let reserve_start = end - 4 * PAGE_BYTES;
    assert_eq!(heap.available_page_frames_in(whole), initial - 1);
    assert_eq!(heap.available_page_frames_in(reserve_start..end), 3);
}

/// An 8 MiB heap whose last `reserve` MiB are its reserve, at the address
/// this returns.
fn reserving(heap: &Heap, reserve: usize) -> usize {
    let layout = Layout::from_size_align(8 << 20, 4096).unwrap();
    // SAFETY: a nonzero layout; the test leaks this backing.
    let ram = unsafe { std::alloc::alloc(layout) };
    // SAFETY: exclusive backing for the test's whole run.
    unsafe { heap.initialize(ram, layout.size()) };
    heap.reserve_tail(reserve << 20);
    ram as usize + ((8 - reserve) << 20)
}

#[test]
fn a_large_request_the_holes_cannot_serve_takes_the_reserve() {
    let heap = Heap::new();
    let floor = reserving(&heap, 2);
    let hole = Layout::from_size_align(256 << 10, 16).unwrap();
    let large = Layout::from_size_align(1 << 20, 16).unwrap();
    // SAFETY: valid layouts, each block freed once below.
    unsafe {
        let blocks: Vec<*mut u8> = (0..23).map(|_| heap.alloc(hole)).collect();
        assert!(blocks.iter().all(|&block| !block.is_null()));
        assert!(blocks.iter().all(|&block| (block as usize) < floor));
        for block in blocks.iter().step_by(2) {
            heap.dealloc(*block, hole);
        }
        let big = heap.alloc(large);
        assert!(big as usize >= floor);
        // The holes serve what fits them, not the reserve.
        let small = heap.alloc(hole);
        assert!((small as usize) < floor);
        heap.dealloc(small, hole);
        heap.dealloc(big, large);
        for block in blocks.iter().skip(1).step_by(2) {
            heap.dealloc(*block, hole);
        }
    }
    let usage = heap.usage();
    assert_eq!((usage.extents, usage.carved), (2, 0));
    assert_eq!(usage.largest, 6 << 20);
}

#[test]
fn a_full_main_part_is_low_and_small_blocks_then_take_the_reserve() {
    let heap = Heap::new();
    let floor = reserving(&heap, 2);
    let layout = Layout::from_size_align(256 << 10, 16).unwrap();
    let served: Vec<usize> = (0..24)
        // SAFETY: a valid layout, for blocks the test keeps.
        .map(|_| unsafe { heap.alloc(layout) } as usize)
        .collect();
    assert!(heap.low());
    assert!(served.iter().all(|&block| block != 0));
    assert!(served.iter().any(|&block| block >= floor));
}

#[test]
fn a_released_tail_leaves_the_heap_serving_from_what_remains() {
    let heap = Heap::new();
    let mut ram = backing();
    let start = ram.0.as_ptr() as usize;
    assert_eq!(heap.release_tail(0), None);
    // SAFETY: exclusive test backing, kept through every exact free.
    unsafe { heap.initialize(ram.0.as_mut_ptr(), ram.0.len()) };
    let layout = Layout::from_size_align(8192, 16).unwrap();
    // SAFETY: a valid layout, freed below.
    let first = unsafe { heap.alloc(layout) };
    assert!(!first.is_null());

    let released = heap.release_tail(16 * 1024).unwrap();
    assert!(released.start.is_multiple_of(PAGE_BYTES));
    assert_eq!(released.end, start + ram.0.len());
    assert_eq!(heap.usage().total, released.start - start);
    assert_eq!(heap.release_tail(1 << 20), None);

    // SAFETY: a valid layout, freed below.
    let second = unsafe { heap.alloc(layout) };
    assert!(!second.is_null() && (second as usize + layout.size()) <= released.start);
    // More than the kept bytes can hold is refused, not served past the end.
    let large = Layout::from_size_align(32 * 1024, 16).unwrap();
    // SAFETY: a valid layout that fails.
    assert!(unsafe { heap.alloc(large) }.is_null());
    // SAFETY: the blocks and exact layouts allocated above.
    unsafe {
        heap.dealloc(first, layout);
        heap.dealloc(second, layout);
    }
    assert_eq!(heap.usage().live, 0);
    // Everything coalesced back, up to the new end and not past it.
    assert_eq!(heap.usage().largest, released.start - start);
}

#[test]
fn attempted_unready_zero_and_contended_requests_preserve_state() {
    let heap = Heap::new();
    let mut ram = backing();
    let layouts = [
        Layout::from_size_align(17, 16).unwrap(),
        Layout::from_size_align(256, 128).unwrap(),
        Layout::from_size_align(4097, 4096).unwrap(),
    ];
    // SAFETY: these are valid layouts; unready allocation cannot access RAM.
    unsafe {
        for layout in layouts {
            assert!(heap.try_allocate(layout).is_null());
            assert!(heap.try_allocate_zeroed(layout).is_null());
        }
    }
    assert_eq!(usage_values(&heap), [0; 5]);
    // SAFETY: exclusive test backing, kept through every exact free.
    unsafe { heap.initialize(ram.0.as_mut_ptr(), ram.0.len()) };
    let before = usage_values(&heap);
    let image = ram.0.to_vec();
    let zero = Layout::from_size_align(0, 128).unwrap();
    // SAFETY: the new APIs explicitly reject zero without allocation.
    unsafe {
        assert!(heap.try_allocate(zero).is_null());
        assert!(heap.try_allocate_zeroed(zero).is_null());
    }
    assert_eq!(ram.0.as_slice(), image.as_slice());
    assert_eq!(usage_values(&heap), before);
    let guard = heap.lock();
    // A blocking or recursively acquired implementation cannot pass this.
    // SAFETY: the existing guard keeps exclusive metadata ownership;
    // attempted allocation must fail without accessing that metadata.
    unsafe {
        for layout in layouts {
            assert!(heap.try_allocate(layout).is_null());
            assert!(heap.try_allocate_zeroed(layout).is_null());
            assert!(heap.held.load(Ordering::Relaxed));
        }
    }
    assert_eq!(ram.0.as_slice(), image.as_slice());
    drop(guard);
    assert_eq!(usage_values(&heap), before);
}

#[test]
fn attempted_zeroed_reuse_keeps_other_live_payloads_and_global_free_rules() {
    let heap = Heap::new();
    let mut ram = backing();
    let layout = Layout::from_size_align(17, 16).unwrap();
    // SAFETY: fresh exclusive backing; every successful block is freed once
    // through this exact heap/layout while the backing remains live.
    unsafe {
        heap.initialize(ram.0.as_mut_ptr(), ram.0.len());
        let a = heap.alloc(layout);
        let b = heap.alloc(layout);
        assert!(!a.is_null() && !b.is_null() && a != b);
        a.write_bytes(0xa5, layout.size());
        b.write_bytes(0x5a, layout.size());
        heap.dealloc(a, layout);
        let tried = heap.try_allocate_zeroed(layout);
        assert_eq!(tried, a);
        assert!(
            core::slice::from_raw_parts(tried, layout.size())
                .iter()
                .all(|v| *v == 0)
        );
        assert!(
            core::slice::from_raw_parts(b, layout.size())
                .iter()
                .all(|v| *v == 0x5a)
        );
        assert_eq!(heap.usage().live, 64);
        heap.dealloc(tried, layout);
        let ordinary = heap.alloc(layout);
        assert_eq!(ordinary, tried);
        ordinary.write_bytes(0x7c, layout.size());
        heap.dealloc(ordinary, layout);
        let unzeroed = heap.try_allocate(layout);
        assert_eq!(unzeroed, ordinary);
        heap.dealloc(unzeroed, layout);
        heap.dealloc(b, layout);
    }
    assert_eq!(heap.usage().live, 0);
    // The existing small-object cache still keeps its own two blocks.
    assert!(heap.usage().carved > 0);
}

#[test]
fn attempted_general_alignment_zeroing_and_ordinary_free_coalesce() {
    let heap = Heap::new();
    let mut ram = backing();
    // SAFETY: all disjoint payloads keep this exclusive backing/layout.
    unsafe {
        heap.initialize(ram.0.as_mut_ptr(), ram.0.len());
        for (bytes, align) in [(256, 128), (4097, 4096), (8193, 32768)] {
            let layout = Layout::from_size_align(bytes, align).unwrap();
            let tried = heap.try_allocate_zeroed(layout);
            let other = heap.alloc(layout);
            assert!(!tried.is_null() && !other.is_null() && tried != other);
            assert!((tried as usize).is_multiple_of(align));
            assert!(
                core::slice::from_raw_parts(tried, bytes)
                    .iter()
                    .all(|v| *v == 0)
            );
            tried.write_bytes(0xa5, bytes);
            other.write_bytes(0x5a, bytes);
            heap.dealloc(tried, layout);
            let again = heap.try_allocate_zeroed(layout);
            assert!(!again.is_null() && (again as usize).is_multiple_of(align));
            assert!(
                core::slice::from_raw_parts(again, bytes)
                    .iter()
                    .all(|v| *v == 0)
            );
            assert!(
                core::slice::from_raw_parts(other, bytes)
                    .iter()
                    .all(|v| *v == 0x5a)
            );
            heap.dealloc(other, layout);
            heap.dealloc(again, layout);
            assert_eq!(heap.usage().live, 0);
            assert_eq!(heap.usage().carved, 0);
            assert_eq!(heap.usage().largest, ram.0.len());
        }
    }
}

#[test]
fn attempted_exhaustion_preserves_live_bytes_then_reclaims_the_full_extent() {
    let heap = Heap::new();
    let mut ram = backing();
    let layout = Layout::from_size_align(256, 128).unwrap();
    // SAFETY: each raw allocation keeps the same real backing and exact
    // layout; no payload slice survives subsequent allocation/deallocation.
    unsafe {
        heap.initialize(ram.0.as_mut_ptr(), ram.0.len());
        let mut blocks = Vec::new();
        loop {
            let block = heap.try_allocate(layout);
            if block.is_null() {
                break;
            }
            assert!((block as usize).is_multiple_of(128));
            block.write_bytes(0x39, layout.size());
            blocks.push(block);
        }
        assert!(blocks.len() > 32);
        let before = usage_values(&heap);
        let image = ram.0.to_vec();
        assert!(heap.try_allocate(layout).is_null());
        assert!(heap.try_allocate_zeroed(layout).is_null());
        assert!(heap.alloc(layout).is_null());
        assert_eq!(ram.0.as_slice(), image.as_slice());
        assert_eq!(usage_values(&heap), before);
        for block in blocks {
            assert!(
                core::slice::from_raw_parts(block, layout.size())
                    .iter()
                    .all(|v| *v == 0x39)
            );
            heap.dealloc(block, layout);
        }
    }
    assert_eq!(heap.usage().live, 0);
    assert_eq!(heap.usage().carved, 0);
    assert_eq!(heap.usage().largest, ram.0.len());
}

#[test]
fn aligned_extents_coalesce_after_exhaustion() {
    let heap = Heap::new();
    let mut ram = backing();
    let layout = Layout::from_size_align(4097, 4096).unwrap();
    // SAFETY: `ram` is this test's own backing, handed to the heap once and outliving every
    // block.
    unsafe {
        assert!(heap.alloc(layout).is_null());
        heap.initialize(ram.0.as_mut_ptr(), ram.0.len());
        let mut blocks = Vec::new();
        loop {
            let block = heap.alloc(layout);
            if block.is_null() {
                break;
            }
            assert!((block as usize).is_multiple_of(4096));
            blocks.push(block);
        }
        assert!(blocks.len() > 8);
        // Interleave holes to exercise coalescing on both sides.
        for block in blocks.iter().step_by(2) {
            heap.dealloc(*block, layout);
        }
        for block in blocks.iter().skip(1).step_by(2) {
            heap.dealloc(*block, layout);
        }
    }
    let usage = heap.usage();
    assert_eq!(usage.live, 0);
    assert_eq!(usage.carved, 0);
    assert_eq!(usage.largest, ram.0.len());
    assert!(usage.peak > 0);
}

#[test]
fn size_class_reuse_preserves_other_live_bytes() {
    let heap = Heap::new();
    let mut ram = backing();
    let layout = Layout::from_size_align(17, 16).unwrap();
    // SAFETY: `ram` is this test's own backing, handed to the heap once and outliving every
    // block.
    unsafe {
        heap.initialize(ram.0.as_mut_ptr(), ram.0.len());
        let a = heap.alloc(layout);
        let b = heap.alloc(layout);
        assert!(!a.is_null() && !b.is_null() && a != b);
        b.write_bytes(0x5a, layout.size());
        heap.dealloc(a, layout);
        let reused = heap.alloc(layout);
        assert_eq!(reused, a);
        reused.write_bytes(0xa5, layout.size());
        assert!(
            core::slice::from_raw_parts(b, layout.size())
                .iter()
                .all(|v| *v == 0x5a)
        );
        heap.dealloc(reused, layout);
        heap.dealloc(b, layout);
    }
    assert_eq!(heap.usage().live, 0);
}

#[test]
fn resizing_within_a_size_class_keeps_the_allocation() {
    let heap = Heap::new();
    let mut ram = backing();
    let layout = Layout::from_size_align(50, 16).unwrap();
    let smaller = Layout::from_size_align(34, 16).unwrap();
    let larger = Layout::from_size_align(62, 16).unwrap();
    let next_class = Layout::from_size_align(65, 16).unwrap();
    // SAFETY: exclusive backing, valid nonzero layouts, and each resize
    // receives the immediately preceding live allocation and layout.
    unsafe {
        heap.initialize(ram.0.as_mut_ptr(), ram.0.len());
        let before = heap.alloc(layout);
        assert!(!before.is_null());
        before.write_bytes(0x5a, layout.size());
        let usage = usage_values(&heap);
        let after = heap.realloc(before, layout, larger.size());
        assert_eq!(after, before);
        assert_eq!(usage_values(&heap), usage);
        let shrunk = heap.realloc(after, larger, smaller.size());
        assert_eq!(shrunk, after);
        assert_eq!(usage_values(&heap), usage);
        let moved = heap.realloc(shrunk, smaller, next_class.size());
        assert!(!moved.is_null() && moved != shrunk);
        assert_eq!(
            core::slice::from_raw_parts(moved, smaller.size()),
            &[0x5a; 34]
        );
        heap.dealloc(moved, next_class);
    }
    assert_eq!(heap.usage().live, 0);
}

#[test]
fn large_resize_uses_the_right_extent_and_preserves_live_neighbours() {
    let heap = Heap::new();
    let mut ram = backing();
    let layout = |kib: usize| Layout::from_size_align(kib * 1024, 16).unwrap();
    let (original, gap, guard) = (layout(28), layout(68), layout(28));
    let (grown, shrunk, reused) = (layout(90), layout(20), layout(70));
    // SAFETY: the backing stays exclusive, and each operation uses a live
    // allocation with the layout most recently assigned to it.
    unsafe {
        heap.initialize(ram.0.as_mut_ptr(), ram.0.len());
        let block = heap.alloc(original);
        let neighbour = heap.alloc(gap);
        let sentinel = heap.alloc(guard);
        assert!(!block.is_null() && !neighbour.is_null() && !sentinel.is_null());
        block.write_bytes(0x5a, original.size());
        sentinel.write_bytes(0xa5, guard.size());
        heap.dealloc(neighbour, gap);

        let before = heap.usage();
        assert!(before.largest < grown.size());
        let larger = heap.realloc(block, original, grown.size());
        assert_eq!(larger, block);
        let after = heap.usage();
        assert_eq!(after.carves, before.carves);
        assert_eq!(after.live, grown.size() + guard.size());
        assert_eq!(after.large[large_bucket(grown.size())], (1, grown.size()));
        assert!(
            core::slice::from_raw_parts(larger, original.size())
                .iter()
                .all(|&byte| byte == 0x5a)
        );

        let smaller = heap.realloc(larger, grown, shrunk.size());
        assert_eq!(smaller, larger);
        assert_eq!(heap.usage().live, shrunk.size() + guard.size());
        let reuse = heap.alloc(reused);
        assert!(!reuse.is_null());
        assert!(reuse.addr() > smaller.addr() && reuse.addr() < sentinel.addr());
        reuse.write_bytes(0xc3, reused.size());
        assert!(
            core::slice::from_raw_parts(smaller, shrunk.size())
                .iter()
                .all(|&byte| byte == 0x5a)
        );
        assert!(
            core::slice::from_raw_parts(sentinel, guard.size())
                .iter()
                .all(|&byte| byte == 0xa5)
        );
        heap.dealloc(smaller, shrunk);
        heap.dealloc(sentinel, guard);
        heap.dealloc(reuse, reused);
    }
    let after = heap.usage();
    assert_eq!((after.live, after.carved, after.extents), (0, 0, 1));
    assert_eq!(after.largest, ram.0.len());
}

#[test]
fn failed_large_growth_preserves_the_owner_and_the_reserve_boundary() {
    let heap = Heap::new();
    let mut ram = backing();
    let reserve = 32 * 1024;
    let original = Layout::from_size_align(ram.0.len() - reserve - HEADER_BYTES, 16).unwrap();
    // SAFETY: exclusive backing, with one main-pool allocation held through
    // the failed growth and returned using its unchanged layout.
    unsafe {
        heap.initialize(ram.0.as_mut_ptr(), ram.0.len());
        heap.reserve_tail(reserve);
        let block = heap.alloc(original);
        assert!(!block.is_null());
        block.write_bytes(0x96, original.size());
        let before = usage_values(&heap);
        assert!(
            heap.realloc(block, original, original.size() + GRANULE)
                .is_null()
        );
        assert_eq!(usage_values(&heap), before);
        assert!(
            core::slice::from_raw_parts(block, original.size())
                .iter()
                .all(|&byte| byte == 0x96)
        );
        heap.dealloc(block, original);
    }
    let after = heap.usage();
    assert_eq!((after.live, after.carved, after.extents), (0, 0, 2));
}

#[test]
fn the_class_lists_return_to_the_bins_when_a_request_needs_them() {
    let mut ram = backing();
    let heap = Heap::new();
    // SAFETY: exclusive test backing, kept through every exact free.
    unsafe { heap.initialize(ram.0.as_mut_ptr(), ram.0.len()) };
    let small = Layout::from_size_align(64, 16).unwrap();
    let mut blocks = Vec::new();
    loop {
        // SAFETY: a nonzero layout on an initialized heap.
        let block = unsafe { heap.try_allocate(small) };
        if block.is_null() {
            break;
        }
        blocks.push(block);
    }
    for block in blocks.drain(..) {
        // SAFETY: each is a live allocation of `small`.
        unsafe { heap.dealloc(block, small) };
    }
    // Every byte sits on the class list; the bins alone have no room.
    let large = Layout::from_size_align(ram.0.len() / 2, 16).unwrap();
    // SAFETY: a nonzero layout on an initialized heap.
    let block = unsafe { heap.try_allocate(large) };
    assert!(!block.is_null());
    // SAFETY: the live allocation just made.
    unsafe { heap.dealloc(block, large) };
}

#[test]
fn a_release_between_free_neighbours_merges_all_three() {
    let heap = Heap::new();
    let mut ram = backing();
    let layout = Layout::from_size_align(5000, 16).unwrap();
    // SAFETY: `ram` is this test's own backing, handed to the heap once and
    // outliving every block.
    unsafe {
        heap.initialize(ram.0.as_mut_ptr(), ram.0.len());
        let [a, b, c] = [(); 3].map(|()| heap.alloc(layout));
        assert!(!a.is_null() && !b.is_null() && !c.is_null());
        assert_eq!(heap.usage().extents, 1);
        heap.dealloc(a, layout);
        assert_eq!(heap.usage().extents, 2);
        heap.dealloc(c, layout);
        assert_eq!(heap.usage().extents, 2, "c joins the rest");
        heap.dealloc(b, layout);
    }
    let usage = heap.usage();
    assert_eq!((usage.extents, usage.carved), (1, 0));
    assert_eq!(usage.largest, ram.0.len());
}

#[test]
fn an_exact_fit_below_its_rounded_bin_is_found() {
    let heap = Heap::new();
    let mut ram = backing();
    // A 4176-byte block, between the bins' 256-byte steps above 4 KiB.
    let hole = Layout::from_size_align(4160, 16).unwrap();
    let guard = Layout::from_size_align(4112, 16).unwrap();
    let rest = Layout::from_size_align(ram.0.len() - 4176 - 4128 - 16, 16).unwrap();
    // SAFETY: `ram` is this test's own backing, handed to the heap once and
    // outliving every block.
    unsafe {
        heap.initialize(ram.0.as_mut_ptr(), ram.0.len());
        let a = heap.alloc(hole);
        let b = heap.alloc(guard);
        let c = heap.alloc(rest);
        assert!(!a.is_null() && !b.is_null() && !c.is_null());
        assert_eq!(heap.usage().extents, 0);
        heap.dealloc(a, hole);
        let again = heap.alloc(hole);
        assert_eq!(again, a);
        for (block, layout) in [(again, hole), (b, guard), (c, rest)] {
            heap.dealloc(block, layout);
        }
    }
    assert_eq!(heap.usage().largest, ram.0.len());
}

#[test]
fn constrained_pages_use_real_cpu_interval_and_exact_payload_end() {
    let heap = Heap::new();
    let mut ram = backing();
    let base = ram.0.as_mut_ptr() as usize;
    let page = Layout::from_size_align(PAGE_BYTES, PAGE_BYTES).unwrap();
    // SAFETY: this dedicated real RAM stays live through exact deallocation.
    unsafe {
        assert!(
            heap.alloc_pages_zeroed_in(page, base..base + ram.0.len())
                .is_null()
        );
        heap.initialize(base as *mut u8, ram.0.len());
        let window = base + 7 * PAGE_BYTES..base + 8 * PAGE_BYTES;
        let block = heap.alloc_pages_zeroed_in(page, window.clone());
        assert_eq!(block as usize, window.start);
        assert_eq!(block as usize + page.size(), window.end);
        assert!(
            core::slice::from_raw_parts(block, page.size())
                .iter()
                .all(|byte| *byte == 0)
        );
        block.write_bytes(0xa5, page.size());
        assert_eq!(heap.usage().live, page.size());
        heap.dealloc(block, page);
        let again = heap.alloc_pages_zeroed_in(page, window);
        assert_eq!(again, block);
        assert!(
            core::slice::from_raw_parts(again, page.size())
                .iter()
                .all(|byte| *byte == 0)
        );
        heap.dealloc(again, page);
    }
    let usage = heap.usage();
    assert_eq!(usage.live, 0);
    assert_eq!(usage.carved, 0);
    assert_eq!(usage.largest, ram.0.len());
}

#[test]
fn constrained_failure_preserves_all_free_bytes_and_accounting() {
    let heap = Heap::new();
    let mut ram = backing();
    let base = ram.0.as_mut_ptr() as usize;
    // SAFETY: fresh exclusive model RAM; no users before initialization.
    unsafe { heap.initialize(base as *mut u8, ram.0.len()) };
    let page = Layout::from_size_align(PAGE_BYTES, PAGE_BYTES).unwrap();
    let end = base + ram.0.len();
    for (layout, allowed) in [
        (page, base..base),
        (page, end..base),
        (page, base..base + PAGE_BYTES), // no space for canonical Header
        (page, base + PAGE_BYTES..base + 2 * PAGE_BYTES - 1),
        (page, base + PAGE_BYTES + 1..base + 2 * PAGE_BYTES),
        (page, end..end + PAGE_BYTES),
        (Layout::from_size_align(0, PAGE_BYTES).unwrap(), base..end),
        (
            Layout::from_size_align(PAGE_BYTES - 1, PAGE_BYTES).unwrap(),
            base..end,
        ),
        (Layout::from_size_align(PAGE_BYTES, 16).unwrap(), base..end),
    ] {
        let image = ram.0.to_vec();
        let before = heap.usage();
        // SAFETY: valid layout, kept real heap, no other consumers.
        let block = unsafe { heap.alloc_pages_zeroed_in(layout, allowed) };
        assert!(block.is_null());
        assert_eq!(ram.0.as_slice(), image.as_slice());
        let after = heap.usage();
        assert_eq!(
            (after.live, after.carved, after.peak, after.largest),
            (before.live, before.carved, before.peak, before.largest)
        );
    }
}

/// A remainder too small for any block stays in the allocation instead
/// of becoming a free extent that every later walk passes.
#[test]
fn remainders_too_small_for_a_block_stay_allocated() {
    let heap = Heap::new();
    let mut ram = backing();
    let header = core::mem::size_of::<Header>();
    let layout = Layout::from_size_align(ram.0.len() - header - 24, align_of::<usize>()).unwrap();
    // SAFETY: the one payload is freed with its layout before `ram` goes.
    unsafe {
        heap.initialize(ram.0.as_mut_ptr(), ram.0.len());
        let block = heap.alloc(layout);
        assert!(!block.is_null());
        assert_eq!(heap.usage().carved, ram.0.len());
        heap.dealloc(block, layout);
    }
    assert_eq!(heap.usage().carved, 0);
    assert_eq!(heap.usage().largest, ram.0.len());
}

#[test]
fn fragmented_selection_skips_earlier_extents_and_preserves_other_owners() {
    let heap = Heap::new();
    let mut ram = backing();
    let base = ram.0.as_mut_ptr() as usize;
    let page = Layout::from_size_align(PAGE_BYTES, PAGE_BYTES).unwrap();
    let small = Layout::from_size_align(17, 16).unwrap();
    // SAFETY: each raw payload is held until its exact deallocation;
    // no slice or device alias exists while the heap mutates its metadata.
    unsafe {
        heap.initialize(base as *mut u8, ram.0.len());
        let a = heap.alloc(page);
        let b = heap.alloc(page);
        let c = heap.alloc(page);
        assert!(!a.is_null() && !b.is_null() && !c.is_null());
        b.write_bytes(0x39, page.size());
        heap.dealloc(a, page);
        heap.dealloc(c, page);
        // An earlier free extent is large enough but outside this request.
        let chosen = heap.alloc_pages_zeroed_in(page, c as usize..c as usize + PAGE_BYTES);
        assert_eq!(chosen, c);
        let metadata = heap.alloc(small);
        assert!(!metadata.is_null());
        metadata.write_bytes(0x6e, small.size());
        assert!(
            core::slice::from_raw_parts(b, page.size())
                .iter()
                .all(|byte| *byte == 0x39)
        );
        heap.dealloc(chosen, page);
        assert!(
            core::slice::from_raw_parts(metadata, small.size())
                .iter()
                .all(|byte| *byte == 0x6e)
        );
        heap.dealloc(metadata, small);
        heap.dealloc(b, page);
    }
    // The ordinary small-object cache keeps its own block, as before.
    assert_eq!(heap.usage().live, 0);
    assert!(heap.usage().carved < 2 * PAGE_BYTES);
}

#[test]
fn constrained_alignment_and_ordinary_pages_coalesce_together() {
    let heap = Heap::new();
    let mut ram = backing();
    let base = ram.0.as_mut_ptr() as usize;
    let ordinary = Layout::from_size_align(PAGE_BYTES, PAGE_BYTES).unwrap();
    // SAFETY: exclusive fixed test RAM and stopped payload consumers.
    unsafe {
        heap.initialize(base as *mut u8, ram.0.len());
        for alignment in [PAGE_BYTES, 2 * PAGE_BYTES, 4 * PAGE_BYTES, 8 * PAGE_BYTES] {
            let layout = Layout::from_size_align(3 * PAGE_BYTES, alignment).unwrap();
            let other = heap.alloc(ordinary);
            assert!(!other.is_null());
            other.write_bytes(0x7c, ordinary.size());
            let block =
                heap.alloc_pages_zeroed_in(layout, base + PAGE_BYTES + 7..base + ram.0.len());
            assert!(!block.is_null() && (block as usize).is_multiple_of(alignment));
            assert!(block as usize + layout.size() <= base + ram.0.len());
            assert!(
                core::slice::from_raw_parts(other, ordinary.size())
                    .iter()
                    .all(|byte| *byte == 0x7c)
            );
            heap.dealloc(other, ordinary);
            heap.dealloc(block, layout);
            assert_eq!(heap.usage().live, 0);
            assert_eq!(heap.usage().carved, 0);
            assert_eq!(heap.usage().largest, ram.0.len());
        }
    }
}

/// A core's cache and counts share no cache line with another core's.
#[test]
fn each_cores_writes_stay_in_lines_of_its_own() {
    const LINE: usize = 64;
    assert_eq!(core::mem::align_of::<Core>() % LINE, 0);
    assert_eq!(core::mem::size_of::<Core>() % LINE, 0);
    let heap = Heap::new();
    let at = |cpu: usize| &raw const heap.cores[cpu] as usize;
    assert_eq!(at(0) % LINE, 0);
    assert_eq!(at(1) - at(0), core::mem::size_of::<Core>());
}

#[test]
fn core_caches_move_blocks_between_cores_in_batches() {
    let heap = Heap::new();
    let mut ram = backing();
    let layout = Layout::from_size_align(24, 8).unwrap();
    let class = class(layout).unwrap();
    heap.tagged.store(2, Ordering::Relaxed);
    // SAFETY: exclusive test backing; no interrupts reach these calls,
    // and every block goes back to a cache of its class.
    unsafe {
        heap.initialize(ram.0.as_mut_ptr(), ram.0.len());
        let blocks: Vec<_> = (0..CACHED + BATCH)
            .map(|_| heap.take_cached(0, class))
            .collect();
        assert!(blocks.iter().all(|block| !block.is_null()));
        let mut distinct = blocks.clone();
        distinct.sort();
        distinct.dedup();
        assert_eq!(distinct.len(), blocks.len());
        assert_eq!(heap.usage().live, blocks.len() * 32);
        // Freed on another core than allocated them: its cache fills
        // and moves a batch to the class list.
        for &block in &blocks {
            heap.give_cached(1, class, block);
        }
        assert_eq!(heap.usage().live, 0);
        assert_eq!((*heap.cores[1].cache.get()).counts[class], CACHED);
        // The other core's allocations reuse the blocks the list holds.
        let reused = heap.alloc(layout);
        assert!(blocks.contains(&reused));
        heap.dealloc(reused, layout);
    }
    assert_eq!(heap.usage().live, 0);
}

#[test]
fn concurrent_allocations_preserve_data_and_reclaim_live_bytes() {
    let heap = Heap::new();
    let mut ram = backing();
    // SAFETY: `ram` is this test's own backing, handed to the heap once and outliving every
    // block.
    unsafe { heap.initialize(ram.0.as_mut_ptr(), ram.0.len()) };
    let barrier = Barrier::new(8);
    thread::scope(|scope| {
        for id in 1..=8 {
            let heap = &heap;
            let barrier = &barrier;
            scope.spawn(move || {
                let layout = Layout::from_size_align(257, 64).unwrap();
                for _ in 0..64 {
                    // SAFETY: `ram` is this test's own backing, handed to the heap once and
                    // outliving every block.
                    unsafe {
                        let block = heap.alloc(layout);
                        assert!(!block.is_null() && (block as usize).is_multiple_of(64));
                        block.write_bytes(id, layout.size());
                        barrier.wait();
                        assert!(
                            core::slice::from_raw_parts(block, layout.size())
                                .iter()
                                .all(|byte| *byte == id)
                        );
                        heap.dealloc(block, layout);
                    }
                }
            });
        }
    });
    assert_eq!(heap.usage().live, 0);
    assert_eq!(heap.usage().carved, 0);
    assert_eq!(heap.usage().largest, ram.0.len());
}

/// The range of the heap the large-hook test watches, and what its hook
/// heard there: live tracked bytes and the calls.
static HOOKED_RANGE: [AtomicUsize; 2] = [AtomicUsize::new(0), AtomicUsize::new(0)];
static HOOKED_LIVE: AtomicUsize = AtomicUsize::new(0);
static HOOKED_CALLS: AtomicUsize = AtomicUsize::new(0);

fn hooked(pointer: usize, bytes: usize, allocated: bool) {
    let range = HOOKED_RANGE[0].load(Ordering::Relaxed)..HOOKED_RANGE[1].load(Ordering::Relaxed);
    if !range.contains(&pointer) {
        return;
    }
    HOOKED_CALLS.fetch_add(1, Ordering::Relaxed);
    if allocated {
        HOOKED_LIVE.fetch_add(bytes, Ordering::Relaxed);
    } else {
        HOOKED_LIVE.fetch_sub(bytes, Ordering::Relaxed);
    }
}

#[test]
fn the_large_hook_hears_every_tracked_block_made_resized_and_freed() {
    const BYTES: usize = 32 << 20;
    let mut ram = std::vec![0u8; BYTES + 4096];
    let start = ram.as_mut_ptr().addr().next_multiple_of(4096);
    HOOKED_RANGE[0].store(start, Ordering::Relaxed);
    HOOKED_RANGE[1].store(start + BYTES, Ordering::Relaxed);
    set_large_hook(hooked);
    let heap = Heap::new();
    // SAFETY: exclusive test backing, kept through every exact free.
    unsafe { heap.initialize(start as *mut u8, BYTES) };
    let large = Layout::from_size_align(TRACKED_BYTES, 16).unwrap();
    let below = Layout::from_size_align(TRACKED_BYTES - 16, 16).unwrap();
    // SAFETY: valid layouts, each block freed once with its layout.
    unsafe {
        let small = heap.alloc(below);
        assert_eq!(HOOKED_CALLS.load(Ordering::Relaxed), 0);
        let block = heap.alloc(large);
        assert_eq!(HOOKED_LIVE.load(Ordering::Relaxed), TRACKED_BYTES);
        let grown = heap.realloc(block, large, 2 * TRACKED_BYTES);
        assert!(!grown.is_null());
        assert_eq!(HOOKED_LIVE.load(Ordering::Relaxed), 2 * TRACKED_BYTES);
        let marked = heap.mark();
        let peak_block = heap.alloc(large);
        assert!(heap.marked_peak() >= marked + TRACKED_BYTES);
        heap.dealloc(peak_block, large);
        assert!(heap.marked_peak() >= marked + TRACKED_BYTES);
        assert_eq!(heap.mark(), marked);
        assert_eq!(heap.marked_peak(), marked);
        heap.dealloc(
            grown,
            Layout::from_size_align(2 * TRACKED_BYTES, 16).unwrap(),
        );
        heap.dealloc(small, below);
    }
    assert_eq!(HOOKED_LIVE.load(Ordering::Relaxed), 0);
    assert!(HOOKED_CALLS.load(Ordering::Relaxed) >= 5);
}

#[test]
fn the_room_is_what_no_block_was_carved_from_and_comes_back_when_freed() {
    let heap = Heap::new();
    let mut ram = backing();
    // SAFETY: exclusive test backing, kept through every exact free.
    unsafe { heap.initialize(ram.0.as_mut_ptr(), ram.0.len()) };
    heap.reserve_tail(ram.0.len() / 16);
    let total = heap.total();
    assert_eq!(total, ram.0.len());
    assert_eq!(heap.room(), total);
    // Fill the heap with large blocks: the room shrinks by each, into the
    // reserve, down to nothing a block fits.
    let layout = Layout::from_size_align(8192, 16).unwrap();
    let mut blocks = Vec::new();
    loop {
        let before = heap.room();
        // SAFETY: a valid layout; every block is freed below.
        let block = unsafe { heap.alloc(layout) };
        if block.is_null() {
            break;
        }
        blocks.push(block);
        assert!(
            heap.room() <= before - layout.size(),
            "{} of {before}",
            heap.room()
        );
    }
    assert!(blocks.len() >= 14, "{}", blocks.len());
    assert!(heap.room() < 2 * layout.size(), "{}", heap.room());
    // Small blocks freed stay carved in their class; large ones return.
    for block in blocks {
        // SAFETY: each block once, with its layout.
        unsafe { heap.dealloc(block, layout) };
    }
    assert_eq!(heap.room(), total);
}

#[test]
fn owners_count_their_blocks_until_freed_wherever_neighbours_merge() {
    let heap = Heap::new();
    let mut ram = backing();
    // SAFETY: exclusive test backing, kept through every exact free.
    unsafe { heap.initialize(ram.0.as_mut_ptr(), ram.0.len()) };
    let large = Layout::from_size_align(8192, 16).unwrap();
    let small = Layout::from_size_align(48, 8).unwrap();
    // SAFETY: valid layouts; each block is freed once with its layout.
    unsafe {
        heap.set_owner(3);
        let before = heap.alloc(large);
        heap.set_owner(5);
        let middle = heap.alloc(large);
        let little = heap.alloc(small);
        heap.set_owner(0);
        let after = heap.alloc(large);
        let mut owners = [0; OWNERS];
        owners[3] = 8192;
        owners[5] = 8192 + 64;
        assert_eq!(heap.owners(), owners);
        // Freeing the neighbours rewrites the middle block's `previous`
        // and merges around it: its owner stays.
        heap.dealloc(before, large);
        heap.dealloc(after, large);
        owners[3] = 0;
        assert_eq!(heap.owners(), owners);
        // A block grown in place keeps its owner and counts its new size.
        let grown = heap.realloc(middle, large, 12288);
        assert_eq!(grown, middle);
        owners[5] = 12288 + 64;
        assert_eq!(heap.owners(), owners);
        heap.dealloc(grown, Layout::from_size_align(12288, 16).unwrap());
        // A size-class block reused under another owner counts there.
        heap.dealloc(little, small);
        heap.set_owner(7);
        let reused = heap.alloc(small);
        assert_eq!(reused, little);
        owners[5] = 0;
        owners[7] = 64;
        assert_eq!(heap.owners(), owners);
        heap.dealloc(reused, small);
    }
    assert_eq!(heap.owners(), [0; OWNERS]);
    assert_eq!(heap.usage().live, 0);
}
