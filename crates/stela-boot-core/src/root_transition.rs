//! Allocation-free physical-backing equivalence for an owned root transition.
//! Metadata equality alone never grants native memory or publication permission.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Error {
    EmptyRange,
    Overflow,
    Overlap,
}
fn validate(ranges: impl Iterator<Item = (u64, u64)> + Clone) -> Result<(), Error> {
    for (index, (start, bytes)) in ranges.clone().enumerate() {
        if bytes == 0 {
            return Err(Error::EmptyRange);
        }
        let end = start.checked_add(bytes).ok_or(Error::Overflow)?;
        for (other, size) in ranges.clone().take(index) {
            let top = other.checked_add(size).ok_or(Error::Overflow)?;
            if start < top && other < end {
                return Err(Error::Overlap);
            }
        }
    }
    Ok(())
}
fn covers(start: u64, bytes: u64, parts: impl Iterator<Item = (u64, u64)> + Clone) -> bool {
    let Some(end) = start.checked_add(bytes) else {
        return false;
    };
    let mut cursor = start;
    while cursor < end {
        let Some(top) = parts.clone().find_map(|(first, size)| {
            let top = first.checked_add(size)?;
            (first <= cursor && cursor < top).then_some(top)
        }) else {
            return false;
        };
        cursor = top.min(end);
    }
    true
}
/// Compare unions while allowing reordering and adjacent split/coalesced spans.
/// Empty sets are equivalent. Every nonempty span must be well-formed/disjoint;
/// overlap/duplicate metadata never conceals an ownership mismatch.
pub fn same_backing(left: &[(u64, u64)], right: &[(u64, u64)]) -> Result<bool, Error> {
    same_backing_iter(left.iter().copied(), right.iter().copied())
}
/// The same check without scratch allocation for native reservation catalogs.
pub fn same_backing_iter(
    left: impl Iterator<Item = (u64, u64)> + Clone,
    right: impl Iterator<Item = (u64, u64)> + Clone,
) -> Result<bool, Error> {
    validate(left.clone())?;
    validate(right.clone())?;
    Ok(left
        .clone()
        .all(|(start, bytes)| covers(start, bytes, right.clone()))
        && right
            .clone()
            .all(|(start, bytes)| covers(start, bytes, left.clone())))
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn reordering_and_splitting_do_not_change_physical_ownership() {
        assert_eq!(
            same_backing(
                &[(0x1000, 0x3000), (0x9000, 0x1000)],
                &[(0x9000, 0x1000), (0x3000, 0x1000), (0x1000, 0x2000)]
            ),
            Ok(true)
        );
        assert_eq!(same_backing(&[], &[]), Ok(true));
    }
    #[test]
    fn holes_growth_shrink_and_duplicate_ranges_cannot_join() {
        for other in [
            &[(0x1000, 0x1000), (0x3000, 0x1000)][..],
            &[(0x1000, 0x4000)],
            &[(0x1000, 0x2000)],
            &[],
        ] {
            assert_eq!(same_backing(&[(0x1000, 0x3000)], other), Ok(false));
        }
        assert_eq!(
            same_backing(&[(0x1000, 0x1000), (0x1000, 0x1000)], &[]),
            Err(Error::Overlap)
        );
        assert_eq!(
            same_backing(&[(0x1000, 0x2000), (0x2000, 0x2000)], &[]),
            Err(Error::Overlap)
        );
        assert_eq!(same_backing(&[(0, 0)], &[]), Err(Error::EmptyRange));
        assert_eq!(same_backing(&[(u64::MAX, 1)], &[]), Err(Error::Overflow));
    }
}

/// Admit IRQ/queue identity in a retained root, including its exact guest PCID.
/// This grants no continuation switch: execution still enters the loader guard.
/// Loader PCID 1 is deliberately absent from the guest catalog.
pub fn irq_root_allowed(current: u64, roots: &[u64], tagged_roots: &[u64]) -> bool {
    const ADDRESS: u64 = 0x000f_ffff_ffff_f000;
    if current & !(ADDRESS | 0xfff) != 0 {
        return false;
    }
    if current & 0xfff == 0 {
        return roots.contains(&current);
    }
    current & 0xfff >= 2 && tagged_roots.binary_search(&current).is_ok()
}

#[cfg(test)]
mod irq_tests {
    use super::irq_root_allowed;
    #[test]
    fn tagged_guest_irq_retains_exact_root_and_context_identity() {
        let roots = [0x1000, 0x2000, 0x3000];
        let tagged = [0x2002, 0x3003];
        assert!(irq_root_allowed(0x1000, &roots, &tagged));
        assert!(irq_root_allowed(0x2002, &roots, &tagged));
        assert!(irq_root_allowed(0x3003, &roots, &tagged));
        for bad in [
            0x2001,
            0x2003,
            0x3002,
            0x1002,
            0x4000,
            0x4002,
            0x8000_0000_0000_2002,
            0x0010_0000_0000_2002,
        ] {
            assert!(!irq_root_allowed(bad, &roots, &tagged), "{bad:#x}");
        }
    }
}
