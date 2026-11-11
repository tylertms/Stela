use super::*;
use crate::metal::nvidia::{CHIPS, fixture};
use alloc::{boxed::Box, vec};

const COMPILER: &[u8] = b"compiler build a";

fn gpu() -> Identity {
    fixture::nvk(&CHIPS[0], 16 << 30).identity
}

/// A store of `payload` for `key`, leaked as the boot's bytes are.
fn stored(key: &Key, payload: &[u8]) -> &'static [u8] {
    let mut bytes = header(key, payload).to_vec();
    bytes.extend_from_slice(payload);
    Box::leak(bytes.into_boxed_slice())
}

#[test]
fn the_key_names_the_compiler_build_and_every_gpu_identity_field() {
    let base = Key::new(COMPILER, &gpu());
    assert_eq!(base, Key::new(COMPILER, &gpu()));
    let changed: [fn(&mut Identity); 5] = [
        |gpu| gpu.vendor_id ^= 1,
        |gpu| gpu.device_id ^= 1,
        |gpu| gpu.driver_version ^= 1,
        |gpu| gpu.driver_id = crate::metal::vulkan::DriverId::MesaRadv,
        |gpu| gpu.pipeline_cache_uuid[15] ^= 1,
    ];
    for change in changed {
        let mut other = gpu();
        change(&mut other);
        assert_ne!(Key::new(COMPILER, &other), base);
    }
    assert_ne!(Key::new(b"compiler build b", &gpu()), base);
    // A length prefix keeps identities from running into the GPU's fields.
    assert_ne!(Key::new(b"", &gpu()), base);
}

#[test]
fn a_sealed_store_opens_for_its_key_alone() {
    let key = Key::new(COMPILER, &gpu());
    let bytes = stored(&key, b"vk pipeline cache data");
    assert_eq!(stored_bytes(&bytes[..HEADER_BYTES]), Ok(bytes.len() as u64));
    assert_eq!(
        payload(bytes, &key, MAX_BYTES),
        Ok(&b"vk pipeline cache data"[..])
    );
    let other = Key::new(b"compiler build b", &gpu());
    assert_eq!(payload(bytes, &other, MAX_BYTES), Err(Rejection::Stale));
}

#[test]
fn blank_foreign_damaged_and_oversized_stores_are_refused() {
    let key = Key::new(COMPILER, &gpu());
    let good = stored(&key, &[7; 300]).to_vec();
    assert_eq!(payload(&[], &key, MAX_BYTES), Err(Rejection::Blank));
    assert_eq!(payload(&[0; 4096], &key, MAX_BYTES), Err(Rejection::Blank));
    assert_eq!(
        payload(&good[..40], &key, MAX_BYTES),
        Err(Rejection::Format)
    );

    let mut magic = good.clone();
    magic[0] ^= 1;
    assert_eq!(payload(&magic, &key, MAX_BYTES), Err(Rejection::Format));
    let mut version = good.clone();
    version[8] = 2;
    assert_eq!(payload(&version, &key, MAX_BYTES), Err(Rejection::Format));

    let mut flipped = good.clone();
    flipped[HEADER_BYTES + 100] ^= 1;
    assert_eq!(payload(&flipped, &key, MAX_BYTES), Err(Rejection::Damaged));
    let mut checksum = good.clone();
    checksum[CHECKSUM_AT] ^= 1;
    assert_eq!(payload(&checksum, &key, MAX_BYTES), Err(Rejection::Damaged));

    // A file cut short, a bound below the store, and a length past any file.
    assert_eq!(
        payload(&good[..good.len() - 1], &key, MAX_BYTES),
        Err(Rejection::Length)
    );
    assert_eq!(
        payload(&good, &key, good.len() as u64 - 1),
        Err(Rejection::Length)
    );
    assert_eq!(
        payload(&good, &key, good.len() as u64).map(<[u8]>::len),
        Ok(300)
    );
    let mut huge = good.clone();
    huge[LENGTH_AT..CHECKSUM_AT].copy_from_slice(&u64::MAX.to_le_bytes());
    assert_eq!(stored_bytes(&huge), Err(Rejection::Length));
    huge[LENGTH_AT..CHECKSUM_AT].copy_from_slice(&MAX_BYTES.to_le_bytes());
    assert_eq!(stored_bytes(&huge), Err(Rejection::Length));
}

#[test]
fn snapshots_follow_new_compiles_and_stop_at_the_bound() {
    let mut policy = Policy::new(HEADER_BYTES as u64 + 1000);
    assert!(policy.grows() && !policy.unsaved());
    // Hits add nothing to save.
    for _ in 0..100 {
        assert!(!policy.created(true));
    }
    assert!(!policy.unsaved());
    assert_eq!(policy.counts(), (100, 0));
    for _ in 1..SNAPSHOT_COMPILES {
        assert!(!policy.created(false));
    }
    assert!(policy.unsaved());
    assert!(policy.created(false));
    assert_eq!(policy.counts(), (100, u64::from(SNAPSHOT_COMPILES)));
    assert!(policy.snapshotted(1000));
    assert!(policy.grows() && !policy.unsaved());

    policy.created(false);
    assert!(!policy.snapshotted(1001));
    assert!(!policy.grows());
    // A full store snapshots no more.
    for _ in 0..2 * SNAPSHOT_COMPILES {
        assert!(!policy.created(false));
    }
    assert!(!policy.unsaved());

    let mut failed = Policy::new(MAX_BYTES);
    failed.created(false);
    failed.stop();
    assert!(!failed.grows() && !failed.unsaved());
    // No bound passes the store's own.
    assert!(!Policy::new(u64::MAX).snapshotted(MAX_BYTES as usize));
}

#[test]
fn a_claim_seeds_from_a_matching_store_and_reports_any_other() {
    let gpu = gpu();
    let key = Key::new(COMPILER, &gpu);
    let mut store = Store::new(stored(&key, b"cached"), COMPILER, MAX_BYTES);
    let claim = store.claim(&gpu).unwrap();
    assert_eq!(claim.seed.bytes(), b"cached");
    assert_eq!(claim.refused, None);

    let mut blank = Store::new(&[], COMPILER, MAX_BYTES);
    let claim = blank.claim(&gpu).unwrap();
    assert!(claim.seed.bytes().is_empty() && claim.refused.is_none());

    let other = Key::new(b"an older build", &gpu);
    let mut stale = Store::new(stored(&other, b"old"), COMPILER, MAX_BYTES);
    let claim = stale.claim(&gpu).unwrap();
    assert!(claim.seed.bytes().is_empty());
    assert_eq!(claim.refused, Some(Rejection::Stale));

    let mut small = Store::new(stored(&key, &[1; 64]), COMPILER, HEADER_BYTES as u64);
    assert_eq!(small.claim(&gpu).unwrap().refused, Some(Rejection::Length));
}

#[test]
fn the_newest_holder_keeps_snapshots_and_each_is_offered_until_written() {
    let gpu = gpu();
    let key = Key::new(COMPILER, &gpu);
    let mut store = Store::new(&[], COMPILER, MAX_BYTES);
    assert!(store.update().is_none(), "nothing holds it");
    let first = store.claim(&gpu).unwrap();
    assert!(store.update().is_none(), "nothing was added");
    store.keep(&first, vec![1; 10]);

    // Another GPU has no store while the first holds it.
    let mut other = gpu.clone();
    other.device_id ^= 1;
    assert!(store.claim(&other).is_none());

    // A reopened device of the same GPU takes over, seeded with the newest.
    let second = store.claim(&gpu).unwrap();
    assert_eq!(second.seed.bytes(), &[1; 10][..]);
    store.keep(&first, vec![2; 10]);
    store.release(&first);
    store.keep(&second, vec![3; 10]);

    let update = store.update().unwrap();
    assert_eq!(update.key, key);
    assert_eq!(update.cache.as_deref(), Some(&vec![3; 10]));
    // Offered again until written, and not after.
    assert!(store.update().is_some());
    store.written(&update);
    assert!(store.update().is_none());
    // A newer snapshot is offered again, the guest still running.
    store.keep(&second, vec![4; 10]);
    store.written(&update);
    assert_eq!(store.update().unwrap().cache.as_deref(), Some(&vec![4; 10]));

    // Released, the store serves another GPU.
    let mut store = Store::new(&[], COMPILER, MAX_BYTES);
    let claim = store.claim(&gpu).unwrap();
    store.release(&claim);
    assert!(store.claim(&other).is_some());
}

/// The layout a boot reads back: header, cache, NIR.
fn sealed(key: &Key, cache: &[u8], nir: &[u8]) -> &'static [u8] {
    let mut bytes = header_with_nir(key, cache, nir).to_vec();
    bytes.extend_from_slice(cache);
    bytes.extend_from_slice(nir);
    Box::leak(bytes.into_boxed_slice())
}

#[test]
fn nir_compiled_this_boot_is_offered_once_and_read_back_after_the_cache() {
    let gpu = gpu();
    let key = Key::new(COMPILER, &gpu);
    let mut store = Store::new(&[], COMPILER, MAX_BYTES);
    let claim = store.claim(&gpu).unwrap();
    store.add_nir([1; 32], b"first nir");
    store.add_nir([2; 32], b"second");
    store.keep(&claim, vec![5; 20]);
    let update = store.update().unwrap();
    let entries: Vec<_> = NirEntries(&update.nir).collect();
    assert_eq!(
        entries,
        [([1; 32], &b"first nir"[..]), ([2; 32], &b"second"[..])]
    );
    store.written(&update);
    store.add_nir([4; 32], b"later");
    let later = store.update().unwrap();
    assert!(later.cache.is_none(), "the snapshot is written");
    assert_eq!(
        NirEntries(&later.nir)
            .map(|(key, _)| key)
            .collect::<Vec<_>>(),
        [[4; 32]]
    );

    let bytes = sealed(&key, &[5; 20], &update.nir);
    assert_eq!(stored_bytes(bytes), Ok(bytes.len() as u64));
    assert_eq!(payload(bytes, &key, MAX_BYTES), Ok(&[5; 20][..]));

    // The next boot's device holds it and offers only what is new; the
    // whole NIR is the stored entries and the added ones.
    let mut next = Store::new(bytes, COMPILER, MAX_BYTES);
    assert!(next.stored_nir().next().is_none(), "no device claimed it");
    next.claim(&gpu).unwrap();
    assert_eq!(next.stored_nir().count(), 2);
    next.add_nir([1; 32], b"first nir");
    next.add_nir([3; 32], b"third");
    let update = next.update().unwrap();
    assert!(update.cache.is_none());
    assert_eq!(
        NirEntries(&update.nir)
            .map(|(key, _)| key)
            .collect::<Vec<_>>(),
        [[3; 32]]
    );
    let keys: Vec<_> = NirEntries(&next.all_nir(&key))
        .map(|(key, _)| key)
        .collect();
    assert_eq!(keys, [[1; 32], [2; 32], [3; 32]]);
}

#[test]
fn damaged_nir_is_dropped_and_the_cache_kept() {
    let gpu = gpu();
    let key = Key::new(COMPILER, &gpu);
    let mut nir = Vec::new();
    push_entry(&mut nir, &[1; 32], b"nir");
    let mut bytes = header_with_nir(&key, b"cache", &nir).to_vec();
    bytes.extend_from_slice(b"cache");
    bytes.extend_from_slice(&nir);
    let last = bytes.len() - 1;
    bytes[last] ^= 1;
    assert_eq!(payload(&bytes, &key, MAX_BYTES), Ok(&b"cache"[..]));
    assert!(nir_section(&bytes, &key, MAX_BYTES).is_empty());
}

#[test]
fn lookups_count_hits_and_compiles_apart_and_clear_when_taken() {
    let lookups = Lookups::new();
    lookups.note(true, 10);
    lookups.note(false, 300);
    lookups.note(false, 1_000);
    lookups.note(true, 20);
    assert_eq!(
        lookups.take(),
        LookupCounts {
            hits: 2,
            misses: 2,
            hit_ns: 30,
            miss_ns: 1_300,
            miss_longest_ns: 1_000,
        }
    );
    assert_eq!(lookups.take(), LookupCounts::default());
}
