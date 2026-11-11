//! Compiled pipelines kept across boots: the pipeline store.
//!
//! The boot reads the store's file before the device opens and hands its
//! bytes here ([`provide`]). The first backend to open claims it and, when
//! the key matches, seeds the pipeline cache its compiled pipelines read and
//! fill; its snapshots of that cache replace the kept bytes. The boot writes
//! what was added ([`update`]) back to the file while the guest runs and as
//! it stops, each piece once ([`written`]), in a journal a cut write cannot
//! damage (`stela_formats::journal`), and hands the next boot's device the
//! store it reads from that journal in the layout below.
//!
//! A store is a header, the driver's `vkGetPipelineCacheData` bytes, then
//! the AIR compiler's NIR by the digest of what each compilation read
//! ([`NirEntries`]), which a warm boot deserializes instead of lowering AIR
//! again; the header gives the NIR's length and digest after the cache's,
//! zero for none. The
//! header names a [`Key`]: the build identity of the compiler that turns NIR
//! into machine code, and the GPU's vendor, device, driver and pipeline cache
//! UUID. NVK's pipeline cache UUID is the same for every Stela build
//! (`compat/mesa/build_id.c`), so only the build identity tells builds
//! apart. Inside the cache NVK keys each shader by a digest of its
//! serialized NIR, the pipeline state and the layout, so a function's AIR,
//! constants, formats and linked functions select their own entries. A store
//! of another key, a damaged one or one past the bound is reported once and
//! ignored: its pipelines compile again.
use crate::metal::vulkan::physical::Identity;
use alloc::{sync::Arc, vec::Vec};
use ash::vk;
use core::{
    cell::UnsafeCell,
    sync::atomic::{AtomicBool, AtomicU64, Ordering},
};
use stela_crypto::{Sha256, digest::Digest};

/// A store's first bytes.
pub const MAGIC: [u8; 8] = *b"STLGPIPE";
/// The header layout this build reads and writes.
pub const VERSION: u32 = 1;
/// The header: magic, version, a zero word, the key, the payload's length
/// and its SHA-256, then the NIR's length and its SHA-256, zeros for none.
pub const HEADER_BYTES: usize = 128;
const KEY_AT: usize = 16;
const LENGTH_AT: usize = 48;
const CHECKSUM_AT: usize = 56;
const NIR_LENGTH_AT: usize = 88;
const NIR_CHECKSUM_AT: usize = 96;
/// The most bytes of NIR a store keeps.
pub const MAX_NIR_BYTES: u64 = 96 << 20;
/// The largest store kept, header included, whatever the file allows: the
/// boot holds its bytes and the driver a cache of them for the whole boot.
pub const MAX_BYTES: u64 = 256 << 20;
/// A backend snapshots its cache after this many pipelines it did not find
/// there, and when the device goes idle with any.
pub const SNAPSHOT_COMPILES: u32 = 16;

/// What a store's pipelines are valid for.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Key(pub [u8; 32]);

impl Key {
    /// The key of pipelines the compiler of build identity `compiler`
    /// builds for `gpu`.
    pub fn new(compiler: &[u8], gpu: &Identity) -> Self {
        let mut hasher = Sha256::new();
        hasher.update(MAGIC);
        hasher.update(VERSION.to_le_bytes());
        hasher.update((compiler.len() as u64).to_le_bytes());
        hasher.update(compiler);
        hasher.update(gpu.vendor_id.to_le_bytes());
        hasher.update(gpu.device_id.to_le_bytes());
        hasher.update(gpu.driver_id.raw().to_le_bytes());
        hasher.update(gpu.driver_version.to_le_bytes());
        hasher.update(gpu.pipeline_cache_uuid);
        Self(hasher.finalize().into())
    }
}

/// Why a store's bytes seed nothing.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Rejection {
    /// A blank file: nothing was kept yet.
    Blank,
    /// Not a store, or one of another header version.
    Format,
    /// Another compiler build's or GPU's pipelines.
    Stale,
    /// A payload longer than the file or the bound.
    Length,
    /// A payload whose checksum does not match.
    Damaged,
}

impl Rejection {
    fn describe(self) -> &'static str {
        match self {
            Self::Blank => "it is blank",
            Self::Format => "it is not a pipeline store of this version",
            Self::Stale => "it holds another compiler build's or GPU's pipelines",
            Self::Length => "its length is past the file or the bound",
            Self::Damaged => "its checksum does not match",
        }
    }
}

fn word(bytes: &[u8], at: usize) -> u32 {
    u32::from_le_bytes(bytes[at..at + 4].try_into().expect("four bytes"))
}

/// The bytes a store whose file begins with `header` takes, header
/// included, for a reader that has only its first [`HEADER_BYTES`].
pub fn stored_bytes(header: &[u8]) -> Result<u64, Rejection> {
    let head = &header[..header.len().min(HEADER_BYTES)];
    if head.iter().all(|&byte| byte == 0) {
        return Err(Rejection::Blank);
    }
    let header = header.get(..HEADER_BYTES).ok_or(Rejection::Format)?;
    if header[..8] != MAGIC || word(header, 8) != VERSION {
        return Err(Rejection::Format);
    }
    let length = u64::from_le_bytes(header[LENGTH_AT..CHECKSUM_AT].try_into().expect("a length"));
    let nir = nir_length(header);
    length
        .checked_add(nir)
        .and_then(|length| length.checked_add(HEADER_BYTES as u64))
        .filter(|&bytes| bytes <= MAX_BYTES)
        .ok_or(Rejection::Length)
}

fn nir_length(header: &[u8]) -> u64 {
    u64::from_le_bytes(
        header[NIR_LENGTH_AT..NIR_CHECKSUM_AT]
            .try_into()
            .expect("a length"),
    )
}

/// The header of a store of `payload` for `key`.
pub fn header(key: &Key, payload: &[u8]) -> [u8; HEADER_BYTES] {
    header_with_nir(key, payload, &[])
}

/// The header of a store of `payload` and then `nir` for `key`.
pub fn header_with_nir(key: &Key, payload: &[u8], nir: &[u8]) -> [u8; HEADER_BYTES] {
    let mut header = [0; HEADER_BYTES];
    header[..8].copy_from_slice(&MAGIC);
    header[8..12].copy_from_slice(&VERSION.to_le_bytes());
    header[KEY_AT..LENGTH_AT].copy_from_slice(&key.0);
    header[LENGTH_AT..CHECKSUM_AT].copy_from_slice(&(payload.len() as u64).to_le_bytes());
    header[CHECKSUM_AT..CHECKSUM_AT + 32].copy_from_slice(&Sha256::digest(payload));
    if !nir.is_empty() {
        header[NIR_LENGTH_AT..NIR_CHECKSUM_AT].copy_from_slice(&(nir.len() as u64).to_le_bytes());
        header[NIR_CHECKSUM_AT..NIR_CHECKSUM_AT + 32].copy_from_slice(&Sha256::digest(nir));
    }
    header
}

/// The NIR of `stored`, whose payload [`payload`] accepted for `key`
/// within `bound`: empty for none, or for NIR that is damaged, which then
/// compiles again.
pub fn nir_section<'a>(stored: &'a [u8], key: &Key, bound: u64) -> &'a [u8] {
    let Ok(payload) = payload(stored, key, bound) else {
        return &[];
    };
    let start = HEADER_BYTES + payload.len();
    let length = nir_length(stored);
    let Some(nir) = usize::try_from(length)
        .ok()
        .and_then(|length| stored.get(start..start.checked_add(length)?))
    else {
        return &[];
    };
    if Sha256::digest(nir)[..] != stored[NIR_CHECKSUM_AT..NIR_CHECKSUM_AT + 32] {
        return &[];
    }
    nir
}

/// A store's NIR: entries of a 32-byte key, a 32-bit little-endian length
/// and that many bytes of serialized NIR.
pub struct NirEntries<'a>(pub &'a [u8]);

impl<'a> Iterator for NirEntries<'a> {
    type Item = ([u8; 32], &'a [u8]);

    fn next(&mut self) -> Option<Self::Item> {
        let key: [u8; 32] = self.0.get(..32)?.try_into().ok()?;
        let length = u32::from_le_bytes(self.0.get(32..36)?.try_into().ok()?) as usize;
        let blob = self.0.get(36..36usize.checked_add(length)?)?;
        self.0 = &self.0[36 + length..];
        Some((key, blob))
    }
}

/// Appends an entry of `key` and `blob` to a store's NIR.
fn push_entry(nir: &mut Vec<u8>, key: &[u8; 32], blob: &[u8]) {
    nir.extend_from_slice(key);
    nir.extend_from_slice(&(blob.len() as u32).to_le_bytes());
    nir.extend_from_slice(blob);
}

/// The payload of `stored`, a store's file or its first bytes, when it holds
/// `key`'s pipelines within `bound` bytes.
pub fn payload<'a>(stored: &'a [u8], key: &Key, bound: u64) -> Result<&'a [u8], Rejection> {
    let bytes = stored_bytes(stored)?;
    if stored[KEY_AT..LENGTH_AT] != key.0 {
        return Err(Rejection::Stale);
    }
    if bytes > bound.min(MAX_BYTES) || bytes > stored.len() as u64 {
        return Err(Rejection::Length);
    }
    // The cache's bytes, which the NIR follows.
    let cache = bytes - nir_length(stored) - HEADER_BYTES as u64;
    let payload = &stored[HEADER_BYTES..HEADER_BYTES + cache as usize];
    if Sha256::digest(payload)[..] != stored[CHECKSUM_AT..CHECKSUM_AT + 32] {
        return Err(Rejection::Damaged);
    }
    Ok(payload)
}

/// When a backend snapshots its cache, and whether the cache still grows.
#[derive(Debug)]
pub struct Policy {
    bound: u64,
    unsaved: u32,
    grows: bool,
    /// Pipelines created through the cache this boot that it held, and
    /// that it did not.
    hits: u64,
    misses: u64,
}

impl Policy {
    pub fn new(bound: u64) -> Self {
        Self {
            bound: bound.min(MAX_BYTES),
            unsaved: 0,
            grows: true,
            hits: 0,
            misses: 0,
        }
    }

    /// Pipelines created through the cache this boot: found there, and not.
    pub fn counts(&self) -> (u64, u64) {
        (self.hits, self.misses)
    }

    /// Whether compiled pipelines are added to the cache. Once a snapshot
    /// passes the bound they are only looked up there.
    pub fn grows(&self) -> bool {
        self.grows
    }

    /// A pipeline was created through the cache, `hit` when it was found
    /// there: whether to snapshot now.
    pub fn created(&mut self, hit: bool) -> bool {
        if hit {
            self.hits += 1;
        } else {
            self.misses += 1;
        }
        if self.grows && !hit {
            self.unsaved += 1;
        }
        self.grows && self.unsaved >= SNAPSHOT_COMPILES
    }

    /// Whether an idle device has pipelines to snapshot.
    pub fn unsaved(&self) -> bool {
        self.grows && self.unsaved > 0
    }

    /// A snapshot of `payload` bytes was taken: whether it is kept. One past
    /// the bound stops the cache growing.
    pub fn snapshotted(&mut self, payload: usize) -> bool {
        self.unsaved = 0;
        let fits = (HEADER_BYTES as u64).saturating_add(payload as u64) <= self.bound;
        self.grows &= fits;
        fits
    }

    /// Snapshots failed: the cache stops growing.
    pub fn stop(&mut self) {
        self.unsaved = 0;
        self.grows = false;
    }
}

/// The bytes a backend seeds its cache with.
#[derive(Clone, Debug)]
pub enum Seed {
    Empty,
    /// The payload the boot read.
    Stored(&'static [u8]),
    /// A previous backend's newest snapshot, for a reopened device.
    Kept(Arc<Vec<u8>>),
}

impl Seed {
    pub fn bytes(&self) -> &[u8] {
        match self {
            Self::Empty => &[],
            Self::Stored(bytes) => bytes,
            Self::Kept(bytes) => bytes,
        }
    }
}

/// A backend's hold on the store.
#[derive(Debug)]
pub struct Claim {
    generation: u64,
    pub key: Key,
    pub bound: u64,
    pub seed: Seed,
    /// Why the stored bytes seed nothing, unless they are blank.
    pub refused: Option<Rejection>,
}

/// The boot's store: what it read, the newest snapshot, and what of both
/// the boot wrote back since ([`Store::update`]).
#[derive(Debug)]
pub struct Store {
    compiler: Vec<u8>,
    bound: u64,
    stored: &'static [u8],
    owner: Option<(u64, Key)>,
    claims: u64,
    kept: Option<(Key, Arc<Vec<u8>>)>,
    /// Snapshots kept so far, and the newest of them written.
    kept_version: u64,
    written_version: u64,
    /// Keys of the NIR added this boot that is written.
    written_nir: alloc::collections::BTreeSet<[u8; 32]>,
    /// The key the stored bytes matched at a claim, and their NIR.
    matched: Option<Key>,
    /// The keys of the matched NIR, verified once at the claim, so that
    /// adding NIR does not hash the stored file again.
    stored_keys: alloc::collections::BTreeSet<[u8; 32]>,
    /// NIR compiled this boot, by key, and its bytes.
    nir: alloc::collections::BTreeMap<[u8; 32], Vec<u8>>,
    nir_bytes: u64,
}

impl Store {
    /// A store of `stored`, its file's bytes (empty for a blank one), for
    /// the compiler of build identity `compiler`, holding up to `bound`
    /// bytes.
    pub fn new(stored: &'static [u8], compiler: &[u8], bound: u64) -> Self {
        Self {
            compiler: compiler.to_vec(),
            bound: bound.min(MAX_BYTES),
            stored,
            owner: None,
            claims: 0,
            kept: None,
            kept_version: 0,
            written_version: 0,
            written_nir: alloc::collections::BTreeSet::new(),
            matched: None,
            stored_keys: alloc::collections::BTreeSet::new(),
            nir: alloc::collections::BTreeMap::new(),
            nir_bytes: 0,
        }
    }

    /// The stored NIR, for the device whose claim matched the store.
    pub fn stored_nir(&self) -> NirEntries<'static> {
        NirEntries(match self.matched {
            Some(key) => nir_section(self.stored, &key, self.bound),
            None => &[],
        })
    }

    /// Keeps `blob`, NIR of `key` compiled this boot, for the next.
    pub fn add_nir(&mut self, key: [u8; 32], blob: &[u8]) {
        let bytes = blob.len() as u64 + 36;
        if self.matched.is_none() && self.owner.is_none()
            || self.nir_bytes + bytes > MAX_NIR_BYTES
            || self.stored_keys.contains(&key)
        {
            return;
        }
        if self.nir.insert(key, blob.to_vec()).is_none() {
            self.nir_bytes += bytes;
        }
    }

    /// The serialized NIR of `key` that [`Self::add_nir`] kept this boot.
    pub fn recall_nir(&self, key: &[u8; 32]) -> Option<Vec<u8>> {
        self.nir.get(key).cloned()
    }

    /// The store for a backend of `gpu`. A newer backend of the same key
    /// replaces the holder, as a reopened device does; another GPU's while
    /// one holds it, has none.
    pub fn claim(&mut self, gpu: &Identity) -> Option<Claim> {
        let key = Key::new(&self.compiler, gpu);
        if self.owner.is_some_and(|(_, owner)| owner != key) {
            return None;
        }
        self.claims += 1;
        self.owner = Some((self.claims, key));
        let matched = payload(self.stored, &key, self.bound).is_ok();
        if matched && self.matched != Some(key) {
            self.matched = Some(key);
            self.stored_keys = self.stored_nir().map(|(stored, _)| stored).collect();
        }
        let (seed, refused) = match &self.kept {
            Some((kept, payload)) if *kept == key => (Seed::Kept(payload.clone()), None),
            _ => match payload(self.stored, &key, self.bound) {
                Ok(payload) => (Seed::Stored(payload), None),
                Err(Rejection::Blank) => (Seed::Empty, None),
                Err(rejection) => (Seed::Empty, Some(rejection)),
            },
        };
        Some(Claim {
            generation: self.claims,
            key,
            bound: self.bound,
            seed,
            refused,
        })
    }

    /// Keep `payload`, the holder's newest snapshot.
    pub fn keep(&mut self, claim: &Claim, payload: Vec<u8>) {
        if self.holds(claim) {
            self.kept = Some((claim.key, Arc::new(payload)));
            self.kept_version += 1;
        }
    }

    /// Ends `claim`'s hold, when it still holds the store.
    pub fn release(&mut self, claim: &Claim) {
        if self.holds(claim) {
            self.owner = None;
        }
    }

    fn holds(&self, claim: &Claim) -> bool {
        self.owner
            .is_some_and(|(generation, _)| generation == claim.generation)
    }

    /// What was added since the last [`Self::written`]: the newest snapshot
    /// when newer than the one written, and the NIR not written, for the key
    /// the store is held for. `None` when nothing is new.
    pub fn update(&self) -> Option<Update> {
        let key = self
            .kept
            .as_ref()
            .map(|(key, _)| *key)
            .or(self.owner.map(|(_, key)| key))
            .or(self.matched)?;
        let cache = match &self.kept {
            Some((kept, payload)) if *kept == key && self.kept_version > self.written_version => {
                Some(payload.clone())
            }
            _ => None,
        };
        let mut nir = Vec::new();
        let mut nir_keys = Vec::new();
        for (entry, blob) in &self.nir {
            if !self.written_nir.contains(entry) {
                push_entry(&mut nir, entry, blob);
                nir_keys.push(*entry);
            }
        }
        (cache.is_some() || !nir.is_empty()).then_some(Update {
            key,
            cache,
            nir,
            cache_version: self.kept_version,
            nir_keys,
        })
    }

    /// `update` is durable: what it holds is not offered again.
    pub fn written(&mut self, update: &Update) {
        if update.cache.is_some() {
            self.written_version = self.written_version.max(update.cache_version);
        }
        self.written_nir.extend(update.nir_keys.iter().copied());
    }

    /// Every NIR entry the store holds for `key`, stored and added, as one
    /// section: what a store rewritten whole keeps.
    pub fn all_nir(&self, key: &Key) -> Vec<u8> {
        let stored = match self.matched {
            Some(matched) if matched == *key => nir_section(self.stored, key, self.bound),
            _ => &[],
        };
        let mut nir = stored.to_vec();
        for (entry, blob) in &self.nir {
            push_entry(&mut nir, entry, blob);
        }
        nir
    }
}

/// What a store added since it was last written ([`Store::update`]).
#[derive(Debug)]
pub struct Update {
    pub key: Key,
    /// The newest cache snapshot, which replaces the written one.
    pub cache: Option<Arc<Vec<u8>>>,
    /// NIR entries ([`NirEntries`]) added to the written ones.
    pub nir: Vec<u8>,
    cache_version: u64,
    nir_keys: Vec<[u8; 32]>,
}

/// Pipelines created through the store's cache since the last
/// [`Lookups::take`]: those the cache held and those compiled, with the
/// driver's creation time of each kind and the longest compile.
pub struct Lookups {
    hits: AtomicU64,
    misses: AtomicU64,
    hit_ns: AtomicU64,
    miss_ns: AtomicU64,
    miss_longest_ns: AtomicU64,
}

/// What [`Lookups::take`] answers.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct LookupCounts {
    pub hits: u64,
    pub misses: u64,
    pub hit_ns: u64,
    pub miss_ns: u64,
    pub miss_longest_ns: u64,
}

impl Lookups {
    pub const fn new() -> Self {
        Self {
            hits: AtomicU64::new(0),
            misses: AtomicU64::new(0),
            hit_ns: AtomicU64::new(0),
            miss_ns: AtomicU64::new(0),
            miss_longest_ns: AtomicU64::new(0),
        }
    }

    /// A creation of `ns` nanoseconds, `hit` when the cache held it.
    pub fn note(&self, hit: bool, ns: u64) {
        if hit {
            self.hits.fetch_add(1, Ordering::Relaxed);
            self.hit_ns.fetch_add(ns, Ordering::Relaxed);
        } else {
            self.misses.fetch_add(1, Ordering::Relaxed);
            self.miss_ns.fetch_add(ns, Ordering::Relaxed);
            self.miss_longest_ns.fetch_max(ns, Ordering::Relaxed);
        }
    }

    /// The counts since the last call, cleared.
    pub fn take(&self) -> LookupCounts {
        let take = |counter: &AtomicU64| counter.swap(0, Ordering::Relaxed);
        LookupCounts {
            hits: take(&self.hits),
            misses: take(&self.misses),
            hit_ns: take(&self.hit_ns),
            miss_ns: take(&self.miss_ns),
            miss_longest_ns: take(&self.miss_longest_ns),
        }
    }
}

impl Default for Lookups {
    fn default() -> Self {
        Self::new()
    }
}

static LOOKUPS: Lookups = Lookups::new();

/// The boot's pipeline creations through the store's cache since the last
/// call: what the platform reports (`GPU STORE`).
pub fn take_lookups() -> LookupCounts {
    LOOKUPS.take()
}

/// Counts a creation through the store's cache.
pub(super) fn note_lookup(hit: bool, ns: u64) {
    LOOKUPS.note(hit, ns);
}

/// The boot's store, behind a lock no interrupt handler takes.
struct Cell {
    held: AtomicBool,
    store: UnsafeCell<Option<Store>>,
}
// SAFETY: `with_store` holds `held` for every access.
unsafe impl Sync for Cell {}
static STORE: Cell = Cell {
    held: AtomicBool::new(false),
    store: UnsafeCell::new(None),
};

fn with_store<R>(f: impl FnOnce(&mut Option<Store>) -> R) -> R {
    while STORE
        .held
        .compare_exchange_weak(false, true, Ordering::Acquire, Ordering::Relaxed)
        .is_err()
    {
        while STORE.held.load(Ordering::Relaxed) {
            core::hint::spin_loop();
        }
    }
    // SAFETY: `held` excludes every other access until it is released.
    let result = f(unsafe { &mut *STORE.store.get() });
    STORE.held.store(false, Ordering::Release);
    result
}

/// Give the devices that open later the store the boot read: `stored`, the
/// file's bytes, for the compiler of build identity `compiler`, holding up
/// to `bound` bytes. Without it compiled pipelines keep no cache.
pub fn provide(stored: &'static [u8], compiler: &[u8], bound: u64) {
    let store = Store::new(stored, compiler, bound);
    with_store(|slot| *slot = Some(store));
}

/// What the boot's store added since it was last written, for the boot to
/// write while the guest runs and as it stops ([`Store::update`]).
pub fn update() -> Option<Update> {
    with_store(|slot| slot.as_ref()?.update())
}

/// `update` is durable on the file.
pub fn written(update: &Update) {
    with_store(|slot| {
        if let Some(store) = slot {
            store.written(update);
        }
    });
}

/// Every NIR entry the boot's store holds for `key` ([`Store::all_nir`]).
pub fn all_nir(key: &Key) -> Vec<u8> {
    with_store(|slot| {
        slot.as_ref()
            .map(|store| store.all_nir(key))
            .unwrap_or_default()
    })
}

/// The NIR the store holds for the device that claimed it, by key.
pub fn stored_nir() -> Vec<([u8; 32], &'static [u8])> {
    with_store(|slot| {
        slot.as_ref()
            .map(|store| store.stored_nir().collect())
            .unwrap_or_default()
    })
}

/// Keeps `blob`, NIR of `key` compiled this boot, for the next boot.
pub fn add_nir(key: &[u8; 32], blob: &[u8]) {
    with_store(|slot| {
        if let Some(store) = slot {
            store.add_nir(*key, blob);
        }
    });
}

/// The serialized NIR of `key` compiled this boot, while the store keeps it.
pub fn recall_nir(key: &[u8; 32]) -> Option<Vec<u8>> {
    with_store(|slot| slot.as_ref()?.recall_nir(key))
}

/// The cache a backend's compiled pipelines read and fill, seeded from the
/// store, and when it is snapshotted.
pub(super) struct Persisted {
    claim: Option<Claim>,
    /// The bytes the cache was seeded with that the driver took.
    seeded: usize,
    cache: vk::PipelineCache,
    policy: Policy,
    log: fn(&str),
}

impl Persisted {
    /// No store: compiled pipelines keep no cache.
    pub(super) fn none(log: fn(&str)) -> Self {
        Self {
            claim: None,
            seeded: 0,
            cache: vk::PipelineCache::null(),
            policy: Policy::new(0),
            log,
        }
    }

    /// Claims the boot's store for `gpu` and seeds a cache from it. A seed
    /// the driver refuses leaves the cache empty.
    ///
    /// # Safety
    ///
    /// `device` is the backend's live device, whose physical device `gpu`
    /// describes.
    pub(super) unsafe fn open(device: &ash::Device, gpu: &Identity, log: fn(&str)) -> Self {
        let Some(mut claim) = with_store(|slot| slot.as_mut()?.claim(gpu)) else {
            return Self::none(log);
        };
        if let Some(rejection) = claim.refused {
            log(&alloc::format!(
                "pipeline store ignored: {}; its pipelines compile again",
                rejection.describe()
            ));
        }
        let seed = core::mem::replace(&mut claim.seed, Seed::Empty);
        let create = |data: &[u8]| {
            let info = vk::PipelineCacheCreateInfo::default().initial_data(data);
            // SAFETY: the create info and the data outlive the call; the
            // driver ignores data whose header is not its own.
            unsafe { device.create_pipeline_cache(&info, None) }
        };
        let mut seeded = seed.bytes().len();
        let cache = create(seed.bytes()).or_else(|error| {
            if seed.bytes().is_empty() {
                return Err(error);
            }
            log(&alloc::format!(
                "pipeline store refused by the driver ({error:?}); its pipelines compile again"
            ));
            seeded = 0;
            create(&[])
        });
        match cache {
            Ok(cache) => Self {
                seeded,
                policy: Policy::new(claim.bound),
                claim: Some(claim),
                cache,
                log,
            },
            Err(error) => {
                log(&alloc::format!(
                    "pipeline store unused: no pipeline cache ({error:?}); pipelines compile \
                     every boot"
                ));
                with_store(|slot| {
                    if let Some(store) = slot {
                        store.release(&claim);
                    }
                });
                Self::none(log)
            }
        }
    }

    /// The cache a compiled pipeline is looked up in, and whether it is
    /// added there when it is missing.
    pub(super) fn cache(&self) -> Option<(vk::PipelineCache, bool)> {
        self.claim
            .is_some()
            .then_some((self.cache, self.policy.grows()))
    }

    /// A compiled pipeline was created through [`Self::cache`], `hit` when
    /// the cache held it.
    pub(super) fn created(&mut self, device: &ash::Device, hit: bool) {
        if self.claim.is_some() && self.policy.created(hit) {
            self.snapshot(device);
        }
    }

    /// The device is idle: snapshot the pipelines added since the last.
    pub(super) fn idle(&mut self, device: &ash::Device) {
        if self.claim.is_some() && self.policy.unsaved() {
            self.snapshot(device);
        }
    }

    fn snapshot(&mut self, device: &ash::Device) {
        let Some(claim) = &self.claim else {
            return;
        };
        // SAFETY: the cache is this device's and alive; the backend creates
        // no pipeline meanwhile.
        match unsafe { device.get_pipeline_cache_data(self.cache) } {
            Ok(payload) => {
                let bytes = payload.len();
                // A warm boot whose cache holds nothing it was seeded with
                // shows as misses with no hits; one with keys that change
                // between boots, as misses of the same pipelines each boot.
                let (hits, misses) = self.policy.counts();
                (self.log)(&alloc::format!(
                    "pipeline store snapshot bytes={bytes} seeded={} hits={hits} misses={misses}",
                    self.seeded
                ));
                if self.policy.snapshotted(bytes) {
                    with_store(|slot| {
                        if let Some(store) = slot {
                            store.keep(claim, payload);
                        }
                    });
                } else {
                    (self.log)(&alloc::format!(
                        "pipeline store full: {bytes} bytes pass its bound of {}; newer \
                         pipelines compile every boot",
                        claim.bound
                    ));
                }
            }
            Err(error) => {
                self.policy.stop();
                (self.log)(&alloc::format!(
                    "pipeline store not snapshotted ({error:?}); newer pipelines compile \
                     every boot"
                ));
            }
        }
    }

    /// Releases the store and destroys the cache.
    ///
    /// # Safety
    ///
    /// `device` is the live device the cache was created on, and no
    /// pipeline creation uses the cache.
    pub(super) unsafe fn destroy(&mut self, device: &ash::Device) {
        if let Some(claim) = self.claim.take() {
            with_store(|slot| {
                if let Some(store) = slot {
                    store.release(&claim);
                }
            });
            // SAFETY: the caller's guarantees.
            unsafe { device.destroy_pipeline_cache(self.cache, None) };
            self.cache = vk::PipelineCache::null();
        }
    }
}

#[cfg(test)]
mod tests;
