//! IPSW-backed verified random reads. Existing ZIP64 and bounded inflater own
//! archive parsing. A member is scanned once against immutable payload SHA-256;
//! 64-KiB block hashes then detect changes before returning any read.
//! BuildManifest's component digest is a different (Image4) digest and remains
//! attached to the selected identity; it is never replaced with the ZIP hash.
//!
//! A manifest that carries recorded hashes (`payload::BlockHashes`, which
//! the host that wrote it computed while hashing every member) needs no
//! scan to build them: [`Store::adopt_block_hashes`] takes the block hashes
//! and SHA-384s from the file the manifest names by SHA-256. Every read
//! checks its blocks against them, and a member read from its first byte to
//! its last is also checked against its SHA-384. [`Verify`] says when the
//! whole of each member is checked against those records: by default every
//! member, before StartRestore ([`Store::prepare`]); with
//! [`Verify::OnRead`] only the blocks the restore reads, as it reads them.
//! Both use the one block check, and a mismatch refuses the member by name.
//!
//! A remote image's device-verified members (`payload::Member::sha256`
//! absent) have no hashes to check: their bytes pass through as read, and
//! the device's own Image4 and AEA checks verify them.
use super::{
    Error, Result,
    plist::{self, Limits, Value},
};
use alloc::{
    borrow::Cow,
    boxed::Box,
    collections::{BTreeMap, VecDeque},
    string::{String, ToString},
    vec::Vec,
};
use sha2::{Digest, Sha384};
use stela_contracts::refuse;
use stela_crypto::Sha256;
use {
    stela_formats::payload::PayloadManifest,
    stela_formats::payload::Purpose,
    stela_formats::payload::ReadAt,
    stela_formats::payload::zip::{Archive, Entry, Progress},
};

pub const BLOCK_BYTES: usize = stela_formats::payload::HASH_BLOCK_BYTES as usize;
pub const MAX_READ_BYTES: usize = 1024 * 1024;
const MAX_BLOCKS: usize = 1 << 20;
const MAX_BUILD_MANIFEST_BYTES: usize = 64 * 1024 * 1024;
/// `Info.RestoreBehavior` of a BuildManifest identity that erases the
/// target, and of one that updates its installed system in place, keeping
/// the user's data.
pub const ERASE_BEHAVIOR: &str = "Erase";
pub const UPDATE_BEHAVIOR: &str = "Update";
#[derive(Debug, Clone)]
pub struct Identity {
    pub index: usize,
    pub value: Value,
}
impl Identity {
    pub fn info(&self) -> Result<&Value> {
        self.value.required("Info")
    }
    pub fn variant(&self) -> Result<&str> {
        self.info()?.required("Variant")?.string()
    }
    pub fn component(&self, name: &str) -> Result<&Value> {
        self.value.required("Manifest")?.required(name)
    }
    pub fn path(&self, name: &str) -> Result<&str> {
        self.component(name)?
            .required("Info")?
            .required("Path")?
            .string()
    }
    pub fn number(&self, key: &str) -> Result<u64> {
        let v = self.value.required(key)?;
        match v {
            Value::Integer(n) => Ok(*n),
            Value::String(s) => u64::from_str_radix(s.strip_prefix("0x").unwrap_or(s), 16)
                .map_err(|_| Error::Invalid("identity hex number")),
            _ => Err(Error::Invalid("identity number")),
        }
    }
}
/// When recorded hashes are checked against the whole of each member.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Verify {
    /// Every member, read whole by [`Store::prepare`] before StartRestore.
    #[default]
    Full,
    /// Only the blocks each request reads, as it reads them.
    OnRead,
}
struct Verified {
    entry: Entry,
    blocks: Cow<'static, [[u8; 32]]>,
    /// Absent for a device-verified member, which has no blocks either.
    sha384: Option<[u8; 48]>,
    /// For recorded hashes not yet checked against the whole member: the
    /// SHA-384 of its bytes read in order from its start, and how far they
    /// reach.
    whole: Option<(u64, Sha384)>,
}
/// A few simultaneous streams can share one IPSW. Keep their positions
/// separate, with bounded inflater buffers, so alternating reads do not
/// replay each deflated member from byte zero.
#[derive(Default)]
struct Cursors(VecDeque<(String, Progress)>);

impl Cursors {
    // Each deflate cursor owns a 64 KiB input buffer, a 32 KiB window,
    // and the decoder state. Stored cursors have no inflater allocation.
    // A cursor whose read is pending also holds that read's bytes so far,
    // at most MAX_READ_BYTES plus a block.
    const LIMIT: usize = 6;

    fn read<R: ReadAt>(
        &mut self,
        archive: &Archive,
        reader: &mut R,
        entry: &Entry,
        offset: u64,
        output: &mut [u8],
    ) -> Result<()> {
        let cached = self
            .0
            .iter()
            .position(|(name, _)| *name == entry.name)
            .and_then(|index| self.0.remove(index));
        let (name, mut progress) = match cached {
            Some((name, progress)) => (Some(name), Some(progress)),
            None => (None, None),
        };
        let result = archive
            .read_member_resuming(reader, entry, &mut progress, offset, output)
            .map_err(|error| source_error(reader, error));
        // A pending read keeps its cursor, with the bytes it produced, so
        // asking again goes on from there instead of inflating the member's
        // prefix again; any other failure drops it.
        let keep = match &result {
            Ok(()) => true,
            Err(error) => matches!(error, Error::Pending),
        };
        if keep
            && let Some(progress) = progress
            && progress.resumes_from() < entry.bytes
        {
            if self.0.len() == Self::LIMIT {
                self.0.pop_front();
            }
            self.0
                .push_back((name.unwrap_or_else(|| entry.name.clone()), progress));
        }
        result
    }
}

pub struct Store<R> {
    reader: R,
    archive: Archive,
    manifest: PayloadManifest,
    /// This board's BuildIdentities, each with its original index.
    identities: Vec<Identity>,
    /// The selected identity's position in `identities`.
    current: usize,
    verified: BTreeMap<String, Verified>,
    block_count: usize,
    verify: Verify,
    cursors: Cursors,
    /// The member scan a pending read stopped ([`Store::verify`]).
    scan: Option<Box<Scan>>,
}
/// A member's SHA-256 scan in progress: where it is in the member and what
/// it has hashed, so a read that runs out of time costs no re-read.
struct Scan {
    path: String,
    progress: Progress,
    sha256: Sha256,
    sha384: Sha384,
    hashes: Vec<[u8; 32]>,
    /// The block being filled, and how far.
    block: Vec<u8>,
    filled: usize,
    /// Bytes hashed so far.
    total: u64,
}
fn payload(error: impl core::fmt::Debug) -> Error {
    Error::Payload(refuse!("payload: {error}", ?error))
}
/// A failed read of `reader`: [`Error::Pending`] when the source only ran
/// out of the time it may wait (`ReadAt::blocked`), so the caller asks
/// again later, else the payload's refusal.
fn source_error<R: ReadAt>(reader: &mut R, error: impl core::fmt::Debug) -> Error {
    match reader.blocked() {
        true => Error::Pending,
        false => payload(error),
    }
}
impl<R: ReadAt> Store<R> {
    /// `selected` is supplied by the inventory (20 for the acceptance input),
    /// never selected by a build/version exception in the runtime.
    pub fn open(mut reader: R, manifest: PayloadManifest, selected: usize) -> Result<Self> {
        manifest.validate().map_err(payload)?;
        let archive = Archive::open(&mut reader).map_err(payload)?;
        if archive.entries().windows(2).any(|p| p[0].name == p[1].name) {
            return Err(Error::Invalid("duplicate ZIP member"));
        }
        let mut names: Vec<_> = archive.entries().iter().map(|e| e.name.as_str()).collect();
        names.sort_unstable();
        if names.windows(2).any(|p| p[0] == p[1]) {
            return Err(Error::Invalid("duplicate ZIP member"));
        }
        let entry = archive
            .find("BuildManifest.plist")
            .ok_or(Error::Invalid("missing BuildManifest"))?;
        if entry.bytes > MAX_BUILD_MANIFEST_BYTES as u64 {
            return Err(Error::Limit("BuildManifest bytes"));
        }
        let mut bytes = vec![0; entry.bytes as usize];
        archive
            .read_member_at(&mut reader, entry, 0, &mut bytes)
            .map_err(payload)?;
        if stela_formats::sha256(&bytes) != manifest.source.build_manifest_sha256 {
            return Err(Error::Invalid("BuildManifest SHA-256"));
        }
        // A universal BuildManifest names every board. Only this board's
        // identities are kept, as they are decoded: the selected one and the
        // RecoveryOS identity `answers` looks up by ApChipID/ApBoardID.
        let mut identities = Vec::new();
        let mut count = 0;
        let mut sink = |index: usize, value: Value| {
            count = index + 1;
            let class = value
                .get("Info")
                .and_then(|info| info.get("DeviceClass"))
                .map(Value::string)
                .transpose()?;
            if index == selected || class == Some(manifest.device_class.as_str()) {
                identities.push(Identity { index, value });
            }
            Ok(())
        };
        plist::decode_streaming(
            &bytes,
            Limits {
                bytes: MAX_BUILD_MANIFEST_BYTES,
                nodes: 2_000_000,
                depth: 64,
            },
            plist::Stream {
                path: &["BuildIdentities"],
                sink: &mut sink,
            },
        )?;
        if count == 0 {
            return Err(Error::Invalid("BuildManifest has no BuildIdentities"));
        }
        drop(bytes);
        let current = identities
            .iter()
            .position(|identity| identity.index == selected)
            .ok_or(Error::Invalid("BuildManifest identity index"))?;
        let identity = &identities[current];
        // A restore runs an erase or an update identity; the inventory's
        // purpose must name the same behavior.
        let purpose = match identity.info()?.required("RestoreBehavior")?.string()? {
            ERASE_BEHAVIOR => Purpose::Erase,
            UPDATE_BEHAVIOR => Purpose::Update,
            _ => {
                return Err(Error::Invalid(
                    "selected identity is neither erase nor update",
                ));
            }
        };
        let set = manifest
            .identities
            .iter()
            .find(|s| s.index as usize == selected)
            .ok_or(Error::Invalid("identity absent from payload inventory"))?;
        if set.variant != identity.variant()?
            || set.purpose != purpose
            || identity.info()?.required("DeviceClass")?.string()? != manifest.device_class
        {
            return Err(Error::Invalid("payload/BuildManifest identity mismatch"));
        }
        Ok(Self {
            reader,
            archive,
            manifest,
            identities,
            current,
            verified: BTreeMap::new(),
            block_count: 0,
            verify: Verify::Full,
            cursors: Cursors::default(),
            scan: None,
        })
    }
    pub fn manifest(&self) -> &PayloadManifest {
        &self.manifest
    }
    pub(super) fn release_cursors(&mut self) {
        self.cursors.0.clear();
    }

    pub fn reader_mut(&mut self) -> &mut R {
        &mut self.reader
    }
    pub fn identity(&self) -> &Identity {
        &self.identities[self.current]
    }
    pub fn zip_entries(&self) -> &[Entry] {
        self.archive.entries()
    }
    pub fn identities(&self) -> &[Identity] {
        &self.identities
    }
    pub fn member_len(&self, path: &str) -> Result<u64> {
        self.manifest
            .member(path)
            .map(|m| m.bytes)
            .ok_or(Error::Invalid(
                "member absent from immutable payload inventory",
            ))
    }
    /// Admission/preflight work, before StartRestore: every inventory member
    /// is read whole and checked, against its recorded hashes or, without
    /// them, its SHA-256 (building its block hashes). With
    /// [`Verify::OnRead`] recorded members are left to the reads.
    pub fn prepare(&mut self) -> Result<()> {
        let paths: Vec<_> = self
            .manifest
            .members
            .iter()
            .map(|m| m.name.clone())
            .collect();
        let mut buffer = Vec::new();
        for path in paths {
            match self.verified.get(&path) {
                Some(recorded) if recorded.whole.is_some() && self.verify == Verify::Full => {
                    self.check_whole(&path, &mut buffer)?
                }
                Some(_) => {}
                None => self.verify(&path)?,
            }
        }
        Ok(())
    }
    /// When [`prepare`](Self::prepare) checks members with recorded hashes;
    /// [`Verify::Full`] unless set.
    pub fn set_verify(&mut self, verify: Verify) {
        self.verify = verify;
    }
    /// Take the manifest's recorded block hashes and SHA-384s from `file`,
    /// the contents of the file it names, in place of scanning to build
    /// them; members are checked against them as [`Verify`] says.
    pub fn adopt_block_hashes(&mut self, file: &[u8]) -> Result<()> {
        self.adopt_hashes(file, |blocks| Cow::Owned(blocks.to_vec()))
    }

    /// Adopt hashes from an immutable file that outlives the store.
    pub fn adopt_static_block_hashes(&mut self, file: &'static [u8]) -> Result<()> {
        self.adopt_hashes(file, Cow::Borrowed)
    }

    fn adopt_hashes<'a>(
        &mut self,
        file: &'a [u8],
        retain: impl Fn(&'a [[u8; 32]]) -> Cow<'static, [[u8; 32]]>,
    ) -> Result<()> {
        let blocks = self
            .manifest
            .block_hashes
            .as_ref()
            .ok_or(Error::Invalid("the payload names no block hashes"))?;
        if stela_formats::sha256(file) != blocks.sha256 {
            return Err(Error::Payload(refuse!(
                "{file}: SHA-256 differs from the payload manifest",
                file = blocks.file
            )));
        }
        let count = usize::try_from(self.manifest.hash_blocks())
            .map_err(|_| Error::Limit("member blocks"))?;
        if count > MAX_BLOCKS || file.len() != count * 32 {
            return Err(Error::Invalid("block hash file size"));
        }
        let hashes = file.as_chunks::<32>().0;
        let mut at = 0;
        let mut adopted = BTreeMap::new();
        for member in &self.manifest.members {
            if member.sha256.is_none() {
                adopted.insert(member.name.clone(), self.unhashed(&member.name)?);
                continue;
            }
            let entry = self
                .archive
                .find(&member.name)
                .ok_or(Error::Invalid("missing ZIP member"))?
                .clone();
            if entry.bytes != member.bytes {
                return Err(Error::Invalid("ZIP/payload member size mismatch"));
            }
            let mut sha384 = [0u8; 48];
            hex::decode_to_slice(member.sha384.as_deref().unwrap_or(""), &mut sha384)
                .map_err(|_| Error::Invalid("member SHA-384"))?;
            let n = member.bytes.div_ceil(BLOCK_BYTES as u64) as usize;
            let blocks = retain(&hashes[at..at + n]);
            at += n;
            adopted.insert(
                member.name.clone(),
                Verified {
                    entry,
                    blocks,
                    sha384: Some(sha384),
                    whole: Some((0, Sha384::new())),
                },
            );
        }
        self.block_count = count;
        self.verified = adopted;
        Ok(())
    }
    /// Whether every member is checked as [`Verify`] requires before
    /// StartRestore.
    pub fn prepared(&self) -> bool {
        self.manifest.members.iter().all(|m| {
            self.verified
                .get(&m.name)
                .is_some_and(|v| v.whole.is_none() || self.verify == Verify::OnRead)
        })
    }
    /// Blocks `blocks` of a member with recorded or scanned hashes, each
    /// checked against its hash: one read of the source, so the storage
    /// under it moves the whole run in its largest transfers.
    fn read_blocks(
        &mut self,
        path: &str,
        blocks: core::ops::Range<u64>,
        bytes: &mut Vec<u8>,
    ) -> Result<()> {
        let verified = &self.verified[path];
        let start = blocks.start * BLOCK_BYTES as u64;
        let end = (blocks.end * BLOCK_BYTES as u64).min(verified.entry.bytes);
        bytes.resize((end - start) as usize, 0);
        self.cursors.read(
            &self.archive,
            &mut self.reader,
            &verified.entry,
            start,
            bytes,
        )?;
        let expected = &verified.blocks[blocks.start as usize..blocks.end as usize];
        for (index, (block, expected)) in bytes.chunks(BLOCK_BYTES).zip(expected).enumerate() {
            let actual: [u8; 32] = Sha256::digest(block).into();
            if actual != *expected {
                return Err(Error::Payload(refuse!(
                    "{path}: source changed at block {block}",
                    path,
                    block = blocks.start + index as u64
                )));
            }
        }
        Ok(())
    }
    /// Read a member with recorded hashes whole, checking each block and
    /// then its SHA-384.
    fn check_whole(&mut self, path: &str, buffer: &mut Vec<u8>) -> Result<()> {
        let blocks = self.verified[path].blocks.len() as u64;
        let run = (MAX_READ_BYTES / BLOCK_BYTES) as u64;
        let mut hash = Sha384::new();
        for first in (0..blocks).step_by(run as usize) {
            self.read_blocks(path, first..(first + run).min(blocks), buffer)?;
            hash.update(&*buffer);
        }
        let verified = self.verified.get_mut(path).expect("recorded");
        if Some(<[u8; 48]>::from(hash.finalize())) != verified.sha384 {
            return Err(Error::Payload(refuse!(
                "{path}: SHA-384 differs from the payload manifest",
                path
            )));
        }
        verified.whole = None;
        Ok(())
    }
    /// A prepared member's SHA-384; `None` for a device-verified one.
    pub fn digest384(&self, path: &str) -> Result<Option<&[u8; 48]>> {
        self.verified
            .get(path)
            .map(|v| v.sha384.as_ref())
            .ok_or(Error::Invalid("member not prepared"))
    }
    /// A device-verified member's entry, which no hash covers.
    fn unhashed(&self, path: &str) -> Result<Verified> {
        let member = self
            .manifest
            .member(path)
            .ok_or(Error::Invalid("unlisted payload member"))?;
        let entry = self
            .archive
            .find(path)
            .ok_or(Error::Invalid("missing ZIP member"))?
            .clone();
        if entry.bytes != member.bytes {
            return Err(Error::Invalid("ZIP/payload member size mismatch"));
        }
        Ok(Verified {
            entry,
            blocks: Cow::Borrowed(&[]),
            sha384: None,
            whole: None,
        })
    }
    pub fn verify(&mut self, path: &str) -> Result<()> {
        if self.verified.contains_key(path) {
            return Ok(());
        }
        let member = self
            .manifest
            .member(path)
            .ok_or(Error::Invalid("unlisted payload member"))?;
        let Some(sha256) = member.sha256.clone() else {
            let unhashed = self.unhashed(path)?;
            self.verified.insert(path.into(), unhashed);
            return Ok(());
        };
        let entry = self
            .archive
            .find(path)
            .ok_or(Error::Invalid("missing ZIP member"))?
            .clone();
        if entry.bytes != member.bytes {
            return Err(Error::Invalid("ZIP/payload member size mismatch"));
        }
        let blocks = usize::try_from(member.bytes.div_ceil(BLOCK_BYTES as u64))
            .map_err(|_| Error::Limit("member blocks"))?;
        if blocks > MAX_BLOCKS - self.block_count {
            return Err(Error::Limit("payload hash index"));
        }
        // A scan a pending read stopped goes on where it stopped; any other
        // member's is dropped.
        let mut scan = match self.scan.take() {
            Some(scan) if scan.path == path => scan,
            _ => {
                let progress = self
                    .archive
                    .start_member(&mut self.reader, &entry)
                    .map_err(|error| source_error(&mut self.reader, error))?;
                Box::new(Scan {
                    path: path.into(),
                    progress,
                    sha256: Sha256::new(),
                    sha384: Sha384::new(),
                    hashes: Vec::new(),
                    block: vec![0; BLOCK_BYTES],
                    filled: 0,
                    total: 0,
                })
            }
        };
        loop {
            let at = scan.filled;
            let n =
                match self
                    .archive
                    .pump(&mut self.reader, &mut scan.progress, &mut scan.block[at..])
                {
                    Ok(n) => n,
                    Err(error) => {
                        let error = source_error(&mut self.reader, error);
                        if matches!(error, Error::Pending) {
                            self.scan = Some(scan);
                        }
                        return Err(error);
                    }
                };
            scan.filled += n;
            if n != 0 && scan.filled < BLOCK_BYTES {
                continue;
            }
            if scan.filled == 0 {
                break;
            }
            let done = core::mem::take(&mut scan.filled);
            scan.total += done as u64;
            if scan.total > member.bytes {
                return Err(Error::Invalid("member exceeds inventory"));
            }
            let Scan {
                sha256: hash,
                sha384: hash384,
                hashes,
                block,
                ..
            } = &mut *scan;
            hash.update(&block[..done]);
            hash384.update(&block[..done]);
            hashes.push(Sha256::digest(&block[..done]).into());
            if n == 0 {
                break;
            }
        }
        let Scan {
            sha256: hash,
            sha384: hash384,
            hashes,
            total,
            ..
        } = *scan;
        if total != member.bytes || hex::encode(hash.finalize()) != sha256 {
            return Err(Error::Payload(refuse!(
                "{path}: immutable SHA-256/length mismatch",
                path
            )));
        }
        self.block_count += hashes.len();
        self.verified.insert(
            path.into(),
            Verified {
                entry,
                blocks: Cow::Owned(hashes),
                sha384: Some(hash384.finalize().into()),
                whole: None,
            },
        );
        Ok(())
    }
    pub fn read_at(&mut self, path: &str, offset: u64, len: usize) -> Result<Vec<u8>> {
        self.read_at_reusing(path, offset, len, Vec::new())
    }

    pub(super) fn read_at_reusing(
        &mut self,
        path: &str,
        offset: u64,
        len: usize,
        mut result: Vec<u8>,
    ) -> Result<Vec<u8>> {
        if len > MAX_READ_BYTES {
            return Err(Error::Limit("payload random read"));
        }
        let size = self.member_len(path)?;
        if offset.checked_add(len as u64).is_none_or(|end| end > size) {
            return Err(Error::Invalid("payload read range"));
        }
        self.verify(path)?;
        result.clear();
        if self.verified[path].sha384.is_none() {
            return self.read_unhashed(path, offset, len, result);
        }
        if len == 0 {
            return Ok(result);
        }
        let block = BLOCK_BYTES as u64;
        let first = offset / block;
        self.read_blocks(
            path,
            first..(offset + len as u64).div_ceil(block),
            &mut result,
        )?;
        result.drain(..(offset - first * block) as usize);
        result.truncate(len);
        let verified = self.verified.get_mut(path).expect("verified above");
        if let Some((reached, hash)) = &mut verified.whole
            && *reached == offset
            && !result.is_empty()
        {
            hash.update(&result);
            *reached += result.len() as u64;
            if *reached == size {
                let actual: [u8; 48] = hash.clone().finalize().into();
                if Some(actual) != verified.sha384 {
                    return Err(Error::Payload(refuse!(
                        "{path}: SHA-384 differs from the payload manifest",
                        path
                    )));
                }
                verified.whole = None;
            }
        }
        Ok(result)
    }
    /// A device-verified member's bytes, as read.
    fn read_unhashed(
        &mut self,
        path: &str,
        offset: u64,
        len: usize,
        mut bytes: Vec<u8>,
    ) -> Result<Vec<u8>> {
        let entry = &self.verified[path].entry;
        bytes.resize(len, 0);
        self.cursors
            .read(&self.archive, &mut self.reader, entry, offset, &mut bytes)?;
        Ok(bytes)
    }
    pub fn read_component(&mut self, name: &str, offset: u64, len: usize) -> Result<Vec<u8>> {
        let path = self.identity().path(name)?.to_string();
        self.read_at(&path, offset, len)
    }
    pub fn read_small(&mut self, path: &str) -> Result<Vec<u8>> {
        let n =
            usize::try_from(self.member_len(path)?).map_err(|_| Error::Limit("small member"))?;
        self.read_at(path, 0, n)
    }
}

#[cfg(test)]
pub(crate) mod tests;
