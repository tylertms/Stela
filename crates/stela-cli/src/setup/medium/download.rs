//! Download straight into FAT32 pieces; no complete IPSW is cached on the PC.
//!
//! Each piece is synced before the next starts, and [`RESUME_FILE`] beside
//! the pieces records those done. A download that stopped goes on when the
//! image on the server is the same one (URL, entity tag, size) and the piece
//! size is too: each recorded piece is read back from the stick, in order, to
//! rebuild the whole image's hashes, and kept when it matches its recorded
//! hash; every missing or damaged piece is asked for again by range. The
//! catalog's SHA-1 of the whole image still decides.
use super::*;
use std::io;

/// The record of a download's finished pieces, in its build's directory
/// until the download is published.
pub(super) const RESUME_FILE: &str = "download-resume.json";

const BLOCK: usize = HASH_BLOCK_BYTES as usize;
const WINDOW: usize = 16 * BLOCK;

pub(in super::super) struct Download {
    pub image: restore::Image,
    pub bytes: u64,
    etag: String,
}

impl Download {
    /// Open the catalog's image on Apple's servers, pinning its size and
    /// entity tag, and copy its boot files to `boot` when bundles are
    /// prepared; the catalog's build is checked either way.
    pub(in super::super) fn prepare(
        image: restore::Image,
        model: &str,
        boot: Option<&Path>,
    ) -> Result<Self> {
        ensure!(
            image.sha1.is_some(),
            "the catalog provides no checksum for this macOS image, which an Offline installer needs"
        );
        let source = restore::open_image(&image)?;
        let download = Self {
            bytes: source.size(),
            etag: source.etag().into(),
            image,
        };
        let build = match boot {
            Some(boot) => {
                restore::inputs_from(source, model, boot)?;
                restore::payload::identity(boot, model)?.1
            }
            None => restore::payload::identity_of(source, model)?.1,
        };
        ensure!(
            build == download.image.build,
            "the catalog download contains a different macOS build"
        );
        Ok(download)
    }

    pub(in super::super) fn write(
        &self,
        volume: &Path,
        model: &str,
        piece_bytes: u64,
    ) -> Result<Written> {
        let source = restore::open_image(&self.image)?;
        ensure!(
            source.size() == self.bytes && source.etag() == self.etag,
            "the download changed after boot-file preparation; run setup again"
        );
        write_from(
            source,
            volume,
            model,
            &self.image,
            &self.etag,
            self.bytes,
            piece_bytes,
        )
    }
}

/// What [`RESUME_FILE`] holds: the object the pieces came from, and the
/// pieces synced so far, in order.
#[derive(Debug, Serialize, Deserialize, PartialEq, Eq)]
struct Resume {
    url: String,
    etag: String,
    total: u64,
    piece_bytes: u64,
    done: Vec<Piece>,
}

/// The pieces of the image a stopped setup left in its build's
/// `directory`: how many are there at their recorded size, and of how many
/// pieces the image is when a stopped download records it. Their bytes are
/// checked against their recorded hashes when the setup goes on.
pub(in super::super) fn pieces_on_usb(directory: &Path) -> (usize, Option<usize>) {
    let at_size = |path: PathBuf, bytes: u64| fs::metadata(path).is_ok_and(|m| m.len() == bytes);
    if let Ok(record) = stela_engine::read_json::<Resume>(&directory.join(RESUME_FILE)) {
        let total = piece_sizes(record.total, record.piece_bytes).len();
        let present = record
            .done
            .iter()
            .enumerate()
            .filter(|(index, piece)| {
                at_size(partial_path(directory, *index), piece.bytes)
                    || at_size(directory.join(piece_name(*index)), piece.bytes)
            })
            .count();
        return (present, Some(total));
    }
    // Pieces copied from a local IPSW, without a download record: every
    // whole one has the first one's size.
    let size = |index| fs::metadata(directory.join(piece_name(index))).map(|m| m.len());
    let Ok(first) = size(0) else {
        return (0, None);
    };
    let present = (0..MAX_PIECES)
        .take_while(|&index| size(index).is_ok())
        .filter(|&index| size(index).ok() == Some(first))
        .count();
    (present, None)
}

/// The pieces a stopped download of the same object recorded as synced;
/// an unreadable or different record resumes nothing. Each is checked
/// against its hash when the download reaches it ([`kept_piece`]).
fn recorded(directory: &Path, record: &Resume) -> Vec<Piece> {
    let Ok(previous) = stela_engine::read_json::<Resume>(&directory.join(RESUME_FILE)) else {
        return Vec::new();
    };
    if (
        &previous.url,
        &previous.etag,
        previous.total,
        previous.piece_bytes,
    ) != (&record.url, &record.etag, record.total, record.piece_bytes)
    {
        return Vec::new();
    }
    previous.done
}

/// The pieces of a complete medium of the same image in `manifest_path`
/// (an earlier setup's, downloaded or copied from a local IPSW): a repair
/// checks them as a stopped download's and downloads only the missing or
/// damaged ones. Empty for another image or piece size.
fn finished(manifest_path: &Path, image: &restore::Image, total: u64, sizes: &[u64]) -> Vec<Piece> {
    let Some(manifest) = fs::read(manifest_path)
        .ok()
        .and_then(|bytes| stela_formats::payload::PayloadManifest::parse(&bytes).ok())
    else {
        return Vec::new();
    };
    let Layout::Pieces { pieces, .. } = manifest.layout else {
        return Vec::new();
    };
    // A medium copied from a local IPSW names no catalog digest; the whole
    // image's is checked against the catalog's all the same.
    let same_image = manifest.source.bytes == total
        && manifest.product_build == image.build
        && manifest.source.catalog_sha1.as_ref().is_none_or(|digest| {
            image
                .sha1
                .as_ref()
                .is_some_and(|expected| expected.eq_ignore_ascii_case(digest))
        });
    match same_image
        && pieces
            .iter()
            .map(|piece| piece.bytes)
            .eq(sizes.iter().copied())
    {
        true => pieces,
        false => Vec::new(),
    }
}

/// Whether the piece `index` a stopped download recorded is on the stick as
/// recorded: read back once, its bytes go into the whole image's hashes and
/// its block hashes into `blocks`. A piece already promoted by a stop
/// during publication becomes a partial again. A missing, short or
/// damaged piece leaves the hashes as they were.
fn kept_piece(
    directory: &Path,
    index: usize,
    piece: &Piece,
    wholes: (&mut Sha256, &mut Sha1),
    blocks: &mut Vec<u8>,
    progress: &crate::progress::Phase,
) -> bool {
    let partial = partial_path(directory, index);
    let at_size = |path: &Path| fs::metadata(path).is_ok_and(|meta| meta.len() == piece.bytes);
    let promoted = directory.join(piece_name(index));
    if !partial.exists() && at_size(&promoted) && fs::rename(&promoted, &partial).is_err() {
        return false;
    }
    // The partial is not longer than its piece, or its tail would sit
    // where the next piece's first bytes go.
    if !at_size(&partial) {
        return false;
    }
    let (whole_sha256, whole_sha1) = wholes;
    let (mut hashes, mut sha256, mut sha1) =
        (Hashes::default(), whole_sha256.clone(), whole_sha1.clone());
    let read = stream_file(&partial, piece.bytes, progress, |bytes| {
        hashes.update(bytes);
        sha256.update(bytes);
        sha1.update(bytes);
    });
    let (digest, digests) = hashes.finish();
    if read.is_err() || digest != piece.sha256 {
        return false;
    }
    (*whole_sha256, *whole_sha1) = (sha256, sha1);
    blocks.extend(digests);
    true
}

fn write_from(
    mut source: impl Read + Seek,
    volume: &Path,
    model: &str,
    image: &restore::Image,
    etag: &str,
    total: u64,
    piece_bytes: u64,
) -> Result<Written> {
    ensure!(
        piece_bytes > 0
            && piece_bytes <= stela_formats::payload::MAX_SLICE_BYTES
            && piece_bytes.is_multiple_of(HASH_BLOCK_BYTES),
        "invalid FAT32 piece size"
    );
    let sizes = piece_sizes(total, piece_bytes);
    ensure!(
        !sizes.is_empty() && sizes.len() <= MAX_PIECES,
        "invalid download size or piece size"
    );
    let directory = directory(volume, &image.build);
    fs::create_dir_all(&directory)
        .with_context(|| format!("cannot create {}", directory.display()))?;
    let manifest_path = directory.join(MANIFEST_FILE);
    let written = finished(&manifest_path, image, total, &sizes);
    if manifest_path.exists() {
        fs::remove_file(&manifest_path)
            .with_context(|| format!("cannot remove {}", manifest_path.display()))?;
    }
    let mut whole_sha256 = Sha256::new();
    let mut whole_sha1 = Sha1::new();
    let mut record = Resume {
        url: image.url.clone(),
        etag: etag.into(),
        total,
        piece_bytes,
        done: Vec::new(),
    };
    let resume_path = directory.join(RESUME_FILE);
    // The record keeps every piece a stopped download synced, also those
    // after a damaged one, until each is checked or downloaded again.
    record.done = recorded(&directory, &record);
    if record.done.is_empty() {
        record.done = written;
    }
    if !record.done.is_empty() {
        eprintln!(
            "note: checking the {} of {} pieces already on the stick; only missing or damaged \
             pieces are downloaded again",
            record.done.len().min(sizes.len()),
            sizes.len()
        );
    }
    let progress = crate::progress::Phase::bytes("Downloading macOS onto the stick", total);
    let (mut pieces, mut piece_blocks) = (Vec::new(), Vec::new());
    let (mut kept, mut position) = (0, 0);
    let mut offset = 0;
    for (index, &size) in sizes.iter().enumerate() {
        let previous = record.done.get(index).filter(|piece| piece.bytes == size);
        if let Some(piece) = previous
            && kept_piece(
                &directory,
                index,
                piece,
                (&mut whole_sha256, &mut whole_sha1),
                &mut piece_blocks,
                &progress,
            )
        {
            pieces.push(piece.clone());
            kept += 1;
            offset += size;
            continue;
        }
        // A damaged piece's check counts no progress.
        progress.position(offset);
        if position != offset {
            source.seek(SeekFrom::Start(offset))?;
        }
        // A piece of the medium that was not kept is replaced: it goes
        // first, so a full USB has room for the new one.
        let promoted = directory.join(piece_name(index));
        if promoted.exists() {
            fs::remove_file(&promoted)
                .with_context(|| format!("cannot remove {}", promoted.display()))?;
        }
        let partial = partial_path(&directory, index);
        let writing = || format!("cannot write {}", partial.display());
        let mut file = fs::File::create(&partial).with_context(writing)?;
        let mut hashes = Hashes::default();
        stream(&mut source, size, |bytes| {
            file.write_all(bytes).with_context(writing)?;
            hashes.update(bytes);
            whole_sha256.update(bytes);
            whole_sha1.update(bytes);
            progress.advance(bytes.len() as u64);
            Ok(())
        })?;
        file.sync_all().with_context(writing)?;
        crate::progress::drop_cached(&file).with_context(writing)?;
        offset += size;
        position = offset;
        let (sha256, blocks) = hashes.finish();
        let piece = Piece {
            bytes: size,
            sha256,
        };
        match record.done.get_mut(index) {
            Some(recorded) => *recorded = piece.clone(),
            None => record.done.push(piece.clone()),
        }
        pieces.push(piece);
        piece_blocks.extend(blocks);
        efi::write_atomic(&resume_path, &serde_json::to_vec(&record)?, false)?;
    }
    if position != total {
        source.seek(SeekFrom::Start(total))?;
    }
    ensure!(
        source.read(&mut [0])? == 0,
        "download exceeds the expected image size"
    );
    let sha256 = hex::encode(whole_sha256.finalize());
    let sha1 = hex::encode(whole_sha1.finalize());
    if !image
        .sha1
        .as_ref()
        .is_some_and(|expected| expected.eq_ignore_ascii_case(&sha1))
    {
        // Nothing of it can be trusted to go on from.
        let _ = fs::remove_file(&resume_path);
        bail!("download differs from the catalog digest");
    }
    progress.finish();
    let mut completing = false;
    let inventory = (|| {
        let joined = Joined::new(&directory, piece_bytes, total, &piece_blocks)?;
        restore::payload::inventory_from(
            joined,
            restore::payload::VerifiedSource {
                image: Path::new(&image.url)
                    .file_name()
                    .map(Path::new)
                    .context("image URL has no file name")?,
                model,
                digest: (total, sha256),
            },
            |joined| {
                completing = true;
                joined.complete(&pieces)
            },
        )
    })();
    if !completing {
        // If inventory failed early, retain the old ordered verification and
        // promotion. A media failure still takes precedence and leaves the
        // same verified prefix promoted before that failure.
        let progress = crate::progress::Phase::bytes("Verifying the stick", total);
        promote(&directory, &pieces, |_, piece, partial| {
            ensure!(
                hash_file(partial, piece.bytes, &progress)? == piece.sha256,
                "the stick's data does not read back as written"
            );
            Ok(())
        })?;
        progress.finish();
    }
    let mut inventory = inventory?;
    ensure!(
        inventory.manifest.product_build == image.build,
        "download contains a different macOS build"
    );
    inventory.manifest.source.catalog_sha1 = Some(sha1);
    inventory.manifest.layout = Layout::Pieces {
        piece_bytes,
        pieces,
        block_hashes: BlockHashes {
            file: PIECE_HASHES_FILE.into(),
            block_bytes: HASH_BLOCK_BYTES,
            sha256: stela_formats::sha256(&piece_blocks),
        },
    };
    inventory.manifest.validate().map_err(anyhow::Error::msg)?;
    efi::write_atomic(&directory.join(PIECE_HASHES_FILE), &piece_blocks, false)?;
    efi::write_atomic(&directory.join(BLOCK_HASHES_FILE), &inventory.blocks, false)?;
    let json = inventory.manifest.to_json();
    efi::write_atomic(&manifest_path, json.as_bytes(), false)?;
    match fs::remove_file(&resume_path) {
        Err(error) if error.kind() != io::ErrorKind::NotFound => {
            return Err(error).with_context(|| format!("cannot remove {}", resume_path.display()));
        }
        _ => {}
    }
    Ok(Written {
        wrote: sizes.len() - kept,
        kept,
        manifest_sha256: stela_formats::sha256(json.as_bytes()),
    })
}

fn partial_path(directory: &Path, index: usize) -> PathBuf {
    directory.join(format!("{}.partial", piece_name(index)))
}

/// Verify and promote each piece in the original order, leaving later
/// partials alone on failure.
fn promote(
    directory: &Path,
    pieces: &[Piece],
    mut verify: impl FnMut(usize, &Piece, &Path) -> Result<()>,
) -> Result<()> {
    for (index, piece) in pieces.iter().enumerate() {
        let path = directory.join(piece_name(index));
        let partial = partial_path(directory, index);
        verify(index, piece, &partial)?;
        if path.exists() {
            fs::remove_file(&path).with_context(|| format!("cannot remove {}", path.display()))?;
        }
        fs::rename(&partial, &path).with_context(|| {
            format!("cannot rename {} to {}", partial.display(), path.display())
        })?;
    }
    Ok(())
}

/// ZIP's seekable view of the synced partials. The bitmap records readback
/// coverage, but every window fetched from disk is verified again.
struct Joined<'a> {
    directory: &'a Path,
    piece_bytes: u64,
    total: u64,
    position: u64,
    current: Option<(usize, fs::File)>,
    hashes: &'a [[u8; 32]],
    checked: Vec<u8>,
    checked_bytes: u64,
    buffer: Vec<u8>,
    window: Option<(u64, usize)>,
}

impl<'a> Joined<'a> {
    fn new(directory: &'a Path, piece_bytes: u64, total: u64, hashes: &'a [u8]) -> Result<Self> {
        let (hashes, tail) = hashes.as_chunks::<32>();
        ensure!(
            tail.is_empty() && hashes.len() as u64 == total.div_ceil(HASH_BLOCK_BYTES),
            "the stick's readback has the wrong number of block digests"
        );
        Ok(Self {
            directory,
            piece_bytes,
            total,
            position: 0,
            current: None,
            hashes,
            checked: vec![0; hashes.len().div_ceil(8)],
            checked_bytes: 0,
            buffer: vec![0; WINDOW],
            window: None,
        })
    }

    fn checked(&self, block: usize) -> bool {
        self.checked[block / 8] & (1 << (block % 8)) != 0
    }

    fn load(&mut self, start: u64, bytes: usize) -> io::Result<()> {
        self.window = None;
        let index = (start / self.piece_bytes) as usize;
        let offset = start % self.piece_bytes;
        debug_assert!(start.is_multiple_of(HASH_BLOCK_BYTES));
        debug_assert!(bytes <= WINDOW && bytes as u64 <= self.piece_bytes - offset);
        let path = partial_path(self.directory, index);
        let reading = |error: io::Error| {
            io::Error::new(
                error.kind(),
                format!("cannot read {}: {error}", path.display()),
            )
        };
        if self.current.as_ref().is_none_or(|(old, _)| *old != index) {
            self.current = Some((index, fs::File::open(&path).map_err(reading)?));
        }
        let file = &mut self.current.as_mut().unwrap().1;
        file.seek(SeekFrom::Start(offset)).map_err(reading)?;
        file.read_exact(&mut self.buffer[..bytes])
            .map_err(reading)?;
        let first = (start / HASH_BLOCK_BYTES) as usize;
        for (index, bytes) in self.buffer[..bytes].chunks(BLOCK).enumerate() {
            let block = first + index;
            if Sha256::digest(bytes).as_slice() != self.hashes[block] {
                return Err(io::Error::new(
                    io::ErrorKind::InvalidData,
                    "the stick's data does not read back as written",
                ));
            }
            let bits = &mut self.checked[block / 8];
            let bit = 1 << (block % 8);
            if *bits & bit == 0 {
                *bits |= bit;
                self.checked_bytes += bytes.len() as u64;
            }
        }
        self.window = Some((start, bytes));
        Ok(())
    }

    fn complete(mut self, pieces: &[Piece]) -> Result<()> {
        self.current = None;
        let directory = self.directory;
        let progress = crate::progress::Phase::bytes(
            "Verifying the rest of the stick",
            self.total - self.checked_bytes,
        );
        promote(directory, pieces, |index, piece, _| {
            let start = index as u64 * self.piece_bytes;
            let end = start + piece.bytes;
            let mut block = (start / HASH_BLOCK_BYTES) as usize;
            let last = end.div_ceil(HASH_BLOCK_BYTES) as usize;
            while block < last {
                if self.checked(block) {
                    block += 1;
                    continue;
                }
                let blocks = (block..last)
                    .take(WINDOW / BLOCK)
                    .take_while(|&block| !self.checked(block))
                    .count();
                let start = block as u64 * HASH_BLOCK_BYTES;
                let bytes = (end - start).min((blocks * BLOCK) as u64) as usize;
                self.load(start, bytes)?;
                progress.advance(bytes as u64);
                block += blocks;
            }
            self.current = None;
            Ok(())
        })?;
        progress.finish();
        Ok(())
    }
}

impl Read for Joined<'_> {
    fn read(&mut self, bytes: &mut [u8]) -> io::Result<usize> {
        if self.position == self.total || bytes.is_empty() {
            return Ok(0);
        }
        if self
            .window
            .is_none_or(|(start, len)| !(start..start + len as u64).contains(&self.position))
        {
            let offset = self.position % self.piece_bytes;
            let start = self.position - offset % WINDOW as u64;
            let bytes = (self.total - start)
                .min(self.piece_bytes - start % self.piece_bytes)
                .min(WINDOW as u64);
            self.load(start, bytes as usize)?;
        }
        let (start, len) = self.window.unwrap();
        let offset = (self.position - start) as usize;
        let count = bytes.len().min(len - offset);
        bytes[..count].copy_from_slice(&self.buffer[offset..offset + count]);
        self.position += count as u64;
        Ok(count)
    }
}

impl Seek for Joined<'_> {
    fn seek(&mut self, from: SeekFrom) -> io::Result<u64> {
        let position = match from {
            SeekFrom::Start(n) => i128::from(n),
            SeekFrom::Current(n) => i128::from(self.position) + i128::from(n),
            SeekFrom::End(n) => i128::from(self.total) + i128::from(n),
        };
        if position < 0 || position > i128::from(self.total) {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "seek outside the stick's image",
            ));
        }
        self.position = position as u64;
        Ok(self.position)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const ETAG: &str = "\"pinned\"";

    /// A seekable source that fails once it reaches `stop_at`, as a stopped
    /// download does, and counts what was read and where it was sought.
    struct Tracked {
        data: io::Cursor<Vec<u8>>,
        stop_at: Option<u64>,
        seeks: Vec<u64>,
        read: u64,
    }

    impl Tracked {
        fn new(data: &[u8], stop_at: Option<u64>) -> Self {
            Self {
                data: io::Cursor::new(data.to_vec()),
                stop_at,
                seeks: Vec::new(),
                read: 0,
            }
        }
    }

    impl Read for Tracked {
        fn read(&mut self, output: &mut [u8]) -> io::Result<usize> {
            let at = self.data.position();
            let room = match self.stop_at {
                Some(stop) if at >= stop => return Err(io::Error::other("connection lost")),
                Some(stop) => (stop - at) as usize,
                None => usize::MAX,
            };
            let take = output.len().min(room);
            let n = self.data.read(&mut output[..take])?;
            self.read += n as u64;
            Ok(n)
        }
    }

    impl Seek for Tracked {
        fn seek(&mut self, to: SeekFrom) -> io::Result<u64> {
            let at = self.data.seek(to)?;
            self.seeks.push(at);
            Ok(at)
        }
    }

    fn synthetic(temp: &Path) -> Result<(Vec<u8>, restore::Image)> {
        let ipsw = temp.join("source.ipsw");
        crate::efi::synthetic_ipsw(&ipsw)?;
        let bytes = fs::read(&ipsw)?;
        let image = restore::Image {
            version: "test".into(),
            build: restore::payload::SYNTHETIC_BUILD.into(),
            models: vec![restore::payload::SYNTHETIC_MODEL.into()],
            url: "https://updates.cdn-apple.com/source.ipsw".into(),
            sha1: Some(hex::encode(Sha1::digest(&bytes))),
        };
        Ok((bytes, image))
    }

    fn write(
        volume: &Path,
        image: &restore::Image,
        etag: &str,
        source: &mut Tracked,
    ) -> Result<Written> {
        let total = source.data.get_ref().len() as u64;
        write_from(
            source,
            volume,
            restore::payload::SYNTHETIC_MODEL,
            image,
            etag,
            total,
            HASH_BLOCK_BYTES,
        )
    }

    #[test]
    fn a_stopped_download_goes_on_after_its_last_synced_piece() -> Result<()> {
        let temp = tempfile::tempdir()?;
        let (bytes, image) = synthetic(temp.path())?;
        let piece = HASH_BLOCK_BYTES;
        assert!(bytes.len() as u64 > piece + 100, "the fixture needs pieces");
        let volume = temp.path().join("usb");
        let path = directory(&volume, &image.build);
        // Stopped in the second piece: the first is synced and recorded.
        let mut stopped = Tracked::new(&bytes, Some(piece + 100));
        assert!(write(&volume, &image, ETAG, &mut stopped).is_err());
        assert!(path.join(RESUME_FILE).exists());
        let count = bytes.len().div_ceil(piece as usize);
        assert_eq!(pieces_on_usb(&path), (1, Some(count)));
        let mut resumed = Tracked::new(&bytes, None);
        let written = write(&volume, &image, ETAG, &mut resumed)?;
        assert_eq!(resumed.seeks, [piece]);
        assert_eq!(resumed.read, bytes.len() as u64 - piece);
        assert!(!path.join(RESUME_FILE).exists());
        let manifest = fs::read(path.join(MANIFEST_FILE))?;
        assert_eq!(written.manifest_sha256, stela_formats::sha256(&manifest));
        for (index, part) in bytes.chunks(piece as usize).enumerate() {
            assert_eq!(fs::read(path.join(piece_name(index)))?, part);
        }
        Ok(())
    }

    /// [`synthetic`] with a stored member of `padding` bytes added, so the
    /// image takes more pieces.
    fn padded(temp: &Path, padding: usize) -> Result<(Vec<u8>, restore::Image)> {
        let ipsw = temp.join("padded.ipsw");
        crate::efi::synthetic_ipsw(&ipsw)?;
        let file = fs::File::options().read(true).write(true).open(&ipsw)?;
        let mut zip = zip::ZipWriter::new_append(file)?;
        let options = zip::write::SimpleFileOptions::default()
            .compression_method(zip::CompressionMethod::Stored);
        zip.start_file("Padding.bin", options)?;
        let fill: Vec<u8> = (0..padding).map(|at| (at % 251) as u8).collect();
        zip.write_all(&fill)?;
        zip.finish()?;
        let bytes = fs::read(&ipsw)?;
        let (_, mut image) = synthetic(temp)?;
        image.sha1 = Some(hex::encode(Sha1::digest(&bytes)));
        Ok((bytes, image))
    }

    #[test]
    fn only_a_damaged_piece_is_downloaded_again_and_promoted_pieces_are_kept() -> Result<()> {
        let temp = tempfile::tempdir()?;
        let (bytes, image) = padded(temp.path(), 4 * HASH_BLOCK_BYTES as usize)?;
        let piece = HASH_BLOCK_BYTES;
        assert!(
            bytes.len() as u64 > 3 * piece + 100,
            "the fixture needs pieces"
        );
        let volume = temp.path().join("usb");
        let path = directory(&volume, &image.build);
        // Stopped in the fourth piece: three are synced and recorded.
        let mut stopped = Tracked::new(&bytes, Some(3 * piece + 100));
        assert!(write(&volume, &image, ETAG, &mut stopped).is_err());
        // The second no longer reads back as recorded, and the first was
        // promoted by a stop while the download was being published.
        fs::write(partial_path(&path, 1), vec![0; piece as usize])?;
        fs::rename(partial_path(&path, 0), path.join(piece_name(0)))?;
        let mut again = Tracked::new(&bytes, None);
        let written = write(&volume, &image, ETAG, &mut again)?;
        assert_eq!(again.seeks, [piece, 3 * piece]);
        assert_eq!(again.read, bytes.len() as u64 - 2 * piece);
        assert_eq!(written.kept, 2);
        assert_eq!(written.wrote, bytes.len().div_ceil(piece as usize) - 2);
        assert!(!path.join(RESUME_FILE).exists());
        for (index, part) in bytes.chunks(piece as usize).enumerate() {
            assert_eq!(fs::read(path.join(piece_name(index)))?, part);
        }
        Ok(())
    }

    #[test]
    fn a_repair_of_a_complete_medium_downloads_only_its_damaged_pieces() -> Result<()> {
        let temp = tempfile::tempdir()?;
        let (bytes, image) = padded(temp.path(), 4 * HASH_BLOCK_BYTES as usize)?;
        let piece = HASH_BLOCK_BYTES;
        let volume = temp.path().join("usb");
        let path = directory(&volume, &image.build);
        write(&volume, &image, ETAG, &mut Tracked::new(&bytes, None))?;
        assert!(!path.join(RESUME_FILE).exists());
        // One piece's bytes changed, another cut short.
        fs::write(path.join(piece_name(1)), vec![0; piece as usize])?;
        fs::write(path.join(piece_name(3)), b"short")?;
        let mut again = Tracked::new(&bytes, None);
        let written = write(&volume, &image, ETAG, &mut again)?;
        // The two pieces, then the end, to find nothing after it.
        assert_eq!(again.seeks, [piece, 3 * piece, bytes.len() as u64]);
        assert_eq!(again.read, 2 * piece);
        let count = bytes.len().div_ceil(piece as usize);
        assert_eq!((written.kept, written.wrote), (count - 2, 2));
        for (index, part) in bytes.chunks(piece as usize).enumerate() {
            assert_eq!(fs::read(path.join(piece_name(index)))?, part);
        }
        // A medium of another image keeps nothing.
        let elsewhere = tempfile::tempdir()?;
        let (other, mut changed) = padded(elsewhere.path(), 5 * HASH_BLOCK_BYTES as usize)?;
        changed.sha1 = Some(hex::encode(Sha1::digest(&other)));
        let mut fresh = Tracked::new(&other, None);
        assert_eq!(write(&volume, &changed, ETAG, &mut fresh)?.kept, 0);
        assert_eq!(fresh.read, other.len() as u64);
        Ok(())
    }

    #[test]
    fn a_stop_after_a_damaged_piece_is_fetched_again_keeps_the_later_pieces() -> Result<()> {
        let temp = tempfile::tempdir()?;
        let (bytes, image) = padded(temp.path(), 4 * HASH_BLOCK_BYTES as usize)?;
        let piece = HASH_BLOCK_BYTES;
        let volume = temp.path().join("usb");
        let path = directory(&volume, &image.build);
        let mut stopped = Tracked::new(&bytes, Some(3 * piece + 100));
        assert!(write(&volume, &image, ETAG, &mut stopped).is_err());
        fs::write(partial_path(&path, 0), vec![0; piece as usize])?;
        // Stopped again once the first was fetched again: the second and
        // third are still recorded.
        let mut again = Tracked::new(&bytes, Some(piece + 1));
        assert!(write(&volume, &image, ETAG, &mut again).is_err());
        assert_eq!(pieces_on_usb(&path).0, 3);
        let mut last = Tracked::new(&bytes, None);
        let written = write(&volume, &image, ETAG, &mut last)?;
        assert_eq!(last.seeks, [3 * piece]);
        assert_eq!(written.kept, 3);
        for (index, part) in bytes.chunks(piece as usize).enumerate() {
            assert_eq!(fs::read(path.join(piece_name(index)))?, part);
        }
        Ok(())
    }

    #[test]
    fn a_changed_object_or_a_damaged_piece_is_downloaded_again() -> Result<()> {
        let temp = tempfile::tempdir()?;
        let (bytes, image) = synthetic(temp.path())?;
        let piece = HASH_BLOCK_BYTES;
        let volume = temp.path().join("usb");
        let path = directory(&volume, &image.build);
        // The synced piece no longer reads back as recorded.
        let mut stopped = Tracked::new(&bytes, Some(piece + 1));
        assert!(write(&volume, &image, ETAG, &mut stopped).is_err());
        fs::write(partial_path(&path, 0), vec![0; piece as usize])?;
        let mut again = Tracked::new(&bytes, None);
        write(&volume, &image, ETAG, &mut again)?;
        assert!(again.seeks.is_empty());
        assert_eq!(again.read, bytes.len() as u64);
        // Another entity tag is another object.
        let mut stopped = Tracked::new(&bytes, Some(piece + 1));
        assert!(write(&volume, &image, ETAG, &mut stopped).is_err());
        let mut fresh = Tracked::new(&bytes, None);
        write(&volume, &image, "\"other\"", &mut fresh)?;
        assert!(fresh.seeks.is_empty());
        assert_eq!(fresh.read, bytes.len() as u64);
        assert!(!path.join(RESUME_FILE).exists());
        for (index, part) in bytes.chunks(piece as usize).enumerate() {
            assert_eq!(fs::read(path.join(piece_name(index)))?, part);
        }
        Ok(())
    }

    #[test]
    fn a_download_that_misses_the_catalog_digest_forgets_its_pieces() -> Result<()> {
        let temp = tempfile::tempdir()?;
        let (bytes, mut image) = synthetic(temp.path())?;
        image.sha1 = Some(hex::encode(Sha1::digest(b"another image")));
        let volume = temp.path().join("usb");
        let mut source = Tracked::new(&bytes, None);
        let error = write(&volume, &image, ETAG, &mut source).unwrap_err();
        assert_eq!(
            error.to_string(),
            "download differs from the catalog digest"
        );
        assert!(!directory(&volume, &image.build).join(RESUME_FILE).exists());
        Ok(())
    }

    fn partials(
        directory: &Path,
        data: &[u8],
        piece_bytes: usize,
    ) -> Result<(Vec<Piece>, Vec<u8>)> {
        let mut pieces = Vec::new();
        let mut blocks = Vec::new();
        for (index, part) in data.chunks(piece_bytes).enumerate() {
            fs::write(partial_path(directory, index), part)?;
            let mut hashes = Hashes::default();
            hashes.update(part);
            let (sha256, digests) = hashes.finish();
            pieces.push(Piece {
                bytes: part.len() as u64,
                sha256,
            });
            blocks.extend(digests);
        }
        Ok((pieces, blocks))
    }

    #[test]
    fn readback_tracks_unique_windows_and_a_short_final_block() -> Result<()> {
        let directory = tempfile::tempdir()?;
        let piece = WINDOW + BLOCK;
        let data: Vec<u8> = (0..2 * piece + 29).map(|at| (at / 103) as u8).collect();
        let (pieces, hashes) = partials(directory.path(), &data, piece)?;
        let mut joined = Joined::new(directory.path(), piece as u64, data.len() as u64, &hashes)?;
        let mut bytes = [0; 4];
        joined.seek(SeekFrom::Start(piece as u64 - 2))?;
        joined.read_exact(&mut bytes)?;
        assert_eq!(bytes, data[piece - 2..piece + 2]);
        assert_eq!(joined.checked_bytes, (WINDOW + BLOCK) as u64);
        joined.seek(SeekFrom::End(-4))?;
        joined.read_exact(&mut bytes)?;
        assert_eq!(bytes, data[data.len() - 4..]);
        assert_eq!(joined.checked_bytes, (WINDOW + BLOCK + 29) as u64);
        joined.seek(SeekFrom::Start(piece as u64 - 2))?;
        joined.read_exact(&mut bytes)?;
        assert_eq!(bytes, data[piece - 2..piece + 2]);
        assert_eq!(joined.checked_bytes, (WINDOW + BLOCK + 29) as u64);
        joined.complete(&pieces)?;
        for (index, part) in data.chunks(piece).enumerate() {
            assert_eq!(fs::read(directory.path().join(piece_name(index)))?, part);
            assert!(!partial_path(directory.path(), index).exists());
        }
        Ok(())
    }

    #[test]
    fn rereading_a_window_checks_for_corruption_before_returning_bytes() -> Result<()> {
        let directory = tempfile::tempdir()?;
        let data = vec![7; 2 * BLOCK];
        let (_, hashes) = partials(directory.path(), &data, BLOCK)?;
        let mut joined = Joined::new(directory.path(), BLOCK as u64, data.len() as u64, &hashes)?;
        let mut byte = [0];
        joined.read_exact(&mut byte)?;
        assert_eq!(byte, [7]);
        joined.seek(SeekFrom::Start(BLOCK as u64))?;
        joined.read_exact(&mut byte)?;
        fs::write(partial_path(directory.path(), 0), vec![8; BLOCK])?;
        joined.seek(SeekFrom::Start(0))?;
        let mut output = [42; 2];
        assert_eq!(
            joined.read(&mut output).unwrap_err().kind(),
            io::ErrorKind::InvalidData
        );
        assert_eq!(joined.position, 0);
        assert_eq!(output, [42; 2]);
        Ok(())
    }

    #[test]
    fn untouched_readback_failure_keeps_the_original_promotion_prefix() -> Result<()> {
        let directory = tempfile::tempdir()?;
        let data = vec![7; 2 * BLOCK + 11];
        for truncate in [false, true] {
            let path = directory
                .path()
                .join(if truncate { "short" } else { "corrupt" });
            fs::create_dir(&path)?;
            let (pieces, hashes) = partials(&path, &data, BLOCK)?;
            let mut joined = Joined::new(&path, BLOCK as u64, data.len() as u64, &hashes)?;
            let mut byte = [0];
            joined.read_exact(&mut byte)?;
            let old = path.join(piece_name(1));
            fs::write(&old, b"old piece")?;
            fs::write(
                partial_path(&path, 1),
                vec![9; if truncate { 4 } else { BLOCK }],
            )?;
            let error = joined.complete(&pieces).unwrap_err();
            assert_eq!(
                error.downcast_ref::<io::Error>().unwrap().kind(),
                if truncate {
                    io::ErrorKind::UnexpectedEof
                } else {
                    io::ErrorKind::InvalidData
                }
            );
            assert_eq!(fs::read(path.join(piece_name(0)))?, &data[..BLOCK]);
            assert!(!partial_path(&path, 0).exists());
            assert_eq!(fs::read(old)?, b"old piece");
            assert!(partial_path(&path, 1).exists());
            assert!(partial_path(&path, 2).exists());
        }
        Ok(())
    }

    #[test]
    fn invalid_archive_keeps_the_old_fallback_promotion() -> Result<()> {
        struct CorruptAtEof<'a> {
            data: &'a [u8],
            partial: PathBuf,
        }

        impl Seek for CorruptAtEof<'_> {
            fn seek(&mut self, _: SeekFrom) -> io::Result<u64> {
                Err(io::Error::other("this source does not seek"))
            }
        }

        impl Read for CorruptAtEof<'_> {
            fn read(&mut self, output: &mut [u8]) -> io::Result<usize> {
                if self.data.is_empty() {
                    fs::write(&self.partial, [8; 17])?;
                }
                self.data.read(output)
            }
        }

        let temp = tempfile::tempdir()?;
        let data = vec![9; BLOCK + 17];
        let image = restore::Image {
            version: "test".into(),
            build: restore::payload::SYNTHETIC_BUILD.into(),
            models: vec![restore::payload::SYNTHETIC_MODEL.into()],
            url: "https://updates.cdn-apple.com/source.ipsw".into(),
            sha1: Some(hex::encode(Sha1::digest(&data))),
        };
        let volume = temp.path();
        assert!(
            write_from(
                io::Cursor::new(&data[..]),
                volume,
                restore::payload::SYNTHETIC_MODEL,
                &image,
                ETAG,
                data.len() as u64,
                BLOCK as u64,
            )
            .is_err()
        );
        let path = directory(volume, &image.build);
        for (index, part) in data.chunks(BLOCK).enumerate() {
            assert_eq!(fs::read(path.join(piece_name(index)))?, part);
            assert!(!partial_path(&path, index).exists());
        }
        assert!(!path.join(MANIFEST_FILE).exists());

        let volume = temp.path().join("corrupt");
        let path = directory(&volume, &image.build);
        let partial = partial_path(&path, 1);
        let error = write_from(
            CorruptAtEof {
                data: &data,
                partial: partial.clone(),
            },
            &volume,
            restore::payload::SYNTHETIC_MODEL,
            &image,
            ETAG,
            data.len() as u64,
            BLOCK as u64,
        )
        .unwrap_err();
        assert_eq!(
            error.to_string(),
            "the stick's data does not read back as written"
        );
        assert_eq!(fs::read(path.join(piece_name(0)))?, &data[..BLOCK]);
        assert!(partial.exists());
        assert!(!path.join(piece_name(1)).exists());
        assert!(!path.join(MANIFEST_FILE).exists());
        Ok(())
    }

    #[test]
    fn download_verifies_before_publication() -> Result<()> {
        let temp = tempfile::tempdir()?;
        let ipsw = temp.path().join("source.ipsw");
        crate::efi::synthetic_ipsw(&ipsw)?;
        let bytes = fs::read(&ipsw)?;
        let image = restore::Image {
            version: "test".into(),
            build: restore::payload::SYNTHETIC_BUILD.into(),
            models: vec![restore::payload::SYNTHETIC_MODEL.into()],
            url: "https://updates.cdn-apple.com/source.ipsw".into(),
            sha1: Some(hex::encode(Sha1::digest(&bytes))),
        };
        let volume = temp.path().join("usb");
        let written = write_from(
            io::Cursor::new(&bytes[..]),
            &volume,
            restore::payload::SYNTHETIC_MODEL,
            &image,
            ETAG,
            bytes.len() as u64,
            HASH_BLOCK_BYTES,
        )?;
        assert!(!written.manifest_sha256.is_empty());
        assert!(
            written.wrote > 1,
            "the fixture must exercise joined-piece reads"
        );
        let written_dir = directory(&volume, &image.build);
        let manifest = fs::read(written_dir.join(MANIFEST_FILE))?;
        assert_eq!(written.manifest_sha256, stela_formats::sha256(&manifest));
        let manifest = stela_formats::payload::PayloadManifest::parse(&manifest)
            .map_err(anyhow::Error::msg)?;
        assert_eq!(manifest.source.sha256, Some(stela_formats::sha256(&bytes)));
        assert_eq!(manifest.source.catalog_sha1, image.sha1);
        let Layout::Pieces {
            pieces,
            block_hashes,
            ..
        } = manifest.layout
        else {
            panic!("download did not publish a pieces layout");
        };
        let mut expected_blocks = Vec::new();
        assert_eq!(
            pieces.len(),
            bytes.len().div_ceil(HASH_BLOCK_BYTES as usize)
        );
        for (index, (piece, expected)) in pieces
            .iter()
            .zip(bytes.chunks(HASH_BLOCK_BYTES as usize))
            .enumerate()
        {
            let digest = Sha256::digest(expected);
            expected_blocks.extend_from_slice(&digest);
            assert_eq!(piece.sha256, hex::encode(digest));
            assert_eq!(fs::read(written_dir.join(piece_name(index)))?, expected);
        }
        assert_eq!(
            fs::read(written_dir.join(block_hashes.file))?,
            expected_blocks
        );
        let mut damaged = bytes.clone();
        damaged[0] ^= 1;
        let mut oversized = bytes.clone();
        oversized.push(0);
        for (index, bad_bytes) in [damaged, bytes[..bytes.len() - 1].to_vec(), oversized]
            .into_iter()
            .enumerate()
        {
            let bad = temp.path().join(format!("bad-{index}"));
            assert!(
                write_from(
                    io::Cursor::new(&bad_bytes[..]),
                    &bad,
                    restore::payload::SYNTHETIC_MODEL,
                    &image,
                    ETAG,
                    bytes.len() as u64,
                    HASH_BLOCK_BYTES
                )
                .is_err()
            );
            assert!(!directory(&bad, &image.build).join(MANIFEST_FILE).exists());
        }
        Ok(())
    }
}
