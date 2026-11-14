//! Bounds-checked Metal library containers and LLVM bitcode wrappers.
//!
//! The MTLB layout is reverse-engineered, not a public Apple ABI; it follows
//! MetalLibraryArchive (<https://github.com/YuAo/MetalLibraryArchive>) and
//! LLVM's bitcode wrapper format, and copies no external parser. The reader
//! checks section and record bounds, required tags, metadata group framing
//! and the selected AIR module's SHA-256; wrapper parsing checks framing
//! only, so LLVM must still verify the bitcode. Shader bytes, metadata and
//! digest remain borrowed from the original input.
use alloc::vec::Vec;
use core::{fmt, ops::Range, str};
use stela_contracts::bytes::{le16, le32, le64};
use stela_crypto::{Sha256, digest::Digest};

const HEADER_BYTES: usize = 88;
const LLVM_WRAPPER_MAGIC: u32 = 0x0b17_c0de;
const LLVM_WRAPPER_BYTES: usize = 20;
const LLVM_MAGIC: &[u8; 4] = b"BC\xc0\xde";

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ErrorKind {
    Truncated,
    Overflow,
    Magic,
    Version,
    FileSize,
    SectionOverlap,
    FunctionCount,
    TagSize,
    DuplicateTag([u8; 4]),
    MissingTag([u8; 4]),
    Terminator,
    Name,
    Metadata,
    Hash,
    Bitcode,
    Allocation,
    ResourceLimit,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Error {
    /// Byte offset in the original input, not an extracted temporary buffer.
    pub offset: usize,
    pub kind: ErrorKind,
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "Metal library {:?} at {:#x}", self.kind, self.offset)
    }
}

impl core::error::Error for Error {}

type Result<T> = core::result::Result<T, Error>;

fn error(offset: usize, kind: ErrorKind) -> Error {
    Error { offset, kind }
}

fn bytes(data: &[u8], offset: usize, size: usize) -> Result<&[u8]> {
    let end = offset
        .checked_add(size)
        .ok_or_else(|| error(offset, ErrorKind::Overflow))?;
    data.get(offset..end)
        .ok_or_else(|| error(offset, ErrorKind::Truncated))
}

fn u16_at(data: &[u8], offset: usize) -> Result<u16> {
    bytes(data, offset, 2).map(|field| le16(field, 0))
}

fn u32_at(data: &[u8], offset: usize) -> Result<u32> {
    bytes(data, offset, 4).map(|field| le32(field, 0))
}

fn u64_at(data: &[u8], offset: usize) -> Result<u64> {
    bytes(data, offset, 8).map(|field| le64(field, 0))
}

fn host_size(value: u64, offset: usize) -> Result<usize> {
    usize::try_from(value).map_err(|_| error(offset, ErrorKind::Overflow))
}

fn section(data: &[u8], header_offset: usize, count_prefix: bool) -> Result<Range<usize>> {
    let offset = host_size(u64_at(data, header_offset)?, header_offset)?;
    let mut size = host_size(u64_at(data, header_offset + 8)?, header_offset + 8)?;
    // MTLB's function-list size excludes its leading u32 function count.
    if count_prefix {
        size = size
            .checked_add(4)
            .ok_or_else(|| error(header_offset, ErrorKind::Overflow))?;
    }
    bytes(data, offset, size)?;
    Ok(offset..offset + size)
}

/// Raw tagged metadata. Unknown tags remain visible to later compiler stages.
#[derive(Clone, Copy, Debug)]
pub struct Tags<'a> {
    bytes: &'a [u8],
    offset: usize,
}

#[derive(Clone, Copy, Debug)]
pub struct Tag<'a> {
    pub name: [u8; 4],
    pub value: &'a [u8],
    pub offset: usize,
}

impl<'a> Iterator for Tags<'a> {
    type Item = Result<Tag<'a>>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.bytes.is_empty() {
            return None;
        }
        let offset = self.offset;
        let parsed = (|| {
            let name: [u8; 4] = bytes(self.bytes, 0, 4)?.try_into().unwrap();
            if &name == b"ENDT" {
                if self.bytes.len() != 4 {
                    return Err(error(0, ErrorKind::Terminator));
                }
                return Ok((4, None));
            }
            let size = usize::from(u16_at(self.bytes, 4)?);
            let value = bytes(self.bytes, 6, size)?;
            Ok((
                6 + size,
                Some(Tag {
                    name,
                    value,
                    offset,
                }),
            ))
        })();
        match parsed {
            Ok((used, tag)) => {
                self.bytes = &self.bytes[used..];
                self.offset += used;
                tag.map(Ok)
            }
            Err(mut failure) => {
                failure.offset += offset;
                self.bytes = &[];
                Some(Err(failure))
            }
        }
    }
}

impl<'a> Tags<'a> {
    /// Remaining original tag-stream range, including its ENDT terminator.
    pub fn range(&self) -> Range<usize> {
        self.offset..self.offset + self.bytes.len()
    }

    fn checked(data: &'a [u8], offset: usize) -> Result<Self> {
        if !data.ends_with(b"ENDT") {
            return Err(error(offset + data.len(), ErrorKind::Terminator));
        }
        let tags = Self {
            bytes: data,
            offset,
        };
        for tag in tags {
            tag?;
        }
        Ok(tags)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Version {
    pub major: u16,
    pub minor: u16,
}

#[derive(Clone, Copy, Debug)]
pub struct Header {
    pub platform: u16,
    pub format: Version,
    /// Preserve the container's library kind and target OS without inventing
    /// support for a dynamic library, a companion, or a future target.
    pub kind: u8,
    pub target_os: u8,
    pub minimum_os: Version,
}

impl Header {
    /// The kind of a dynamic library (`MTLLibraryTypeDynamic`).
    pub const DYNAMIC: u8 = 2;
}

/// A structurally validated library. AIR digests are checked by [`Function::air`]
/// when selected for compilation, avoiding hashing every unused shader.
#[derive(Debug)]
pub struct Library<'a> {
    pub header: Header,
    functions: Vec<Function<'a>>,
    pub header_extensions: Option<Tags<'a>>,
    /// A dynamic library's install name, in the section its `HDYN` tag
    /// locates.
    pub install_name: Option<&'a str>,
    /// The install names of the dynamic libraries it links, that section's
    /// `DYNL` tags in order, which Metal loads to resolve its imports.
    pub linked: Vec<&'a str>,
}

impl<'a> Library<'a> {
    pub fn parse(data: &'a [u8]) -> Result<Self> {
        Self::parse_bounded(data, usize::MAX)
    }

    /// Apply the caller's entry budget before allocating the function table.
    pub fn parse_bounded(data: &'a [u8], max_functions: usize) -> Result<Self> {
        bytes(data, 0, HEADER_BYTES)?;
        if &data[..4] != b"MTLB" {
            return Err(error(0, ErrorKind::Magic));
        }
        let header = Header {
            platform: u16_at(data, 4)?,
            format: Version {
                major: u16_at(data, 6)?,
                minor: u16_at(data, 8)?,
            },
            kind: data[10],
            target_os: data[11],
            minimum_os: Version {
                major: u16_at(data, 12)?,
                minor: u16_at(data, 14)?,
            },
        };
        if header.format.major != 2 {
            return Err(error(6, ErrorKind::Version));
        }
        if u64_at(data, 16)? != data.len() as u64 {
            return Err(error(16, ErrorKind::FileSize));
        }
        let functions_range = section(data, 24, true)?;
        let public = section(data, 40, false)?;
        let private = section(data, 56, false)?;
        let code = section(data, 72, false)?;
        let sections = [&functions_range, &public, &private, &code];
        for (index, a) in sections.iter().enumerate() {
            if !a.is_empty() && a.start < HEADER_BYTES {
                return Err(error(a.start, ErrorKind::SectionOverlap));
            }
            for b in &sections[..index] {
                if !a.is_empty() && !b.is_empty() && a.start < b.end && b.start < a.end {
                    return Err(error(a.start, ErrorKind::SectionOverlap));
                }
            }
        }
        let count = u32_at(data, functions_range.start)? as usize;
        if count > functions_range.len().saturating_sub(4) / 8 {
            return Err(error(functions_range.start, ErrorKind::FunctionCount));
        }
        if count > max_functions {
            return Err(error(functions_range.start, ErrorKind::ResourceLimit));
        }
        let mut functions = Vec::new();
        functions
            .try_reserve_exact(count)
            .map_err(|_| error(functions_range.start, ErrorKind::Allocation))?;
        let mut offset = functions_range.start + 4;
        for _ in 0..count {
            let size = u32_at(data, offset)? as usize;
            if size < 8 || size > functions_range.end - offset {
                return Err(error(offset, ErrorKind::TagSize));
            }
            functions.push(Function::parse(
                data, offset, size, &public, &private, &code,
            )?);
            offset += size;
        }
        if offset != functions_range.end {
            return Err(error(offset, ErrorKind::FunctionCount));
        }
        let header_extensions = if functions_range.end < public.start {
            Some(Tags::checked(
                &data[functions_range.end..public.start],
                functions_range.end,
            )?)
        } else {
            None
        };
        let (install_name, linked) = dynamic_names(data, header_extensions)?;
        Ok(Self {
            header,
            functions,
            header_extensions,
            install_name,
            linked,
        })
    }

    pub fn functions(&self) -> &[Function<'a>] {
        &self.functions
    }
}

fn tag(out: &mut Vec<u8>, name: &[u8; 4], value: &[u8]) {
    out.extend_from_slice(name);
    out.extend_from_slice(&(value.len() as u16).to_le_bytes());
    out.extend_from_slice(value);
}

fn name_tag(out: &mut Vec<u8>, name: &str) {
    tag(out, b"NAME", &[name.as_bytes(), &[0]].concat());
}

/// A container of one AIR module's `functions`, as the source compiler's
/// module has none: each entry names the same module, and a dynamic library
/// has an `install_name`. Each function is a name and its MTLB stage.
pub fn assemble(
    air: &[u8],
    functions: &[(&str, u8)],
    kind: u8,
    install_name: Option<&str>,
) -> Vec<u8> {
    let digest = Sha256::digest(air);
    let mut entries = Vec::new();
    for (function, stage) in functions {
        let mut entry = alloc::vec![0; 4];
        name_tag(&mut entry, function);
        tag(&mut entry, b"TYPE", &[*stage]);
        tag(&mut entry, b"HASH", &digest);
        tag(&mut entry, b"MDSZ", &(air.len() as u64).to_le_bytes());
        tag(&mut entry, b"OFFT", &[0; 24]);
        tag(&mut entry, b"VERS", &[0; 8]);
        entry.extend_from_slice(b"ENDT");
        let size = entry.len() as u32;
        entry[..4].copy_from_slice(&size.to_le_bytes());
        entries.extend_from_slice(&entry);
    }
    let mut data = alloc::vec![0; HEADER_BYTES];
    data[..4].copy_from_slice(b"MTLB");
    data[4..6].copy_from_slice(&0x8001u16.to_le_bytes());
    data[6..8].copy_from_slice(&2u16.to_le_bytes());
    data[10] = kind;
    data[11] = 0x81;
    data.extend_from_slice(&(functions.len() as u32).to_le_bytes());
    data.extend_from_slice(&entries);
    let dynamic_header = install_name.map(|_| {
        tag(&mut data, b"HDYN", &[0; 16]);
        data.len() - 16
    });
    data.extend_from_slice(b"ENDT");
    let field = |data: &mut [u8], at: usize, value: usize| {
        data[at..at + 8].copy_from_slice(&(value as u64).to_le_bytes());
    };
    field(&mut data, 24, HEADER_BYTES);
    field(&mut data, 32, entries.len());
    for at in [40, 56] {
        let metadata = data.len();
        data.extend_from_slice(&8u32.to_le_bytes());
        data.extend_from_slice(b"ENDT");
        field(&mut data, at, metadata);
        field(&mut data, at + 8, 8);
    }
    let code = data.len();
    data.extend_from_slice(air);
    field(&mut data, 72, code);
    field(&mut data, 80, air.len());
    if let (Some(at), Some(install_name)) = (dynamic_header, install_name) {
        let section = data.len();
        name_tag(&mut data, install_name);
        data.extend_from_slice(b"ENDT");
        let size = data.len() - section;
        field(&mut data, at, section);
        field(&mut data, at + 8, size);
    }
    let size = data.len();
    field(&mut data, 16, size);
    data
}

/// One NUL-terminated name.
fn name(value: &[u8], offset: usize) -> Result<&str> {
    let name = value
        .strip_suffix(&[0])
        .filter(|name| !name.is_empty() && !name.contains(&0))
        .ok_or_else(|| error(offset, ErrorKind::Name))?;
    str::from_utf8(name).map_err(|_| error(offset, ErrorKind::Name))
}

/// The `NAME` of the tagged section an `HDYN` tag locates by offset and
/// size, and the install names its `DYNL` tags list.
fn dynamic_names<'a>(
    data: &'a [u8],
    extensions: Option<Tags<'a>>,
) -> Result<(Option<&'a str>, Vec<&'a str>)> {
    for tag in extensions.into_iter().flatten() {
        let tag = tag?;
        if tag.name != *b"HDYN" {
            continue;
        }
        if tag.value.len() != 16 {
            return Err(error(tag.offset, ErrorKind::TagSize));
        }
        let offset = host_size(u64_at(tag.value, 0)?, tag.offset)?;
        let size = host_size(u64_at(tag.value, 8)?, tag.offset)?;
        let (mut install_name, mut linked) = (None, Vec::new());
        for tag in Tags::checked(bytes(data, offset, size)?, offset)? {
            let tag = tag?;
            if tag.name == *b"NAME" && install_name.is_none() {
                install_name = Some(name(tag.value, tag.offset)?);
            } else if tag.name == *b"DYNL" {
                linked.push(name(tag.value, tag.offset)?);
            }
        }
        return Ok((install_name, linked));
    }
    Ok((None, Vec::new()))
}

#[derive(Clone, Copy, Debug)]
pub struct Function<'a> {
    pub name: &'a str,
    /// MTLB stage value, not the differently numbered MTLFunctionType.
    pub kind: u8,
    pub air_version: Version,
    pub language_version: Version,
    pub tags: Tags<'a>,
    pub public_metadata: Option<Tags<'a>>,
    pub private_metadata: Option<Tags<'a>>,
    pub digest: &'a [u8; 32],
    pub module_offset: usize,
    module: &'a [u8],
}

impl Function<'_> {
    /// The MTLB stage of an extern function: a declaration, with its
    /// argument and return reflection, of a function another library or the
    /// source that links it defines. `functionNames` omits it and
    /// `externFunctionNames` lists it.
    pub const EXTERN: u8 = 5;
    /// The MTLB stage of a function a dynamic library exports for other
    /// libraries to link: a plain definition, not an entry point.
    pub const EXPORTED: u8 = 3;
}

impl<'a> Function<'a> {
    fn parse(
        data: &'a [u8],
        offset: usize,
        size: usize,
        public: &Range<usize>,
        private: &Range<usize>,
        code: &Range<usize>,
    ) -> Result<Self> {
        let tags = Tags::checked(bytes(data, offset + 4, size - 4)?, offset + 4)?;
        let mut name = None;
        let mut kind = None;
        let mut digest = None;
        let mut module_size = None;
        let mut offsets = None;
        let mut versions = None;
        for tag in tags {
            let tag = tag?;
            let slot = match &tag.name {
                b"NAME" => &mut name,
                b"TYPE" => &mut kind,
                b"HASH" => &mut digest,
                b"MDSZ" => &mut module_size,
                b"OFFT" => &mut offsets,
                b"VERS" => &mut versions,
                _ => continue,
            };
            if slot.replace(tag.value).is_some() {
                return Err(error(tag.offset, ErrorKind::DuplicateTag(tag.name)));
            }
        }
        let required = |value: Option<&'a [u8]>, tag| {
            value.ok_or_else(|| error(offset, ErrorKind::MissingTag(tag)))
        };
        let name = self::name(required(name, *b"NAME")?, offset)?;
        let exact = |value: &'a [u8], size| {
            if value.len() == size {
                Ok(value)
            } else {
                Err(error(offset, ErrorKind::TagSize))
            }
        };
        let kind = exact(required(kind, *b"TYPE")?, 1)?[0];
        let digest = exact(required(digest, *b"HASH")?, 32)?.try_into().unwrap();
        let module_size = host_size(
            u64_at(exact(required(module_size, *b"MDSZ")?, 8)?, 0)?,
            offset,
        )?;
        let offsets = exact(required(offsets, *b"OFFT")?, 24)?;
        let versions = exact(required(versions, *b"VERS")?, 8)?;
        let relative_code = host_size(u64_at(offsets, 16)?, offset)?;
        let module_offset = code
            .start
            .checked_add(relative_code)
            .ok_or_else(|| error(offset, ErrorKind::Overflow))?;
        if relative_code > code.len() || module_size > code.len() - relative_code {
            return Err(error(module_offset, ErrorKind::Truncated));
        }
        let module = bytes(data, module_offset, module_size)?;
        let public_metadata = metadata(data, public, u64_at(offsets, 0)?, offset)?;
        let private_metadata = metadata(data, private, u64_at(offsets, 8)?, offset)?;
        Ok(Self {
            name,
            kind,
            digest,
            module_offset,
            module,
            air_version: Version {
                major: u16_at(versions, 0)?,
                minor: u16_at(versions, 2)?,
            },
            language_version: Version {
                major: u16_at(versions, 4)?,
                minor: u16_at(versions, 6)?,
            },
            tags,
            public_metadata,
            private_metadata,
        })
    }

    /// Original wrapped/raw module extent; no payload copy or semantic rewrite.
    pub fn module_range(&self) -> Range<usize> {
        self.module_offset..self.module_offset + self.module.len()
    }

    /// Verify the archive's digest before returning the LLVM reader's input.
    pub fn air(&self) -> Result<Air<'a>> {
        Air::verified(self.module, self.digest).map_err(|mut failure| {
            failure.offset += self.module_offset;
            failure
        })
    }
}

fn metadata<'a>(
    data: &'a [u8],
    section: &Range<usize>,
    relative: u64,
    source: usize,
) -> Result<Option<Tags<'a>>> {
    let relative = host_size(relative, source)?;
    if relative == 0 && section.is_empty() {
        return Ok(None);
    }
    if relative > section.len() || section.len() - relative < 4 {
        return Err(error(source, ErrorKind::Metadata));
    }
    let offset = section.start + relative;
    let size = u32_at(data, offset)? as usize;
    let available = section.len() - relative;
    // A block is its size and then tags ending in ENDT. Apple's toolchains
    // record the size either with its own four bytes or without them, both
    // within one MTLB version, so the tags are those that end in ENDT at
    // either length.
    let tags = |length: usize| {
        (length >= 4 && 4 + length <= available)
            .then(|| Tags::checked(&data[offset + 4..offset + 4 + length], offset + 4).ok())
            .flatten()
    };
    size.checked_sub(4)
        .and_then(tags)
        .or_else(|| tags(size))
        .map(Some)
        .ok_or_else(|| error(offset, ErrorKind::Metadata))
}

/// Validated LLVM wrapper framing, not LLVM IR validation or shader compilation.
#[derive(Clone, Copy, Debug)]
pub struct Air<'a> {
    pub bitcode: &'a [u8],
    pub cpu_type: Option<u32>,
}

impl<'a> Air<'a> {
    pub(super) fn verified(data: &'a [u8], digest: &[u8; 32]) -> Result<Self> {
        if Sha256::digest(data).as_slice() != digest {
            return Err(error(0, ErrorKind::Hash));
        }
        Self::parse(data)
    }

    pub fn parse(data: &'a [u8]) -> Result<Self> {
        if data.starts_with(LLVM_MAGIC) {
            return Ok(Self {
                bitcode: data,
                cpu_type: None,
            });
        }
        if u32_at(data, 0)? != LLVM_WRAPPER_MAGIC {
            return Err(error(0, ErrorKind::Bitcode));
        }
        bytes(data, 0, LLVM_WRAPPER_BYTES)?;
        if u32_at(data, 4)? != 0 {
            return Err(error(4, ErrorKind::Version));
        }
        let offset = u32_at(data, 8)? as usize;
        let size = u32_at(data, 12)? as usize;
        if offset < LLVM_WRAPPER_BYTES || size < 4 {
            return Err(error(8, ErrorKind::Bitcode));
        }
        let bitcode = bytes(data, offset, size)?;
        if !bitcode.starts_with(LLVM_MAGIC) || data[offset + size..].iter().any(|byte| *byte != 0) {
            return Err(error(offset, ErrorKind::Bitcode));
        }
        Ok(Self {
            bitcode,
            cpu_type: Some(u32_at(data, 16)?),
        })
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;
    use alloc::{vec, vec::Vec};

    fn u64_field(out: &mut [u8], at: usize, value: usize) {
        out[at..at + 8].copy_from_slice(&(value as u64).to_le_bytes());
    }

    /// An original synthetic container. Its payload is only LLVM framing, not a
    /// valid shader; AIR semantics are the LLVM reader/compiler's responsibility.
    pub(crate) fn fixture(duplicate: bool) -> Vec<u8> {
        container(duplicate, 0, u32::MAX)
    }

    /// A library of `kind` whose module's wrapper names `cpu_type`.
    pub(crate) fn container(duplicate: bool, kind: u8, cpu_type: u32) -> Vec<u8> {
        container_named(duplicate, kind, cpu_type, None, &[], 32)
    }

    /// [`fixture`] whose module holds `module_bytes` bytes, its padding
    /// after the bitcode filled with `fill`: a library of that size.
    pub(crate) fn padded(module_bytes: usize, fill: u8) -> Vec<u8> {
        let mut data = container_named(false, 0, u32::MAX, None, &[], module_bytes);
        let code = u64::from_le_bytes(data[72..80].try_into().unwrap()) as usize;
        let end = code + module_bytes;
        data[code + 32..end].fill(fill);
        // The module's digest covers the padding.
        let digest = Sha256::digest(&data[code..end]);
        let at = data
            .windows(4)
            .position(|tag| tag == b"HASH")
            .expect("the entry names its module's digest")
            + 6;
        data[at..at + 32].copy_from_slice(&digest);
        data
    }

    /// A dynamic library of the given install name tag value.
    pub(crate) fn dynamic(install_name: &[u8]) -> Vec<u8> {
        container_named(
            false,
            Header::DYNAMIC,
            u32::MAX,
            Some(install_name),
            &[],
            32,
        )
    }

    /// An executable library named `file` that links the dynamic libraries
    /// of the install name tag values `linked`, as Metal's linker writes it.
    pub(crate) fn linking(file: &[u8], linked: &[&[u8]]) -> Vec<u8> {
        container_named(false, 0, u32::MAX, Some(file), linked, 32)
    }

    fn container_named(
        duplicate: bool,
        kind: u8,
        cpu_type: u32,
        install_name: Option<&[u8]>,
        linked: &[&[u8]],
        module_bytes: usize,
    ) -> Vec<u8> {
        let mut module = vec![];
        for word in [LLVM_WRAPPER_MAGIC, 0, 20, 8, cpu_type] {
            module.extend_from_slice(&word.to_le_bytes());
        }
        module.extend_from_slice(b"BC\xc0\xde\0\0\0\0");
        module.resize(module_bytes.max(32), 0);
        let mut entry = vec![0; 4];
        tag(&mut entry, b"NAME", b"kernel\0");
        if duplicate {
            tag(&mut entry, b"NAME", b"second\0");
        }
        tag(&mut entry, b"TYPE", &[2]);
        tag(&mut entry, b"HASH", &Sha256::digest(&module));
        tag(&mut entry, b"MDSZ", &(module.len() as u64).to_le_bytes());
        tag(&mut entry, b"OFFT", &[0; 24]);
        tag(&mut entry, b"VERS", &[2, 0, 8, 0, 4, 0, 0, 0]);
        tag(&mut entry, b"FUTR", &[0xa7, 0xb8, 0xc9]);
        entry.extend_from_slice(b"ENDT");
        let size = entry.len();
        entry[..4].copy_from_slice(&(size as u32).to_le_bytes());
        let mut data = vec![0; HEADER_BYTES];
        data[..4].copy_from_slice(b"MTLB");
        data[4..6].copy_from_slice(&0x8001u16.to_le_bytes());
        data[6..8].copy_from_slice(&2u16.to_le_bytes());
        data[8..10].copy_from_slice(&9u16.to_le_bytes());
        data[10] = kind;
        data[11] = 0x81;
        data[12] = 26;
        u64_field(&mut data, 24, HEADER_BYTES);
        u64_field(&mut data, 32, size);
        data.extend_from_slice(&1u32.to_le_bytes());
        data.extend_from_slice(&entry);
        // Header extension is not part of the four principal section spans.
        tag(&mut data, b"UUID", &[0x42; 16]);
        let dynamic_header = install_name.map(|_| {
            tag(&mut data, b"HDYN", &[0; 16]);
            data.len() - 16
        });
        data.extend_from_slice(b"ENDT");
        let public = data.len();
        data.extend_from_slice(&8u32.to_le_bytes());
        data.extend_from_slice(b"ENDT");
        u64_field(&mut data, 40, public);
        u64_field(&mut data, 48, 8);
        let private = data.len();
        data.extend_from_slice(&8u32.to_le_bytes());
        data.extend_from_slice(b"ENDT");
        u64_field(&mut data, 56, private);
        u64_field(&mut data, 64, 8);
        let code = data.len();
        data.extend_from_slice(&module);
        u64_field(&mut data, 72, code);
        u64_field(&mut data, 80, module.len());
        if let (Some(field), Some(name)) = (dynamic_header, install_name) {
            let section = data.len();
            tag(&mut data, b"NAME", name);
            for linked in linked {
                tag(&mut data, b"DYNL", linked);
            }
            data.extend_from_slice(b"ENDT");
            let size = data.len() - section;
            u64_field(&mut data, field, section);
            u64_field(&mut data, field + 8, size);
        }
        let file_size = data.len();
        u64_field(&mut data, 16, file_size);
        data
    }

    #[test]
    fn metadata_sizes_with_or_without_their_own_field_parse() {
        let mut data = fixture(false);
        for at in [40, 56] {
            let block = u64::from_le_bytes(data[at..at + 8].try_into().unwrap()) as usize;
            // Each block holds only ENDT: four bytes after the size field.
            data[block..block + 4].copy_from_slice(&4u32.to_le_bytes());
        }
        let library = Library::parse(&data).unwrap();
        let function = &library.functions()[0];
        assert_eq!(function.public_metadata.unwrap().count(), 0);
        assert_eq!(function.private_metadata.unwrap().count(), 0);
        let block = u64::from_le_bytes(data[40..48].try_into().unwrap()) as usize;
        data[block..block + 4].copy_from_slice(&3u32.to_le_bytes());
        assert_eq!(Library::parse(&data).unwrap_err().kind, ErrorKind::Metadata);
    }

    #[test]
    fn a_padded_library_parses_with_a_module_of_its_size() {
        let data = padded(4096, 0x5a);
        let library = Library::parse(&data).unwrap();
        let module = library.functions()[0].module_range();
        assert_eq!(module.len(), 4096);
        assert!(data[module][32..].iter().all(|byte| *byte == 0x5a));
        assert_ne!(data, padded(4096, 0xa5));
    }

    #[test]
    fn original_sections_and_unknown_tags_are_preserved() {
        let data = fixture(false);
        let library = Library::parse(&data).unwrap();
        assert_eq!(library.header.platform, 0x8001);
        assert_eq!(library.header.format, Version { major: 2, minor: 9 });
        assert_eq!(
            library.header.minimum_os,
            Version {
                major: 26,
                minor: 0
            }
        );
        let function = &library.functions()[0];
        assert_eq!(function.name, "kernel");
        assert_eq!(function.kind, 2);
        assert_eq!(function.air_version, Version { major: 2, minor: 8 });
        let future = function
            .tags
            .into_iter()
            .find(|tag| tag.as_ref().unwrap().name == *b"FUTR")
            .unwrap()
            .unwrap();
        assert_eq!(future.value, &[0xa7, 0xb8, 0xc9]);
        let uuid = library.header_extensions.unwrap().next().unwrap().unwrap();
        assert_eq!(uuid.name, *b"UUID");
        assert_eq!(uuid.value, &[0x42; 16]);
        assert!(function.public_metadata.unwrap().next().is_none());
        let air = function.air().unwrap();
        assert_eq!(air.cpu_type, Some(u32::MAX));
        assert_eq!(air.bitcode, b"BC\xc0\xde\0\0\0\0");
        let bitcode_at = function.module_offset + LLVM_WRAPPER_BYTES;
        assert_eq!(air.bitcode.as_ptr(), data[bitcode_at..].as_ptr());
    }

    #[test]
    fn assembled_libraries_share_their_module() {
        let air = b"BC\xc0\xde\0\0\0\0";
        let data = assemble(air, &[("first", 2), ("second", 4)], 0, None);
        let library = Library::parse(&data).unwrap();
        assert_eq!(library.install_name, None);
        let functions = library.functions();
        assert_eq!(functions.len(), 2);
        assert_eq!((functions[0].name, functions[0].kind), ("first", 2));
        assert_eq!((functions[1].name, functions[1].kind), ("second", 4));
        for function in functions {
            assert_eq!(function.air().unwrap().bitcode, air);
        }
        let data = assemble(air, &[("first", 2)], Header::DYNAMIC, Some("lib/name"));
        let library = Library::parse(&data).unwrap();
        assert_eq!(library.header.kind, Header::DYNAMIC);
        assert_eq!(library.install_name, Some("lib/name"));
        assert_eq!(library.functions()[0].air().unwrap().bitcode, air);
    }

    #[test]
    fn libraries_name_the_dynamic_libraries_they_link() {
        let data = linking(
            b"user.metallib\0",
            &[b"@loader_path/dynamic.metallib\0", b"/abs/other.metallib\0"],
        );
        let library = Library::parse(&data).unwrap();
        assert_eq!(
            library.linked,
            ["@loader_path/dynamic.metallib", "/abs/other.metallib"]
        );
        assert!(Library::parse(&fixture(false)).unwrap().linked.is_empty());
        assert!(
            Library::parse(&dynamic(b"lib\0"))
                .unwrap()
                .linked
                .is_empty()
        );
        let data = linking(b"user.metallib\0", &[b"unterminated"]);
        assert_eq!(Library::parse(&data).unwrap_err().kind, ErrorKind::Name);
    }

    #[test]
    fn dynamic_libraries_name_themselves() {
        let data = dynamic(b"@executable_path/stela.metallib\0");
        let library = Library::parse(&data).unwrap();
        assert_eq!(library.header.kind, Header::DYNAMIC);
        assert_eq!(
            library.install_name,
            Some("@executable_path/stela.metallib")
        );
        assert_eq!(Library::parse(&fixture(false)).unwrap().install_name, None);
        for name in [&b"unterminated"[..], b"\0", b"in\0side\0", b"\xff\0"] {
            let data = dynamic(name);
            assert_eq!(Library::parse(&data).unwrap_err().kind, ErrorKind::Name);
        }
    }

    #[test]
    fn a_dynamic_header_stays_in_the_file() {
        let base = dynamic(b"name\0");
        let library = Library::parse(&base).unwrap();
        let tag = library
            .header_extensions
            .unwrap()
            .find(|tag| tag.as_ref().unwrap().name == *b"HDYN")
            .unwrap()
            .unwrap();
        for field in [6, 14] {
            let mut data = base.clone();
            data[tag.offset + field..tag.offset + field + 8]
                .copy_from_slice(&u64::MAX.to_le_bytes());
            assert!(Library::parse(&data).is_err(), "field {field}");
        }
    }

    #[test]
    fn every_truncated_prefix_and_duplicate_required_tag_is_rejected() {
        let data = fixture(false);
        for end in 0..data.len() {
            assert!(Library::parse(&data[..end]).is_err(), "prefix {end}");
        }
        assert!(
            matches!(Library::parse(&fixture(true)).unwrap_err().kind, ErrorKind::DuplicateTag(tag) if tag == *b"NAME")
        );
    }

    #[test]
    fn payload_integrity_is_checked_when_selected() {
        let mut data = fixture(false);
        let module_at = Library::parse(&data).unwrap().functions()[0].module_offset;
        data[module_at + 24] ^= 1;
        let library = Library::parse(&data).unwrap();
        let failure = library.functions()[0].air().unwrap_err();
        assert_eq!(failure, error(module_at, ErrorKind::Hash));
    }

    #[test]
    fn overlapping_out_of_bounds_and_overflowed_sections_fail() {
        let mut data = fixture(false);
        let public = u64_at(&data, 40).unwrap();
        data[72..80].copy_from_slice(&public.to_le_bytes());
        assert_eq!(
            Library::parse(&data).unwrap_err().kind,
            ErrorKind::SectionOverlap
        );
        for field in [24, 32, 40, 48, 56, 64, 72, 80] {
            let mut data = fixture(false);
            data[field..field + 8].copy_from_slice(&u64::MAX.to_le_bytes());
            assert!(Library::parse(&data).is_err(), "field {field}");
        }
    }

    #[test]
    fn wrapper_offsets_versions_padding_and_magic_are_checked() {
        let data = fixture(false);
        let function = Library::parse(&data).unwrap().functions()[0];
        assert!(Air::parse(b"BC\xc0\xde").is_ok());
        for (at, word) in [(0, 0), (4, 1), (8, 0), (8, u32::MAX), (12, u32::MAX)] {
            let mut module = function.module.to_vec();
            module[at..at + 4].copy_from_slice(&word.to_le_bytes());
            assert!(Air::parse(&module).is_err(), "wrapper field {at}");
        }
        let mut module = function.module.to_vec();
        module[31] = 1;
        assert_eq!(Air::parse(&module).unwrap_err().kind, ErrorKind::Bitcode);
        let mut module = function.module.to_vec();
        module[20] = 0;
        assert_eq!(Air::parse(&module).unwrap_err().kind, ErrorKind::Bitcode);
    }

    #[test]
    fn malformed_tag_and_metadata_offsets_never_escape_their_sections() {
        let base = fixture(false);
        let function = Library::parse(&base).unwrap().functions()[0];
        for name in [*b"OFFT", *b"VERS", *b"HASH", *b"MDSZ"] {
            let mut data = base.clone();
            let tag = function
                .tags
                .into_iter()
                .find(|tag| tag.as_ref().unwrap().name == name)
                .unwrap()
                .unwrap();
            data[tag.offset + 4..tag.offset + 6].copy_from_slice(&u16::MAX.to_le_bytes());
            assert!(Library::parse(&data).is_err());
        }
        let tag = function
            .tags
            .into_iter()
            .find(|tag| tag.as_ref().unwrap().name == *b"OFFT")
            .unwrap()
            .unwrap();
        for field in [0, 8, 16] {
            let mut data = base.clone();
            data[tag.offset + 6 + field..tag.offset + 14 + field]
                .copy_from_slice(&u64::MAX.to_le_bytes());
            assert!(Library::parse(&data).is_err());
        }
        let mut data = base.clone();
        data[HEADER_BYTES..HEADER_BYTES + 4].copy_from_slice(&u32::MAX.to_le_bytes());
        assert_eq!(
            Library::parse(&data).unwrap_err().kind,
            ErrorKind::FunctionCount
        );
    }

    #[test]
    fn bounded_mutations_cannot_panic_or_allocate_from_an_unchecked_count() {
        let base = fixture(false);
        let mut random = 0x51ae_17d3u32;
        for _ in 0..10_000 {
            random = random.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            let index = random as usize % base.len();
            let mut data = base.clone();
            data[index] ^= (random >> 24) as u8 | 1;
            if let Ok(library) = Library::parse(&data) {
                for function in library.functions() {
                    let _ = function.air();
                }
            }
        }
    }
}
