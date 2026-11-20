//! Bounded host-device firmware manifest. This is independent of Apple boot
//! bundles: names select exact driver requirements, paths select files beneath
//! `EFI/stela/firmware` on the same boot volume. Hashes establish integrity
//! against this manifest, not vendor authentication or permission to distribute.
use alloc::{format, string::String, vec::Vec};
use core::fmt;
use serde::{
    Deserialize, Deserializer,
    de::{self, SeqAccess, Visitor},
};
use stela_driver_core::firmware::{Catalog, Provenance, Resource, Target};

/// Native host capability, independent of the guest bundle/options schema.
/// Revision 1 permits 16 MiB per image and 64 MiB in all, revision 2 permits
/// 64 MiB per image, and revision 3 permits 256 MiB in all. Revision 4 keeps
/// revision 3's limits and also reads the complete bundle ([`bundle`]) from
/// the USB's data volume. Each keeps schema 1 and 64 images; changing these
/// requires a revision.
pub const MOUNT_CAPABILITY: [u8; 21] = *b"STELA-HOST-FIRMWARE\0\x04";
pub fn supports_mount(image: &[u8]) -> bool {
    MountLimits::from_image(image).is_some()
}

/// Whether the EFI image reads the complete bundle ([`bundle::MANIFEST`]).
pub fn reads_bundle(image: &[u8]) -> bool {
    MountLimits::from_image(image).is_some_and(|limits| limits.revision >= 4)
}

/// Bounds advertised by an EFI image, independent of manifest schema 1.
#[derive(Clone, Copy, Debug)]
pub struct MountLimits {
    max_image_bytes: usize,
    max_total_bytes: usize,
    revision: u8,
}

impl MountLimits {
    pub fn from_image(image: &[u8]) -> Option<Self> {
        let mut revision = 0;
        for marker in image.windows(MOUNT_CAPABILITY.len()) {
            if marker[..20] != MOUNT_CAPABILITY[..20] {
                continue;
            }
            match marker[20] {
                1..=4 => revision = revision.max(marker[20]),
                // Never interpret an unknown mount contract as an older one.
                _ => return None,
            }
        }
        match revision {
            1 => Some(Self {
                max_image_bytes: 16 * 1024 * 1024,
                max_total_bytes: 64 * 1024 * 1024,
                revision,
            }),
            2 => Some(Self {
                max_image_bytes: MAX_IMAGE_BYTES,
                max_total_bytes: 64 * 1024 * 1024,
                revision,
            }),
            3 | 4 => Some(Self {
                max_image_bytes: MAX_IMAGE_BYTES,
                max_total_bytes: MAX_TOTAL_BYTES,
                revision,
            }),
            _ => None,
        }
    }

    /// Check metadata before payload allocation, download or publication.
    pub fn check_lengths(&self, lengths: impl IntoIterator<Item = usize>) -> Result<(), Error> {
        let mut count = 0usize;
        let mut total = 0usize;
        for length in lengths {
            if length == 0 || length > self.max_image_bytes {
                return Err(Error(format!(
                    "stela.efi host firmware image limit is {} bytes; update Stela before installing this package",
                    self.max_image_bytes
                )));
            }
            count += 1;
            total = total
                .checked_add(length)
                .ok_or_else(|| fail("host firmware total overflows its budget"))?;
            if count > MAX_IMAGES || total > self.max_total_bytes {
                return Err(fail("host firmware resources exceed the mount budget"));
            }
        }
        Ok(())
    }
}

pub const MANIFEST_PATH: &str = "EFI/stela/firmware/manifest.json";
pub const DIRECTORY: &str = "EFI/stela/firmware";
pub const MAX_MANIFEST_BYTES: usize = 512 * 1024;
pub const MAX_IMAGES: usize = 64;
pub const MAX_IMAGE_BYTES: usize = 64 * 1024 * 1024;
/// Room for a GSP-RM image per GPU family, beside every other driver's firmware.
pub const MAX_TOTAL_BYTES: usize = 256 * 1024 * 1024;
pub const MAX_LABEL_BYTES: usize = 256;
pub const MAX_PATH_BYTES: usize = 240;

pub mod bundle;
pub mod recipes;
pub mod requirements;

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Manifest<'a> {
    pub schema: u32,
    #[serde(borrow, deserialize_with = "bounded_images")]
    pub images: Vec<Image<'a>>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Image<'a> {
    /// Logical Linux/vendor name, case-exact and independent of the FAT path.
    pub name: &'a str,
    /// Canonical lower-case FAT-safe path relative to DIRECTORY.
    pub path: &'a str,
    #[serde(borrow)]
    pub target: Identity<'a>,
    #[serde(borrow)]
    pub provenance: Source<'a>,
    pub length: usize,
    pub sha256: &'a str,
}

#[derive(Debug, Deserialize, PartialEq, Eq)]
#[serde(deny_unknown_fields)]
pub struct Identity<'a> {
    pub driver: &'a str,
    pub hardware: &'a str,
    pub revision: &'a str,
    pub board: Option<&'a str>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Source<'a> {
    pub source: &'a str,
    pub revision: &'a str,
    pub member: &'a str,
    pub license: &'a str,
}

fn bounded_images<'de, D: Deserializer<'de>>(deserializer: D) -> Result<Vec<Image<'de>>, D::Error> {
    struct Images;
    impl<'de> Visitor<'de> for Images {
        type Value = Vec<Image<'de>>;
        fn expecting(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
            write!(f, "at most {MAX_IMAGES} firmware resources")
        }
        fn visit_seq<A: SeqAccess<'de>>(self, mut seq: A) -> Result<Self::Value, A::Error> {
            let mut images = Vec::new();
            while let Some(image) = seq.next_element()? {
                if images.len() == MAX_IMAGES {
                    return Err(de::Error::custom("too many firmware resources"));
                }
                images.push(image);
            }
            Ok(images)
        }
    }
    deserializer.deserialize_seq(Images)
}

/// Plain, bounded diagnostic suitable for a setup error or pre-boot refusal.
#[derive(Debug)]
pub struct Error(pub String);
impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.0)
    }
}
impl core::error::Error for Error {}
fn fail(message: &str) -> Error {
    Error(message.into())
}

impl<'a> Manifest<'a> {
    pub fn parse(bytes: &'a [u8]) -> Result<Self, Error> {
        if bytes.len() > MAX_MANIFEST_BYTES {
            return Err(fail("host firmware manifest exceeds 512 KiB"));
        }
        let manifest: Self = serde_json::from_slice(bytes)
            .map_err(|e| Error(format!("host firmware manifest: {e}")))?;
        if manifest.schema != 1 {
            return Err(fail("unsupported host firmware manifest schema"));
        }
        let mut total = 0usize;
        for (index, image) in manifest.images.iter().enumerate() {
            if !logical_path(image.name)
                || !storage_path(image.path)
                || image.path == "manifest.json"
            {
                return Err(fail("invalid host firmware name or storage path"));
            }
            for label in [
                image.target.driver,
                image.target.hardware,
                image.target.revision,
                image.provenance.source,
                image.provenance.revision,
                image.provenance.license,
            ] {
                if !valid_label(label) {
                    return Err(fail("invalid host firmware identity/provenance label"));
                }
            }
            if image.target.board.is_some_and(|board| !valid_label(board))
                || !logical_path(image.provenance.member)
            {
                return Err(fail("invalid host firmware board or source member"));
            }
            if image.length == 0 || image.length > MAX_IMAGE_BYTES {
                return Err(fail("host firmware image exceeds the nonzero 64 MiB limit"));
            }
            total = total
                .checked_add(image.length)
                .filter(|n| *n <= MAX_TOTAL_BYTES)
                .ok_or_else(|| fail("host firmware total exceeds its budget"))?;
            image.digest()?;
            for previous in &manifest.images[..index] {
                if previous.path == image.path {
                    return Err(fail("duplicate host firmware storage path"));
                }
                if previous.name == image.name && previous.target == image.target {
                    return Err(fail("duplicate host firmware identity"));
                }
            }
        }
        Ok(manifest)
    }

    pub fn total_bytes(&self) -> usize {
        self.images.iter().map(|image| image.length).sum()
    }

    pub fn check_mount(&self, image: &[u8]) -> Result<(), Error> {
        MountLimits::from_image(image)
            .ok_or_else(|| fail("stela.efi has no supported host firmware mount capability"))?
            .check_lengths(self.images.iter().map(|image| image.length))
    }

    /// Build metadata in the active allocator; permanent pages are never copied.
    /// Each slice must remain immutable and mapped for this catalog's
    /// lifetime. Physical-to-reference construction belongs to the platform.
    pub fn catalog(&self, bytes: &[&'static [u8]]) -> Result<Catalog, Error> {
        self.catalog_with(bytes, Vec::new())
    }

    /// Add platform-observed permanent resources in the same validation pass.
    /// Descriptor borrows are copied into catalog metadata; image bytes remain
    /// in the caller's immutable permanent mappings.
    pub fn catalog_with<'b>(
        &'b self,
        bytes: &[&'static [u8]],
        additional: Vec<Resource<'b, &'static [u8]>>,
    ) -> Result<Catalog, Error> {
        if bytes.len() != self.images.len() {
            return Err(fail("host firmware handoff count mismatch"));
        }
        if self
            .images
            .len()
            .checked_add(additional.len())
            .is_none_or(|count| count > MAX_IMAGES)
            || additional
                .iter()
                .any(|resource| resource.bytes.len() > MAX_IMAGE_BYTES)
            || self
                .images
                .iter()
                .map(|image| image.length)
                .chain(additional.iter().map(|resource| resource.bytes.len()))
                .try_fold(0usize, |total, bytes| total.checked_add(bytes))
                .is_none_or(|total| total > MAX_TOTAL_BYTES)
        {
            return Err(fail("combined host firmware resources exceed their budget"));
        }
        let mut resources = self
            .images
            .iter()
            .zip(bytes)
            .map(|(image, bytes)| {
                Ok(Resource {
                    name: image.name,
                    target: Target {
                        driver: image.target.driver,
                        hardware: image.target.hardware,
                        revision: image.target.revision,
                        board: image.target.board,
                    },
                    provenance: Provenance {
                        source: image.provenance.source,
                        revision: image.provenance.revision,
                        member: image.provenance.member,
                        license: image.provenance.license,
                    },
                    expected_length: image.length,
                    expected_sha256: image.digest()?,
                    bytes: *bytes,
                })
            })
            .collect::<Result<Vec<_>, Error>>()?;
        resources.extend(additional);
        Catalog::from_persistent_handoff(resources).map_err(|e| Error(format!("host {e}")))
    }
}

impl Image<'_> {
    pub fn digest(&self) -> Result<[u8; 32], Error> {
        if self.sha256.len() != 64
            || !self
                .sha256
                .bytes()
                .all(|b| b.is_ascii_digit() || (b'a'..=b'f').contains(&b))
        {
            return Err(fail(
                "host firmware SHA-256 must be 64 lower-case hex digits",
            ));
        }
        let mut out = [0; 32];
        for (index, byte) in out.iter_mut().enumerate() {
            *byte = u8::from_str_radix(&self.sha256[index * 2..index * 2 + 2], 16)
                .map_err(|_| fail("invalid host firmware digest"))?;
        }
        Ok(out)
    }
}

fn valid_label(label: &str) -> bool {
    !label.is_empty()
        && label.len() <= MAX_LABEL_BYTES
        && label.bytes().all(|b| (0x20..=0x7e).contains(&b))
}
fn logical_path(path: &str) -> bool {
    valid_label(path)
        && !path.contains('\\')
        && path.split('/').all(|part| !matches!(part, "" | "." | ".."))
}
/// Storage paths have one representation: no drive names, UEFI separators,
/// traversal, trailing-dot/space aliases, DOS device names or 8.3 tilde aliases.
pub fn storage_path(path: &str) -> bool {
    if path.is_empty() || path.len() > MAX_PATH_BYTES {
        return false;
    }
    path.split('/').all(|part| {
        if part.is_empty() || part.len() > 64 || part.starts_with('.') || part.ends_with('.') {
            return false;
        }
        if !part.bytes().all(|b| {
            b.is_ascii_lowercase() || b.is_ascii_digit() || matches!(b, b'.' | b'_' | b'-')
        }) {
            return false;
        }
        let stem = part.split('.').next().unwrap_or("");
        !matches!(stem, "con" | "prn" | "aux" | "nul")
            && !["com", "lpt"].iter().any(|prefix| {
                stem.strip_prefix(prefix).is_some_and(|suffix| {
                    suffix.len() == 1 && matches!(suffix.as_bytes()[0], b'1'..=b'9')
                })
            })
    })
}

#[cfg(feature = "std")]
mod package;
#[cfg(feature = "std")]
pub use package::{Package, check_installed_mount, sync_directory};

#[cfg(test)]
mod tests;
