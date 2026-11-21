//! Immutable DART family, stream-remap and register-window contracts.
//! Resolving a mapper's declared route establishes no physical DMA permission.
use crate::Result;
use stela_contracts::refuse;
pub const DAPF_BYTES: u64 = 0x4000;
/// The largest DT translation window: M4/M5-generation `dart,t8110` nodes
/// declare 0xc000 or 0x20000 bytes (J604AP, J700AP, J714cAP).
pub const TRANSLATION_WINDOW_LIMIT: u64 = 0x20000;
pub const T8110_RECORDS: u64 = 0x8000;
pub const T8110_RECORD: u64 = 0x40;
pub const T8110_REGISTER_BYTES_2_2: u64 = T8110_RECORDS + 256 * T8110_RECORD;

/// Register and page-entry format, independently selected for each DT node.
/// This does not check every optional feature of a physical chip.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Profile {
    T8020,
    T6000,
    /// `dart,t8110` revision 1.0: one 16 KiB register file.
    T8110,
    /// The pre-2.2 16 KiB register file with a wide DVA space and ranged flush.
    T8110Rev20,
    /// `dart,t8110` revision 2.2: per-stream exception records above 0x4000
    /// make the register file 0xc000 bytes. A compatible string never names
    /// it; the node's translation window selects it.
    T8110Rev22,
}

impl Profile {
    /// Select one native family from an entire DT compatible string list.
    /// A conflicting or unknown DART family cannot fall back to another
    /// entry in the list. Selection alone does not check optional features.
    pub fn from_compatible(compatible: &[u8]) -> Option<Self> {
        let mut selected = None;
        for name in compatible
            .split(|&byte| byte == 0)
            .filter(|name| !name.is_empty())
        {
            let profile = match name {
                b"dart,t8020" => Self::T8020,
                b"dart,t6000" => Self::T6000,
                b"dart,t8110" => Self::T8110,
                name if name.starts_with(b"dart,") => return None,
                _ => continue,
            };
            if selected.is_some_and(|previous| previous != profile) {
                return None;
            }
            selected = Some(profile);
        }
        selected
    }

    pub const fn stream_limit(self) -> usize {
        match self {
            Self::T8020 | Self::T6000 => 16,
            Self::T8110 | Self::T8110Rev20 | Self::T8110Rev22 => 256,
        }
    }

    /// Whether the profile is a revision of the `dart,t8110` family, which
    /// shares its 16-byte DT instance descriptors and modern properties.
    pub const fn is_t8110(self) -> bool {
        matches!(self, Self::T8110 | Self::T8110Rev20 | Self::T8110Rev22)
    }

    /// Refine a compatible family by the bytes of its first translation
    /// window. Registers above 0x4000 exist only from revision 2.2: SPTM's
    /// error-mask table moves ERROR_MASK to 0x4060 at 0x202 (T8112 file
    /// 0x15a38/0x159d0) and the per-stream exception records reach 0xc000.
    /// A larger window implies at least revision 2.2. The default for a
    /// 16 KiB window is 1.0 unless the node requires wide DVA. The silicon's own
    /// revision is read from PARAMS3 by iBoot, SPTM and the kernel and is not
    /// recoverable statically, so this is an inference.
    pub fn for_translation_window(self, bytes: u64) -> Option<Self> {
        let profile = match self {
            Self::T8110 | Self::T8110Rev22 if bytes > DAPF_BYTES => Self::T8110Rev22,
            Self::T8110 | Self::T8110Rev22 => Self::T8110,
            profile => profile,
        };
        profile.admits_window(bytes).then_some(profile)
    }

    /// A 16 KiB T8110 window with an explicit DVA-flush request and a
    /// high DMA window needs at least 2.0. 26B123 T6020 SPTM rejects the
    /// request on 1.0/1.1 (file 0xdbdc0..0xdbdf0); the 2.2 fault register
    /// bank cannot fit this window. Infer 2.0 as the earliest matching profile.
    pub fn for_device_window(self, bytes: u64, flush_by_dva: bool, vm_base: u64) -> Option<Self> {
        if self.is_t8110() && bytes == DAPF_BYTES && flush_by_dva && vm_base >> 36 == 0x10 {
            Some(Self::T8110Rev20)
        } else {
            self.for_translation_window(bytes)
        }
    }

    /// Whether a translation window of `bytes` fits this register file: one
    /// 16 KiB block before revision 2.2, else 16 KiB blocks covering the
    /// 0xc000-byte register file, at most 0x20000. Registers between the
    /// register file and the window's end are unrecovered and refuse.
    pub fn admits_window(self, bytes: u64) -> bool {
        match self {
            Self::T8110Rev22 => {
                bytes.is_multiple_of(DAPF_BYTES)
                    && (T8110_REGISTER_BYTES_2_2..=TRANSLATION_WINDOW_LIMIT).contains(&bytes)
            }
            _ => bytes == DAPF_BYTES,
        }
    }

    /// Validate a DART's `remap` (source, target) pairs as every
    /// generation's 26A434 SPTM does (T8103 VA 0xfffffff0270e4858, T8112 VA
    /// 0xfffffff0270d976c..0xfffffff0270d9a68): no source remapped twice or
    /// to itself, no target that is itself remapped, every SID in range.
    pub fn validate_remaps(self, streams: usize, remaps: &[(u8, u8)]) -> Result<()> {
        if streams == 0 || streams > self.stream_limit() {
            return Err(refuse!(
                "DART stream count is outside the selected native profile"
            ));
        }
        for (index, &(source, target)) in remaps.iter().enumerate() {
            if usize::from(source) >= streams
                || usize::from(target) >= streams
                || source == target
                || remaps[..index].iter().any(|&(s, _)| s == source)
                || remaps.iter().any(|&(s, _)| s == target)
            {
                return Err(refuse!(
                    "DART remaps chain, repeat or exceed the native profile"
                ));
            }
        }
        Ok(())
    }

    /// Resolve an OS mapper context to the incoming stream declared by DT.
    /// Several sources may share a target (J473AP `dart-dcp`: 6 and 7 to 5).
    /// Every generation's SPTM programs the
    /// routes before enabling DMA and its mapper entry points reject remapped
    /// sources. Sources sharing a context reach the same tables, so the
    /// endpoint uses the lowest. This is a board route declaration, not an
    /// observation of a physical bus transaction.
    pub fn mapper_input_stream(
        self,
        streams: usize,
        context: usize,
        remaps: &[(u8, u8)],
    ) -> Result<usize> {
        if context >= streams {
            return Err(refuse!("DART mapper context is outside the native profile"));
        }
        self.validate_remaps(streams, remaps)?;
        if remaps
            .iter()
            .any(|&(source, _)| usize::from(source) == context)
        {
            return Err(refuse!(
                "DART mapper names an incoming remapped source instead of its context"
            ));
        }
        Ok(remaps
            .iter()
            .filter(|&&(_, target)| usize::from(target) == context)
            .map(|&(source, _)| usize::from(source))
            .min()
            .unwrap_or(context))
    }
}
