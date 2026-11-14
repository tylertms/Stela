use super::*;
use {
    stela_formats::payload::IdentitySet, stela_formats::payload::Member,
    stela_formats::payload::test_support::Memory, stela_formats::payload::test_support::manifest,
    stela_formats::payload::zip::test_support::archive,
};
pub(crate) fn fixture(data: &[u8], deflate: bool) -> Store<Memory> {
    with_components(data, deflate, &[])
}
/// Give a store's manifest AEA keys, for the staged-answer tests.
pub(crate) fn set_aea_keys(store: &mut Store<Memory>, keys: Vec<stela_formats::payload::AeaKey>) {
    store.manifest.aea_keys = keys;
}
/// Set one Info field on the selected fixture identity.
pub(crate) fn set_info(store: &mut Store<Memory>, key: &str, value: Value) {
    let Value::Dict(identity) = &mut store.identities[store.current].value else {
        panic!("fixture identity is not a dictionary")
    };
    let Value::Dict(info) = identity.get_mut("Info").expect("fixture Info") else {
        panic!("fixture Info is not a dictionary")
    };
    info.insert(key.into(), value);
}
pub(crate) fn set_component_info(
    store: &mut Store<Memory>,
    component: &str,
    key: &str,
    value: Value,
) {
    let Value::Dict(identity) = &mut store.identities[store.current].value else {
        panic!("fixture identity")
    };
    let Value::Dict(manifest) = identity.get_mut("Manifest").unwrap() else {
        panic!("fixture manifest")
    };
    let Value::Dict(entry) = manifest.get_mut(component).unwrap() else {
        panic!("fixture component")
    };
    let Value::Dict(info) = entry.get_mut("Info").unwrap() else {
        panic!("fixture component Info")
    };
    info.insert(key.into(), value);
}
/// Drop a member from a store's inventory, as a trimmed medium leaves it.
pub(crate) fn drop_member(store: &mut Store<Memory>, name: &str) {
    store.manifest.members.retain(|m| m.name != name);
}
/// The fixture with more BuildManifest components, each `(name, path,
/// bytes)` with its SHA-384 as the Digest.
pub(crate) fn with_components(
    data: &[u8],
    deflate: bool,
    extra: &[(&str, &str, &[u8])],
) -> Store<Memory> {
    with_digests(data, deflate, extra, &[])
}
/// `with_components` with the named components' Digest replaced, for a
/// manifest whose digest covers something other than the member's
/// bytes. Components naming one path share one member.
pub(crate) fn with_digests(
    data: &[u8],
    deflate: bool,
    extra: &[(&str, &str, &[u8])],
    digests: &[(&str, [u8; 48])],
) -> Store<Memory> {
    let (reader, manifest) = parts(data, deflate, extra, digests, ERASE);
    Store::open(reader, manifest, 0).unwrap()
}
/// A BuildManifest identity's variant and behavior, and the purpose the
/// inventory records for it.
pub(crate) type Behavior = (&'static str, &'static str, Purpose);
pub(crate) const ERASE: Behavior = (
    "Customer Erase Install (IPSW)",
    ERASE_BEHAVIOR,
    Purpose::Erase,
);
pub(crate) const UPDATE: Behavior = (
    "Customer Upgrade Install (IPSW)",
    UPDATE_BEHAVIOR,
    Purpose::Update,
);
/// The fixture's IPSW and inventory, its one identity `behavior`'s.
pub(crate) fn parts(
    data: &[u8],
    deflate: bool,
    extra: &[(&str, &str, &[u8])],
    digests: &[(&str, [u8; 48])],
    behavior: Behavior,
) -> (Memory, PayloadManifest) {
    use sha2::{Digest, Sha384};
    let (variant, restore_behavior, purpose) = behavior;
    let mut components: BTreeMap<String, Value> = [(
        "OS".into(),
        Value::dict([
            ("Info", Value::dict([("Path", Value::from("os.dmg"))])),
            ("Digest", Value::Data(vec![1; 48])),
        ]),
    )]
    .into();
    for (name, path, bytes) in extra {
        components.insert(
            (*name).into(),
            Value::dict([
                ("Info", Value::dict([("Path", Value::from(*path))])),
                (
                    "Digest",
                    Value::Data(
                        digests
                            .iter()
                            .find(|(component, _)| component == name)
                            .map_or_else(|| Sha384::digest(bytes).to_vec(), |(_, d)| d.to_vec()),
                    ),
                ),
            ]),
        );
    }
    let identity = Value::dict([
        (
            "Info",
            Value::dict([
                ("Variant", Value::from(variant)),
                ("MacOSVariant", Value::from("macOS Customer")),
                ("RestoreBehavior", Value::from(restore_behavior)),
                ("DeviceClass", Value::from("testap")),
                ("OSDiskImageSize", Value::Integer(123)),
            ]),
        ),
        ("ApChipID", Value::from("0x8112")),
        ("ApBoardID", Value::from("0x24")),
        ("ApSecurityDomain", Value::from("0x1")),
        ("UniqueBuildID", Value::Data(vec![0x12; 8])),
        ("Manifest", Value::Dict(components)),
    ]);
    let build = plist::encode_xml(
        &Value::dict([("BuildIdentities", Value::Array(vec![identity]))]),
        Limits::default(),
    )
    .unwrap();
    let (mut m, _) = manifest();
    m.device_class = "testap".into();
    m.source.build_manifest_sha256 = stela_formats::sha256(&build);
    m.global_manifests.clear();
    m.aea_keys.clear();
    let mut files: Vec<(&str, &[u8])> = vec![("os.dmg", data)];
    for (_, path, bytes) in extra {
        if !files.iter().any(|(name, _)| name == path) {
            files.push((*path, *bytes));
        }
    }
    m.members = files
        .iter()
        .map(|(name, bytes)| Member {
            name: (*name).into(),
            bytes: bytes.len() as u64,
            sha256: Some(stela_formats::sha256(bytes)),
            sha384: None,
        })
        .collect();
    m.identities = vec![IdentitySet {
        index: 0,
        variant: variant.into(),
        purpose,
        members: files.iter().map(|(name, _)| (*name).into()).collect(),
    }];
    let entries: Vec<(&str, &[u8], bool)> =
        core::iter::once(("BuildManifest.plist", build.as_slice(), false))
            .chain(files.iter().map(|(name, bytes)| (*name, *bytes, deflate)))
            .collect();
    (Memory(archive(&entries, true)), m)
}
/// `s`'s member hashes as a block hash file, its manifest naming it.
fn precomputed(s: &mut Store<Memory>, data: &[u8]) -> Vec<u8> {
    let file: Vec<u8> = data
        .chunks(BLOCK_BYTES)
        .flat_map(|block| Sha256::digest(block).to_vec())
        .collect();
    s.manifest.members[0].sha384 = Some(hex::encode(Sha384::digest(data)));
    s.manifest.block_hashes = Some(stela_formats::payload::BlockHashes {
        file: stela_formats::payload::BLOCK_HASHES_FILE.into(),
        block_bytes: stela_formats::payload::HASH_BLOCK_BYTES,
        sha256: stela_formats::sha256(&file),
    });
    s.manifest.validate().unwrap();
    file
}

#[test]
fn recorded_hashes_check_every_member_before_the_restore_by_default() {
    let data: Vec<u8> = (0..MAX_READ_BYTES + 5).map(|i| (i * 7) as u8).collect();
    let mut s = fixture(&data, true);
    let file = precomputed(&mut s, &data);
    s.adopt_block_hashes(&file).unwrap();
    assert!(!s.prepared(), "the whole member is still to be checked");
    s.prepare().unwrap();
    assert!(s.prepared());
    assert_eq!(s.read_at("os.dmg", 5, 10).unwrap(), data[5..15]);

    // The same records refuse a changed block or SHA-384 by name.
    let mut s = fixture(&data, true);
    let mut file = precomputed(&mut s, &data);
    let last = file.len() - 32;
    file[last] ^= 1;
    s.manifest.block_hashes.as_mut().unwrap().sha256 = stela_formats::sha256(&file);
    s.adopt_block_hashes(&file).unwrap();
    let error = s.prepare().unwrap_err().to_string();
    assert!(
        error.contains("os.dmg: source changed at block 16"),
        "{error}"
    );
    let mut s = fixture(&data, false);
    let file = precomputed(&mut s, &data);
    s.manifest.members[0].sha384 = Some("0".repeat(96));
    s.adopt_block_hashes(&file).unwrap();
    let error = s.prepare().unwrap_err().to_string();
    assert!(error.contains("os.dmg: SHA-384 differs"), "{error}");
    assert!(!s.prepared());
}

#[test]
fn verify_on_read_checks_recorded_hashes_as_reads_reach_them() {
    let data: Vec<u8> = (0..BLOCK_BYTES * 2 + 5).map(|i| (i * 3) as u8).collect();
    let mut s = fixture(&data, false);
    let file = alloc::boxed::Box::leak(precomputed(&mut s, &data).into_boxed_slice());
    assert!(!s.prepared());
    let error = s
        .adopt_static_block_hashes(&file[1..])
        .unwrap_err()
        .to_string();
    assert!(error.contains("blocks.bin"), "{error}");
    s.set_verify(Verify::OnRead);
    s.adopt_static_block_hashes(file).unwrap();
    assert!(core::ptr::eq(
        s.verified["os.dmg"].blocks.as_ref(),
        file.as_chunks::<32>().0
    ));
    // Nothing is left to read before the restore.
    assert!(s.prepared());
    s.prepare().unwrap();
    assert_eq!(
        s.digest384("os.dmg").unwrap().unwrap(),
        &Sha384::digest(&data)[..]
    );
    let mut buffer = Vec::with_capacity(BLOCK_BYTES);
    buffer.extend_from_slice(b"old");
    let pointer = buffer.as_ptr();
    let bytes = s.read_at_reusing("os.dmg", 10, 20, buffer).unwrap();
    assert_eq!(bytes, data[10..30], "a random read");
    assert_eq!(bytes.as_ptr(), pointer);
    // Read in order from the start, the whole member meets its SHA-384.
    let mut whole = s.read_at("os.dmg", 0, BLOCK_BYTES).unwrap();
    whole.extend(
        s.read_at("os.dmg", BLOCK_BYTES as u64, data.len() - BLOCK_BYTES)
            .unwrap(),
    );
    assert_eq!(whole, data);

    // A block hash that differs refuses that block by number.
    let mut s = fixture(&data, true);
    let mut file = precomputed(&mut s, &data);
    file[32] ^= 1;
    s.manifest.block_hashes.as_mut().unwrap().sha256 = stela_formats::sha256(&file);
    s.set_verify(Verify::OnRead);
    s.adopt_block_hashes(&file).unwrap();
    s.prepare().unwrap();
    s.read_at("os.dmg", 0, 10).unwrap();
    let error = s
        .read_at("os.dmg", BLOCK_BYTES as u64, 1)
        .unwrap_err()
        .to_string();
    assert!(
        error.contains("os.dmg: source changed at block 1"),
        "{error}"
    );

    // A SHA-384 that differs refuses the member once it is read whole.
    let mut s = fixture(&data, false);
    let file = precomputed(&mut s, &data);
    s.manifest.members[0].sha384 = Some("0".repeat(96));
    s.set_verify(Verify::OnRead);
    s.adopt_block_hashes(&file).unwrap();
    s.read_at("os.dmg", 0, BLOCK_BYTES).unwrap();
    let error = s
        .read_at("os.dmg", BLOCK_BYTES as u64, data.len() - BLOCK_BYTES)
        .unwrap_err()
        .to_string();
    assert!(error.contains("os.dmg: SHA-384 differs"), "{error}");
}

#[test]
fn random_reads_refuse_changed_bytes() {
    let data: Vec<_> = (0..BLOCK_BYTES * 3 + 7).map(|i| i as u8).collect();
    for deflate in [false, true] {
        let mut s = fixture(&data, deflate);
        assert_eq!(s.identity().index, 0);
        assert_eq!(
            s.read_component("OS", BLOCK_BYTES as u64 - 3, 12).unwrap(),
            data[BLOCK_BYTES - 3..BLOCK_BYTES + 9]
        );
        assert!(s.read_at("os.dmg", u64::MAX, 2).is_err());
        assert!(s.read_at("unknown", 0, 0).is_err());
        assert!(s.read_at("os.dmg", 0, MAX_READ_BYTES + 1).is_err());
        assert_eq!(
            s.read_at("os.dmg", data.len() as u64, 0).unwrap(),
            Vec::<u8>::new()
        );
        if !deflate {
            let at = s
                .reader
                .0
                .windows(16)
                .position(|w| w == &data[..16])
                .unwrap();
            s.reader.0[at] ^= 1;
            assert!(
                s.read_at_reusing("os.dmg", 0, 1, Vec::with_capacity(BLOCK_BYTES))
                    .is_err()
            );
        }
    }
}
#[test]
fn a_restore_opens_an_erase_or_update_identity_its_inventory_names_alike() {
    let (reader, manifest) = parts(b"os", false, &[], &[], UPDATE);
    let store = Store::open(reader, manifest, 0).unwrap();
    assert_eq!(
        store
            .identity()
            .info()
            .unwrap()
            .required("RestoreBehavior")
            .unwrap()
            .string()
            .unwrap(),
        UPDATE_BEHAVIOR
    );
    // The inventory's purpose must name the identity's behavior.
    let (reader, mut manifest) = parts(b"os", false, &[], &[], UPDATE);
    manifest.identities[0].purpose = Purpose::Erase;
    let error = Store::open(reader, manifest, 0).err().unwrap().to_string();
    assert!(error.contains("identity mismatch"), "{error}");
    // An installed-system identity is no restore.
    let (reader, manifest) = parts(
        b"os",
        false,
        &[],
        &[],
        ("macOS Customer", "Install", Purpose::Installed),
    );
    let error = Store::open(reader, manifest, 0).err().unwrap().to_string();
    assert!(error.contains("neither erase nor update"), "{error}");
}
#[test]
fn a_remote_images_device_verified_member_passes_through_unhashed() {
    let data = b"bytes the device verifies";
    let mut s = fixture(data, true);
    s.manifest.layout = stela_formats::payload::Layout::Remote {
        url: "http://updates-http.cdn-apple.com/a.ipsw".into(),
        etag: "\"e\"".into(),
    };
    s.manifest.source.sha256 = None;
    s.manifest.members[0].sha256 = None;
    s.prepare().unwrap();
    assert!(s.prepared());
    assert_eq!(s.read_at("os.dmg", 6, 3).unwrap(), &data[6..9]);
    assert_eq!(s.read_at("os.dmg", 0, data.len()).unwrap(), data);
    assert_eq!(s.digest384("os.dmg").unwrap(), None);
}
#[test]
fn corrupt_inventory_fails_before_any_bytes_are_returned() {
    let mut s = fixture(b"hello", false);
    s.manifest.members[0].sha256 = Some("0".repeat(64));
    assert!(s.read_at("os.dmg", 0, 1).is_err());
    assert!(s.verified.is_empty());
}

#[test]
fn interleaved_deflated_members_resume_without_replaying_prefixes() {
    struct Counting {
        source: Memory,
        bytes: usize,
    }
    impl ReadAt for Counting {
        type Error = &'static str;

        fn len(&self) -> u64 {
            self.source.len()
        }

        fn read_at(
            &mut self,
            offset: u64,
            buffer: &mut [u8],
        ) -> core::result::Result<(), Self::Error> {
            self.bytes += buffer.len();
            self.source.read_at(offset, buffer)
        }
    }

    let contents = |mut state: u32| -> Vec<u8> {
        (0..8 * BLOCK_BYTES)
            .map(|_| {
                state ^= state << 13;
                state ^= state >> 17;
                state ^= state << 5;
                state as u8
            })
            .collect()
    };
    let data = contents(0x5a);
    let other = contents(0x61);
    let (source, manifest) = parts(&data, true, &[("Extra", "extra.dmg", &other)], &[], ERASE);
    let mut store = Store::open(Counting { source, bytes: 0 }, manifest, 0).unwrap();
    store.prepare().unwrap();
    let mut read_both = |discard_cursors| {
        store.reader.bytes = 0;
        for offset in (0..data.len()).step_by(BLOCK_BYTES) {
            for (path, expected) in [("os.dmg", &data), ("extra.dmg", &other)] {
                if discard_cursors {
                    store.cursors.0.clear();
                }
                assert_eq!(
                    store.read_at(path, offset as u64, BLOCK_BYTES).unwrap(),
                    expected[offset..offset + BLOCK_BYTES]
                );
            }
        }
        store.reader.bytes
    };

    let resumed = read_both(false);
    let replayed = read_both(true);
    assert!(
        replayed >= 4 * resumed,
        "resumed {resumed}, replayed {replayed}"
    );
}

#[test]
fn a_read_whose_source_ran_out_of_time_is_pending_and_goes_on_later() {
    /// A network source bounded for its caller: its next `stalls` reads
    /// run out of time; `broken` fails every read for another reason.
    struct Stalling {
        source: Memory,
        stalls: usize,
        broken: bool,
        blocked: bool,
    }
    impl ReadAt for Stalling {
        type Error = &'static str;

        fn len(&self) -> u64 {
            self.source.len()
        }

        fn read_at(
            &mut self,
            offset: u64,
            buffer: &mut [u8],
        ) -> core::result::Result<(), Self::Error> {
            if self.stalls > 0 {
                self.stalls -= 1;
                self.blocked = true;
                return Err("the read's time ran out");
            }
            if self.broken {
                return Err("connection refused");
            }
            self.source.read_at(offset, buffer)
        }

        fn blocked(&mut self) -> bool {
            core::mem::take(&mut self.blocked)
        }
    }

    let data: Vec<u8> = (0..3 * BLOCK_BYTES).map(|n| (n % 251) as u8).collect();
    for deflate in [false, true] {
        let (source, manifest) = parts(&data, deflate, &[], &[], ERASE);
        let reader = Stalling {
            source,
            stalls: 0,
            broken: false,
            blocked: false,
        };
        let mut store = Store::open(reader, manifest, 0).unwrap();
        store.prepare().unwrap();
        store.reader.stalls = 2;
        for _ in 0..2 {
            assert!(matches!(
                store.read_at("os.dmg", 5, BLOCK_BYTES),
                Err(Error::Pending)
            ));
        }
        assert_eq!(
            store.read_at("os.dmg", 5, BLOCK_BYTES).unwrap(),
            data[5..5 + BLOCK_BYTES]
        );
        // Any other failure stays the payload's refusal.
        store.cursors.0.clear();
        store.reader.broken = true;
        let error = store.read_at("os.dmg", 2 * BLOCK_BYTES as u64, 16);
        assert!(matches!(error, Err(Error::Payload(_))), "{error:?}");
    }
}

#[test]
fn a_scan_a_pending_read_stops_goes_on_without_reading_again() {
    /// Every third read runs out of time; counts the bytes it serves.
    struct Stalling {
        source: Memory,
        reads: usize,
        served: u64,
        blocked: bool,
        /// Reads are bounded once the store is open.
        bounded: bool,
    }
    impl ReadAt for Stalling {
        type Error = &'static str;

        fn len(&self) -> u64 {
            self.source.len()
        }

        fn read_at(
            &mut self,
            offset: u64,
            buffer: &mut [u8],
        ) -> core::result::Result<(), Self::Error> {
            self.reads += 1;
            if self.bounded && self.reads.is_multiple_of(3) {
                self.blocked = true;
                return Err("the read's time ran out");
            }
            self.served += buffer.len() as u64;
            self.source.read_at(offset, buffer)
        }

        fn blocked(&mut self) -> bool {
            core::mem::take(&mut self.blocked)
        }
    }

    // Barely compressible, so a deflate member refills its input often.
    let mut state = 0x2545_f491_u32;
    let data: Vec<u8> = (0..16 * BLOCK_BYTES + 777)
        .map(|_| {
            state ^= state << 13;
            state ^= state >> 17;
            state ^= state << 5;
            (state >> 24) as u8
        })
        .collect();
    for deflate in [false, true] {
        let mut clean = fixture(&data, deflate);
        clean.prepare().unwrap();
        let (source, manifest) = parts(&data, deflate, &[], &[], ERASE);
        let archive = source.len();
        let reader = Stalling {
            source,
            reads: 0,
            served: 0,
            blocked: false,
            bounded: false,
        };
        let mut store = Store::open(reader, manifest, 0).unwrap();
        store.reader.served = 0;
        store.reader.bounded = true;
        let mut pending = 0;
        loop {
            match store.prepare() {
                Ok(()) => break,
                Err(Error::Pending) => pending += 1,
                Err(error) => panic!("deflate={deflate}: {error:?}"),
            }
            assert!(pending < 10_000);
        }
        assert!(pending > 2, "deflate={deflate}");
        // Each byte is read once, plus member headers.
        assert!(
            store.reader.served < archive + 64 * 1024,
            "deflate={deflate}: served {} of {archive}",
            store.reader.served
        );
        assert_eq!(
            store.verified["os.dmg"].blocks,
            clean.verified["os.dmg"].blocks
        );
        assert_eq!(
            store.digest384("os.dmg").unwrap(),
            clean.digest384("os.dmg").unwrap()
        );
    }
}

/// A remote image's device-verified members have no scan: their reads go
/// straight to the cursors. A read longer than a few network refills that
/// runs out of time again and again still goes on where it stopped, and
/// reads each compressed byte once; any other failure drops the cursor.
#[test]
fn a_pending_device_verified_read_goes_on_from_its_bytes_so_far() {
    /// Every third read runs out of time once reads are bounded; counts
    /// the bytes it serves.
    struct Stalling {
        source: Memory,
        reads: usize,
        served: u64,
        blocked: bool,
        bounded: bool,
        broken: bool,
    }
    impl ReadAt for Stalling {
        type Error = &'static str;

        fn len(&self) -> u64 {
            self.source.len()
        }

        fn read_at(
            &mut self,
            offset: u64,
            buffer: &mut [u8],
        ) -> core::result::Result<(), Self::Error> {
            self.reads += 1;
            if self.broken {
                return Err("connection refused");
            }
            if self.bounded && self.reads.is_multiple_of(3) {
                self.blocked = true;
                return Err("the read's time ran out");
            }
            self.served += buffer.len() as u64;
            self.source.read_at(offset, buffer)
        }

        fn blocked(&mut self) -> bool {
            core::mem::take(&mut self.blocked)
        }
    }

    // Barely compressible, so one read spans many input refills.
    let mut state = 0x1b87_3593_u32;
    let data: Vec<u8> = (0..3 * MAX_READ_BYTES + 777)
        .map(|_| {
            state ^= state << 13;
            state ^= state >> 17;
            state ^= state << 5;
            (state >> 24) as u8
        })
        .collect();
    for deflate in [false, true] {
        let (source, mut manifest) = parts(&data, deflate, &[], &[], ERASE);
        manifest.layout = stela_formats::payload::Layout::Remote {
            url: "http://updates-http.cdn-apple.com/a.ipsw".into(),
            etag: "\"e\"".into(),
        };
        manifest.source.sha256 = None;
        manifest.members[0].sha256 = None;
        let archive = source.len();
        let reader = Stalling {
            source,
            reads: 0,
            served: 0,
            blocked: false,
            bounded: false,
            broken: false,
        };
        let mut store = Store::open(reader, manifest, 0).unwrap();
        store.prepare().unwrap();
        store.reader.served = 0;
        store.reader.bounded = true;
        let mut pending = 0;
        let mut offset = 0;
        while offset < data.len() {
            let len = MAX_READ_BYTES.min(data.len() - offset);
            let bytes = loop {
                match store.read_at("os.dmg", offset as u64, len) {
                    Ok(bytes) => break bytes,
                    Err(Error::Pending) => pending += 1,
                    Err(error) => panic!("deflate={deflate}: {error:?}"),
                }
                assert!(pending < 1000, "deflate={deflate}");
            };
            assert!(bytes == data[offset..offset + len], "deflate={deflate}");
            offset += len;
        }
        assert!(pending >= 2, "deflate={deflate}");
        assert!(
            store.reader.served < archive + 64 * 1024,
            "deflate={deflate}: served {} of {archive}",
            store.reader.served
        );
        // A failure that is not the source's time running out drops the
        // cursor: the next read starts the member again.
        store.reader.bounded = false;
        assert!(store.read_at("os.dmg", 0, 16).is_ok());
        store.reader.broken = true;
        let error = store.read_at("os.dmg", 16, MAX_READ_BYTES);
        assert!(matches!(error, Err(Error::Payload(_))), "{error:?}");
        assert!(store.cursors.0.is_empty(), "deflate={deflate}");
    }
}
