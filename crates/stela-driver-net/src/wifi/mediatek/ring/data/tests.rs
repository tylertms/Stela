extern crate std;
use super::super::{
    rx::{Assembly, RxRing},
    tests::{Domain, Io, Memory, Source, State},
};
use super::*;
use core::sync::atomic::AtomicUsize;
use std::sync::Mutex;
struct Context(Arc<AtomicUsize>, u64);
impl Drop for Context {
    fn drop(&mut self) {
        self.0.fetch_add(1, Ordering::Relaxed);
    }
}
struct Model {
    tx: Arc<Mutex<State>>,
    rx: Arc<Mutex<State>>,
    memory: Arc<Mutex<Memory>>,
    rx_ring: RxRing<Io>,
    assembly: Assembly,
    drops: Arc<AtomicUsize>,
}
fn setup(chip: Chip, count: usize) -> (DataRing<Io, Context>, Model) {
    let memory = Arc::new(Mutex::new(Memory::default()));
    let tx = Arc::new(Mutex::new(State {
        memory: memory.clone(),
        ..State::default()
    }));
    let rx = Arc::new(Mutex::new(State {
        memory: memory.clone(),
        ..State::default()
    }));
    let domain: Arc<dyn DmaDomain> = Arc::new(Domain(memory.clone()));
    // SAFETY: one fresh, stopped modeled requester; both queues resolve DMA
    // through its one mapping registry. No physical claim is made.
    let ring =
        unsafe { DataRing::install(Io(tx.clone()), &Source, domain.clone(), count, chip) }.unwrap();
    // SAFETY: the model device, its pages and its domain belong to this test alone; no other
    // driver, worker or DMA touches them.
    let rx_ring = unsafe { RxRing::install(Io(rx.clone()), &Source, domain, 8) }.unwrap();
    (
        ring,
        Model {
            tx,
            rx,
            memory,
            rx_ring,
            assembly: Assembly::default(),
            drops: Arc::new(AtomicUsize::new(0)),
        },
    )
}
fn config(chip: Chip, bytes: usize) -> txwi::Config {
    txwi::Config {
        chip,
        wcid: 7,
        vif: Some(txwi::Vif {
            omac: 2,
            wmm: 1,
            band: 0,
            mld: false,
        }),
        phy_band_one: false,
        queue: txwi::Queue::Data { access_category: 0 },
        pid: 1,
        frame: txwi::Frame {
            bytes,
            header_bytes: 26,
            kind: txwi::Kind::Data,
            subtype: 8,
            tid: 5,
            multicast: false,
            fragment: txwi::Fragment::None,
            sequence: txwi::Sequence::Hardware,
            minimum_rate: false,
        },
        protect: true,
        no_ack: false,
        hardware_amsdu: false,
        fixed_rate: None,
    }
}
fn pointer(memory: &Memory, address: u64, bytes: usize) -> *const u8 {
    let (&base, (pages, pointer, direction)) = memory
        .mappings
        .range(..=address)
        .next_back()
        .expect("actual mapped IOVA");
    let offset = usize::try_from(address - base).unwrap();
    assert!(offset + bytes <= pages.bytes());
    assert_ne!(*direction, Direction::FromDevice);
    assert_ne!(address, (*pointer + offset) as u64);
    (*pointer + offset) as *const u8
}
struct DevicePacket {
    address: u64,
    token: u16,
    wcid: u16,
    segments: Vec<(u64, usize)>,
}
fn fetch(model: &Model, expected_txd: [u8; 32]) -> DevicePacket {
    let mut tx = model.tx.lock().unwrap();
    assert_ne!(tx.index, tx.head);
    let memory = model.memory.lock().unwrap();
    // SAFETY: the pointer addresses live memory this test's model owns, in bounds, and nothing
    // accesses it concurrently.
    let descriptor = unsafe {
        core::slice::from_raw_parts(
            pointer(&memory, tx.base + u64::from(tx.index) * 16, 16).cast::<u32>(),
            4,
        )
    };
    assert_eq!(u32::from_le(descriptor[1]), LAST | (64 << 16));
    assert_eq!(descriptor[2..], [0, 0]);
    let address = u64::from(u32::from_le(descriptor[0]));
    // SAFETY: the pointer addresses live memory this test's model owns, in bounds, and nothing
    // accesses it concurrently.
    let txwi = unsafe { core::slice::from_raw_parts(pointer(&memory, address, 64), 64) };
    assert_eq!(&txwi[..32], expected_txd);
    assert_eq!(
        u16::from_le_bytes(txwi[32..34].try_into().unwrap()) & 0x8000,
        0x8000
    );
    let token = u16::from_le_bytes(txwi[32..34].try_into().unwrap()) & 0x7fff;
    assert!(token < 8192);
    assert_eq!(&txwi[34..40], &[0; 6]);
    let mut segments = Vec::new();
    for i in 0..4 {
        let at = 40 + i / 2 * 12;
        let (address_at, length_at) = if i % 2 == 0 {
            (at, at + 4)
        } else {
            (at + 8, at + 6)
        };
        let addr = u64::from(u32::from_le_bytes(
            txwi[address_at..address_at + 4].try_into().unwrap(),
        ));
        let len = u16::from_le_bytes(txwi[length_at..length_at + 2].try_into().unwrap());
        assert!((1..=4095).contains(&(len & 0x7fff)));
        assert_eq!(len & 0x7000, 0);
        segments.push((addr, usize::from(len & 0xfff)));
        if len & 0x8000 != 0 {
            break;
        }
    }
    let wcid = u32::from_le_bytes(txwi[4..8].try_into().unwrap()) as u16 & 0x3ff;
    // Only WFDMA's TXWI fetch has completed. Firmware may read both TXWI
    // and all referenced MSDUs later, after this index is visible to host.
    tx.index = (tx.index + 1) % tx.count;
    DevicePacket {
        address,
        token,
        wcid,
        segments,
    }
}
fn finish(model: &Model, packet: &DevicePacket) -> Vec<u8> {
    let memory = model.memory.lock().unwrap();
    // SAFETY: the pointer addresses live memory this test's model owns, in bounds, and nothing
    // accesses it concurrently.
    let txwi = unsafe { core::slice::from_raw_parts(pointer(&memory, packet.address, 64), 64) };
    assert_eq!(
        u16::from_le_bytes(txwi[32..34].try_into().unwrap()) & 0x7fff,
        packet.token
    );
    let mut payload = Vec::new();
    for &(address, bytes) in &packet.segments {
        // SAFETY: the pointer addresses live memory this test's model owns, in bounds, and
        // nothing accesses it concurrently.
        payload.extend_from_slice(unsafe {
            core::slice::from_raw_parts(pointer(&memory, address, bytes), bytes)
        });
    }
    payload
}
fn notification(chip: Chip, tokens: &[u16], wcid: u16) -> Vec<u8> {
    let mut words = vec![
        (6 << 27) | ((tokens.len() as u32) << 16),
        if chip == Chip::Mt7925 { 4 << 16 } else { 0 },
    ];
    words.push((1 << 31) | (u32::from(wcid) << if chip == Chip::Mt7925 { 12 } else { 14 }));
    if chip == Chip::Mt7925 {
        words.push((1 << 30) | (3 << 24));
        for pair in tokens.chunks(2) {
            words.push(u32::from(pair[0]) | (u32::from(*pair.get(1).unwrap_or(&0x7fff)) << 15));
        }
    } else {
        for &token in tokens {
            words.push((u32::from(token) << 16) | 3);
        }
    }
    words.iter().flat_map(|w| w.to_le_bytes()).collect()
}
fn deliver(model: &mut Model, bytes: &[u8]) -> Vec<u8> {
    super::super::rx::tests::device_receive(&model.rx, bytes, true, 0);
    let fragment = model.rx_ring.poll().unwrap().expect("actual DDONE");
    let frame = model
        .assembly
        .push(fragment)
        .unwrap()
        .expect("complete packet");
    assert!(!frame.drop);
    frame.bytes
}
fn quiesce(ring: &mut DataRing<Io, Context>, model: &mut Model) -> Vec<Context> {
    // SAFETY: modeled requester no longer accesses any queue/token buffer.
    let contexts = unsafe { ring.release_after_quiesce() }.unwrap();
    // SAFETY: the model device is stopped and drained, so nothing still uses what this
    // releases.
    unsafe { model.rx_ring.release_after_quiesce() }.unwrap();
    assert!(model.memory.lock().unwrap().mappings.is_empty());
    contexts
}
#[test]
fn refused_final_admission_changes_no_descriptor_token_or_context_owner() {
    if crate::test_process::isolate_model() {
        return;
    }
    let (mut ring, mut model) = setup(Chip::Mt7925, 8);
    let before = model.memory.lock().unwrap().mappings.len();
    let descriptor = ring.descriptors.read32(0).unwrap();
    let bytes = vec![0x5a; 100];
    let calls = AtomicUsize::new(0);
    let result = ring.submit_if(
        &Source,
        config(Chip::Mt7925, bytes.len()),
        &bytes,
        Context(model.drops.clone(), 1),
        || {
            calls.fetch_add(1, Ordering::Relaxed);
            // Both payload and TXWI are mapped before the gate.
            assert_eq!(model.memory.lock().unwrap().mappings.len(), before + 2);
            false
        },
    );
    assert_eq!(result, Err(Error::PublicationRefused));
    assert_eq!(calls.load(Ordering::Relaxed), 1);
    assert_eq!(model.memory.lock().unwrap().mappings.len(), before);
    assert_eq!(ring.descriptors.read32(0).unwrap(), descriptor);
    assert_eq!(ring.descriptors.read32(4).unwrap(), DONE);
    assert_eq!(model.tx.lock().unwrap().head, 0);
    assert_eq!(ring.pending_descriptors(), 0);
    assert_eq!(ring.outstanding_tokens(), 0);
    assert_eq!(model.drops.load(Ordering::Relaxed), 1);
    assert!(!ring.is_poisoned());
    let receipt = ring
        .submit(
            &Source,
            config(Chip::Mt7925, bytes.len()),
            &bytes,
            Context(model.drops.clone(), 2),
        )
        .unwrap();
    assert_eq!(
        (receipt.serial, receipt.token, receipt.descriptor),
        (1, 0, 0)
    );
    drop(quiesce(&mut ring, &mut model));
    assert_eq!(model.drops.load(Ordering::Relaxed), 2);
}
#[test]
fn requester_reads_txwi_then_late_payload_until_both_completions_through_wrap_and_reuse() {
    if crate::test_process::isolate_model() {
        return;
    }
    for chip in [Chip::Mt7921, Chip::Mt7922, Chip::Mt7925] {
        let (mut ring, mut model) = setup(chip, if chip == Chip::Mt7925 { 2048 } else { 8 });
        let baseline = model.memory.lock().unwrap().mappings.len();
        for n in 0..8201u64 {
            let bytes = vec![n as u8; [26, 4095, 4096, 8190, 16380][n as usize % 5]];
            let c = config(chip, bytes.len());
            let receipt = ring
                .submit(&Source, c, &bytes, Context(model.drops.clone(), n))
                .unwrap();
            assert_eq!(receipt.token, (n % 8192) as u16);
            assert!(ring.poll_consumed().unwrap().is_none());
            let packet = fetch(&model, txwi::encode(c).unwrap());
            assert_eq!(packet.token, receipt.token);
            if n % 2 == 0 {
                assert_eq!(ring.poll_consumed().unwrap(), Some(receipt));
                assert!(ring.take_released().unwrap().is_none());
                assert!(model.memory.lock().unwrap().mappings.len() > baseline);
            }
            assert_eq!(finish(&model, &packet), bytes);
            let wire = notification(chip, &[packet.token], packet.wcid);
            let complete = deliver(&mut model, &wire);
            // SAFETY: modeled firmware finished all TXP reads and
            // delivered this once through this requester's completed RX.
            unsafe { ring.apply_tx_free(ring.epoch(), &complete) }.unwrap();
            if n % 2 == 1 {
                assert!(ring.take_released().unwrap().is_none());
                assert_eq!(ring.poll_consumed().unwrap(), Some(receipt));
            }
            let released = ring.take_released().unwrap().expect("both receipts");
            assert_eq!(released.receipt, receipt);
            assert_eq!(released.context.1, n);
            assert_eq!(released.evidence.accounting.unwrap().attempts, 3);
            assert_eq!(released.evidence.accounting.unwrap().status, 0);
            assert_eq!(model.drops.load(Ordering::Relaxed), n as usize);
            drop(released);
            assert_eq!(model.memory.lock().unwrap().mappings.len(), baseline);
            assert_eq!(ring.outstanding_tokens(), 0);
        }
        assert!(quiesce(&mut ring, &mut model).is_empty());
        assert_eq!(model.drops.load(Ordering::Relaxed), 8201);
    }
}
#[test]
fn free_batches_release_held_tokens_and_skip_the_rest_as_mt7925_does() {
    if crate::test_process::isolate_model() {
        return;
    }
    for chip in [Chip::Mt7921, Chip::Mt7925] {
        for case in 0..5 {
            let (mut ring, mut model) = setup(chip, 8);
            let a = ring
                .submit(
                    &Source,
                    config(chip, 30),
                    &[7; 30],
                    Context(model.drops.clone(), 1),
                )
                .unwrap();
            let b = ring
                .submit(
                    &Source,
                    config(chip, 30),
                    &[8; 30],
                    Context(model.drops.clone(), 2),
                )
                .unwrap();
            let tokens = match case {
                // An unknown token is skipped; a repeated one frees once.
                0 => vec![a.token, 8192],
                1 => vec![a.token, a.token],
                // The token decides, whatever station the pair names.
                _ => vec![a.token, b.token],
            };
            let wire = notification(chip, &tokens, if case == 2 { 8 } else { 7 });
            let epoch = if case == 3 {
                Epoch(ring.epoch.0 + 1)
            } else {
                ring.epoch()
            };
            if case == 4 {
                let one = notification(chip, &[b.token], 7);
                // SAFETY: the model device, its pages and its domain belong to this test alone;
                // no other driver, worker or DMA touches them.
                unsafe { ring.apply_tx_free(epoch, &one) }.unwrap();
            }
            // SAFETY: the model device, its pages and its domain belong to this test alone; no
            // other driver, worker or DMA touches them.
            let result = unsafe { ring.apply_tx_free(epoch, &wire) };
            if case == 3 {
                // A notification of another epoch is a caller error.
                assert!(result.is_err());
                assert!(ring.is_poisoned());
            } else {
                result.unwrap();
                assert!(!ring.is_poisoned());
                let freed = |token: u16| ring.tokens[usize::from(token)].as_ref().unwrap().free;
                assert!(freed(a.token).is_some());
                assert_eq!(freed(b.token).is_some(), case >= 2);
                assert_eq!(ring.ignored_frees(), u64::from(matches!(case, 0 | 4)));
            }
            // A notification that does not parse is skipped whole.
            if case != 3 {
                // SAFETY: as above.
                unsafe { ring.apply_tx_free(epoch, &[0; 3]) }.unwrap();
                assert!(!ring.is_poisoned());
            }
            assert_eq!(ring.outstanding_tokens(), 2);
            assert_eq!(model.drops.load(Ordering::Relaxed), 0);
            drop(quiesce(&mut ring, &mut model));
            assert_eq!(model.drops.load(Ordering::Relaxed), 2);
        }
    }
}
#[test]
fn token_pressure_refuses_without_new_mappings() {
    if crate::test_process::isolate_model() {
        return;
    }
    let chip = Chip::Mt7925;
    let (mut ring, mut model) = setup(chip, 4);
    for n in 0..8192 {
        ring.submit(
            &Source,
            config(chip, 30),
            &[n as u8; 30],
            Context(model.drops.clone(), n),
        )
        .unwrap();
        fetch(&model, txwi::encode(config(chip, 30)).unwrap());
        assert!(ring.poll_consumed().unwrap().is_some());
    }
    let before = model.memory.lock().unwrap().mappings.len();
    assert_eq!(
        ring.submit(
            &Source,
            config(chip, 30),
            &[0; 30],
            Context(model.drops.clone(), 9999)
        ),
        Err(Error::Full)
    );
    assert_eq!(ring.outstanding_tokens(), 8192);
    assert_eq!(ring.pending_descriptors(), 0);
    assert_eq!(model.memory.lock().unwrap().mappings.len(), before);
    assert!(!ring.is_poisoned());
    drop(quiesce(&mut ring, &mut model));
    assert_eq!(model.drops.load(Ordering::Relaxed), 8193);
}

#[test]
fn ring_pressure_and_bad_frames_publish_nothing() {
    if crate::test_process::isolate_model() {
        return;
    }
    let chip = Chip::Mt7925;
    let (mut ring, mut model) = setup(chip, 4);
    for n in 0..2 {
        ring.submit(
            &Source,
            config(chip, 30),
            &[0; 30],
            Context(model.drops.clone(), n),
        )
        .unwrap();
    }
    assert_eq!(
        ring.submit(
            &Source,
            config(chip, 30),
            &[0; 30],
            Context(model.drops.clone(), 2)
        ),
        Err(Error::Full)
    );
    assert_eq!(ring.pending_descriptors(), 2);
    assert_eq!(
        ring.submit(
            &Source,
            config(Chip::Mt7921, 30),
            &[0; 30],
            Context(model.drops.clone(), 3)
        ),
        Err(Error::InvalidFrame)
    );
    assert_eq!(
        ring.submit(
            &Source,
            config(chip, 31),
            &[0; 30],
            Context(model.drops.clone(), 4)
        ),
        Err(Error::InvalidFrame)
    );
    drop(quiesce(&mut ring, &mut model));
    assert_eq!(model.drops.load(Ordering::Relaxed), 5);
}
#[test]
fn uncertain_publication_drop_keeps_real_queue_pins_and_context() {
    if crate::test_process::isolate_model() {
        return;
    }
    for failure in [false, true] {
        let chip = Chip::Mt7925;
        let (mut ring, mut model) = setup(chip, 8);
        model.tx.lock().unwrap().fail_publish = failure;
        let result = ring.submit(
            &Source,
            config(chip, 5000),
            &[0; 5000],
            Context(model.drops.clone(), 1),
        );
        assert_eq!(result.is_err(), failure);
        let before = model.memory.lock().unwrap().mappings.len();
        drop(ring);
        assert_eq!(model.tx.lock().unwrap().queue_drops, 0);
        assert_eq!(model.drops.load(Ordering::Relaxed), 0);
        assert_eq!(model.memory.lock().unwrap().mappings.len(), before);
        // SAFETY: the model device is stopped and drained, so nothing still uses what this
        // releases.
        unsafe { model.rx_ring.release_after_quiesce() }.unwrap();
        assert_eq!(model.rx.lock().unwrap().queue_drops, 0);
        drop(model.rx_ring);
        assert_eq!(model.rx.lock().unwrap().queue_drops, 1);
    }
}
#[test]
fn unmap_and_completion_visibility_failures_keep_context_until_quiescent_retry() {
    if crate::test_process::isolate_model() {
        return;
    }
    for failure in 0..6 {
        let chip = Chip::Mt7925;
        let (mut ring, mut model) = setup(chip, 8);
        ring.submit(
            &Source,
            config(chip, 5000),
            &[0; 5000],
            Context(model.drops.clone(), 1),
        )
        .unwrap();
        let packet = fetch(&model, txwi::encode(config(chip, 5000)).unwrap());
        assert_eq!(finish(&model, &packet), [0; 5000]);
        let complete = deliver(&mut model, &notification(chip, &[packet.token], 7));
        // SAFETY: the model device, its pages and its domain belong to this test alone; no
        // other driver, worker or DMA touches them.
        unsafe { ring.apply_tx_free(ring.epoch(), &complete) }.unwrap();
        assert!(ring.poll_consumed().unwrap().is_some());
        {
            let mut memory = model.memory.lock().unwrap();
            if failure < 3 {
                memory.fail_sync_at = Some(memory.syncs.len() + failure + 1);
            } else {
                memory.fail_unmap_at = Some(memory.unmaps + failure - 2);
            }
        }
        assert!(ring.take_released().is_err());
        assert!(ring.is_poisoned());
        assert_eq!(ring.outstanding_tokens(), 1);
        assert_eq!(model.drops.load(Ordering::Relaxed), 0);
        assert_eq!(model.tx.lock().unwrap().queue_drops, 0);
        {
            let mut memory = model.memory.lock().unwrap();
            memory.fail_sync_at = None;
            memory.fail_unmap_at = None;
        }
        drop(quiesce(&mut ring, &mut model));
        assert_eq!(model.drops.load(Ordering::Relaxed), 1);
    }
}

#[test]
fn every_submission_visibility_failure_distinguishes_unpublished_from_reachable_context() {
    if crate::test_process::isolate_model() {
        return;
    }
    for fail in 1..=5 {
        let chip = Chip::Mt7921;
        let (mut ring, mut model) = setup(chip, 8);
        let baseline = model.memory.lock().unwrap().mappings.len();
        {
            let mut memory = model.memory.lock().unwrap();
            memory.fail_sync_at = Some(memory.syncs.len() + fail);
        }
        assert!(
            ring.submit(
                &Source,
                config(chip, 10000),
                &[0; 10000],
                Context(model.drops.clone(), 1)
            )
            .is_err()
        );
        if fail < 5 {
            assert!(!ring.is_poisoned());
            assert_eq!(ring.outstanding_tokens(), 0);
            assert_eq!(model.memory.lock().unwrap().mappings.len(), baseline);
            assert_eq!(model.drops.load(Ordering::Relaxed), 1);
        } else {
            assert!(ring.is_poisoned());
            assert_eq!(ring.outstanding_tokens(), 1);
            assert!(model.memory.lock().unwrap().mappings.len() > baseline);
            assert_eq!(model.drops.load(Ordering::Relaxed), 0);
        }
        model.memory.lock().unwrap().fail_sync_at = None;
        drop(quiesce(&mut ring, &mut model));
        assert_eq!(model.drops.load(Ordering::Relaxed), 1);
    }
}

#[test]
fn invalid_indices_and_uncertain_constructor_keep_owner() {
    if crate::test_process::isolate_model() {
        return;
    }
    for actual in [1, u16::MAX] {
        let (mut ring, mut model) = setup(Chip::Mt7921, 8);
        model.tx.lock().unwrap().index = actual;
        assert_eq!(
            ring.poll_consumed(),
            Err(Error::Ring(super::super::Error::InvalidDeviceIndex))
        );
        assert!(ring.is_poisoned());
        quiesce(&mut ring, &mut model);
    }
    let memory = Arc::new(Mutex::new(Memory::default()));
    let state = Arc::new(Mutex::new(State {
        memory: memory.clone(),
        fail_configure: true,
        ..State::default()
    }));
    let domain: Arc<dyn DmaDomain> = Arc::new(Domain(memory.clone()));
    // SAFETY: the model device, its pages and its domain belong to this test alone; no other
    // driver, worker or DMA touches them.
    let result = unsafe {
        DataRing::<_, Context>::install(Io(state.clone()), &Source, domain, 2048, Chip::Mt7925)
    };
    assert!(matches!(
        result,
        Err(Error::Ring(super::super::Error::RegisterFailure))
    ));
    assert_eq!(state.lock().unwrap().queue_drops, 0);
    assert_eq!(memory.lock().unwrap().mappings.len(), 1);
}

#[test]
fn failed_partial_quiescent_unmap_drop_keeps_context_with_remaining_mappings() {
    if crate::test_process::isolate_model() {
        return;
    }
    let (mut ring, mut model) = setup(Chip::Mt7925, 8);
    ring.submit(
        &Source,
        config(Chip::Mt7925, 10000),
        &[0; 10000],
        Context(model.drops.clone(), 1),
    )
    .unwrap();
    {
        let mut memory = model.memory.lock().unwrap();
        memory.fail_unmap_at = Some(memory.unmaps + 2);
    }
    // SAFETY: the model device is stopped and drained, so nothing still uses what this
    // releases.
    assert!(unsafe { ring.release_after_quiesce() }.is_err());
    assert_eq!(model.drops.load(Ordering::Relaxed), 0);
    let remaining = model.memory.lock().unwrap().mappings.len();
    drop(ring);
    assert_eq!(model.memory.lock().unwrap().mappings.len(), remaining);
    assert_eq!(model.tx.lock().unwrap().queue_drops, 0);
    assert_eq!(model.drops.load(Ordering::Relaxed), 0);
    model.memory.lock().unwrap().fail_unmap_at = None;
    // SAFETY: the model device is stopped and drained, so nothing still uses what this
    // releases.
    unsafe { model.rx_ring.release_after_quiesce() }.unwrap();
}

#[test]
#[ignore = "oracle: STELA_MT792X_DATA_ORACLE"]
fn unchanged_dma_add_buf_matches_owned_queue_descriptors() {
    if crate::test_process::isolate_model() {
        return;
    }
    use std::{
        io::Write,
        process::{Command, Stdio},
    };
    let directory = std::env::var_os("STELA_MT792X_DATA_ORACLE").expect("STELA_MT792X_DATA_ORACLE");
    for count in [4, 8, 2048] {
        let chip = Chip::Mt7925;
        let (mut ring, mut model) = setup(chip, count);
        let mut input = Vec::new();
        let mut expected = Vec::new();
        for n in 0..100u64 {
            let head = ring.head;
            let queued = ring.queued;
            ring.submit(
                &Source,
                config(chip, 100),
                &[n as u8; 100],
                Context(model.drops.clone(), n),
            )
            .unwrap();
            let memory = model.memory.lock().unwrap();
            let descriptor = ring.descriptors.pointer(head * 16, 16).unwrap();
            // SAFETY: the pointer addresses live memory this test's model owns, in bounds, and
            // nothing accesses it concurrently.
            let bytes = unsafe { core::slice::from_raw_parts(descriptor, 16) };
            expected.extend_from_slice(bytes);
            for value in [
                ring.head as u32,
                ring.queued as u32,
                head as u32,
                1,
                1,
                0xffff,
            ] {
                expected.extend_from_slice(&value.to_le_bytes());
            }
            let address = u32::from_le_bytes(bytes[..4].try_into().unwrap());
            assert!(memory.mappings.contains_key(&u64::from(address)));
            for value in [address, count as u32, head as u32, queued as u32] {
                input.extend_from_slice(&value.to_le_bytes());
            }
            drop(memory);
            fetch(&model, txwi::encode(config(chip, 100)).unwrap());
            ring.poll_consumed().unwrap().unwrap();
        }
        let mut child = Command::new(std::path::Path::new(&directory).join("data"))
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .stderr(Stdio::piped())
            .spawn()
            .unwrap();
        child.stdin.take().unwrap().write_all(&input).unwrap();
        let result = child.wait_with_output().unwrap();
        assert!(
            result.status.success(),
            "{}",
            std::string::String::from_utf8_lossy(&result.stderr)
        );
        assert_eq!(result.stdout, expected, "actual owned queue count {count}");
        drop(quiesce(&mut ring, &mut model));
        assert_eq!(model.drops.load(Ordering::Relaxed), 100);
    }
}
