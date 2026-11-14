// SPDX-License-Identifier: GPL-2.0-only
//! VMD hierarchy/lifecycle composed with the shared independent NVMe peer.
//! All effects run synchronously in one thread. No MappingPlatform callback is
//! treated as a register-access hook, and no raw device memory races CPU memory.
use super::*;
use crate::bus::{
    dma::{
        mapping::{Direction, DmaError, DomainMapping},
        pages::DmaPages,
    },
    pci::regs,
    vmd::endpoint::test_support::*,
};
use alloc::{collections::BTreeMap, vec::Vec};
use core::{
    ops::Range,
    sync::atomic::{AtomicU64, AtomicUsize, Ordering},
};
extern crate std;
use crate::{block::BlockDevice, nvme::owned::protocol};
use protocol::{Fault, Protocol};
use std::sync::Mutex;

const PATH: [usize; 3] = [ROOT, BRIDGE, NVME];
const SIBLING: usize = NVME + (1 << 15);
/// The buffers an owned controller maps: four queues, the admin page and
/// the I/O pool.
const BUFFERS: usize = 5 + crate::nvme::POOL_PAGES;

struct Configuration(Arc<Mutex<State>>);
impl protocol::Access for Configuration {
    fn registers_available(&self) -> bool {
        let state = self.0.lock().unwrap();
        // Observe stored posted-write effects, not Lifecycle::phase or
        // the requested Command value. All three PM capabilities exist here.
        state.parent(regs::COMMAND, 2) & u32::from(regs::COMMAND_MEMORY) != 0
            && PATH.into_iter().all(|base| {
                state.child(base + COMMAND_OFFSET, 2) & u32::from(regs::COMMAND_MEMORY) != 0
                    && state.child(base + 0x84, 2) & 3 == 0
            })
    }
    fn dma_enabled(&self) -> bool {
        let state = self.0.lock().unwrap();
        state.parent(regs::COMMAND, 2) & u32::from(regs::COMMAND_MASTER) != 0
            && PATH.into_iter().all(|base| {
                state.child(base + COMMAND_OFFSET, 2) & u32::from(regs::COMMAND_MASTER) != 0
                    && state.child(base + 0x84, 2) & 3 == 0
            })
    }
    fn any_bus_master(&self) -> bool {
        self.0.lock().unwrap().any_master()
    }
    fn observe(&self, event: &'static str) {
        self.0.lock().unwrap().events.push(Event::Nvme(event));
    }
}

struct Domain {
    original: Arc<dyn DmaDomain>,
    state: Arc<Mutex<State>>,
    epoch: u64,
}
impl Domain {
    fn current(&self) -> Result<(), DmaError> {
        if self.state.lock().unwrap().epoch != self.epoch {
            Err(DmaError::MappingFailed)
        } else {
            Ok(())
        }
    }
    fn contained(&self) -> Result<(), DmaError> {
        self.current()?;
        if self.state.lock().unwrap().any_master() {
            Err(DmaError::DrainUnavailable)
        } else {
            Ok(())
        }
    }
}
// SAFETY: the independent peer owns CPU pins/device snapshots; this additional
// observer checks the original configuration epoch and every stored BM.
// It grants no native requester/alias identity and never invents a drain ACK.
unsafe impl DmaDomain for Domain {
    unsafe fn map(
        &self,
        pages: &DmaPages,
        direction: Direction,
        limit: u64,
    ) -> Result<DomainMapping, DmaError> {
        self.current()?;
        let physical = pages.physical_pages();
        if physical.len() > 1 {
            assert!(physical.windows(2).all(|pair| pair[1] != pair[0] + 4096));
        }
        // SAFETY: forwarding the model pins and request.
        let mapping = unsafe { self.original.map(pages, direction, limit) }?;
        assert!(physical.iter().all(|page| *page != mapping.address));
        Ok(mapping)
    }
    unsafe fn unmap(&self, mapping: DomainMapping) -> Result<(), DmaError> {
        self.contained()?;
        // SAFETY: original peer independently checks stop, drain and this token.
        unsafe { self.original.unmap(mapping) }
    }
    unsafe fn sync_for_device(
        &self,
        mapping: DomainMapping,
        range: Range<usize>,
        direction: Direction,
    ) -> Result<(), DmaError> {
        self.current()?;
        // SAFETY: exact peer token/range; peer owns separate device visibility.
        unsafe { self.original.sync_for_device(mapping, range, direction) }
    }
    unsafe fn sync_for_cpu(
        &self,
        mapping: DomainMapping,
        range: Range<usize>,
        direction: Direction,
    ) -> Result<(), DmaError> {
        self.current()?;
        // SAFETY: original peer refuses outstanding device ownership.
        unsafe { self.original.sync_for_cpu(mapping, range, direction) }
    }
    unsafe fn read_coherent_u32(
        &self,
        mapping: DomainMapping,
        offset: usize,
        direction: Direction,
    ) -> Result<u32, DmaError> {
        self.current()?;
        // SAFETY: original peer performs the isolated coherent read.
        unsafe { self.original.read_coherent_u32(mapping, offset, direction) }
    }
    unsafe fn drain_posted_writes(&self) -> Result<(), DmaError> {
        self.contained()?;
        // SAFETY: independently observed stored BM-clear; peer also requires
        // CC.EN0, no pending command and its own successful drain.
        unsafe { self.original.drain_posted_writes() }
    }
}

/// A one-controller attachment, as the single-controller tests see it.
type Single = Result<(Attached, RetiredHierarchy<Model>), (BlockError, RetiredHierarchy<Model>)>;

struct Fixture {
    host: Option<Host<Model>>,
    bar: Option<FirmwareBar>,
    state: Arc<Mutex<State>>,
    drops: Arc<AtomicUsize>,
    protocol: Protocol,
    domain: Option<Arc<dyn DmaDomain>>,
}
impl Fixture {
    fn new() -> Self {
        let model = Model::default();
        let state = model.state.clone();
        let drops = model.drops.clone();
        {
            let mut state = state.lock().unwrap();
            for base in PATH {
                // Preserve the PCIe capability and add linked PM v3.
                state.set(base + 0x40, 2, 0x8010);
                state.set(base + 0x80, 2, 1);
                state.set(base + 0x82, 2, 3);
            }
            state.add_function(SIBLING, 0x0200_0000, 0, 0);
        }
        let (host, bar) = fixture(model);
        let protocol = Protocol::new(Arc::new(Configuration(state.clone())));
        let domain: Arc<dyn DmaDomain> = Arc::new(Domain {
            original: protocol.domain(),
            state: state.clone(),
            epoch: 1,
        });
        Self {
            host: Some(host),
            bar: Some(bar),
            state,
            drops,
            protocol,
            domain: Some(domain),
        }
    }
    fn attach(&mut self) -> Single {
        self.attach_with_clock(model_clock())
    }
    fn attach_with_clock(&mut self, clock: Clock) -> Single {
        let protocol = &self.protocol;
        // SAFETY: this isolated fixture keeps one configuration model,
        // its selected independent register peer and original model domain.
        // The private sequence is exactly the public native sequence; no fake
        // native MMIO permission or physical requester claim is supplied.
        let attachment = unsafe {
            attach_with(
                self.host.take().unwrap(),
                vec![Controller {
                    bar: self.bar.take().unwrap(),
                    label: "composed VMD behind root and bridge".into(),
                    quirks: crate::nvme::Quirks::NONE,
                }],
                self.domain.as_ref().unwrap().clone(),
                self.protocol.pages(),
                clock,
                model_timer(),
                (|_: &FirmwareBar| Ok(protocol.registers()), protocol::attach),
            )
        };
        // One controller: its own result is the attachment's.
        match attachment {
            Ok((mut controllers, returned)) => {
                assert_eq!(controllers.len(), 1);
                match controllers.pop().unwrap().1 {
                    Ok(attached) => Ok((attached, returned)),
                    Err(error) => Err((error, returned)),
                }
            }
            Err(refused) => Err(refused),
        }
    }
    fn attached(&mut self) -> (Attached, RetiredHierarchy<Model>) {
        match self.attach() {
            Ok(value) => value,
            Err((error, _)) => panic!("composed attach: {error:?}"),
        }
    }
    fn assert_retained(&self, returned: &RetiredHierarchy<Model>, maps: usize) {
        let snapshot = self.protocol.snapshot();
        assert!(!returned.is_ready());
        assert_eq!(self.drops.load(Ordering::Relaxed), 0);
        assert_eq!(snapshot.mappings.len(), maps);
        assert_eq!(snapshot.register_owner_drops, 0);
        assert!(Arc::strong_count(self.domain.as_ref().unwrap()) > 1);
    }
    fn take_stopped_model(&mut self, returned: &mut RetiredHierarchy<Model>) -> Model {
        assert!(returned.is_ready());
        assert!(!self.state.lock().unwrap().any_master());
        assert!(self.protocol.snapshot().mappings.is_empty());
        // All namespace/controller handles must already have been dropped.
        // Dropping the final *model* domain owner retires this isolated binding;
        // no native DmaDomain detach operation is implied or invented.
        assert_eq!(Arc::strong_count(self.domain.as_ref().unwrap()), 1);
        drop(self.domain.take());
        self.state.lock().unwrap().epoch += 1;
        // SAFETY: isolated binding retired, no stale namespace/controller, no
        // model DMA/MMIO consumer; original stopped model is now recoverable.
        unsafe { returned.take_after_retirement() }.unwrap()
    }
}

#[test]
fn composed_bridge_nvme_identify_prp_read_write_flush_and_original_hierarchy_return() {
    let mut f = Fixture::new();
    // Exercise bridge and endpoint D3hot->D0 posted PMCSR transitions.
    for base in PATH {
        f.state.lock().unwrap().set(base + 0x84, 2, 0x8103);
    }
    let (mut attached, mut returned) = f.attached();
    assert!(attached.failures.is_empty());
    assert_eq!(attached.namespaces.len(), 1);
    let disk = &mut attached.namespaces[0];
    assert_eq!(
        (disk.identity().block_size, disk.identity().blocks),
        (512, 1024)
    );
    let pattern: Vec<u8> = (0..128 * 1024).map(|n| (n * 17) as u8).collect();
    disk.write(8, &pattern).unwrap();
    let mut actual = vec![0xa5; pattern.len()];
    disk.read(8, &mut actual).unwrap();
    assert_eq!(actual, pattern);
    let replacement: Vec<u8> = pattern.iter().map(|byte| byte ^ 0xa5).collect();
    disk.write(8, &replacement).unwrap();
    disk.read(8, &mut actual).unwrap();
    assert_eq!(actual, replacement);
    for _ in 0..35 {
        disk.flush().unwrap();
    }
    assert_eq!(
        // SAFETY: intentional negative model probe. The original domain observes enabled BM
        // bits and must refuse before any drain acknowledgement.
        unsafe { f.domain.as_ref().unwrap().drain_posted_writes() },
        Err(DmaError::DrainUnavailable)
    );
    assert!(!f.protocol.snapshot().events.contains(&"drain"));
    let mut stale = Protocol::sibling(disk);
    assert!(!returned.is_ready());
    disk.retire_owned().unwrap();
    let before = f.protocol.snapshot();
    assert!(stale.flush().is_err());
    assert!(disk.read(0, &mut [0; 512]).is_err());
    let after = f.protocol.snapshot();
    assert_eq!(
        (before.commands, before.mmio_accesses),
        (after.commands, after.mmio_accesses)
    );
    assert_eq!(after.pages_released, BUFFERS);
    assert!(after.retired && after.drained && !after.enabled && after.mappings.is_empty());
    assert!(returned.is_ready());
    // Ready is only publication; stale handles still keep the original domain.
    assert!(Arc::strong_count(f.domain.as_ref().unwrap()) > 1);
    {
        let state = f.state.lock().unwrap();
        let events = &state.events;
        // Original PMCSR W1C status survives every path member's wake.
        for base in PATH {
            assert_eq!(state.child(base + 0x84, 2) & 0x8000, 0x8000);
        }
        let last = |wanted: Event| events.iter().rposition(|event| *event == wanted).unwrap();
        let stop = last(Event::Nvme("stop"));
        let parent_bm_clear = events
            .iter()
            .enumerate()
            .filter_map(|(i, event)| match event {
                Event::Parent(value) if value & regs::COMMAND_MASTER == 0 => Some(i),
                _ => None,
            })
            .find(|i| *i > stop)
            .unwrap();
        let drain = last(Event::Nvme("drain"));
        let unmap = last(Event::Nvme("unmap"));
        let registers = last(Event::Nvme("retire_regs"));
        let release = events
            .iter()
            .rposition(
                |event| matches!(event, Event::Parent(value) if value & regs::COMMAND_MEMORY == 0),
            )
            .unwrap();
        assert!(
            stop < parent_bm_clear
                && parent_bm_clear < drain
                && drain < unmap
                && unmap < registers
                && registers < release
        );
        assert_eq!(state.child(SIBLING + 4, 2) & 7, 0);
    }
    drop(stale);
    drop(attached);
    let original = f.take_stopped_model(&mut returned);
    assert!(Arc::ptr_eq(&original.state, &f.state));
    drop(original);
    assert_eq!(f.drops.load(Ordering::Relaxed), 1);
}

#[test]
fn unavailable_native_clock_refuses_before_discovery_mapping_or_queue_effects() {
    for rate in [0, u64::MAX, u64::MAX / 2 / 127_500_000 + 1] {
        let mut fixture = Fixture::new();
        let events = fixture.state.lock().unwrap().events.clone();
        let (error, returned) = fixture
            .attach_with_clock(Clock {
                ticks_per_microsecond: rate,
            })
            .err()
            .unwrap();
        assert_eq!(
            error,
            BlockError::Unsupported("NVMe requires a calibrated, representable operation clock")
        );
        assert_eq!(fixture.state.lock().unwrap().events, events);
        let snapshot = fixture.protocol.snapshot();
        assert_eq!(snapshot.commands, 0);
        assert_eq!(snapshot.mmio_accesses, 0);
        assert!(snapshot.mappings.is_empty());
        assert!(!returned.is_ready());
        assert_eq!(fixture.drops.load(Ordering::Relaxed), 0);
        assert!(Arc::strong_count(fixture.domain.as_ref().unwrap()) > 1);
    }
}

#[test]
fn composed_failed_d0_and_posted_bme_readback_never_issue_queue_dma() {
    for failed_power_at in [Some(ROOT), Some(BRIDGE), Some(NVME), None] {
        let mut f = Fixture::new();
        {
            let mut state = f.state.lock().unwrap();
            if let Some(base) = failed_power_at {
                state.set(base + 0x84, 2, 0x8103);
                state.pm_failure = true;
            } else {
                state.ack_failure = Some(BRIDGE + 4);
            }
        }
        let returned = match f.attach() {
            Err((_, returned)) => returned,
            Ok(_) => panic!("failed config ACK accepted"),
        };
        let snapshot = f.protocol.snapshot();
        assert_eq!(snapshot.commands, 0);
        assert!(!snapshot.enabled && !snapshot.retired);
        assert_eq!(snapshot.unmap_attempts, 0);
        assert_eq!(snapshot.pages_released, 0);
        if failed_power_at.is_some() {
            assert_eq!(snapshot.mmio_accesses, 0);
            assert!(!f.state.lock().unwrap().any_master());
        } else {
            // The posted BME write really happened; only its equality readback
            // was corrupted. No CC.EN or queue doorbell may follow that failure.
            assert_ne!(
                f.state.lock().unwrap().child(BRIDGE + 4, 2) & u32::from(regs::COMMAND_MASTER),
                0
            );
        }
        f.assert_retained(&returned, BUFFERS);
    }
}

/// Completions reporting another submission queue or a moved head complete
/// their commands: Linux's `nvme_handle_cqe` reads neither field.
#[test]
fn composed_completion_queue_identifier_and_head_are_not_read() {
    for fault in [Fault::Sqid, Fault::Head] {
        let mut f = Fixture::new();
        let (mut attached, mut returned) = f.attached();
        let disk = &mut attached.namespaces[0];
        f.protocol.fault(fault);
        disk.write(0, &[0x3c; 512]).unwrap();
        let mut output = [0; 512];
        disk.read(0, &mut output).unwrap();
        assert_eq!(output, [0x3c; 512]);
        f.protocol.fault(Fault::None);
        disk.retire_owned().unwrap();
        drop(attached);
        drop(f.take_stopped_model(&mut returned));
        assert_eq!(f.drops.load(Ordering::Relaxed), 1);
    }
}

#[test]
fn composed_bad_cq_and_cache_failures_fence_all_stale_siblings() {
    for fault in [Fault::DeviceVisibility, Fault::CpuVisibility] {
        let mut f = Fixture::new();
        let (mut attached, mut returned) = f.attached();
        let disk = &mut attached.namespaces[0];
        let mut sibling = Protocol::sibling(disk);
        let before = f.protocol.snapshot();
        f.protocol.fault(fault);
        let mut output = [0xa5; 512];
        assert!(disk.read(0, &mut output).is_err());
        assert_eq!(output, [0xa5; 512]);
        let failed = f.protocol.snapshot();
        assert_eq!(failed.sync_cpu, before.sync_cpu);
        f.assert_retained(&returned, BUFFERS);
        assert!(sibling.flush().is_err());
        assert!(disk.write(0, &[1; 512]).is_err());
        assert!(sibling.read(0, &mut output).is_err());
        assert_eq!(output, [0xa5; 512]);
        assert_eq!(f.protocol.snapshot().commands, failed.commands);
        f.protocol.fault(Fault::None);
        disk.retire_owned().unwrap();
        drop(sibling);
        drop(attached);
        drop(f.take_stopped_model(&mut returned));
        assert_eq!(f.drops.load(Ordering::Relaxed), 1);
    }
}

#[test]
fn composed_timeouts_reset_the_controller_with_dma_contained_like_linux() {
    for fault in [Fault::Phase, Fault::Cid, Fault::Delayed] {
        let mut f = Fixture::new();
        let (mut attached, mut returned) = f.attached();
        let disk = &mut attached.namespaces[0];
        let mut sibling = Protocol::sibling(disk);
        let before = f.protocol.snapshot();
        f.protocol.fault(fault);
        let mut output = [0xa5; 512];
        let clock = model_clock();
        let started = clock.try_start().unwrap();
        assert!(disk.read(0, &mut output).is_err());
        assert!(clock.try_elapsed_us(started).unwrap() >= 1_000);
        assert_eq!(output, [0xa5; 512]);
        // nvme_timeout aborts, then nvme_dev_disable stops the controller
        // and its DMA before the CPU reads any shared memory.
        let events = f.protocol.snapshot().events;
        let events = &events[before.events.len()..];
        let drained = events.iter().position(|event| *event == "drain").unwrap();
        assert!(!events[..drained].contains(&"cpu"));
        f.protocol.fault(Fault::None);
        if matches!(fault, Fault::Phase | Fault::Cid) {
            // No completion was ever visible or named a command, so the
            // reset's own commands failed and the controller was removed as
            // dead.
            assert!(sibling.flush().is_err());
            assert!(disk.write(0, &[1; 512]).is_err());
        } else {
            // Every reset succeeded; only the held I/O kept timing out.
            disk.read(0, &mut output).unwrap();
            sibling.flush().unwrap();
        }
        disk.retire_owned().unwrap();
        drop(sibling);
        drop(attached);
        drop(f.take_stopped_model(&mut returned));
        assert_eq!(f.drops.load(Ordering::Relaxed), 1);
    }
}

#[test]
fn composed_domain_epoch_drain_unmap_and_register_failures_preserve_original_owners() {
    for stage in 0..4 {
        let mut f = Fixture::new();
        let (mut attached, mut returned) = f.attached();
        let ids = f.protocol.snapshot().mappings;
        match stage {
            0 => f.state.lock().unwrap().epoch = 2,
            1 => f.protocol.fault(Fault::Drain),
            2 => f.protocol.fault(Fault::Unmap),
            _ => f.protocol.fault(Fault::RegistersRetire),
        }
        assert!(attached.namespaces[0].retire_owned().is_err());
        let failed = f.protocol.snapshot();
        f.assert_retained(&returned, if stage == 3 { 0 } else { BUFFERS });
        assert_eq!(failed.pages_released, if stage == 3 { BUFFERS } else { 0 });
        assert_eq!(failed.mappings, if stage == 3 { Vec::new() } else { ids });
        assert!(!failed.retired);
        f.state.lock().unwrap().epoch = 1;
        f.protocol.fault(Fault::None);
        attached.namespaces[0].retire_owned().unwrap();
        let succeeded = f.protocol.snapshot();
        assert_eq!(succeeded.mmio_accesses, failed.mmio_accesses);
        assert_eq!(
            succeeded
                .events
                .iter()
                .filter(|event| **event == "stop")
                .count(),
            failed
                .events
                .iter()
                .filter(|event| **event == "stop")
                .count()
        );
        drop(attached);
        drop(f.take_stopped_model(&mut returned));
        assert_eq!(f.drops.load(Ordering::Relaxed), 1);
    }
}

#[test]
fn composed_final_host_publication_failure_cannot_reuse_retired_registers_or_hierarchy() {
    let mut f = Fixture::new();
    let (mut attached, returned) = f.attached();
    f.state.lock().unwrap().release_failure = true;
    assert!(attached.namespaces[0].retire_owned().is_err());
    let failed = f.protocol.snapshot();
    assert!(failed.retired && failed.drained && failed.mappings.is_empty());
    assert_eq!(failed.pages_released, BUFFERS);
    f.assert_retained(&returned, 0);
    f.state.lock().unwrap().release_failure = false;
    assert!(attached.namespaces[0].retire_owned().is_err());
    let retried = f.protocol.snapshot();
    assert_eq!(
        (retried.mmio_accesses, retried.unmap_attempts),
        (failed.mmio_accesses, failed.unmap_attempts)
    );
    let mut sibling = Protocol::sibling(&attached.namespaces[0]);
    assert!(sibling.flush().is_err());
    drop(sibling);
    drop(attached);
    f.assert_retained(&returned, 0);
    assert_eq!(f.protocol.snapshot().register_owner_drops, 0);
}

#[test]
fn every_hierarchy_refusal_names_its_own_cause() {
    let errors = [
        Error::UnsupportedDevice,
        Error::InvalidResource,
        Error::Mapping,
        Error::ReadFailed,
        Error::Readback,
        Error::Address,
        Error::Capability,
        Error::Exhausted,
        Error::Suspended,
        Error::Quarantined,
        Error::ActiveChild,
        Error::UnstoppedChild(1, 2, 3),
    ];
    let mut messages: Vec<_> = errors
        .into_iter()
        .map(|error| match block(error) {
            BlockError::Protocol(message) | BlockError::Unsupported(message) => message,
            other => panic!("{error:?} became {other:?}"),
        })
        .collect();
    messages.sort_unstable();
    messages.dedup();
    assert_eq!(messages.len(), errors.len());
}

/// One of two controllers' view of the shared configuration model: its own
/// path, the root port and bridge both share, and its own endpoint.
struct Controller2 {
    state: Arc<Mutex<State>>,
    endpoint: usize,
    index: usize,
    stopping: Arc<AtomicUsize>,
}
impl protocol::Access for Controller2 {
    fn registers_available(&self) -> bool {
        let state = self.state.lock().unwrap();
        state.parent(regs::COMMAND, 2) & u32::from(regs::COMMAND_MEMORY) != 0
            && [ROOT, BRIDGE, self.endpoint].into_iter().all(|base| {
                state.child(base + COMMAND_OFFSET, 2) & u32::from(regs::COMMAND_MEMORY) != 0
                    && state.child(base + 0x84, 2) & 3 == 0
            })
    }
    fn dma_enabled(&self) -> bool {
        let state = self.state.lock().unwrap();
        state.parent(regs::COMMAND, 2) & u32::from(regs::COMMAND_MASTER) != 0
            && [ROOT, BRIDGE, self.endpoint].into_iter().all(|base| {
                state.child(base + COMMAND_OFFSET, 2) & u32::from(regs::COMMAND_MASTER) != 0
            })
    }
    /// This controller's own requests: the bridges above may master for
    /// the other one.
    fn any_bus_master(&self) -> bool {
        self.state
            .lock()
            .unwrap()
            .child(self.endpoint + COMMAND_OFFSET, 2)
            & u32::from(regs::COMMAND_MASTER)
            != 0
    }
    fn observe(&self, event: &'static str) {
        if event == "stop" {
            self.stopping.store(self.index, Ordering::Relaxed);
        }
        self.state.lock().unwrap().events.push(Event::Nvme(event));
    }
}

/// The one domain both controllers' requests reach the root through, under
/// the VMD parent's requester ID, over each controller's independent peer:
/// a mapping goes to the controller being attached, and every later use of
/// it to the peer that made it.
struct Shared2 {
    peers: [Arc<dyn DmaDomain>; 2],
    attaching: Arc<AtomicUsize>,
    stopping: Arc<AtomicUsize>,
    next: AtomicU64,
    mappings: Mutex<BTreeMap<u64, (usize, DomainMapping)>>,
}
impl Shared2 {
    fn peer(&self, mapping: DomainMapping) -> (usize, DomainMapping) {
        let (peer, original) = self.mappings.lock().unwrap()[&mapping.id];
        assert_eq!(original.bytes, mapping.bytes);
        (peer, original)
    }
}
// SAFETY: forwards each operation unchanged to the independent peer that
// owns the mapping; no drain acknowledgement is invented.
unsafe impl DmaDomain for Shared2 {
    unsafe fn map(
        &self,
        pages: &DmaPages,
        direction: Direction,
        limit: u64,
    ) -> Result<DomainMapping, DmaError> {
        let peer = self.attaching.load(Ordering::Relaxed);
        // SAFETY: forwarding the model pins and request.
        let original = unsafe { self.peers[peer].map(pages, direction, limit) }?;
        let id = self.next.fetch_add(1, Ordering::Relaxed);
        self.mappings.lock().unwrap().insert(id, (peer, original));
        Ok(DomainMapping { id, ..original })
    }
    unsafe fn unmap(&self, mapping: DomainMapping) -> Result<(), DmaError> {
        let (peer, original) = self.peer(mapping);
        // SAFETY: the owning peer checks stop, drain and its own token.
        unsafe { self.peers[peer].unmap(original) }?;
        self.mappings.lock().unwrap().remove(&mapping.id);
        Ok(())
    }
    unsafe fn sync_for_device(
        &self,
        mapping: DomainMapping,
        range: Range<usize>,
        direction: Direction,
    ) -> Result<(), DmaError> {
        let (peer, original) = self.peer(mapping);
        // SAFETY: the owning peer's exact token and range.
        unsafe { self.peers[peer].sync_for_device(original, range, direction) }
    }
    unsafe fn sync_for_cpu(
        &self,
        mapping: DomainMapping,
        range: Range<usize>,
        direction: Direction,
    ) -> Result<(), DmaError> {
        let (peer, original) = self.peer(mapping);
        // SAFETY: the owning peer refuses outstanding device ownership.
        unsafe { self.peers[peer].sync_for_cpu(original, range, direction) }
    }
    unsafe fn read_coherent_u32(
        &self,
        mapping: DomainMapping,
        offset: usize,
        direction: Direction,
    ) -> Result<u32, DmaError> {
        let (peer, original) = self.peer(mapping);
        // SAFETY: the owning peer performs the isolated coherent read.
        unsafe { self.peers[peer].read_coherent_u32(original, offset, direction) }
    }
    unsafe fn drain_posted_writes(&self) -> Result<(), DmaError> {
        // The controller being torn down, which its peer checks stopped.
        let peer = self.stopping.load(Ordering::Relaxed);
        // SAFETY: that peer requires CC.EN0, its endpoint's BM clear and no
        // pending command before its own drain.
        unsafe { self.peers[peer].drain_posted_writes() }
    }
}

/// Where the chosen disk's identity sits on its medium in this model: a
/// GUID partition table's disk GUID at LBA 1.
const DISK_GUID_OFFSET: usize = 512 + 56;

#[test]
fn two_controllers_behind_one_vmd_attach_and_the_chosen_disk_on_the_second_is_found() {
    let model = Model::default();
    let state = model.state.clone();
    let drops = model.drops.clone();
    {
        let mut state = state.lock().unwrap();
        for base in PATH {
            state.set(base + 0x40, 2, 0x8010);
            state.set(base + 0x80, 2, 1);
            state.set(base + 0x82, 2, 3);
        }
        state.second_nvme();
    }
    let (host, bar) = fixture(model);
    let second = second_bar(&host);
    let attaching = Arc::new(AtomicUsize::new(0));
    let stopping = Arc::new(AtomicUsize::new(0));
    let protocols: Vec<Protocol> = [NVME, NVME2]
        .into_iter()
        .enumerate()
        .map(|(index, endpoint)| {
            Protocol::new(Arc::new(Controller2 {
                state: state.clone(),
                endpoint,
                index,
                stopping: stopping.clone(),
            }))
        })
        .collect();
    // The chosen disk, by the identity the stick's configuration names, is
    // the second controller's.
    let chosen = *b"chosen-disk-guid";
    protocols[0].medium([0x11; 8], DISK_GUID_OFFSET, b"another-disk-gui");
    protocols[1].medium([0x22; 8], DISK_GUID_OFFSET, &chosen);
    let domain: Arc<dyn DmaDomain> = Arc::new(Shared2 {
        peers: [protocols[0].domain(), protocols[1].domain()],
        attaching: attaching.clone(),
        stopping: stopping.clone(),
        next: AtomicU64::new(1),
        mappings: Mutex::new(BTreeMap::new()),
    });
    let controllers = vec![
        Controller {
            bar,
            label: "VMD 02:00.0".into(),
            quirks: crate::nvme::Quirks::NONE,
        },
        Controller {
            bar: second,
            label: "VMD 02:01.0".into(),
            quirks: crate::nvme::Quirks::NONE,
        },
    ];
    let registers = |bar: &FirmwareBar| {
        let index = usize::from(bar.child().device);
        attaching.store(index, Ordering::Relaxed);
        Ok(protocols[index].registers())
    };
    // SAFETY: one configuration model, each controller's own independent
    // register peer and the one model domain both forward through.
    let attachment = unsafe {
        attach_with(
            host,
            controllers,
            domain.clone(),
            protocols[0].pages(),
            model_clock(),
            model_timer(),
            (registers, protocol::attach),
        )
    };
    let Ok((controllers, mut returned)) = attachment else {
        panic!("two-controller attach refused");
    };
    let mut disks: Vec<(String, Attached)> = controllers
        .into_iter()
        .map(|(label, result)| (label, result.expect("controller attached")))
        .collect();
    assert_eq!(disks.len(), 2);
    // Both controllers run: their endpoints and the path above master.
    for base in [ROOT, BRIDGE, NVME, NVME2] {
        assert_ne!(
            state.lock().unwrap().child(base + COMMAND_OFFSET, 2) & u32::from(regs::COMMAND_MASTER),
            0
        );
    }
    // Find the chosen disk by its identity, as the boot does among every
    // disk it attached.
    let mut found = Vec::new();
    for (label, attached) in &mut disks {
        for disk in &mut attached.namespaces {
            let mut sector = [0; 512];
            disk.read(1, &mut sector).unwrap();
            if sector[56..72] == chosen {
                found.push(label.clone());
            }
        }
    }
    assert_eq!(found, ["VMD 02:01.0"]);
    // The chosen disk takes writes while the other controller runs too.
    let chosen_disk = &mut disks[1].1.namespaces[0];
    chosen_disk.write(8, &[0x5a; 4096]).unwrap();
    let mut back = [0; 4096];
    chosen_disk.read(8, &mut back).unwrap();
    assert_eq!(back, [0x5a; 4096]);
    // The first controller retires while the second runs; the hierarchy
    // returns only after the second retired too.
    disks[0].1.namespaces[0].retire_owned().unwrap();
    assert!(!returned.is_ready());
    assert_eq!(
        state.lock().unwrap().child(NVME + COMMAND_OFFSET, 2)
            & u32::from(regs::COMMAND_MASTER | regs::COMMAND_MEMORY),
        0
    );
    chosen_disk_still_runs(&mut disks[1].1);
    disks[1].1.namespaces[0].retire_owned().unwrap();
    assert!(returned.is_ready());
    assert!(!state.lock().unwrap().any_master());
    drop(disks);
    drop(domain);
    // SAFETY: every controller retired, its mappings unmapped and its
    // handles dropped; the isolated model domain is gone.
    let original = unsafe { returned.take_after_retirement() }.unwrap();
    drop(original);
    assert_eq!(drops.load(Ordering::Relaxed), 1);
}

fn chosen_disk_still_runs(attached: &mut Attached) {
    let mut back = [0; 4096];
    attached.namespaces[0].read(8, &mut back).unwrap();
    assert_eq!(back, [0x5a; 4096]);
}
