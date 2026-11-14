// SPDX-License-Identifier: GPL-2.0-only
//! Endpoint hierarchy and lifecycle against the independent posted-config and
//! payload-domain models.
use super::test_support::*;
use super::*;
use crate::bus::{
    dma::{
        mapping::{Direction, DmaError, DmaMapping, DomainMapping},
        memory::{PageBacking, PinnedPages},
        pages::DmaPages,
    },
    mmio::owned::{
        AddressSpace, InstalledMapping, MapFailure, MapRequest, MappingPlatform, PhysicalSpan,
        Resource, ResourceBacking, ResourceKind,
        model::{MappingOwner, MappingRetention},
    },
    pci::regs,
    vmd::{VMCAP, VMCONFIG},
};
use alloc::boxed::Box;
use core::{
    cell::UnsafeCell,
    ops::Range,
    ptr::NonNull,
    sync::atomic::{AtomicBool, AtomicUsize, Ordering},
};
extern crate std;
use std::sync::Mutex;

/// The NVMe class code the model's endpoint carries.
const NVME_CLASS: [u8; 3] = [0x01, 0x08, 0x02];

fn lifecycle(model: Model) -> (Endpoint<Model>, Arc<Domain>) {
    let state = Arc::clone(&model.state);
    let (host, bar) = fixture(model);
    let domain = Arc::new(Domain { state, epoch: 1 });
    let (mut endpoint, _) = Endpoint::new(host, domain.clone(), model_timer());
    endpoint.discover(&bar, NVME_CLASS).unwrap();
    (endpoint, domain)
}
struct Domain {
    state: Arc<Mutex<State>>,
    epoch: u64,
}
// SAFETY: isolated model tracks its page pins/IOVA epochs and
// raw config storage. It cannot acknowledge drain while any BM is set.
unsafe impl DmaDomain for Domain {
    unsafe fn map(
        &self,
        pages: &DmaPages,
        _: Direction,
        _: u64,
    ) -> Result<DomainMapping, DmaError> {
        let mut state = self.state.lock().unwrap();
        if state.epoch != self.epoch {
            return Err(DmaError::MappingFailed);
        }
        state.next += 1;
        let mapping = DomainMapping {
            id: state.next,
            address: 0x2000_0000 + state.next * 0x10000,
            bytes: pages.bytes(),
        };
        state.mappings.insert(mapping.id, mapping);
        state.events.push(Event::Map(mapping.id));
        Ok(mapping)
    }
    unsafe fn unmap(&self, mapping: DomainMapping) -> Result<(), DmaError> {
        let mut state = self.state.lock().unwrap();
        if state.epoch != self.epoch
            || state.any_master()
            || !state.drained
            || !state.mappings.contains_key(&mapping.id)
        {
            return Err(DmaError::InvalidationFailed);
        }
        state.mappings.remove(&mapping.id);
        state.events.push(Event::Unmap(mapping.id));
        Ok(())
    }
    unsafe fn sync_for_device(
        &self,
        _: DomainMapping,
        _: Range<usize>,
        _: Direction,
    ) -> Result<(), DmaError> {
        Ok(())
    }
    unsafe fn sync_for_cpu(
        &self,
        _: DomainMapping,
        _: Range<usize>,
        _: Direction,
    ) -> Result<(), DmaError> {
        Ok(())
    }
    unsafe fn drain_posted_writes(&self) -> Result<(), DmaError> {
        let mut state = self.state.lock().unwrap();
        if state.epoch != self.epoch || state.any_master() {
            return Err(DmaError::DrainUnavailable);
        }
        state.drained = true;
        state.events.push(Event::Drain);
        Ok(())
    }
}
#[repr(align(4096))]
struct Buffer([u8; 4096]);
struct Pages {
    buffer: Box<Buffer>,
    drops: Arc<AtomicUsize>,
}
// SAFETY: one owned, aligned, initially zeroed model page with immutable labels.
unsafe impl PageBacking for Pages {
    fn cpu_base(&self) -> NonNull<u8> {
        NonNull::from(&self.buffer.0[0])
    }
    fn physical_pages(&self) -> &[u64] {
        &[0x1000_0000]
    }
    fn page_shift(&self) -> u8 {
        12
    }
    fn cache_attribute(&self) -> CacheAttribute {
        CacheAttribute::WriteBack
    }
}
impl Drop for Pages {
    fn drop(&mut self) {
        self.drops.fetch_add(1, Ordering::Relaxed);
    }
}
fn pin(drops: Arc<AtomicUsize>) -> PinnedPages {
    PinnedPages::new(
        Box::new(Pages {
            buffer: Box::new(Buffer([0; 4096])),
            drops,
        }),
        4096,
        4096,
    )
    .unwrap()
}

#[test]
fn posted_commands_complete_before_payload_unmap() {
    let model = Model::default();
    let state = model.state.clone();
    let drops = model.drops.clone();
    let (mut life, domain) = lifecycle(model);
    let mut returned = RetiredHierarchy(Rc::clone(&life.returned));
    assert!(!returned.is_ready());
    let page_drops = Arc::new(AtomicUsize::new(0));
    let mut mapping = DmaMapping::map(
        domain.clone(),
        pin(page_drops.clone()),
        Direction::Bidirectional,
        u64::MAX,
    )
    .unwrap();
    life.prepare().unwrap();
    assert!(!state.lock().unwrap().any_master());
    life.enable_dma().unwrap();
    assert_eq!(
        // SAFETY: intentional negative probe against the isolated provider; it refuses because the
        // configuration still permits new requests.
        unsafe { domain.drain_posted_writes() },
        Err(DmaError::DrainUnavailable)
    );
    life.disable_dma().unwrap();
    assert_eq!(
        // SAFETY: the model verifies missing-domain-drain refusal.
        unsafe { mapping.unmap() },
        Err(DmaError::InvalidationFailed)
    );
    // SAFETY: all model BM bits have now acknowledged clear.
    unsafe { domain.drain_posted_writes() }.unwrap();
    // SAFETY: this test issues no asynchronous payload accesses; original
    // provider drain completed, while the original host is still held.
    unsafe { mapping.unmap() }.unwrap();
    assert_eq!(page_drops.load(Ordering::Relaxed), 1);
    assert!(!returned.is_ready());
    // SAFETY: the model endpoint is stopped and drained, with no mappings left.
    unsafe { life.release() }.unwrap();
    assert_eq!(drops.load(Ordering::Relaxed), 0);
    assert!(returned.is_ready());
    // SAFETY: this isolated model has no controller handles, real bindings or
    // native reset/removal actors. All model mappings were explicitly unmapped.
    let original = unsafe { returned.take_after_retirement() }.unwrap();
    assert!(!state.lock().unwrap().any_master());
    drop(original);
    assert_eq!(drops.load(Ordering::Relaxed), 1);
    let state = state.lock().unwrap();
    let enabled: Vec<_> = state
        .events
        .iter()
        .filter_map(|event| match event {
            Event::Parent(command) if command & regs::COMMAND_MASTER != 0 => Some(usize::MAX),
            Event::Posted(offset, 2, value) if value & u32::from(regs::COMMAND_MASTER) != 0 => {
                Some(*offset)
            }
            _ => None,
        })
        .collect();
    assert_eq!(enabled, [usize::MAX, ROOT + 4, BRIDGE + 4, NVME + 4]);
    for (index, event) in state.events.iter().enumerate() {
        if let Event::Posted(offset, width, _) = event {
            assert_eq!(state.events[index + 1], Event::Flushed(*offset, *width));
        }
    }
    let drain = state
        .events
        .iter()
        .position(|event| *event == Event::Drain)
        .unwrap();
    let unmap = state
        .events
        .iter()
        .position(|event| matches!(event, Event::Unmap(_)))
        .unwrap();
    let release = state
        .events
        .iter()
        .rposition(
            |event| matches!(event, Event::Parent(command) if command & regs::COMMAND_MEMORY == 0),
        )
        .unwrap();
    assert!(drain < unmap && unmap < release);
}

#[test]
fn abandoned_retired_handle_preserves_the_whole_original_hierarchy() {
    let model = Model::default();
    let drops = model.drops.clone();
    let (mut life, domain) = lifecycle(model);
    let returned = RetiredHierarchy(Rc::clone(&life.returned));
    life.prepare().unwrap();
    life.disable_dma().unwrap();
    // SAFETY: no model engine started; hierarchy BM-clear acknowledged.
    unsafe { domain.drain_posted_writes() }.unwrap();
    // SAFETY: the model endpoint is stopped and drained, with no mappings left.
    unsafe { life.release() }.unwrap();
    assert!(returned.is_ready());
    drop(life);
    drop(returned);
    assert_eq!(drops.load(Ordering::Relaxed), 0);
}

#[test]
fn every_ancestor_window_and_unique_firmware_bus_route_is_required() {
    for (base, offset, value) in [
        (ROOT, 0x20, 0x8010_8010),
        (BRIDGE, 0x20, 0x8010_8010),
        (ROOT, 0x18, 0x0001_0100),
        (BRIDGE, 0x18, 0x0002_0001),
    ] {
        let model = Model::default();
        model.state.lock().unwrap().set(base + offset, 4, value);
        let (mut host, bar) = fixture(model);
        assert!(Hierarchy::discover(&mut host, &bar, NVME_CLASS).is_err());
    }
    let model = Model::default();
    model
        .state
        .lock()
        .unwrap()
        .bridge(ROOT + (1 << 15), 0, 1, 2);
    let (mut host, bar) = fixture(model);
    assert!(Hierarchy::discover(&mut host, &bar, NVME_CLASS).is_err());
}

#[test]
fn stopped_siblings_and_a_second_nvme_remain_contained() {
    // Two NVMe controllers behind one VMD, as laptops with two SSD slots
    // have: the endpoint is the one its BAR was claimed from, and the other
    // stays a sibling without decode or bus mastering, as any other class.
    for extra_class in [0x0200_0000, 0x0108_0200] {
        let model = Model::default();
        let extra = NVME + (1 << 15);
        {
            let mut state = model.state.lock().unwrap();
            state.add_function(extra, extra_class, 0, 0);
            // Firmware left its memory decode on.
            state.set(
                extra + COMMAND_OFFSET,
                2,
                u32::from(regs::COMMAND_INTX_DISABLE | regs::COMMAND_MEMORY),
            );
        }
        let state = model.state.clone();
        let (mut host, bar) = fixture(model);
        let mut hierarchy = Hierarchy::discover(&mut host, &bar, NVME_CLASS).unwrap();
        hierarchy.prepare(&mut host, &*model_timer()).unwrap();
        hierarchy.enable(&mut host).unwrap();
        assert_eq!(state.lock().unwrap().child(extra + 4, 2) & 7, 0);
        assert_ne!(state.lock().unwrap().child(NVME + 4, 2) & 6, 0);
        hierarchy.disable(&mut host).unwrap();
        hierarchy.recheck(&mut host, false).unwrap();
    }
    // A BAR claimed from a function of another class is no endpoint.
    let model = Model::default();
    let (mut host, bar) = fixture(model);
    assert!(matches!(
        Hierarchy::discover(&mut host, &bar, [0x02, 0x00, 0x00]),
        Err(Error::UnsupportedDevice)
    ));
}

/// Give `base` an MSI capability after its PCIe one, enabled as firmware
/// may leave it, and an MSI-X capability after that, enabled and masked.
fn signalling(state: &mut State, base: usize) {
    state.set(base + 0x41, 1, 0x80);
    state.set(base + 0x80, 2, 0x9005);
    state.set(base + 0x82, 2, 1);
    state.set(base + 0x90, 2, 0x0011);
    state.set(base + 0x92, 2, 0xc000);
}

#[test]
fn a_child_firmware_left_mastering_is_stopped_with_the_parent_unmastered() {
    let model = Model::default();
    let state = model.state.clone();
    {
        let mut state = state.lock().unwrap();
        state.set(
            NVME + 4,
            2,
            u32::from(regs::COMMAND_MEMORY | regs::COMMAND_MASTER),
        );
        signalling(&mut state, NVME);
        state.set(BRIDGE + 4, 2, u32::from(regs::COMMAND_MASTER));
    }
    let (mut host, bar) = fixture(model);
    let mut hierarchy = Hierarchy::discover(&mut host, &bar, NVME_CLASS).unwrap();
    {
        let state = state.lock().unwrap();
        assert!(!state.any_master());
        for base in [BRIDGE, NVME] {
            assert_eq!(
                state.child(base + 4, 2) as u16
                    & (regs::COMMAND_MASTER | regs::COMMAND_INTX_DISABLE),
                regs::COMMAND_INTX_DISABLE
            );
        }
        // Decode stays as found until preparation.
        assert_ne!(state.child(NVME + 4, 2) as u16 & regs::COMMAND_MEMORY, 0);
        assert_eq!(state.child(NVME + 0x82, 2), 0);
        assert_eq!(state.child(NVME + 0x92, 2), 0x4000);
        // The parent's Bus Master was never set on the way.
        assert!(!state.events.iter().any(|event| matches!(
            event,
            Event::Parent(command) if command & regs::COMMAND_MASTER != 0
        )));
    }
    hierarchy.prepare(&mut host, &*model_timer()).unwrap();
    hierarchy.enable(&mut host).unwrap();
}

#[test]
fn a_child_that_keeps_mastering_or_signalling_is_refused_and_the_host_poisoned() {
    for (offset, bits) in [
        (NVME + 4, u32::from(regs::COMMAND_MASTER)),
        (NVME + 0x82, 1),
    ] {
        let model = Model::default();
        let state = model.state.clone();
        {
            let mut state = state.lock().unwrap();
            state.set(
                NVME + 4,
                2,
                u32::from(regs::COMMAND_INTX_DISABLE | regs::COMMAND_MASTER),
            );
            signalling(&mut state, NVME);
            state.stuck = Some((offset, bits));
        }
        let (mut host, bar) = fixture(model);
        assert_eq!(
            Hierarchy::discover(&mut host, &bar, NVME_CLASS).err(),
            Some(Error::UnstoppedChild(2, 0, 0))
        );
        assert_eq!(
            Hierarchy::discover(&mut host, &bar, NVME_CLASS).err(),
            Some(Error::Quarantined)
        );
        assert_eq!(
            state.lock().unwrap().parent(regs::COMMAND, 2) & u32::from(regs::COMMAND_MASTER),
            0
        );
    }
}

#[test]
fn failed_posted_bus_master_ack_keeps_original_hierarchy_and_domain() {
    let model = Model::default();
    let state = model.state.clone();
    let drops = model.drops.clone();
    let (mut life, domain) = lifecycle(model);
    life.prepare().unwrap();
    state.lock().unwrap().ack_failure = Some(BRIDGE + 4);
    assert!(life.enable_dma().is_err());
    assert_eq!(life.phase, Phase::Quarantined);
    // The posted write occurred despite the failed equality readback.
    assert_ne!(
        state.lock().unwrap().child(BRIDGE + 4, 2) & u32::from(regs::COMMAND_MASTER),
        0
    );
    assert_eq!(
        // SAFETY: negative model probe; original domain observes BM state.
        unsafe { domain.drain_posted_writes() },
        Err(DmaError::DrainUnavailable)
    );
    drop(life);
    assert_eq!(drops.load(Ordering::Relaxed), 0);
    assert!(Arc::strong_count(&domain) > 1);
}

#[test]
fn d3hot_transition_preserves_pme_status_but_refuses_failed_power_or_lost_assignment() {
    for (fail_power, lose_bar) in [(false, false), (true, false), (false, true)] {
        let model = Model::default();
        {
            let mut state = model.state.lock().unwrap();
            state.set(NVME + 0x40, 2, 0x8010);
            state.set(NVME + 0x80, 2, 1);
            state.set(NVME + 0x82, 2, 3);
            state.set(NVME + 0x84, 2, 0x8103);
            state.pm_failure = fail_power;
            state.pm_loses_bar = lose_bar;
        }
        let state = model.state.clone();
        let drops = model.drops.clone();
        let (mut life, _) = lifecycle(model);
        assert_eq!(life.prepare().is_ok(), !(fail_power || lose_bar));
        assert_eq!(state.lock().unwrap().child(NVME + 0x84, 2) & 0x8000, 0x8000);
        assert!(!state.lock().unwrap().any_master());
        drop(life);
        assert_eq!(drops.load(Ordering::Relaxed), 0);
    }
}

#[test]
fn invalid_timing_precedes_hierarchy_and_capability_config_accesses() {
    for rate in [0, u64::MAX, u64::MAX / 2 / 60_000_000 + 1] {
        let model = Model::default();
        let state = model.state.clone();
        let (mut host, bar) = fixture(model);
        let mut hierarchy = Hierarchy::discover(&mut host, &bar, NVME_CLASS).unwrap();
        let before = state.lock().unwrap().events.clone();
        let clock = ModelTimer {
            budget: crate::Clock {
                ticks_per_microsecond: rate,
            },
        };
        assert_eq!(
            hierarchy.prepare(&mut host, &clock),
            Err(Error::PciPower(crate::bus::pci::power::Error::Clock))
        );
        assert_eq!(state.lock().unwrap().events, before);
        let mut config = host.config(2, 0, 0).unwrap();
        let capabilities = capabilities::Capabilities::inspect(&mut config, false).unwrap();
        let before = state.lock().unwrap().events.clone();
        assert_eq!(
            capabilities.power_up(&mut config, &clock),
            Err(Error::PciPower(crate::bus::pci::power::Error::Clock))
        );
        assert_eq!(state.lock().unwrap().events, before);
    }
}

#[test]
fn exact_pm_wait_requests_follow_word_effect_and_propagate_failure() {
    struct CapturedTime {
        state: Arc<Mutex<State>>,
        fail: bool,
    }
    // SAFETY: explicit same-peer symbolic time. Posted config effects
    // must precede the requested interval; no physical timer is represented.
    unsafe impl crate::bus::pci::power::Timer for CapturedTime {
        fn check(&self) -> Result<(), crate::bus::pci::power::Error> {
            Ok(())
        }
        fn wait_us(&self, microseconds: u64) -> Result<(), crate::bus::pci::power::Error> {
            let mut state = self.state.lock().unwrap();
            assert_eq!(state.child(NVME + 0x84, 2) & 0x8103, 0x8000);
            state.events.push(Event::Wait(microseconds));
            if self.fail {
                Err(crate::bus::pci::power::Error::Clock)
            } else {
                Ok(())
            }
        }
    }
    for power in [2, 3] {
        for fail in [false, true] {
            let model = Model::default();
            let state = model.state.clone();
            {
                let mut state = state.lock().unwrap();
                state.set(NVME + 0x40, 2, 0x8010);
                state.set(NVME + 0x80, 2, 1);
                state.set(NVME + 0x82, 2, 3);
                state.set(NVME + 0x84, 2, 0x8100 | power);
            }
            let (mut host, _) = fixture(model);
            let mut config = host.config(2, 0, 0).unwrap();
            let capabilities = capabilities::Capabilities::inspect(&mut config, false).unwrap();
            let result = capabilities.power_up(
                &mut config,
                &CapturedTime {
                    state: state.clone(),
                    fail,
                },
            );
            assert_eq!(
                result,
                if fail {
                    Err(Error::PciPower(crate::bus::pci::power::Error::Clock))
                } else {
                    Ok(())
                }
            );
            let waits: Vec<_> = state
                .lock()
                .unwrap()
                .events
                .iter()
                .filter_map(|event| match event {
                    Event::Wait(us) => Some(*us),
                    _ => None,
                })
                .collect();
            assert_eq!(waits, vec![if power == 3 { 10_000 } else { 200 }]);
        }
    }
}

#[test]
fn reassignment_removal_and_wrong_host_epoch_never_restart_the_controller() {
    for change in 0..7 {
        let model = Model::default();
        let state = model.state.clone();
        let (mut host, mut bar) = fixture(model);
        if change == 0 {
            bar.domain += 1;
            assert!(Hierarchy::discover(&mut host, &bar, NVME_CLASS).is_err());
            continue;
        }
        let mut hierarchy = Hierarchy::discover(&mut host, &bar, NVME_CLASS).unwrap();
        {
            let mut state = state.lock().unwrap();
            match change {
                1 => state.set(NVME + 0x10, 4, 0x8000_8004),
                2 => state.set(NVME, 4, u32::MAX),
                3 => state.set(BRIDGE + 0x20, 4, 0x8010_8010),
                4 => {
                    state.set_parent(VMCAP, 2, 1);
                    state.set_parent(VMCONFIG, 2, 1 << 8);
                }
                5 => {
                    state.set_parent(6, 2, 0x10);
                    state.set_parent(0x34, 1, 0x80);
                    state.set_parent(0x80, 2, 1);
                    state.set_parent(0x82, 2, 3);
                    state.set_parent(0x84, 2, 3);
                }
                _ => state.set_parent(0, 4, 0x28c1_8086),
            }
        }
        assert!(hierarchy.prepare(&mut host, &*model_timer()).is_err());
        assert!(!state.lock().unwrap().any_master());
    }
}

#[test]
fn provider_epoch_refusal_and_release_failure_keep_original_resources_pinned() {
    let model = Model::default();
    let drops = model.drops.clone();
    let state = model.state.clone();
    let (mut life, domain) = lifecycle(model);
    // This is a model provider refusing its own changed root epoch;
    // it does not claim the shared DmaDomain API can identify a wrong domain.
    state.lock().unwrap().epoch = 2;
    let page_drops = Arc::new(AtomicUsize::new(0));
    assert!(
        DmaMapping::map(
            domain.clone(),
            pin(page_drops.clone()),
            Direction::Bidirectional,
            u64::MAX
        )
        .is_err()
    );
    assert_eq!(page_drops.load(Ordering::Relaxed), 1);
    state.lock().unwrap().epoch = 1;
    life.prepare().unwrap();
    life.disable_dma().unwrap();
    // SAFETY: no engine or payload was started, BM clears acknowledged.
    unsafe { domain.drain_posted_writes() }.unwrap();
    state.lock().unwrap().release_failure = true;
    // SAFETY: the model endpoint is stopped and drained, with no mappings left.
    assert!(unsafe { life.release() }.is_err());
    assert_eq!(drops.load(Ordering::Relaxed), 0);
    assert!(Arc::strong_count(&domain) > 1);
    assert_eq!(life.phase, Phase::Quarantined);
    assert!(life.returned.borrow().0.is_none());
    drop(life);
    assert_eq!(drops.load(Ordering::Relaxed), 0);
}

#[test]
fn malformed_capabilities_active_translation_and_bar_profiles_are_refused() {
    for case in 0..4 {
        let model = Model::default();
        {
            let mut state = model.state.lock().unwrap();
            match case {
                0 => state.set(NVME + 0x40, 2, 0x4410), // Link into PCIe body.
                1 => state.set(NVME + 0x48, 2, 1 << 11), // NoSnoop enabled.
                2 => {
                    state.set(NVME + 0x100, 4, 0x0001_000f);
                    state.set(NVME + 0x106, 2, 0x8000);
                }
                _ => state.set(NVME + 0x100, 4, 0x0001_0010), // Unqualified SR-IOV.
            }
        }
        let (mut host, bar) = fixture(model);
        assert!(Hierarchy::discover(&mut host, &bar, NVME_CLASS).is_err());
    }
    for (low, high, bytes) in [
        (0, 0, 8192),
        (0x8000_4001, 0, 8192),
        (0x8000_4002, 0, 8192),
        (0xffff_e004, u32::MAX, 8192),
        (0x8000_4004, 0, 4096),
        (0x8000_5004, 0, 8192),
    ] {
        assert!(bar_address(low, high, bytes).is_err());
    }
}

#[repr(C, align(4096))]
struct DeviceMemory {
    bytes: UnsafeCell<[u8; 16384]>,
    thread: std::thread::ThreadId,
}
// SAFETY: actual isolated RAM; every accepted model access checks this original
// creating thread before dereference. Registry views serialize its consumers.
unsafe impl Send for DeviceMemory {}
// SAFETY: same original single-thread RAM exclusion; no background device exists.
unsafe impl Sync for DeviceMemory {}
fn memory_context(memory: &Arc<DeviceMemory>) -> Result<(), crate::bus::pci::assignment::Error> {
    if std::thread::current().id() == memory.thread {
        Ok(())
    } else {
        Err(crate::bus::pci::assignment::Error::Context)
    }
}
struct MappedBacking {
    memory: Arc<DeviceMemory>,
    retained: MappingRetention<Arc<DeviceMemory>>,
}
// SAFETY: this separate fixture owns ACTUAL isolated RAM and retains the same
// counted allocation; topology-only Backing remains a denial-only model.
unsafe impl ResourceBacking for MappedBacking {
    fn begin_mapping(
        &self,
    ) -> Result<crate::bus::pci::assignment::OpaqueUse, crate::bus::mmio::owned::Error> {
        self.retained.begin_mapping()
    }
    fn physical_span(&self) -> PhysicalSpan {
        PhysicalSpan::new(NVME_BASE, 16384).unwrap()
    }
    fn kind(&self) -> ResourceKind {
        ResourceKind::Registers
    }
}
struct Platform {
    memory: Arc<DeviceMemory>,
    retained: MappingRetention<Arc<DeviceMemory>>,
    epoch: AtomicUsize,
    fail_second_once: AtomicBool,
    unmaps: Mutex<Vec<u64>>,
}
// SAFETY: isolated CPU mapping fixture; no concurrent register consumers are
// started, and mutation occurs only through real registry-owned bounded views.
unsafe impl Send for Platform {}
// SAFETY: same fixture exclusion; metadata is immutable, atomic or locked.
unsafe impl Sync for Platform {}
// SAFETY: model allocates its aligned CPU storage and validates original
// reservation identity and current provider epoch, not just physical numbers.
unsafe impl MappingPlatform for Platform {
    fn check_context(&self, _: AddressSpace) -> Result<(), crate::bus::mmio::owned::Error> {
        self.retained.check_context()
    }
    unsafe fn map(
        &self,
        resource: &Resource,
        request: MapRequest,
    ) -> Result<InstalledMapping, MapFailure> {
        use crate::bus::mmio::owned::Error as MappingError;
        let Some(backing) = resource.backing().downcast_ref::<MappedBacking>() else {
            return Err(MapFailure::Unchanged(MappingError::Unavailable));
        };
        if !Arc::ptr_eq(&backing.memory, &self.memory)
            || !backing.retained.same_owner(&self.retained)
            || self.epoch.load(Ordering::Relaxed) != 1
        {
            return Err(MapFailure::Unchanged(MappingError::Unavailable));
        }
        let offset = request
            .physical
            .start()
            .checked_sub(NVME_BASE)
            .ok_or(MapFailure::Unchanged(MappingError::OutsideResource))?;
        if offset
            .checked_add(request.physical.bytes())
            .is_none_or(|end| end > 16384)
        {
            return Err(MapFailure::Unchanged(MappingError::OutsideResource));
        }
        // SAFETY: provider owns this exact bounded model allocation; symbolic
        // physical pages are unrelated to its CPU address.
        let cpu_base = unsafe {
            NonNull::new_unchecked(self.memory.bytes.get().cast::<u8>().add(offset as usize))
        };
        Ok(InstalledMapping {
            token: request.id,
            cpu_base,
            physical: request.physical,
            attribute: request.attribute,
            protection: request.protection,
        })
    }
    unsafe fn unmap(
        &self,
        request: MapRequest,
        installed: InstalledMapping,
    ) -> Result<(), crate::bus::mmio::owned::Error> {
        use crate::bus::mmio::owned::Error as MappingError;
        assert_eq!(request.id, installed.token);
        self.unmaps.lock().unwrap().push(request.id);
        if request.physical.start() == NVME_BASE + 8192
            && self.fail_second_once.swap(false, Ordering::Relaxed)
        {
            return Err(MappingError::UnmapFailed);
        }
        Ok(())
    }
}
fn mapping_platform(
    memory: Arc<DeviceMemory>,
    retained: MappingRetention<Arc<DeviceMemory>>,
) -> (Authority, Arc<Platform>) {
    let platform = Arc::new(Platform {
        memory,
        retained,
        epoch: AtomicUsize::new(1),
        fail_second_once: AtomicBool::new(false),
        unmaps: Mutex::new(Vec::new()),
    });
    // SAFETY: separate isolated backing and roots; no production latch or
    // aliases are modified, and all mappings keep this model owner.
    let authority = unsafe {
        Authority::isolated_model(
            platform.clone(),
            AddressSpace {
                identity: 7,
                physical_bits: 48,
                virtual_bits: 48,
            },
            Vec::new(),
        )
    }
    .unwrap();
    (authority, platform)
}

#[test]
fn resource_identity_and_epoch_gate_registry_maps() {
    let (_host, mut bar) = fixture(Model::default());
    let memory = || {
        Arc::new(DeviceMemory {
            bytes: UnsafeCell::new([0; 16384]),
            thread: std::thread::current().id(),
        })
    };
    let ram = memory();
    assert!(
        (ram.bytes.get() as usize).is_multiple_of(4096),
        "actual model buffer must be page-aligned"
    );
    // SAFETY: this owning fixture controls actual private RAM on its original
    // thread. Neither allocation nor its clone owns a Source/Authority/view.
    let mut owner = unsafe { MappingOwner::from_owned(ram.clone(), memory_context) };
    // This is a separate CPU-registry exercise. The unchanged topology-only
    // source's Arc<()> label cannot authorize any CPU mapping.
    bar.resource = Resource::new(Box::new(MappedBacking {
        memory: ram.clone(),
        retained: owner.retention().unwrap(),
    }));
    let other_ram = memory();
    assert!(
        (other_ram.bytes.get() as usize).is_multiple_of(4096),
        "actual model buffer must be page-aligned"
    );
    // SAFETY: distinct actual RAM with its own original pure context guard.
    let mut other_owner = unsafe { MappingOwner::from_owned(other_ram.clone(), memory_context) };
    let (wrong, wrong_platform) =
        mapping_platform(other_ram.clone(), other_owner.retention().unwrap());
    assert!(bar.map(&wrong).is_err());
    assert_eq!(
        owner.uses(),
        0,
        "unchanged foreign-provider refusal kept a map use"
    );
    let (authority, platform) = mapping_platform(ram.clone(), owner.retention().unwrap());
    platform.epoch.store(2, Ordering::Relaxed);
    assert!(bar.map(&authority).is_err());
    assert_eq!(
        owner.uses(),
        0,
        "unchanged provider-profile refusal kept a map use"
    );
    platform.epoch.store(1, Ordering::Relaxed);
    let mut first = bar.map(&authority).unwrap();
    assert_eq!(
        first.physical_span().unwrap(),
        PhysicalSpan::new(NVME_BASE, 8192).unwrap()
    );
    let offset = (NVME_BASE + 8192 - bar.resource.physical_span().start()) as usize;
    let mut second = authority
        .map(
            bar.resource.clone(),
            offset,
            8192,
            CacheAttribute::Uncached,
            Protection::ReadWrite,
        )
        .unwrap();
    platform.fail_second_once.store(true, Ordering::Relaxed);
    // These are the platform's explicit final mapping acknowledgements, not
    // NVMe Lifecycle.release. No model device/CPU users were ever started.
    // SAFETY: exact isolated tokens with no consumers, callbacks or writes.
    unsafe { first.retire() }.unwrap();
    assert_eq!(
        // SAFETY: same model-owner exclusion; failure preserves its token.
        unsafe { second.retire() },
        Err(crate::bus::mmio::owned::Error::UnmapFailed)
    );
    assert!(first.address(0, 4).is_err());
    assert!(second.address(0, 4).is_err());
    // SAFETY: retry the same stopped original token, not a recreated mapping.
    unsafe { second.retire() }.unwrap();
    let unmaps = platform.unmaps.lock().unwrap();
    assert_eq!(unmaps.len(), 3);
    assert_ne!(unmaps[0], unmaps[1]);
    assert_eq!(unmaps[1], unmaps[2]);
    drop(unmaps);
    assert_eq!(
        owner.uses(),
        0,
        "acknowledged original unmaps did not release counted uses"
    );
    drop(first);
    drop(second);
    drop(authority);
    drop(platform);
    drop(wrong);
    drop(wrong_platform);
    drop(bar);
    // SAFETY: exact original RAM views and both acknowledged mapping tokens
    // ended above; no callback/raw alias/device exists or can restart.
    let returned = unsafe { owner.retire() }.unwrap();
    assert!(Arc::ptr_eq(&returned, &ram));
    // SAFETY: the wrong provider never installed a map; its original retained
    // provider views also ended above, so this returns the SAME unused pool.
    let other_returned = unsafe { other_owner.retire() }.unwrap();
    assert!(Arc::ptr_eq(&other_returned, &other_ram));
}

#[test]
fn failed_power_timing_keeps_the_same_owned_minimum_provider_with_the_hierarchy() {
    struct Unavailable {
        _allocation: Box<[u8; 4096]>,
        drops: Rc<core::cell::Cell<usize>>,
    }
    impl Drop for Unavailable {
        fn drop(&mut self) {
            self.drops.set(self.drops.get() + 1);
        }
    }
    // SAFETY: denial-only model; no wait or native timing grant is supplied.
    unsafe impl crate::bus::pci::power::Timer for Unavailable {
        fn check(&self) -> Result<(), crate::bus::pci::power::Error> {
            Err(crate::bus::pci::power::Error::Clock)
        }
        fn wait_us(&self, _: u64) -> Result<(), crate::bus::pci::power::Error> {
            panic!("unavailable model timer cannot wait");
        }
    }
    let model = Model::default();
    let drops = model.drops.clone();
    let state = model.state.clone();
    let (host, bar) = fixture(model);
    let domain = Arc::new(Domain { state, epoch: 1 });
    let timer_drops = Rc::new(core::cell::Cell::new(0));
    let minimum: Rc<dyn crate::bus::pci::power::Timer> = Rc::new(Unavailable {
        _allocation: Box::new([0; 4096]),
        drops: timer_drops.clone(),
    });
    let (mut endpoint, _) = Endpoint::new(host, domain, minimum.clone());
    endpoint.discover(&bar, NVME_CLASS).unwrap();
    assert_eq!(
        endpoint.prepare(),
        Err(Error::PciPower(crate::bus::pci::power::Error::Clock))
    );
    drop(endpoint);
    assert_eq!(Rc::strong_count(&minimum), 2);
    assert_eq!(timer_drops.get(), 0);
    assert_eq!(drops.load(Ordering::Relaxed), 0);
}
