// SPDX-License-Identifier: GPL-2.0-only
//! Stateful software models, authored UNRUN under the driver-stream build lease.
use super::*;
use crate::bus::pci::regs;
use alloc::{collections::BTreeMap, sync::Arc};
use core::sync::atomic::AtomicUsize;
extern crate std;

struct Model {
    parent: [u8; 256],
    bars: [(u64, u64, bool); 3],
    shadow: BTreeMap<usize, u64>,
    child: BTreeMap<usize, u8>,
    /// Writable bits of child dwords, as a BAR's size leaves them.
    child_masks: BTreeMap<usize, u32>,
    table: BTreeMap<u64, u32>,
    writes: Vec<(u8, usize, u8, u32)>,
    parent_reads: Arc<std::sync::Mutex<Vec<(u16, u8)>>>,
    fail_table: bool,
    drops: Arc<AtomicUsize>,
}
impl Model {
    fn new(device: u16) -> Self {
        let mut model = Self {
            parent: [0; 256],
            bars: [
                (0x4000_0000, 32 << 20, false),
                (0x8000_0000, 0x100_0000, false),
                (0x9000_0000, 0x100_0000, true),
            ],
            shadow: BTreeMap::new(),
            child: BTreeMap::new(),
            child_masks: BTreeMap::new(),
            table: BTreeMap::new(),
            writes: Vec::new(),
            parent_reads: Arc::new(std::sync::Mutex::new(Vec::new())),
            fail_table: false,
            drops: Arc::new(AtomicUsize::new(0)),
        };
        model.set(0, 4, 0x8086 | u32::from(device) << 16);
        for (i, (address, _, prefetchable)) in model.bars.into_iter().enumerate() {
            model.set(
                0x10 + i * 8,
                4,
                address as u32 | 4 | if prefetchable { 8 } else { 0 },
            );
            model.set(0x14 + i * 8, 4, (address >> 32) as u32);
        }
        model
    }
    fn set(&mut self, offset: usize, width: u8, value: u32) {
        for i in 0..usize::from(width) {
            self.parent[offset + i] = (value >> (i * 8)) as u8;
        }
    }
    fn child_set(&mut self, base: usize, width: u8, value: u32) {
        for i in 0..usize::from(width) {
            self.child.insert(base + i, (value >> (i * 8)) as u8);
        }
    }
    fn msix(&mut self) {
        self.set(6, 2, 0x10);
        self.set(0x34, 1, 0x80);
        self.set(0x80, 2, 0x11);
        self.set(0x82, 2, 3);
        self.set(0x84, 4, 4);
        self.set(0x88, 4, 0x1004);
    }
}
impl Drop for Model {
    fn drop(&mut self) {
        self.drops.fetch_add(1, Ordering::Relaxed);
    }
}
// SAFETY: this is an isolated stateful model, never physical hardware permission.
unsafe impl Registers for Model {
    fn parent_read(&mut self, offset: u16, width: u8) -> Result<u32, Error> {
        self.parent_reads.lock().unwrap().push((offset, width));
        let mut value = 0;
        for i in 0..usize::from(width) {
            value |= u32::from(self.parent[usize::from(offset) + i]) << (i * 8);
        }
        Ok(value)
    }
    fn parent_write(&mut self, offset: u16, width: u8, value: u32) -> Result<(), Error> {
        self.writes.push((0, usize::from(offset), width, value));
        self.set(usize::from(offset), width, value);
        Ok(())
    }
    fn bar(&self, index: u8) -> (u64, u64, bool) {
        self.bars[usize::from(index / 2)]
    }
    fn shadow_read(&mut self, offset: usize) -> Result<u64, Error> {
        self.shadow.get(&offset).copied().ok_or(Error::ReadFailed)
    }
    fn child_read(&mut self, offset: usize, width: u8) -> Result<u32, Error> {
        let mut value = 0;
        for i in 0..usize::from(width) {
            value |= u32::from(*self.child.get(&(offset + i)).unwrap_or(&0xff)) << (i * 8);
        }
        Ok(value)
    }
    fn child_write(&mut self, offset: usize, width: u8, value: u32) -> Result<(), Error> {
        self.writes.push((1, offset, width, value));
        let value = match self.child_masks.get(&offset).copied() {
            Some(mask) if width == 4 => {
                let old = self.child_read(offset, 4)?;
                value & mask | old & !mask
            }
            _ => value,
        };
        self.child_set(offset, width, value);
        Ok(())
    }
    fn table_read(&mut self, offset: u64) -> Result<u32, Error> {
        if self.fail_table {
            Err(Error::ReadFailed)
        } else {
            Ok(*self.table.get(&offset).unwrap_or(&0))
        }
    }
    fn table_write(&mut self, offset: u64, value: u32) -> Result<(), Error> {
        self.writes.push((2, offset as usize, 4, value));
        self.table.insert(offset, value);
        Ok(())
    }
}
fn start(model: Model) -> Host<Model> {
    match Host::initialize(model) {
        Ok(host) => host,
        Err((e, _)) => panic!("initialize: {e:?}"),
    }
}

#[test]
fn restricted_buses_are_relative_and_bios_buses_are_absolute() {
    for (setting, first) in [(0, 0), (1, 128), (2, 224), (3, 224)] {
        let mut model = Model::new(0x28c0);
        model.set(usize::from(VMCAP), 2, 1);
        model.set(usize::from(VMCONFIG), 2, setting << 8 | 2);
        model.child_set(0, 4, 0x1234_8086);
        let mut host = start(model);
        assert_eq!(host.buses(), first..=first + 31);
        assert_eq!(
            host.config(first, 0, 0).unwrap().read(0, 4),
            Ok(0x1234_8086)
        );
        assert_eq!(host.windows()[1].cpu_start, 0x9000_2010);
        assert!(host.domain() >= 0x1_0000);
        // SAFETY: model has no hardware or asynchronous consumers.
        unsafe { host.retire() }.unwrap();
    }
    let mut model = Model::new(0x28c1);
    model.bars[0].1 = 256 << 20;
    model
        .shadow
        .extend([(0x2818, 0), (0x2820, 0), (0x2840, 224 << 8)]);
    model.child_set(224 << 20, 4, 0xabcd_8086);
    let mut host = start(model);
    assert_eq!(host.buses(), 224..=255);
    assert_eq!(host.config(224, 0, 0).unwrap().read(0, 4), Ok(0xabcd_8086));
    assert_eq!(host.windows()[1].cpu_start, 0x9000_30d0);
}

#[test]
fn shadow_zero_is_a_real_translation_except_for_bios_info() {
    let mut model = Model::new(0x28c0);
    model.set(usize::from(VMLOCK), 4, 2);
    model.shadow.extend([(0x2000, 0), (0x2008, 0x1000_0000)]);
    let host = start(model);
    assert_eq!(host.windows()[0].translate(0x1000, 4096), Ok(0x8000_1000));
    assert_eq!(
        host.windows()[1].translate(0x1000_2010, 16),
        Ok(0x9000_2010)
    );
    assert_eq!(
        host.windows()[1].translate(0x1000_2000, 16),
        Err(Error::Address)
    );
}

fn shadow_capability(model: &mut Model, offset: u16, length: u8, next: u8) {
    model.set(6, 2, 0x10);
    model.set(0x34, 1, u32::from(offset));
    model.set(
        usize::from(offset),
        2,
        u32::from(VENDOR_CAPABILITY) | u32::from(next) << 8,
    );
    model.set(usize::from(offset) + 2, 1, u32::from(length));
    model.set(usize::from(offset) + 4, 4, SHADOW_SIGNATURE);
    if usize::from(offset) + 24 <= 256 {
        model.set(usize::from(offset) + 8, 4, 0x1000_0000);
        model.set(usize::from(offset) + 12, 4, 0);
        model.set(usize::from(offset) + 16, 4, 0x2000_0000);
        model.set(usize::from(offset) + 20, 4, 0);
    }
}

#[test]
fn vendor_shadow_requires_its_complete_declared_payload() {
    for offset in [0x80, 0xe8] {
        let mut model = Model::new(0x201d);
        shadow_capability(&mut model, offset, 24, 0);
        let host = start(model);
        assert_eq!(host.windows()[0].bus_start, 0x1000_0000);
        assert_eq!(host.windows()[1].bus_start, 0x2000_2000);
    }
    for length in 0..8 {
        let mut model = Model::new(0x201d);
        shadow_capability(&mut model, 0x80, length, 0);
        let reads = model.parent_reads.clone();
        let host = start(model);
        // Short vendor data never advertises a complete SHDW signature, and a
        // Length below the entry's own header reserves the header alone.
        // Poison bytes beyond its Length must not become window translations.
        assert_eq!(host.windows()[0].bus_start, 0x8000_0000);
        assert_eq!(host.windows()[1].bus_start, 0x9000_2000);
        assert!(
            reads
                .lock()
                .unwrap()
                .iter()
                .all(|(offset, _)| !(0x84..0x98).contains(offset))
        );
    }
    for length in 8..24 {
        let mut model = Model::new(0x201d);
        shadow_capability(&mut model, 0x80, length, 0);
        let reads = model.parent_reads.clone();
        match Host::initialize(model) {
            Err((error, None)) => assert_eq!(error, Error::Capability),
            _ => panic!("invalid shadow capability must retain the failed host"),
        }
        assert!(
            reads
                .lock()
                .unwrap()
                .iter()
                .all(|(offset, _)| !(0x88..0x98).contains(offset))
        );
    }
    let mut model = Model::new(0x201d);
    shadow_capability(&mut model, 0xec, 24, 0);
    let reads = model.parent_reads.clone();
    match Host::initialize(model) {
        Err((error, None)) => assert_eq!(
            error,
            Error::PciCapability(CapabilityError::OutOfRange { offset: 0xec })
        ),
        _ => panic!("oversized shadow capability must retain the failed host"),
    }
    assert!(
        !reads
            .lock()
            .unwrap()
            .iter()
            .any(|(offset, _)| *offset >= 0xf0)
    );
}

#[test]
fn vendor_shadow_cannot_hide_overlapping_or_cyclic_capability_links() {
    for next in [0x80, 0x84, 0x88, 0x94] {
        let mut model = Model::new(0x201d);
        shadow_capability(&mut model, 0x80, 24, next);
        let reads = model.parent_reads.clone();
        let expected = if next == 0x80 {
            CapabilityError::Cycle { offset: 0x80 }
        } else {
            CapabilityError::Overlap {
                offset: u16::from(next),
            }
        };
        match Host::initialize(model) {
            Err((error, None)) => assert_eq!(error, Error::PciCapability(expected)),
            _ => panic!("shadow aliases must retain the failed host"),
        }
        assert!(
            reads
                .lock()
                .unwrap()
                .iter()
                .all(|(offset, _)| !(0x88..0x98).contains(offset))
        );
    }
    for next in [0x80u8, 0x88] {
        let mut model = Model::new(0x201d);
        shadow_capability(&mut model, 0x80, 24, 0xb0);
        model.set(0xb0, 2, 1 | u32::from(next) << 8);
        let reads = model.parent_reads.clone();
        let expected = if next == 0x80 {
            CapabilityError::Cycle { offset: 0x80 }
        } else {
            CapabilityError::Overlap {
                offset: u16::from(next),
            }
        };
        match Host::initialize(model) {
            Err((error, None)) => assert_eq!(error, Error::PciCapability(expected)),
            _ => panic!("shadow aliases must retain the failed host"),
        }
        assert!(
            reads
                .lock()
                .unwrap()
                .iter()
                .all(|(offset, _)| !(0x88..0x98).contains(offset))
        );
    }
    let mut model = Model::new(0x201d);
    shadow_capability(&mut model, 0x80, 24, 0xa0);
    model.set(0xa0, 2, 1);
    let mut host = start(model);
    assert_eq!(host.windows()[0].bus_start, 0x1000_0000);
    // The valid trailing PM header is traversed before addresses.
    let reads = host.io().parent_reads.lock().unwrap();
    let pm = reads.iter().position(|read| *read == (0xa0, 2)).unwrap();
    let address = reads.iter().position(|read| *read == (0x88, 4)).unwrap();
    assert!(pm < address);
}

#[test]
fn scan_honors_multifunction_and_never_enables_child_dma() {
    let mut model = Model::new(0x201d);
    model.bars[0].1 = 1 << 20;
    model.child_set(0, 4, 0x1234_8086);
    model.child_set(0x0e, 1, 0x80);
    model.child_set(8, 4, 0x0604_0000);
    model.child_set(7 << 12, 4, 0x5678_144d);
    model.child_set((7 << 12) + 0x0e, 1, 0);
    model.child_set((7 << 12) + 8, 4, 0x0108_0200);
    let mut host = start(model);
    let children = host.scan().unwrap();
    assert_eq!(children.len(), 2);
    assert_eq!(children[1].address.function, 7);
    assert_eq!(children[1].class, [1, 8, 2]);
    let mut config = host.config(0, 0, 7).unwrap();
    assert_eq!(config.acpi_address(), Some(0x8007_ffff));
    assert_eq!(config.write(regs::COMMAND, 2, 4), Err(Error::ActiveChild));
    assert_eq!(config.read(4095, 2), Err(Error::Address));
}

#[test]
fn a_child_bar_is_sized_with_decode_off_and_restored_stopped() {
    // The second child was left mastering, without INTx Disable.
    for command in [0x0402, 0x0006] {
        let mut model = Model::new(0x201d);
        model.bars[0].1 = 1 << 20;
        model.child_set(0, 4, 0x5678_144d);
        model.child_set(4, 2, command);
        model.child_set(6, 2, 0);
        model.child_set(8, 4, 0x0108_0200);
        model.child_set(0x0e, 1, 0);
        model.child_set(0x10, 4, 0x8000_4004);
        model.child_set(0x14, 4, 0);
        model.child_masks.insert(0x10, 0xffff_c000);
        model.child_masks.insert(0x14, u32::MAX);
        let mut host = start(model);
        let child = host.scan().unwrap()[0].address;
        assert_eq!(host.memory_bar_bytes(child), Ok(0x4000));
        let mut config = host.config(0, 0, 0).unwrap();
        assert_eq!(config.read(0x10, 4), Ok(0x8000_4004));
        assert_eq!(config.read(0x14, 4), Ok(0));
        assert_eq!(config.read(regs::COMMAND, 2), Ok(0x0402));
        let writes = &host.registers.as_ref().unwrap().writes;
        let sizing = writes
            .iter()
            .position(|write| *write == (1, 0x10, 4, u32::MAX))
            .unwrap();
        assert!(writes[..sizing].contains(&(1, 4, 2, 0x0400)));
    }
}

#[test]
fn native_parent_table_programming_keeps_client_vector_zero_masked() {
    let mut model = Model::new(0x9a0b);
    model.msix();
    let mut host = start(model);
    let messages = [
        crate::bus::irq::MsiMessage {
            address: 0xfee0_1000,
            data: 0x50,
        },
        crate::bus::irq::MsiMessage {
            address: 0xfee0_1000,
            data: 0x51,
        },
    ];
    // SAFETY: isolated register model; no physical IRQ permission is inferred.
    let end = unsafe { host.program_parent_vectors(&messages) }.unwrap();
    assert_eq!(end, 3);
    assert_eq!(host.io().table.get(&12), Some(&1));
    assert_eq!(
        host.io().table.get(&16),
        Some(&(messages[0].address as u32))
    );
    assert_eq!(host.io().table.get(&24), Some(&0x50));
    // SAFETY: same isolated modeled programmed vector range.
    unsafe { host.unmask_parent_vectors(end) }.unwrap();
    assert_eq!(host.io().table.get(&12), Some(&1));
    assert_eq!(host.io().table.get(&28), Some(&0));
    host.mask_parent_vectors().unwrap();
    assert_eq!(host.io().table.get(&28), Some(&1));
    // SAFETY: no native source or callback in this isolated model.
    let mut model = unsafe { host.retire() }.unwrap();
    assert_eq!(
        model.parent_read(0x82, 2).unwrap() & u32::from(1u16 << 15),
        0
    );
}

#[test]
fn uncertain_table_effect_and_abandoned_live_host_keep_owner() {
    let mut model = Model::new(0x28c0);
    model.msix();
    let drops = model.drops.clone();
    let mut host = start(model);
    host.io().fail_table = true;
    assert_eq!(
        // SAFETY: isolated failure-injection model only.
        unsafe {
            host.program_parent_vectors(&[crate::bus::irq::MsiMessage {
                address: 0xfee0_0000,
                data: 0x50,
            }])
        },
        Err(Error::ReadFailed)
    );
    assert!(matches!(host.config(0, 0, 0), Err(Error::Quarantined)));
    drop(host);
    assert_eq!(drops.load(Ordering::Relaxed), 0);
}

#[test]
fn remap_assignments_encode_table_indices_and_balance_nvme_only() {
    let mut slots = remap::Assignments::new(Profile::identify(0x8086, 0x9a0b).unwrap(), 4).unwrap();
    assert_eq!(slots.allocate([6, 4, 0]), Ok(1));
    assert_eq!(slots.allocate([1, 8, 2]), Ok(2));
    assert_eq!(slots.allocate([1, 8, 2]), Ok(3));
    assert_eq!(slots.allocate([1, 8, 2]), Ok(2));
    assert_eq!(slots.message(0), Err(Error::Address));
    assert_eq!(
        slots.message(3),
        Ok(crate::bus::irq::MsiMessage {
            address: 0xfee0_3000,
            data: 0
        })
    );
}

#[test]
fn suspend_resume_requires_unchanged_assignment_and_does_not_restore_old_irqs() {
    let mut host = start(Model::new(0x201d));
    let domain = host.domain();
    // SAFETY: isolated model has no DMA/IRQ/work users.
    unsafe { host.suspend() }.unwrap();
    assert!(matches!(host.config(0, 0, 0), Err(Error::Suspended)));
    host.resume().unwrap();
    assert_eq!(host.domain(), domain);
    // SAFETY: same no-consumer model.
    unsafe { host.suspend() }.unwrap();
    host.io().set(0x10, 4, 0x5000_0004);
    assert_eq!(host.resume(), Err(Error::InvalidResource));
    assert_eq!(host.resume(), Err(Error::Quarantined));
}

struct ChildEffects {
    events: Arc<std::sync::Mutex<Vec<&'static str>>>,
    drops: Arc<AtomicUsize>,
    fail_stop: bool,
}
impl Drop for ChildEffects {
    fn drop(&mut self) {
        self.drops.fetch_add(1, Ordering::Relaxed);
    }
}
// SAFETY: isolated lifecycle model; no physical permission or DMA is inferred.
unsafe impl crate::bus::irq::Source for ChildEffects {
    fn mask(&mut self) -> Result<(), crate::bus::irq::IrqError> {
        self.events.lock().unwrap().push("mask");
        Ok(())
    }
    fn quiesce(&mut self) -> Result<(), crate::bus::irq::IrqError> {
        self.events.lock().unwrap().push("quiesce");
        if self.fail_stop {
            Err(crate::bus::irq::IrqError::Quiesce)
        } else {
            Ok(())
        }
    }
    fn route_kind(&self) -> crate::bus::irq::RouteKind {
        crate::bus::irq::RouteKind::Msi
    }
    fn requester(&self) -> Option<&crate::bus::pci::requester::RequesterId> {
        None
    }
    unsafe fn program(
        &mut self,
        route: &crate::bus::irq::AllocatedRoute<'_>,
    ) -> Result<(), crate::bus::irq::IrqError> {
        assert_eq!(
            route.programming(),
            crate::bus::irq::Route::Msi(crate::bus::irq::MsiMessage {
                address: 0xfee0_1000,
                data: 0
            })
        );
        self.events.lock().unwrap().push("program");
        Ok(())
    }
    unsafe fn unmask(&mut self) -> Result<(), crate::bus::irq::IrqError> {
        self.events.lock().unwrap().push("unmask");
        Ok(())
    }
    unsafe fn synchronize(&mut self) -> Result<(), crate::bus::irq::IrqError> {
        self.events.lock().unwrap().push("synchronize");
        Ok(())
    }
}
struct ChildHandler;
impl crate::bus::irq::Handler for ChildHandler {
    fn handle(&self) -> crate::bus::irq::Disposition {
        crate::bus::irq::Disposition::Handled
    }
}

#[test]
fn child_providers_gate_parent_owner_return_and_uncertain_stop_retention() {
    use crate::bus::irq::{Handler, MsiMessage, Route, Source};
    for fail_stop in [false, true] {
        let mut model = Model::new(0x9a0b);
        model.msix();
        let parent_drops = model.drops.clone();
        let events = Arc::new(std::sync::Mutex::new(Vec::new()));
        let child_drops = Arc::new(AtomicUsize::new(0));
        let children = alloc::vec![ChildSource {
            source: alloc::boxed::Box::new(ChildEffects {
                events: events.clone(),
                drops: child_drops.clone(),
                fail_stop
            }),
            handler: Arc::new(ChildHandler)
        }];
        // SAFETY: isolated source-effect model only, no physical device grant.
        let mut host = start(model);
        // Isolated effect model of an external native domain attach.
        // Production exposes no bus-master-enabling API before that permission
        // exists. This assignment supplies no hardware validation.
        host.io().set(
            usize::from(regs::COMMAND),
            2,
            u32::from(regs::COMMAND_MEMORY | regs::COMMAND_MASTER | regs::COMMAND_INTX_DISABLE),
        );
        // SAFETY: the model device, its pages and its domain belong to this test alone; no other
        // driver, worker or DMA touches them.
        let (mut source, handler, mut returned) = match unsafe { ParentSource::new(host, children) }
        {
            Ok(value) => value,
            Err(_) => panic!("source construction failed"),
        };
        source.mask().unwrap();
        // SAFETY: simulate the owning IRQ registry's published model route.
        unsafe {
            source.program(&crate::bus::irq::AllocatedRoute::model_programming(
                Route::Msi(MsiMessage {
                    address: 0xfee0_2000,
                    data: 0x50,
                }),
            ))
        }
        .unwrap();
        // SAFETY: modeled registry keeps both source and fanout.
        unsafe { source.unmask() }.unwrap();
        assert_eq!(handler.handle(), crate::bus::irq::Disposition::Handled);
        source.mask().unwrap();
        if fail_stop {
            assert_eq!(source.quiesce(), Err(crate::bus::irq::IrqError::Quiesce));
            drop(source);
            assert_eq!(parent_drops.load(Ordering::Relaxed), 0);
            assert_eq!(child_drops.load(Ordering::Relaxed), 0);
        } else {
            source.quiesce().unwrap();
            // SAFETY: model has no asynchronous CPU/callback/DMA users.
            unsafe { source.synchronize() }.unwrap();
            assert_eq!(events.lock().unwrap().last(), Some(&"synchronize"));
            drop(source);
            // SAFETY: modeled terminal registration is now retired/dropped.
            let original = unsafe { returned.take_after_retirement() }.unwrap();
            drop(original);
            assert_eq!(parent_drops.load(Ordering::Relaxed), 1);
            assert_eq!(child_drops.load(Ordering::Relaxed), 1);
        }
    }
}

#[test]
fn each_bar_is_decoded_at_its_own_width() {
    // A 32-bit MEMBAR1 between a 64-bit CFGBAR and MEMBAR2: BAR 3 is a
    // register of its own, not MEMBAR1's upper half.
    let mut model = Model::new(0x28c0);
    model.bars[1] = (0x5000_0000, 0x100_0000, false);
    model.set(0x18, 4, 0x5000_0000);
    model.set(0x1c, 4, 0x6000_0000);
    let host = start(model);
    assert_eq!(host.windows()[0].cpu_start, 0x5000_0000);
    assert_eq!(host.windows()[0].bytes, 0x100_0000);
    assert_eq!(host.windows()[1].cpu_start, 0x9000_2010);
    // SAFETY: model has no hardware or asynchronous consumers.
    unsafe { host.retire() }.unwrap();

    let refused = |configure: fn(&mut Model)| {
        let mut model = Model::new(0x28c0);
        configure(&mut model);
        matches!(
            Host::initialize(model),
            Err((Error::InvalidResource, Some(_)))
        )
    };
    // A 32-bit BAR's reservation that does not match its register.
    assert!(refused(|m| {
        m.bars[1] = (0x5000_0000, 0x100_0000, false);
        m.set(0x18, 4, 0x5100_0000);
    }));
    // A 32-bit BAR cannot decode past 4 GiB.
    assert!(refused(|m| {
        m.bars[1] = (0xff80_0000, 0x100_0000, false);
        m.set(0x18, 4, 0xff80_0000);
    }));
    // I/O space and the below-1 MiB memory type are no VMD aperture.
    assert!(refused(|m| {
        m.bars[1] = (0x5000_0000, 0x100_0000, false);
        m.set(0x18, 4, 0x5000_0002);
    }));
    assert!(refused(|m| m.set(0x20, 4, 0x9000_0001)));
}
