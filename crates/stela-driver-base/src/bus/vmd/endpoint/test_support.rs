// SPDX-License-Identifier: GPL-2.0-only
//! The independent posted-config VMD hierarchy model the endpoint tests and
//! the composed NVMe tests share: a root port, a bridge and one NVMe function
//! behind an Intel VMD parent, with posted child writes.
extern crate std;
use super::*;
use crate::Clock;
use crate::bus::{
    dma::mapping::DomainMapping,
    mmio::owned::{PhysicalSpan, ResourceBacking, ResourceKind},
    pci::regs,
};
use alloc::{boxed::Box, collections::BTreeMap};
use core::sync::atomic::{AtomicUsize, Ordering};
use std::sync::Mutex;

pub const ROOT: usize = 1 << 15;
pub const BRIDGE: usize = 1 << 20;
pub const NVME: usize = 2 << 20;
pub const NVME_BASE: u64 = 0x8000_4000;
/// A second NVMe controller's slot behind the same bridge, and its BAR0,
/// for models that add one ([`State::second_nvme`]).
pub const NVME2: usize = NVME + (1 << 15);
pub const NVME2_BASE: u64 = 0x8000_8000;
pub const COMMAND_OFFSET: usize = regs::COMMAND as usize;

// The same finite counter scale as the shared independent NVMe peer. This is
// an admitted software-test budget, not a measured physical CPU calibration.
// Composed command timeouts use the explicit cfg(test) 1-ms policy; actual
// public native attachment keeps the shared 30-s controller policy.
pub fn model_clock() -> Clock {
    Clock {
        ticks_per_microsecond: 1000,
    }
}

pub(super) struct ModelTimer {
    pub(super) budget: Clock,
}
// SAFETY: explicit shared posted-config/NVMe software model only. The checked
// counter budget advances its modeled intervals; it grants no native minimum.
unsafe impl crate::bus::pci::power::Timer for ModelTimer {
    fn check(&self) -> Result<(), crate::bus::pci::power::Error> {
        self.budget
            .check_duration(60_000_000)
            .map_err(|_| crate::bus::pci::power::Error::Clock)
    }
    fn wait_us(&self, microseconds: u64) -> Result<(), crate::bus::pci::power::Error> {
        self.check()?;
        self.budget
            .try_delay(microseconds)
            .map_err(|_| crate::bus::pci::power::Error::Clock)
    }
}
pub fn model_timer() -> Rc<dyn crate::bus::pci::power::Timer> {
    Rc::new(ModelTimer {
        budget: model_clock(),
    })
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum Event {
    Posted(usize, u8, u32),
    Flushed(usize, u8),
    Parent(u16),
    Map(u64),
    Drain,
    Unmap(u64),
    Nvme(&'static str),
    Wait(u64),
}
pub struct State {
    pub parent: BTreeMap<u16, u8>,
    pub child: BTreeMap<usize, u8>,
    pub pending: Option<(usize, u8, u32)>,
    pub events: Vec<Event>,
    pub ack_failure: Option<usize>,
    /// Bits a child register keeps set whatever is written, by offset.
    pub stuck: Option<(usize, u32)>,
    pub pm_failure: bool,
    pub pm_loses_bar: bool,
    pub release_failure: bool,
    pub epoch: u64,
    pub drained: bool,
    pub mappings: BTreeMap<u64, DomainMapping>,
    pub next: u64,
}
fn get<K: Ord + Copy>(memory: &BTreeMap<K, u8>, keys: impl Iterator<Item = K>) -> u32 {
    keys.enumerate().fold(0, |value, (i, key)| {
        value | u32::from(*memory.get(&key).unwrap_or(&0xff)) << (i * 8)
    })
}
impl State {
    pub fn parent(&self, offset: u16, width: u8) -> u32 {
        get(&self.parent, (0..u16::from(width)).map(|i| offset + i))
    }
    pub fn child(&self, offset: usize, width: u8) -> u32 {
        get(&self.child, (0..usize::from(width)).map(|i| offset + i))
    }
    pub fn set_parent(&mut self, offset: u16, width: u8, value: u32) {
        for i in 0..u16::from(width) {
            self.parent.insert(offset + i, (value >> (i * 8)) as u8);
        }
    }
    pub fn set(&mut self, offset: usize, width: u8, value: u32) {
        for i in 0..usize::from(width) {
            self.child.insert(offset + i, (value >> (i * 8)) as u8);
        }
    }
    pub fn any_master(&self) -> bool {
        self.parent(regs::COMMAND, 2) & u32::from(regs::COMMAND_MASTER) != 0
            || self
                .child
                .keys()
                .filter(|offset| **offset & 0xfff == COMMAND_OFFSET)
                .any(|offset| self.child(*offset, 2) & u32::from(regs::COMMAND_MASTER) != 0)
    }
    pub fn add_function(&mut self, base: usize, class: u32, header: u8, kind: u8) {
        for offset in 0..4096 {
            self.child.insert(base + offset, 0);
        }
        self.set(base, 4, 0x1234_8086);
        self.set(base + 8, 4, class);
        self.set(base + 0xe, 1, u32::from(header));
        self.set(
            base + COMMAND_OFFSET,
            2,
            u32::from(regs::COMMAND_INTX_DISABLE),
        );
        self.set(base + 6, 2, 0x10);
        self.set(base + 0x34, 1, 0x40);
        self.set(base + 0x40, 2, 0x10);
        self.set(base + 0x42, 2, 2 | u32::from(kind) << 4);
    }
    pub fn bridge(&mut self, base: usize, primary: u8, secondary: u8, subordinate: u8) {
        self.add_function(base, 0x0604_0000, 1, if primary == 0 { 4 } else { 5 });
        self.set(
            base + 0x18,
            4,
            u32::from(primary) | u32::from(secondary) << 8 | u32::from(subordinate) << 16,
        );
        self.set(base + 0x20, 4, 0x8000_8000);
        self.set(base + 0x24, 4, 0x0000_fff0); // Disabled prefetch window.
    }
    /// A second NVMe controller at [`NVME2`], its BAR0 at [`NVME2_BASE`].
    pub fn second_nvme(&mut self) {
        self.add_function(NVME2, 0x0108_0200, 0, 0);
        self.set(NVME2 + 0x10, 4, NVME2_BASE as u32 | 4);
    }
}
pub struct Model {
    pub state: Arc<Mutex<State>>,
    pub drops: Arc<AtomicUsize>,
}
impl Default for Model {
    fn default() -> Self {
        let mut state = State {
            parent: BTreeMap::new(),
            child: BTreeMap::new(),
            pending: None,
            events: Vec::new(),
            ack_failure: None,
            stuck: None,
            pm_failure: false,
            pm_loses_bar: false,
            release_failure: false,
            epoch: 1,
            drained: false,
            mappings: BTreeMap::new(),
            next: 0,
        };
        for offset in 0..256 {
            state.parent.insert(offset, 0);
        }
        state.set_parent(0, 4, 0x28c0_8086);
        state.set_parent(regs::COMMAND, 2, u32::from(regs::COMMAND_INTX_DISABLE));
        for (index, (base, prefetch)) in [
            (0x4000_0000u64, false),
            (0x8000_0000, false),
            (0x9000_0000, true),
        ]
        .into_iter()
        .enumerate()
        {
            state.set_parent(
                0x10 + index as u16 * 8,
                4,
                base as u32 | 4 | if prefetch { 8 } else { 0 },
            );
            state.set_parent(0x14 + index as u16 * 8, 4, (base >> 32) as u32);
        }
        state.bridge(ROOT, 0, 1, 2);
        state.bridge(BRIDGE, 1, 2, 2);
        state.add_function(NVME, 0x0108_0200, 0, 0);
        state.set(NVME + 0x10, 4, NVME_BASE as u32 | 4);
        Self {
            state: Arc::new(Mutex::new(state)),
            drops: Arc::new(AtomicUsize::new(0)),
        }
    }
}
impl Drop for Model {
    fn drop(&mut self) {
        self.drops.fetch_add(1, Ordering::Relaxed);
    }
}
// SAFETY: isolated independent configuration storage. Child writes are posted
// until an equal-width model read flushes them; no production permission.
unsafe impl Registers for Model {
    fn parent_read(&mut self, offset: u16, width: u8) -> Result<u32, Error> {
        let state = self.state.lock().unwrap();
        let value = state.parent(offset, width);
        Ok(if state.release_failure && offset == regs::COMMAND {
            value | u32::from(regs::COMMAND_MEMORY)
        } else {
            value
        })
    }
    fn parent_write(&mut self, offset: u16, width: u8, value: u32) -> Result<(), Error> {
        let mut state = self.state.lock().unwrap();
        let old = state.parent(offset, width);
        state.set_parent(offset, width, value);
        if offset == regs::COMMAND {
            if (old | value) & u32::from(regs::COMMAND_MASTER) != 0 {
                state.drained = false;
            }
            state.events.push(Event::Parent(value as u16));
        }
        Ok(())
    }
    fn bar(&self, index: u8) -> (u64, u64, bool) {
        match index {
            0 => (0x4000_0000, 4 << 20, false),
            2 => (0x8000_0000, 0x100_0000, false),
            4 => (0x9000_0000, 0x100_0000, true),
            _ => unreachable!(),
        }
    }
    fn shadow_read(&mut self, _: usize) -> Result<u64, Error> {
        Err(Error::Mapping)
    }
    fn child_read(&mut self, offset: usize, width: u8) -> Result<u32, Error> {
        let mut state = self.state.lock().unwrap();
        if let Some((posted, posted_width, value)) = state.pending
            && posted == offset
            && posted_width == width
        {
            state.pending = None;
            if [ROOT, BRIDGE, NVME, NVME2]
                .into_iter()
                .any(|base| posted == base + COMMAND_OFFSET)
                && (state.child(posted, posted_width) | value) & u32::from(regs::COMMAND_MASTER)
                    != 0
            {
                state.drained = false;
            }
            let mut value = value;
            if [ROOT, BRIDGE, NVME, NVME2]
                .into_iter()
                .any(|base| posted == base + 0x84)
            {
                let old = state.child(posted, 2);
                value = value & !0x8000 | if value & 0x8000 == 0 { old & 0x8000 } else { 0 };
                if state.pm_failure {
                    value |= 3;
                }
                if state.pm_loses_bar {
                    state.set(NVME + 0x10, 4, 0);
                }
            }
            if let Some((offset, bits)) = state.stuck
                && offset == posted
            {
                value |= bits;
            }
            state.set(posted, posted_width, value);
            state.events.push(Event::Flushed(posted, posted_width));
        }
        let value = state.child(offset, width);
        Ok(if state.ack_failure == Some(offset) {
            value & !u32::from(regs::COMMAND_MASTER)
        } else {
            value
        })
    }
    fn child_write(&mut self, offset: usize, width: u8, value: u32) -> Result<(), Error> {
        {
            let mut state = self.state.lock().unwrap();
            assert!(state.pending.is_none());
            state.pending = Some((offset, width, value));
            state.events.push(Event::Posted(offset, width, value));
        }
        // The native Registers contract requires this posted-write
        // readback, independently of lifecycle's subsequent equality check.
        let _ = self.child_read(offset, width)?;
        Ok(())
    }
    fn table_read(&mut self, _: u64) -> Result<u32, Error> {
        Err(Error::Mapping)
    }
    fn table_write(&mut self, _: u64, _: u32) -> Result<(), Error> {
        Err(Error::Mapping)
    }
}
pub struct Backing {
    pub owner: Arc<()>,
}
// SAFETY: symbolic exclusive aperture in an isolated topology model; never
// passed to a production mapping provider or interpreted as RAM.
unsafe impl ResourceBacking for Backing {
    fn begin_mapping(
        &self,
    ) -> Result<crate::bus::pci::assignment::OpaqueUse, crate::bus::mmio::owned::Error> {
        // This topology model supports no CPU mapping (table_read/write also
        // refuse). Arc<()> labels cannot become an allocation/assignment use.
        Err(crate::bus::mmio::owned::Error::Unavailable)
    }

    fn physical_span(&self) -> PhysicalSpan {
        PhysicalSpan::new(0x8000_0000, 0x100_0000).unwrap()
    }
    fn kind(&self) -> ResourceKind {
        ResourceKind::Registers
    }
}
pub fn fixture(model: Model) -> (Host<Model>, FirmwareBar) {
    let host = match Host::initialize(model) {
        Ok(host) => host,
        Err((error, _)) => panic!("host: {error:?}"),
    };
    let bar = FirmwareBar {
        domain: host.domain,
        child: Address {
            domain: host.domain,
            bus: 2,
            device: 0,
            function: 0,
        },
        low: NVME_BASE as u32 | 4,
        high: 0,
        bus_start: NVME_BASE,
        bytes: 8192,
        window: host.windows[0],
        resource: Resource::new(Box::new(Backing {
            owner: Arc::new(()),
        })),
    };
    (host, bar)
}

/// The firmware BAR0 of the second controller [`State::second_nvme`] adds.
pub fn second_bar(host: &Host<Model>) -> FirmwareBar {
    FirmwareBar {
        domain: host.domain,
        child: Address {
            domain: host.domain,
            bus: 2,
            device: 1,
            function: 0,
        },
        low: NVME2_BASE as u32 | 4,
        high: 0,
        bus_start: NVME2_BASE,
        bytes: 8192,
        window: host.windows[0],
        resource: Resource::new(Box::new(Backing {
            owner: Arc::new(()),
        })),
    }
}
