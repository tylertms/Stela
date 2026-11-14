//! The modern (virtio 1.x) PCI transport the virtio drivers share: the
//! register regions, reset, feature negotiation, split virtqueue setup and
//! the MSI-X vector the device signals.
use super::{
    dma::DmaBuffer,
    irq::MsiMessage,
    mmio::Mmio,
    pci::{self, CAPABILITY_VENDOR, ConfigSpace, Function},
};
use crate::{Handoff, block::BlockError, registers};
use core::cell::Cell;

pub const VENDOR: u16 = 0x1af4;

const COMMON: u8 = 1;
const NOTIFY: u8 = 2;
const DEVICE: u8 = 4;

// Common configuration registers.
const DEVICE_FEATURE_SELECT: usize = 0x00;
const DEVICE_FEATURE: usize = 0x04;
const DRIVER_FEATURE_SELECT: usize = 0x08;
const DRIVER_FEATURE: usize = 0x0c;
const CONFIG_VECTOR: usize = 0x10;
const DEVICE_STATUS: usize = 0x14;
const CONFIG_GENERATION: usize = 0x15;
const QUEUE_SELECT: usize = 0x16;
const QUEUE_SIZE: usize = 0x18;
const QUEUE_VECTOR: usize = 0x1a;
const QUEUE_ENABLE: usize = 0x1c;
const QUEUE_NOTIFY_OFF: usize = 0x1e;
const QUEUE_DESCRIPTORS: usize = 0x20;
const QUEUE_DRIVER: usize = 0x28;
const QUEUE_DEVICE: usize = 0x30;
const NO_VECTOR: u16 = 0xffff;
/// An MSI-X table entry's bytes.
const MSIX_ENTRY_BYTES: u64 = 16;

const ACKNOWLEDGE: u8 = 1;
const DRIVER: u8 = 2;
const DRIVER_OK: u8 = 4;
const FEATURES_OK: u8 = 8;
const FAILED: u8 = 0x80;

const FEATURE_VERSION_1: u64 = 1 << 32;

pub const DESCRIPTOR_NEXT: u16 = 1;
pub const DESCRIPTOR_WRITE: u16 = 2;

pub const AVAIL: usize = 0x100;
pub const USED: usize = 0x200;
/// How long a device may take to reset or to complete a request.
pub const TIMEOUT_US: u64 = 10_000_000;

/// A modern virtio PCI function's register regions, reset and acknowledged
/// (virtio 1.2 section 4.1.4 and 3.1.1).
pub struct Modern {
    common: Mmio,
    pub device: Mmio,
    notify: (Option<pci::Bar>, u64, u64, u32),
    bars: [Option<pci::Bar>; 6],
    /// The MSI-X entry configuration changes and the signalled queues
    /// send, [`NO_VECTOR`] when the device is polled.
    vector: u16,
    /// The queues that send `vector`, one bit each.
    signalled: u32,
    /// Set when the device answered a vector write with [`NO_VECTOR`].
    refused: Cell<bool>,
}

impl Modern {
    /// Observed device configuration version (virtio 1.2 sections 2.5 and
    /// 4.1.4.3). Readers bracket their own same-device field accesses with it;
    /// this byte alone supplies no coherent snapshot or current-state claim.
    pub fn config_generation(&self) -> u8 {
        self.common.read8(CONFIG_GENERATION)
    }

    pub fn new(
        function: &mut Function,
        handoff: &Handoff,
        minimum: &(impl pci::power::Timer + ?Sized),
    ) -> Result<Self, BlockError> {
        function.power_up(minimum)?;
        let bars = function.bars()?;
        let mut common = None;
        let mut device = None;
        let mut notify = None;
        for (offset, id) in function.capabilities()? {
            if id != CAPABILITY_VENDOR {
                continue;
            }
            let config = function.config();
            let base = u16::from(offset);
            let kind = config.read8(base + 3)?;
            let bar = bars
                .get(usize::from(config.read8(base + 4)?))
                .copied()
                .flatten();
            let start = u64::from(config.read32(base + 8)?);
            let length = u64::from(config.read32(base + 12)?);
            let region = || registers(handoff, bar, start, length);
            match kind {
                COMMON if common.is_none() => common = Some(region()?),
                DEVICE if device.is_none() => device = Some(region()?),
                NOTIFY if notify.is_none() => {
                    notify = Some((bar, start, length, config.read32(base + 16)?))
                }
                _ => {}
            }
        }
        let (Some(common), Some(device), Some(notify)) = (common, device, notify) else {
            return Err(BlockError::Unsupported(
                "virtio device without modern registers",
            ));
        };
        function.enable_polled()?;
        common.write8(DEVICE_STATUS, 0);
        if !handoff
            .clock
            .wait(TIMEOUT_US, || common.read8(DEVICE_STATUS) == 0)
        {
            return Err(BlockError::Timeout);
        }
        common.write8(DEVICE_STATUS, ACKNOWLEDGE | DRIVER);
        Ok(Self {
            common,
            device,
            notify,
            bars,
            vector: NO_VECTOR,
            signalled: 0,
            refused: Cell::new(false),
        })
    }

    /// Send `message` from MSI-X entry 0 for configuration changes and for
    /// the used buffers of `queues`, as Linux's `vp_find_vqs_msix` shares
    /// vectors, before [`Self::negotiate`] and [`Self::queue`] program them.
    /// A function without MSI-X stays polled.
    pub fn signal(
        &mut self,
        function: &mut Function,
        handoff: &Handoff,
        message: MsiMessage,
        queues: &[u16],
    ) -> Result<(), BlockError> {
        let Some(msix) = function.interrupts()?.msix else {
            return Ok(());
        };
        let bar = self
            .bars
            .get(usize::from(msix.table_bar))
            .copied()
            .flatten();
        let table = registers(handoff, bar, u64::from(msix.table_offset), MSIX_ENTRY_BYTES)?;
        function.enable_msix(msix, || {
            table.write32(0, message.address as u32);
            table.write32(4, (message.address >> 32) as u32);
            table.write32(8, message.data);
            table.write32(12, 0);
            Ok(())
        })?;
        self.vector = 0;
        self.signalled = queues.iter().fold(0, |mask, &queue| {
            mask | 1u32.checked_shl(queue.into()).unwrap_or(0)
        });
        Ok(())
    }

    /// Whether the device sends [`Self::signal`]'s message for every event
    /// it was given.
    pub fn signals(&self) -> bool {
        self.vector != NO_VECTOR && !self.refused.get()
    }

    /// Writes `vector` to the vector register at `offset`; a device that
    /// reads back [`NO_VECTOR`] instead refused it (virtio 1.2 section
    /// 4.1.5.1.2).
    fn route(&self, offset: usize, vector: u16) {
        self.common.write16(offset, vector);
        if vector != NO_VECTOR && self.common.read16(offset) != vector {
            self.refused.set(true);
        }
    }

    /// Accepts the offered features among `wanted`; VERSION_1 is required.
    pub fn negotiate(&self, wanted: u64) -> Result<u64, BlockError> {
        let common = &self.common;
        let features = |select: u32| {
            common.write32(DEVICE_FEATURE_SELECT, select);
            u64::from(common.read32(DEVICE_FEATURE)) << (32 * select)
        };
        let offered = features(0) | features(1);
        if offered & FEATURE_VERSION_1 == 0 {
            common.write8(DEVICE_STATUS, FAILED);
            return Err(BlockError::Unsupported("virtio device without VERSION_1"));
        }
        let accepted = offered & (FEATURE_VERSION_1 | wanted);
        for select in 0..2u32 {
            common.write32(DRIVER_FEATURE_SELECT, select);
            common.write32(DRIVER_FEATURE, (accepted >> (32 * select)) as u32);
        }
        common.write8(DEVICE_STATUS, ACKNOWLEDGE | DRIVER | FEATURES_OK);
        if common.read8(DEVICE_STATUS) & FEATURES_OK == 0 {
            common.write8(DEVICE_STATUS, FAILED);
            return Err(BlockError::Unsupported(
                "virtio device refused its features",
            ));
        }
        self.route(CONFIG_VECTOR, self.vector);
        Ok(accepted)
    }

    /// Enables queue `index` of at most `entries` entries in `memory`
    /// (descriptors at 0, the driver ring at [`AVAIL`], the device ring
    /// at [`USED`]), returning its size and notify register.
    pub fn queue(
        &self,
        handoff: &Handoff,
        index: u16,
        entries: u16,
        memory: &DmaBuffer,
    ) -> Result<(u16, Mmio), BlockError> {
        self.queue_at(handoff, index, entries, memory, AVAIL, USED)
    }

    /// [`Self::queue`] with the driver and device rings at `driver` and
    /// `device` bytes into `memory`: VERSION_1 places each part on its own
    /// (virtio 1.2 section 2.7), so a queue past 16 entries, whose
    /// descriptors outgrow [`AVAIL`], has its rings after them.
    pub fn queue_at(
        &self,
        handoff: &Handoff,
        index: u16,
        entries: u16,
        memory: &DmaBuffer,
        driver: usize,
        device: usize,
    ) -> Result<(u16, Mmio), BlockError> {
        let common = &self.common;
        common.write16(QUEUE_SELECT, index);
        let maximum = common.read16(QUEUE_SIZE);
        if maximum == 0 {
            return Err(BlockError::Unsupported("virtio queue is absent"));
        }
        let entries = entries.min(1 << (15 - maximum.leading_zeros()));
        common.write16(QUEUE_SIZE, entries);
        let signalled = 1u32
            .checked_shl(index.into())
            .is_some_and(|bit| self.signalled & bit != 0);
        self.route(
            QUEUE_VECTOR,
            if signalled { self.vector } else { NO_VECTOR },
        );
        common.write64(QUEUE_DESCRIPTORS, memory.bus_address());
        common.write64(QUEUE_DRIVER, memory.bus_address() + driver as u64);
        common.write64(QUEUE_DEVICE, memory.bus_address() + device as u64);
        let (bar, start, length, multiplier) = self.notify;
        let offset = u64::from(common.read16(QUEUE_NOTIFY_OFF)) * u64::from(multiplier);
        if offset + 2 > length {
            return Err(BlockError::Unsupported(
                "virtio notify register outside its region",
            ));
        }
        let notify = registers(handoff, bar, start + offset, 2)?;
        common.write16(QUEUE_ENABLE, 1);
        Ok((entries, notify))
    }

    pub fn ready(&self) {
        self.common.write8(
            DEVICE_STATUS,
            ACKNOWLEDGE | DRIVER | FEATURES_OK | DRIVER_OK,
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::boxed::Box;

    /// A transport over plain memory: every register reads back what was
    /// written, and the device offers VERSION_1.
    fn modelled(common: &mut [u64], vector: u16) -> Modern {
        // SAFETY: the caller owns the memory for the transport's lifetime.
        let common = unsafe { Mmio::new(common.as_mut_ptr() as u64, size_of_val(common)) };
        common.write32(DEVICE_FEATURE, 1);
        Modern {
            common,
            device: common,
            notify: (None, 0, 0, 0),
            bars: [None; 6],
            vector,
            signalled: 0,
            refused: Cell::new(false),
        }
    }

    #[test]
    fn generation_reads_the_common_byte_without_touching_neighbors() {
        let mut memory = [0xa5a5_a5a5_a5a5_a5a5; 8];
        let transport = modelled(&mut memory, NO_VECTOR);
        transport.common.write8(CONFIG_GENERATION, 0x39);
        let before = memory;
        assert_eq!(transport.config_generation(), 0x39);
        assert_eq!(memory, before);
        assert_eq!(transport.common.read8(DEVICE_STATUS), 0xa5);
        assert_eq!(transport.common.read16(QUEUE_SELECT), 0xa5a5);
    }

    /// Negotiation sends configuration changes to the signal vector, or to
    /// none on a polled device.
    #[test]
    fn negotiation_routes_configuration_changes_to_the_signal_vector() {
        for (vector, signals) in [(0, true), (NO_VECTOR, false)] {
            let mut memory: Box<[u64]> = vec![0u64; 8].into_boxed_slice();
            let transport = modelled(&mut memory, vector);
            transport.negotiate(0).unwrap();
            assert_eq!(transport.common.read16(CONFIG_VECTOR), vector);
            assert_eq!(transport.signals(), signals);
        }
    }
}
