//! usb-storage's `delay_use` on every transport: Linux sends a Bulk-Only
//! device its first class request (GET_MAX_LUN, from `usb_stor_scan_dwork`)
//! one second after the probe that configured it, as some devices fail
//! requests before. Each interface's second runs from its own
//! configuration, so a start that configures several devices, on one
//! controller or many, waits once for the latest. UAS has no such wait.
use super::{
    Binding, BlockError, DmaAllocator, EhciBulk, Interface, Shared, String, UsbDisk, UsbStorage,
    Vec, discover_binding, discover_ehci,
};
use crate::{
    Clock, Deadline,
    usb::xhci::{Device, SlotId},
};

pub(in crate::usb) const SETTLE_US: u64 = 1_000_000;

/// The transport half of a configured Bulk-Only interface.
pub(in crate::usb) enum Configured {
    Xhci {
        shared: Shared,
        device: Device,
        binding: Binding,
    },
    Ehci {
        io: EhciBulk,
        setting: Interface,
        serial: String,
    },
}

/// A configured Bulk-Only interface waiting out [`SETTLE_US`] from its
/// configuration.
pub struct Settling {
    /// `None` when the clock could not time the second: it then refused
    /// every deadline of this controller already, and the interface is due
    /// at once.
    due: Option<Deadline>,
    location: String,
    interface: Configured,
}

/// What discovery found once the interfaces settled.
#[derive(Default)]
pub struct Settled {
    pub storage: Vec<UsbStorage>,
    pub disks: Vec<UsbDisk>,
    pub failures: Vec<(String, BlockError)>,
}

/// The end of an interface's second, taken right after its configuration.
pub(in crate::usb) fn configured(clock: Clock) -> Option<Deadline> {
    clock.try_deadline(SETTLE_US).ok()
}

impl Settling {
    /// The interface, kept until `due` from [`configured`].
    pub(in crate::usb) fn new(
        due: Option<Deadline>,
        location: String,
        interface: Configured,
    ) -> Self {
        Self {
            due,
            location,
            interface,
        }
    }

    pub fn due(&self) -> bool {
        self.due.is_none_or(|due| due.passed())
    }

    /// The xHCI device the interface is on.
    pub(in crate::usb) fn xhci_device(&self) -> Option<SlotId> {
        match &self.interface {
            Configured::Xhci { device, .. } => Some(device.id),
            Configured::Ehci { .. } => None,
        }
    }
}

fn discover<'c>(
    location: String,
    interface: Configured,
    dma: Option<&mut (dyn DmaAllocator + 'c)>,
    out: &mut Settled,
) {
    crate::working(&location);
    let found = match interface {
        Configured::Xhci {
            shared,
            device,
            binding,
        } => {
            // A device gone meanwhile is dropped: the topology reports it.
            if shared.borrow().validate_device_id(device.id).is_err() {
                return;
            }
            match dma {
                Some(dma) => discover_binding(shared, &device, binding, dma, &mut out.failures),
                None => Err(BlockError::Unsupported(
                    "xHCI storage discovery without DMA",
                )),
            }
        }
        Configured::Ehci {
            io,
            setting,
            serial,
        } => discover_ehci(io, &setting, location.clone(), serial, &mut out.failures),
    };
    match found {
        Ok((owner, disks)) => {
            out.storage.push(owner);
            out.disks.extend(disks);
        }
        Err(error) => out.failures.push((location, error)),
    }
}

/// Discover the interfaces whose second passed. With `wait`, first wait
/// until every one's has, once for all: only a device start may, as it
/// holds its core for at most that second. The wait ends with the device
/// start budget too. EHCI discovery takes no `dma`; xHCI's needs it.
pub fn settle<'c>(
    settling: &mut Vec<Settling>,
    mut dma: Option<&mut (dyn DmaAllocator + 'c)>,
    wait: bool,
) -> Settled {
    if wait && settling.iter().any(|entry| !entry.due()) {
        crate::working("USB storage settle");
        while settling.iter().any(|entry| !entry.due()) {
            crate::waiting();
            core::hint::spin_loop();
        }
    }
    let mut settled = Settled::default();
    let mut at = 0;
    while let Some(entry) = settling.get(at) {
        if !entry.due() {
            at += 1;
            continue;
        }
        let Settling {
            location,
            interface,
            ..
        } = settling.remove(at);
        discover(location, interface, dma.as_deref_mut(), &mut settled);
    }
    settled
}
