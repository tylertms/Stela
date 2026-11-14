//! The host's xHCI controllers after probe: each controller's topology
//! owner ([`xhci::Host`]), looked at between engine slices so hotplug and
//! the resets class drivers queue (`Controller::queue_reset`) are serviced,
//! as Linux's `hub_event` services them. A controller that signals its
//! events wakes the engine and is polled once it has work
//! ([`xhci::Host::due`]); another is polled on a timer. A keyboard or
//! pointer that arrives joins the host input; another new device is
//! reported unused and is not started: a disk, network adapter, audio
//! function, Bluetooth controller or hub starts at the next boot. What a
//! new device leaves is kept until every device it came with is removed. The USB storage interfaces probe configured stay owned here
//! for the boot, with the disks the guest reaches through them.
//!
//! A USB4 host whose firmware runs the connection manager is looked at on the
//! same timer: each firmware event is reported, as Linux's
//! `icm_handle_notification` receives it. A software connection manager is
//! polled every slice, since its work waits on the host's control channel;
//! neither takes the host's interrupts. What it tunnels after probe works
//! for USB3 and DisplayPort, and a PCIe device stays unused until the next
//! boot's scan, as a USB device no consumer takes does.
//!
//! The controllers' registers and DMA pages are reached through the
//! loader's identity root, so each poll runs in it (`storage::RestoreRoot`).
use super::firmware_requests::Requests;
use super::storage::RestoreRoot;
use crate::output;
use alloc::{string::String, vec::Vec};
use stela_boot_core::{poll_timer::PollTimer, service_watch::Split};
use stela_drivers::{
    bus::{
        dma::RegionAllocator,
        thunderbolt::{
            icm,
            manager::{Admission, Platform, Report, Uuid},
        },
    },
    input::InputDevice,
    probe::native::usb4,
    usb::{
        storage::UsbStorage,
        xhci::{self, Found},
    },
};

/// Firmware events serviced per USB4 host per poll.
const USB4_EVENTS: usize = 16;

/// What a USB4 host's software connection manager asks of the mount.
pub(super) struct Usb4Platform {
    location: String,
    /// Whether an IOMMU confines the host and its tunnels' bridges.
    isolated: bool,
    /// The devices `--usb4-approve` lists.
    approved: Vec<Uuid>,
    tsc_per_ms: u64,
}

impl Usb4Platform {
    pub(super) fn new(location: &str, isolated: bool, approved: Vec<Uuid>, tsc_hz: u64) -> Self {
        Self {
            location: String::from(location),
            isolated,
            approved,
            tsc_per_ms: (tsc_hz / 1000).max(1),
        }
    }
}

impl Platform for Usb4Platform {
    fn now_ms(&self) -> u64 {
        super::host::rdtsc() / self.tsc_per_ms
    }

    fn report(&mut self, report: Report) {
        let location = &self.location;
        output::arguments(format_args!("HOST USB4 {location}: {report}\n"));
    }

    fn admit_pcie(&self, uuid: Uuid) -> Admission {
        if self.isolated {
            Admission::Isolated
        } else if self.approved.contains(&uuid) {
            Admission::Listed
        } else {
            Admission::Refused
        }
    }
}

/// Report one firmware event of the host at `location`.
pub(super) fn report_usb4(location: &str, family: icm::Family, event: &icm::Event) {
    match event {
        icm::Event::Notification(message) => match icm::Notification::decode(family, message) {
            Ok(notification) => {
                output::arguments(format_args!("HOST USB4 {location}: {notification}\n"))
            }
            Err(error) => output::arguments(format_args!(
                "HOST USB4 {location}: undecodable firmware event {:#x}: {error:?}\n",
                message.code
            )),
        },
        icm::Event::Discarded(error) => output::arguments(format_args!(
            "HOST USB4 {location}: firmware message discarded: {error:?}\n"
        )),
        icm::Event::DriverReady { request, .. } => output::arguments(format_args!(
            "HOST USB4 {location}: unrequested DRIVER_READY answer {request}\n"
        )),
    }
}

/// How often controllers without a signal are polled: Linux's root hub
/// status timer runs every 250 ms; a hub's status endpoint interval is at
/// most 255 ms.
const POLL_HZ: u64 = 16;

/// The pieces of a poll that are timed apart ([`HostUsb::split`]).
const PIECES: usize = 5;
const PIECE_NAMES: [&str; PIECES] = ["usb4", "between", "enter", "driver", "leave"];
const USB4: usize = 0;
const BETWEEN: usize = 1;
const ENTER: usize = 2;
const DRIVER: usize = 3;
const LEAVE: usize = 4;

/// The owners a poll found that no consumer takes, with the locations of
/// the devices they came with that are still attached.
struct Kept {
    attached: Vec<String>,
    _found: Found,
}

/// A USB4 host and what the polls last saw of it.
struct Usb4 {
    host: usb4::Host,
    /// Whether its connection manager still runs.
    running: bool,
    /// A software connection manager's PCIe tunnels.
    pcie_tunnels: usize,
}

/// Drop the `kept` class owners whose devices are all gone, which can stop
/// their transfers, unless another processor is inside the host drivers:
/// they keep for a later turn. `root` is the loader's.
fn drop_detached(kept: &mut Vec<Kept>, root: u64) {
    if kept.iter().any(|kept| kept.attached.is_empty()) {
        // SAFETY: the loader root captured by `storage::open`, as for the
        // controllers' polls; the service runs with the processor's pinned
        // record active.
        if let Some(_root) = unsafe { RestoreRoot::try_enter(root) } {
            kept.retain(|kept| !kept.attached.is_empty());
        }
    }
}

pub struct HostUsb {
    hosts: Vec<xhci::Host>,
    usb4: Vec<Usb4>,
    tsc_per_ms: u64,
    /// The probe's DMA pages, which enumeration of a new device draws on.
    dma: RegionAllocator,
    /// The boot's firmware, through the probe's request record.
    firmware: &'static Requests,
    root: u64,
    /// New devices' class owners that no consumer takes, kept while one of
    /// their devices is attached.
    kept: Vec<Kept>,
    /// The storage interfaces probe configured, whose disks the mount
    /// opened: their class owners outlive `open`.
    _storage: Vec<UsbStorage>,
    /// The timed poll's timer: a poll put off stays due ([`PollTimer`]).
    timer: PollTimer,
    /// Where the last poll's time went ([`Self::split`]).
    split: Split<PIECES>,
}

impl HostUsb {
    pub fn new(
        hosts: Vec<xhci::Host>,
        usb4: Vec<usb4::Host>,
        storage: Vec<UsbStorage>,
        dma: RegionAllocator,
        firmware: &'static Requests,
        root: u64,
        tsc_hz: u64,
    ) -> Self {
        Self {
            hosts,
            usb4: usb4
                .into_iter()
                .map(|host| Usb4 {
                    pcie_tunnels: match host.connection() {
                        usb4::Connection::Software { manager } => manager.pcie_tunnels(),
                        usb4::Connection::Firmware { .. } => 0,
                    },
                    host,
                    running: true,
                })
                .collect(),
            tsc_per_ms: (tsc_hz / 1000).max(1),
            dma,
            firmware,
            root,
            kept: Vec::new(),
            _storage: storage,
            timer: PollTimer::new(tsc_hz / POLL_HZ, super::host::rdtsc()),
            split: Split::new(PIECE_NAMES),
        }
    }

    /// Where the last poll's time went, for the report of a service turn
    /// that held its PE: the USB4 hosts, then over the xHCI controllers the
    /// time before each, entering the host drivers, inside them and leaving
    /// them. What the poll took beyond these is its reports of changes.
    pub(super) fn split(&self) -> &Split<PIECES> {
        &self.split
    }

    /// Whether a host driver found firmware the USB lacks.
    pub fn firmware_missing(&self) -> bool {
        self.firmware.missing()
    }

    /// Poll controllers with work or a due attachment timer, and check
    /// non-signalling controllers on the fallback interval. Return the
    /// keyboards and pointers that arrived.
    ///
    /// No poll waits for the host drivers' lock: the turn may be a step of
    /// a PE that runs guest code, and another processor can hold the lock
    /// for longer than a step may take. A controller whose poll the holder
    /// put off keeps its work, and the timed poll stays due, for the next
    /// turn.
    pub fn poll(&mut self) -> Vec<InputDevice> {
        let mut inputs = Vec::new();
        let now = super::host::rdtsc();
        let timed = self.timer.due(now);
        self.split.clear();
        let mut put_off = self.poll_usb4(now, timed);
        let mut stamp = super::host::rdtsc();
        self.split.add(USB4, now, stamp);
        let firmware = self.firmware;
        for host in &mut self.hosts {
            // Each piece of the poll ends at a stamp, so a turn that holds
            // its PE names the piece ([`Self::split`]).
            let mut lap = |split: &mut Split<PIECES>, piece: usize| {
                let now = super::host::rdtsc();
                split.add(piece, stamp, now);
                stamp = now;
            };
            lap(&mut self.split, BETWEEN);
            // SAFETY: the loader root captured by `storage::open`
            // keeps the engine window and maps the controller and DMA pages.
            // This service runs with the processor's pinned record active.
            // A busy owner leaves the driver state untouched.
            let root = unsafe { RestoreRoot::try_enter(self.root) };
            lap(&mut self.split, ENTER);
            let Some(root) = root else {
                put_off = true;
                continue;
            };
            let due = if host.signals() {
                host.due()
            } else {
                timed || host.attachment_due()
            };
            let changes = due.then(|| {
                let changes = host.poll_after_boot(&mut self.dma, firmware);
                // A new device's driver may have found its firmware missing;
                // the record is reached under the same lock.
                firmware.report();
                changes
            });
            lap(&mut self.split, DRIVER);
            drop(root);
            lap(&mut self.split, LEAVE);
            let Some(mut changes) = changes else {
                continue;
            };
            for device in &changes.removed {
                output::arguments(format_args!("USB removed {}\n", device.location));
                for kept in &mut self.kept {
                    kept.attached
                        .retain(|location| *location != device.location);
                }
            }
            drop_detached(&mut self.kept, self.root);
            let added = &mut changes.added;
            for (location, error) in &added.failures {
                output::arguments(format_args!("{}", xhci::failure_line(location, error)));
            }
            for (location, why) in &added.unused {
                output::arguments(format_args!("USB UNUSED {location}: {why}\n"));
            }
            inputs.extend(
                added
                    .inputs
                    .drain(..)
                    .map(|input| InputDevice::Usb(alloc::boxed::Box::new(input))),
            );
            for device in &added.devices {
                output::arguments(format_args!("USB added {}\n", device.location));
            }
            if !changes.added.devices.is_empty() {
                self.kept.push(Kept {
                    attached: changes
                        .added
                        .devices
                        .iter()
                        .map(|device| device.location.clone())
                        .collect(),
                    _found: changes.added,
                });
            }
        }
        // Owners a busy lock kept on an earlier turn.
        drop_detached(&mut self.kept, self.root);
        if timed {
            self.timer.ended(now, put_off);
        }
        inputs
    }

    /// Run each running USB4 host's software connection manager for up to a
    /// millisecond, and, when `timed`, service each firmware's events. A host whose
    /// transport faults is reported once and left stopped. Whether a host
    /// was put off because another processor is inside the host drivers.
    fn poll_usb4(&mut self, now: u64, timed: bool) -> bool {
        let now_ms = now / self.tsc_per_ms;
        let mut put_off = false;
        for usb4 in &mut self.usb4 {
            if !usb4.running {
                continue;
            }
            let (location, protocol) = usb4.host.protocol();
            match protocol {
                usb4::Connection::Firmware { .. } if !timed => {}
                usb4::Connection::Firmware { icm, .. } => {
                    for _ in 0..USB4_EVENTS {
                        let event = {
                            // SAFETY: as for the xHCI controllers below.
                            let Some(_root) = (unsafe { RestoreRoot::try_enter(self.root) }) else {
                                put_off = true;
                                break;
                            };
                            icm.poll(now_ms)
                        };
                        match event {
                            Ok(Some(event)) => report_usb4(location, icm.family(), &event),
                            Ok(None) => break,
                            Err(error) => {
                                output::arguments(format_args!(
                                    "HOST USB4 {location}: connection manager transport \
                                     stopped: {error:?}\n",
                                ));
                                usb4.running = false;
                                break;
                            }
                        }
                    }
                }
                usb4::Connection::Software { manager } => {
                    {
                        // SAFETY: as for the xHCI controllers below.
                        let Some(_root) = (unsafe { RestoreRoot::try_enter(self.root) }) else {
                            put_off = true;
                            continue;
                        };
                        loop {
                            usb4.running = manager.poll();
                            if !usb4.running
                                || manager.idle()
                                || super::host::rdtsc().wrapping_sub(now) >= self.tsc_per_ms
                            {
                                break;
                            }
                        }
                    }
                    let tunnels = manager.pcie_tunnels();
                    if tunnels > usb4.pcie_tunnels {
                        output::arguments(format_args!(
                            "HOST USB4 {location}: PCIe device added after the scan: \
                             unused until the next boot\n"
                        ));
                    }
                    usb4.pcie_tunnels = tunnels;
                }
            }
        }
        put_off
    }
}
