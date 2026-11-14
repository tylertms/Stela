// SPDX-License-Identifier: GPL-2.0-only
//! Devices that arrive after boot, brought up in steps from the guest's
//! service turns, as Linux's hub_event thread brings them up apart from the
//! interrupt that found them. A port's reset, its device's address,
//! descriptors and configuration, a hub's port power and power-good wait
//! with each of its children, and an input device's start each end a step
//! at every wait, under the same deadlines as at boot, so no turn holds its
//! processor for one of them. One connection runs at a time; the others
//! wait their turn, and the hubs' own status work waits for it, so two
//! owners never share a hub's control endpoint.
use super::*;
use core::task::Poll;
use stela_driver_base::stepped::Stepped;

/// How long one poll steps a connection.
const STEP_US: u64 = 1000;

/// A connection waiting its turn.
pub(super) struct Queued {
    pub(super) location: String,
    pub(super) parent: Option<SlotId>,
    pub(super) reset: enumerate::Reset,
    /// The root port, and its status when the change was seen.
    pub(super) root: Option<(u8, u32)>,
}

/// What a connection found.
struct Attached {
    found: Found,
    connected: enumerate::Connected,
}

/// The connection under way.
pub(super) struct Attaching {
    access: Rc<Access<'static>>,
    work: Stepped<'static, Attached>,
    root: Option<(u8, u32)>,
}

impl Host {
    /// Whether a connection after boot is under way or waiting.
    pub(super) fn attaching(&self) -> bool {
        self.attaching.is_some() || !self.queued.is_empty()
    }

    /// Whether a connection under way or waiting holds root port `port`.
    pub(super) fn holds_root(&self, port: u8) -> bool {
        self.attaching
            .as_ref()
            .is_some_and(|attaching| attaching.root.is_some_and(|(held, _)| held == port))
            || self
                .queued
                .iter()
                .any(|queued| queued.root.is_some_and(|(held, _)| held == port))
    }

    /// Step the connection under way for up to [`STEP_US`], starting the
    /// next one waiting when none is; what one that ends found joins
    /// `changes`.
    pub(super) fn step_attaching(&mut self, dma: &mut dyn DmaAllocator, changes: &mut Changes) {
        if self.attaching.is_none() {
            let Some(queued) = self.queued.pop_front() else {
                return;
            };
            self.attaching = Some(self.start(queued));
        }
        let clock = self.controller.borrow().clock;
        let Ok(deadline) = clock.try_deadline(STEP_US) else {
            return;
        };
        let attaching = self.attaching.as_mut().unwrap();
        let ended = attaching.access.lend(dma, || {
            loop {
                if let Poll::Ready(attached) = attaching.work.step() {
                    return Some(attached);
                }
                if deadline.passed() {
                    return None;
                }
            }
        });
        let Some(Attached {
            mut found,
            connected,
        }) = ended
        else {
            return;
        };
        let root = self.attaching.take().and_then(|attaching| attaching.root);
        if let (Some((port, status)), Some(speed)) = (root, connected.speed) {
            // As `root` baselines a reset that acknowledged a device.
            let controlled = CCS | CAS | PED | (PORT_SPEED_FIELD << PORT_SPEED_SHIFT) | PLS;
            self.roots[usize::from(port - 1)] =
                status & !controlled | CCS | PED | u32::from(speed) << PORT_SPEED_SHIFT;
        }
        self.register_hubs(&found.devices, dma, &mut changes.added.failures);
        for device in &found.devices {
            if !self.devices.iter().any(|known| known.id == device.id) {
                self.devices.push(device.clone());
            }
        }
        let added = &mut changes.added;
        added.devices.append(&mut found.devices);
        added.inputs.append(&mut found.inputs);
        added.unused.append(&mut found.unused);
        added.failures.append(&mut found.failures);
    }

    fn start(&self, queued: Queued) -> Attaching {
        let access = Rc::new(Access::stepped(self.controller.clone()));
        let a = access.clone();
        let Queued {
            location,
            parent,
            reset,
            root,
        } = queued;
        let work = Stepped::new(async move {
            let mut found = Found::default();
            let connected = Controller::connect_with(
                &a,
                location,
                parent,
                reset,
                &mut found.devices,
                &mut found.failures,
            )
            .await;
            for device in found.devices.clone() {
                bind::bind_after_boot_with(&a, &device, &mut found).await;
            }
            Attached { found, connected }
        });
        Attaching { access, work, root }
    }
}
