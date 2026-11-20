// SPDX-License-Identifier: GPL-2.0-or-later
//! Native xHCI adapter. Each direction reserves a monotonic private ring arena.
//! Controller keeps and reactivates the same owned rings only after acknowledged
//! stop/dequeue/drop and event retirement. The class never resets its allocator.
use super::{AudioDevice, Completion, Control, PacketStatus, Plan, Transport};
use crate::{
    audio::Direction,
    block::BlockError,
    bus::dma::{DmaAllocator, DmaBuffer, RegionAllocator},
    usb::descriptor::Endpoint,
    usb::xhci::{Device, IsoStatus, IsoTransfer, Shared, SlotId},
};
use alloc::{string::String, vec::Vec};

struct DiscoveryControl<'a> {
    shared: &'a Shared,
    id: SlotId,
    time: NativeTime,
}
// Four full-speed bInterval16 services are the largest class timeout. UAC3
// recovery (65535 * 50us), native control and feedback budgets are smaller.
const MAX_AUDIO_BUDGET_US: u64 = (8u64 << 15) * 125 * 4;
const CLOCK_REQUIREMENT: &str = "qualified representable USB audio timing";
#[derive(Clone, Copy)]
struct NativeTime {
    clock: crate::Clock,
    origin: u64,
}
impl NativeTime {
    fn new(clock: crate::Clock) -> Result<Self, BlockError> {
        clock
            .check_duration(MAX_AUDIO_BUDGET_US)
            .map_err(|_| BlockError::Unsupported(CLOCK_REQUIREMENT))?;
        let origin = clock
            .try_start()
            .map_err(|_| BlockError::Unsupported(CLOCK_REQUIREMENT))?;
        Ok(Self { clock, origin })
    }
    fn now_us(self) -> u64 {
        self.clock
            .try_elapsed_us(self.origin)
            .expect("admitted native USB audio clock")
    }
}
impl Control for DiscoveryControl<'_> {
    fn control(&mut self, r: [u8; 8], out: &[u8]) -> Result<Vec<u8>, BlockError> {
        self.shared.borrow_mut().device(self.id)?.control(r, out)
    }
    fn now_us(&self) -> u64 {
        self.time.now_us()
    }
    fn relax(&mut self) {
        crate::waiting();
        core::hint::spin_loop();
    }
}

struct Arena {
    _memory: DmaBuffer,
    allocator: RegionAllocator,
    endpoints: Vec<Endpoint>,
}
impl Arena {
    fn new(dma: &mut dyn DmaAllocator) -> Result<Self, BlockError> {
        let memory = dma.allocate(65536, 65536)?;
        // SAFETY: reserved exclusively for this adapter, kept for the boot.
        let allocator = unsafe { RegionAllocator::within(&memory) };
        Ok(Self {
            _memory: memory,
            allocator,
            endpoints: Vec::new(),
        })
    }
    fn released(&mut self) {
        self.endpoints.clear();
        // The controller keeps old ring addresses for the boot. Keep this
        // allocator monotonic even after Drop Context ACK; only PCM can recycle.
    }
}
/// Owns a native controller reference and the audio endpoint arenas.
pub struct Native {
    shared: Shared,
    id: SlotId,
    name: String,
    location: String,
    time: NativeTime,
    playback: Arena,
    capture: Arena,
    status_plan: Result<Option<super::status::StatusPlan>, super::Error>,
    connector_plan: Result<super::connector::ConnectorPlan, super::Error>,
}
impl Native {
    pub(super) fn connector_status(
        &mut self,
        terminal: u8,
    ) -> Result<super::ConnectorStatus, super::Error> {
        self.validate()?;
        self.connector_plan.clone()?.read(self, terminal)
    }
    pub(super) fn status_notifications(
        &self,
        dma: &mut dyn DmaAllocator,
    ) -> Result<Option<super::StatusNotifications>, super::Error> {
        self.shared.borrow_mut().device(self.id)?;
        self.status_plan
            .clone()?
            .map(|plan| super::StatusNotifications::new(self.shared.clone(), self.id, plan, dma))
            .transpose()
    }

    fn arena(&mut self, owner: Direction) -> &mut Arena {
        match owner {
            Direction::Playback => &mut self.playback,
            Direction::Capture => &mut self.capture,
        }
    }
}
impl Control for Native {
    fn control(&mut self, request: [u8; 8], out: &[u8]) -> Result<Vec<u8>, BlockError> {
        self.shared
            .borrow_mut()
            .device(self.id)?
            .control(request, out)
    }
    fn now_us(&self) -> u64 {
        self.time.now_us()
    }
    fn relax(&mut self) {
        crate::waiting();
        core::hint::spin_loop();
    }
}
impl Transport for Native {
    type Ticket = IsoTransfer;
    fn validate(&self) -> Result<(), BlockError> {
        self.shared.borrow_mut().device(self.id)?;
        Ok(())
    }
    fn preflight(&mut self, endpoints: &[Endpoint]) -> Result<(), BlockError> {
        self.shared
            .borrow_mut()
            .device(self.id)?
            .preflight_endpoints(endpoints)
    }
    fn configure(&mut self, owner: Direction, endpoints: &[Endpoint]) -> Result<(), BlockError> {
        let shared = self.shared.clone();
        let id = self.id;
        shared.borrow_mut().device(id)?;
        let arena = self.arena(owner);
        if !arena.endpoints.is_empty() {
            return Err(BlockError::Unsupported(
                "USB audio endpoint owner already configured",
            ));
        }
        // Keep one exclusive controller borrow from the authoritative recheck
        // through publication. A refusal cannot stage a foreign cleanup owner.
        let mut controller = shared.borrow_mut();
        let mut device = controller.device(id)?;
        device.preflight_endpoints(endpoints)?;
        arena.endpoints.extend_from_slice(endpoints);
        device.configure(endpoints, None, &mut arena.allocator)
    }
    fn deconfigure(&mut self, owner: Direction, endpoints: &[Endpoint]) -> Result<(), BlockError> {
        let shared = self.shared.clone();
        let id = self.id;
        shared.borrow_mut().device(id)?;
        let arena = self.arena(owner);
        if arena.endpoints.is_empty() {
            return Ok(());
        }
        if arena.endpoints != endpoints {
            return Err(BlockError::Unsupported(
                "USB audio endpoint teardown owner mismatch",
            ));
        }
        shared.borrow_mut().device(id)?.deconfigure(endpoints)?;
        arena.released();
        Ok(())
    }
    fn submit(
        &mut self,
        endpoint: &Endpoint,
        physical: u64,
        bytes: usize,
    ) -> Result<Self::Ticket, BlockError> {
        self.shared
            .borrow_mut()
            .device(self.id)?
            .submit_iso(endpoint.dci(), physical, bytes)
    }
    fn completed(&mut self, ticket: &Self::Ticket) -> Option<Result<Completion, BlockError>> {
        let mut controller = self.shared.borrow_mut();
        let mut device = match controller.device(self.id) {
            Ok(device) => device,
            Err(error) => return Some(Err(error)),
        };
        device.completed_iso(ticket).map(|r| {
            r.map(|c| Completion {
                bytes: c.bytes,
                status: match c.status {
                    IsoStatus::Success => PacketStatus::Success,
                    IsoStatus::Short => PacketStatus::Short,
                    IsoStatus::MissedService => PacketStatus::MissedService,
                    IsoStatus::Transaction => PacketStatus::Transaction,
                    IsoStatus::Babble => PacketStatus::Babble,
                    IsoStatus::Bandwidth => PacketStatus::Bandwidth,
                    IsoStatus::BufferOverrun => PacketStatus::BufferOverrun,
                },
            })
        })
    }
}
/// Native playback/capture owner published by xHCI and kept in probe::Devices.
pub type UsbAudio = AudioDevice<Native>;
impl UsbAudio {
    pub fn product_name(&self) -> &str {
        &self.transport.name
    }

    pub fn location(&self) -> &str {
        &self.transport.location
    }

    pub(crate) fn attach(
        shared: Shared,
        device: &Device,
        plan: Plan,
        dma: &mut dyn DmaAllocator,
    ) -> Result<Self, BlockError> {
        shared.borrow_mut().device(device.id)?;
        let time = NativeTime::new(shared.borrow().clock)?;
        let binding = plan
            .discover(&mut DiscoveryControl {
                shared: &shared,
                id: device.id,
                time,
            })
            .map_err(|error| match error {
                super::Error::Transport(e) => e,
                _ => BlockError::Unsupported("USB audio high-capability discovery failed"),
            })?;
        // Optional status admission never changes the existing PCM binding.
        // Keep an explicit refusal until the caller requests that service.
        let status_plan = super::status::StatusPlan::discover(&binding, &device.configuration);
        let connector_plan =
            super::connector::ConnectorPlan::discover(&binding, &device.configuration);
        let native = Native {
            shared,
            id: device.id,
            location: device.location.clone(),
            name: device.name(),
            time,
            playback: Arena::new(dma)?,
            capture: Arena::new(dma)?,
            status_plan,
            connector_plan,
        };
        Self::new(native, binding, dma).map_err(|error| match error {
            super::Error::Transport(e) => e,
            _ => BlockError::Unsupported("USB audio allocation failed"),
        })
    }
}

#[cfg(test)]
pub(crate) mod tests {
    pub(crate) mod generation;
    use super::*;
    use crate::usb::xhci;
    use alloc::rc::Rc;
    use core::cell::RefCell;

    #[test]
    fn native_audio_clock_admission_precedes_discovery_ep0_and_dma() {
        struct NoAllocation;
        impl DmaAllocator for NoAllocation {
            fn allocate(&mut self, _: usize, _: usize) -> Result<DmaBuffer, BlockError> {
                panic!("invalid audio clock reached DMA allocation")
            }
        }
        for rate in [0, u64::MAX, u64::MAX / 2 / MAX_AUDIO_BUDGET_US + 1] {
            let mut backing = vec![0u64; 256 * 1024];
            let start = backing.as_mut_ptr() as u64;
            // SAFETY: exclusive native peer RAM outlives controller retirement.
            let mut dma = unsafe { RegionAllocator::new(start..start + backing.len() as u64 * 8) };
            let raw = crate::usb::audio::test_configuration();
            let (mut controller, model) = xhci::model::high_speed_media_controller(&mut dma, &raw);
            controller.set_configuration(1, 1).unwrap();
            let device = Device {
                id: controller.test_device_id(1).unwrap(),
                slot: 1,
                speed: xhci::HIGH_SPEED,
                location: "native audio invalid-clock preflight peer".into(),
                descriptor: [18, 1, 0, 2, 0, 0, 0, 64, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1],
                configurations: vec![raw.clone()],
                configuration: raw.clone(),
                manufacturer: None,
                product: None,
                serial: None,
                language: None,
                vendor_ethernet_tcr: None,
                vendor_ethernet_revision: None,
            };
            let all = crate::usb::descriptor::interfaces(&raw);
            let plan = Plan::parse(&all[0], &all, &raw, xhci::HIGH_SPEED).unwrap();
            controller.clock.ticks_per_microsecond = rate;
            controller.forbid_mmio();
            let setup = model.borrow().setup.clone();
            let commands = model.borrow().seen.clone();
            let before = backing.clone();
            assert!(matches!(
                UsbAudio::attach(
                    Rc::new(RefCell::new(controller)),
                    &device,
                    plan,
                    &mut NoAllocation
                ),
                Err(BlockError::Unsupported(CLOCK_REQUIREMENT))
            ));
            assert_eq!(model.borrow().setup, setup);
            assert_eq!(model.borrow().seen, commands);
            assert_eq!(backing, before);
        }
    }

    #[test]
    fn native_audio_keeps_raw_origin_for_wrapping_elapsed_clock() {
        let time = NativeTime {
            clock: crate::Clock {
                ticks_per_microsecond: 3,
            },
            origin: u64::MAX - 9,
        };
        // Exercise the same shared conversion primitive as try_elapsed_us with
        // deterministic raw observations; no physical RDTSC wrap is asserted.
        assert_eq!(time.clock.elapsed_at(time.origin, 14), Ok(8));
        assert_eq!(time.clock.elapsed_at(time.origin, 20), Ok(10));
    }
    #[test]
    fn native_recheck_never_records_or_retires_foreign_pending_dma() {
        for halted in [false, true] {
            let mut memory = alloc::vec![0u64;256*1024];
            let base = memory.as_mut_ptr() as u64;
            // SAFETY: backing RAM is exclusive and outlives every model/owner.
            let mut dma = unsafe { RegionAllocator::new(base..base + memory.len() as u64 * 8) };
            let foreign = dma.allocate(512, 512).unwrap();
            let shared = xhci::model::controller_with_unresolved_td(
                &mut dma,
                foreign.physical(),
                512,
                5,
                halted,
            );
            let mut native = Native {
                shared: shared.clone(),
                id: shared.borrow().test_device_id(1).unwrap(),
                name: String::new(),
                location: String::new(),
                time: NativeTime::new(shared.borrow().clock).unwrap(),
                playback: Arena::new(&mut dma).unwrap(),
                capture: Arena::new(&mut dma).unwrap(),
                status_plan: Ok(None),
                connector_plan: Err(super::super::Error::Unsupported(
                    "fixture has no connector binding",
                )),
            };
            let ep = Endpoint {
                address: 0x82,
                attributes: 1,
                max_packet: 192,
                interval: 4,
                companion: None,
            };
            let refused = BlockError::Unsupported("xHCI endpoint cancellation failed");
            assert_eq!(
                shared.borrow_mut().preflight_endpoints(1, &[ep]),
                Err(refused.clone())
            );
            let before = memory.clone();
            assert!(native.configure(Direction::Playback, &[ep]).is_err());
            assert!(native.playback.endpoints.is_empty());
            // Cleanup of an unstaged candidate is inert; the pre-existing TD
            // cannot be cancelled, poisoned or dropped by the audio owner.
            native.deconfigure(Direction::Playback, &[ep]).unwrap();
            assert_eq!(
                shared.borrow_mut().preflight_endpoints(1, &[ep]),
                Err(refused)
            );
            assert_eq!(memory, before);
        }
    }
}
