// SPDX-License-Identifier: GPL-2.0-only
//! Repeated unpublished lifetimes, not manufactured cache-return tokens.
use super::*;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Addresses {
    output: u64,
    input: u64,
    control: u64,
    ring: u64,
}

fn addresses(c: &mut Controller, slot: u8) -> Addresses {
    let state = c.slot(slot).unwrap();
    Addresses {
        output: state._output.physical(),
        input: state.input.physical(),
        control: state.control.physical(),
        ring: state.endpoints[1].as_ref().unwrap().ring.buffer.physical(),
    }
}

struct ObserveAddress {
    inner: Box<dyn Registers>,
    hardware: Rc<RefCell<Hardware>>,
    addresses: Addresses,
    context: usize,
    segment: usize,
    observed: Rc<RefCell<usize>>,
}

impl Registers for ObserveAddress {
    fn read32(&mut self, offset: usize) -> u32 {
        self.inner.read32(offset)
    }
    fn write32(&mut self, offset: usize, value: u32) {
        if offset == 0x1000 && self.hardware.borrow().pending_command_kind() == ADDRESS_DEVICE {
            let a = self.addresses;
            assert_eq!(read::<u64>(self.hardware.borrow().dcbaa + 8), a.output);
            assert!(
                bytes(a.output, 32 * self.context)
                    .iter()
                    .all(|byte| *byte == 0)
            );
            assert!(
                bytes(a.control, CONTROL_BYTES)
                    .iter()
                    .all(|byte| *byte == 0)
            );
            let mut expected_ring = vec![0u8; self.segment];
            expected_ring[(RING_TRBS - 1) * TRB..(RING_TRBS - 1) * TRB + 8]
                .copy_from_slice(&a.ring.to_le_bytes());
            assert_eq!(bytes(a.ring, self.segment), expected_ring);
            assert_eq!(
                read::<u64>(a.input + (2 * self.context + 8) as u64),
                a.ring | 1
            );
            // These are the only initialized input words for this root HS peer.
            let mut expected_input = vec![0u8; 33 * self.context];
            let mut word = |at: usize, value: u32| {
                expected_input[at..at + 4].copy_from_slice(&value.to_le_bytes());
            };
            word(4, 3);
            word(self.context, u32::from(HIGH_SPEED) << 20 | 1 << 27);
            word(self.context + 4, 1 << 16);
            word(2 * self.context + 4, 3 << 1 | CONTROL_TYPE << 3 | 64 << 16);
            word(2 * self.context + 16, 8);
            expected_input[2 * self.context + 8..2 * self.context + 16]
                .copy_from_slice(&(a.ring | 1).to_le_bytes());
            assert_eq!(bytes(a.input, 33 * self.context), expected_input);
            *self.observed.borrow_mut() += 1;
        }
        self.inner.write32(offset, value);
    }
    fn write64(&mut self, offset: usize, value: u64) {
        self.inner.write64(offset, value);
    }
}

struct Unused;
impl Registers for Unused {
    fn read32(&mut self, _: usize) -> u32 {
        unreachable!()
    }
    fn write32(&mut self, _: usize, _: u32) {
        unreachable!()
    }
}

#[test]
fn unpublished_retry_reuses_one_resource_set_after_disable_and_physical_reset() {
    for (context, profile) in [
        (32, dma::Profile::SPEC),
        (64, dma::Profile::SPEC),
        (32, dma::Profile::pci(0x1d17, 0x9202)),
        (64, dma::Profile::pci(0x1d17, 0x9202)),
    ] {
        let mut backing = vec![0u64; 512 * 1024];
        let (mut c, mut old, hardware, mut dma) =
            unpublished_profile(&mut backing, HIGH_SPEED, context, 1, profile);
        let slot = old.hardware_slot();
        let original = addresses(&mut c, slot);
        let remaining = dma.remaining();
        let mut previous = vec![old];
        let observed = Rc::new(RefCell::new(0));
        let inner = core::mem::replace(&mut c.registers, Box::new(Unused));
        c.registers = Box::new(ObserveAddress {
            inner,
            hardware: hardware.clone(),
            addresses: original,
            context,
            segment: c.dma_profile.segment_bytes(),
            observed: observed.clone(),
        });
        for retry in 1..=96 {
            // EP0 DMA dirties control storage and advances its producer.
            assert_eq!(
                c.device(old)
                    .unwrap()
                    .control(setup(0x80, GET_DESCRIPTOR, 1 << 8, 0, 18), &[])
                    .unwrap()[0],
                18
            );
            assert_ne!(
                c.slot(slot).unwrap().endpoints[1]
                    .as_ref()
                    .unwrap()
                    .ring
                    .index,
                0
            );
            c.disable_slot(slot, Ownership::Vacant).unwrap();
            assert!(c.slots[usize::from(slot)].is_none());
            assert_eq!(c.dcbaa.read::<u64>(8), 0);
            assert!(
                c.power.retained(slot),
                "Disable is not physical current retirement"
            );
            // Poison every byte of the exclusively retired allocation, proving
            // reuse clears more than the context fields it overwrites.
            copy(original.output, &vec![0xa5; 32 * context]);
            copy(original.input, &vec![0xa5; 33 * context]);
            copy(original.control, &vec![0xa5; CONTROL_BYTES]);
            copy(original.ring, &vec![0xa5; c.dma_profile.segment_bytes()]);
            c.reset_root_device(attachment(HIGH_SPEED)).unwrap();
            assert!(!c.power.retained(slot));
            assert_eq!(hardware.borrow().reset_writes, retry);
            assert_eq!(c.address(attachment(HIGH_SPEED), &mut dma).unwrap(), slot);
            let current = c.device_id(slot).unwrap();
            assert!(!previous.contains(&current));
            let accesses = hardware.borrow().accesses;
            for retired in &previous {
                assert!(c.device(*retired).is_err());
            }
            previous.push(current);
            assert!(c.current_configuration(slot).is_err());
            assert_eq!(hardware.borrow().accesses, accesses);
            assert_eq!(addresses(&mut c, slot), original);
            assert_eq!(dma.remaining(), remaining, "retry allocated fresh DMA");
            assert_eq!(*observed.borrow(), retry);
            old = current;
        }
        assert!(
            hardware.borrow().event_count > RING_TRBS,
            "actual event ring must wrap"
        );
        // Publishing a consumer removes this generation from the cache domain.
        c.claim_native(old).unwrap();
        // Its port still holds the device's current grant, so the ID that
        // comes back stays unaddressed.
        c.disable_slot(slot, Ownership::Retired).unwrap();
        assert!(c.fresh_slot().is_err());
        assert_eq!(c.slot_ownership[usize::from(slot)], Ownership::Tombstone);
        assert_eq!(addresses(&mut c, slot), original);
    }
}

struct HaltControl {
    inner: Box<dyn Registers>,
    hardware: Rc<RefCell<Hardware>>,
    slot: u8,
}
impl Registers for HaltControl {
    fn read32(&mut self, offset: usize) -> u32 {
        self.inner.read32(offset)
    }
    fn write32(&mut self, offset: usize, value: u32) {
        self.inner.write32(offset, value);
        if offset == 0x1000 + 4 * usize::from(self.slot) && value == 1 {
            // The peer has consumed the real Setup/Data/Status TD and written
            // its event. Give that transfer a HALTED/STALL outcome.
            self.hardware
                .borrow_mut()
                .stall_completed_control(self.slot);
        }
    }
    fn write64(&mut self, offset: usize, value: u64) {
        self.inner.write64(offset, value);
    }
}

#[test]
fn unpublished_retry_failures_keep_original_resources_and_refuse_readdress() {
    for context in [32, 64] {
        for (fault, stalled) in [
            (Fault::Stop(1), false),
            (Fault::SilentStop(1), false),
            (Fault::Dequeue(1), false),
            (Fault::Disable, false),
            (Fault::WrongDisableSlot, false),
            (Fault::SilentDisable, false),
            (Fault::Stop(1), true),
        ] {
            let mut backing = vec![0u64; 512 * 1024];
            let (mut c, id, hardware, mut dma) = unpublished(&mut backing, HIGH_SPEED, context, 1);
            let slot = id.hardware_slot();
            let original = addresses(&mut c, slot);
            if stalled {
                let inner = core::mem::replace(&mut c.registers, Box::new(Unused));
                c.registers = Box::new(HaltControl {
                    inner,
                    hardware: hardware.clone(),
                    slot,
                });
                hardware.borrow_mut().fault = fault;
                assert!(
                    c.device(id)
                        .unwrap()
                        .control(setup(0x80, GET_DESCRIPTOR, 1 << 8, 0, 18), &[])
                        .is_err()
                );
            } else {
                c.device(id)
                    .unwrap()
                    .control(setup(0x80, GET_DESCRIPTOR, 1 << 8, 0, 18), &[])
                    .unwrap();
            }
            let control = bytes(original.control, CONTROL_BYTES);
            let input = bytes(original.input, 33 * context);
            let ring = bytes(original.ring, c.dma_profile.segment_bytes());
            let remaining = dma.remaining();
            hardware.borrow_mut().fault = fault;
            assert!(
                c.disable_slot(slot, Ownership::Vacant).is_err(),
                "{fault:?}"
            );
            assert_eq!(addresses(&mut c, slot), original);
            let expected = match (fault, stalled) {
                (_, true) => RESET_ENDPOINT,
                (Fault::Stop(_) | Fault::SilentStop(_), false) => STOP_ENDPOINT,
                (Fault::Dequeue(_), false) => SET_DEQUEUE,
                (Fault::Disable | Fault::WrongDisableSlot | Fault::SilentDisable, false) => {
                    DISABLE_SLOT
                }
                _ => unreachable!(),
            };
            assert!(hardware.borrow().commands_seen.contains(&(
                expected,
                slot,
                if expected == DISABLE_SLOT { 0 } else { 1 }
            )));
            if expected != DISABLE_SLOT {
                assert!(
                    !hardware
                        .borrow()
                        .commands_seen
                        .iter()
                        .any(|(kind, _, _)| *kind == DISABLE_SLOT)
                );
            }
            assert_eq!(c.slot_ownership[usize::from(slot)], Ownership::Enabled);
            assert_eq!(c.dcbaa.read::<u64>(8), original.output);
            assert_eq!(bytes(original.control, CONTROL_BYTES), control);
            assert_eq!(bytes(original.input, 33 * context), input);
            assert_eq!(bytes(original.ring, c.dma_profile.segment_bytes()), ring);
            assert!(c.power.retained(slot));
            assert!(c.device(id).is_err());
            let seen = hardware.borrow().commands_seen.clone();
            assert!(c.address(attachment(HIGH_SPEED), &mut dma).is_err());
            assert_eq!(hardware.borrow().commands_seen, seen);
            assert_eq!(dma.remaining(), remaining);
        }
    }
}

#[test]
fn unpublished_data_or_retired_ring_owner_cannot_return_ep0_resources() {
    for (dci, retired) in [(0, false), (2, false), (2, true)] {
        let mut backing = vec![0u64; 512 * 1024];
        let (mut c, id, hardware, mut dma) = unpublished(&mut backing, HIGH_SPEED, 32, 1);
        let slot = id.hardware_slot();
        let original = addresses(&mut c, slot);
        let ring = Ring::new(&mut dma).unwrap();
        let owned = ring.buffer.physical();
        if retired {
            c.slot(slot).unwrap().retired_rings[dci] = Some(ring);
        } else {
            c.slot(slot).unwrap().endpoints[dci] = Some(Transfer {
                ring,
                streams: None,
                iso: None,
                is_in: false,
                poisoned: false,
                enabled: true,
                in_flight: 0,
                stopped: false,
            });
        }
        let before = backing.clone();
        let accesses = hardware.borrow().accesses;
        assert!(c.disable_slot(slot, Ownership::Vacant).is_err());
        assert_eq!(hardware.borrow().accesses, accesses);
        assert_eq!(backing, before);
        assert_eq!(addresses(&mut c, slot), original);
        let retained = c.slot(slot).unwrap();
        let actual = if retired {
            retained.retired_rings[dci]
                .as_ref()
                .unwrap()
                .buffer
                .physical()
        } else {
            retained.endpoints[dci]
                .as_ref()
                .unwrap()
                .ring
                .buffer
                .physical()
        };
        assert_eq!(actual, owned);
        assert!(!retained.retiring);
    }
}

#[test]
fn unpublished_post_disable_health_failure_does_not_return_resources() {
    for context in [32, 64] {
        let mut backing = vec![0u64; 512 * 1024];
        let (mut c, id, hardware, mut dma) = unpublished(&mut backing, HIGH_SPEED, context, 1);
        let slot = id.hardware_slot();
        let original = addresses(&mut c, slot);
        let remaining = dma.remaining();
        let inner = core::mem::replace(&mut c.registers, Box::new(Unused));
        c.registers = Box::new(FatalAfterDisable {
            inner,
            hardware: hardware.clone(),
            status_reads: None,
        });
        let error = c.disable_slot(slot, Ownership::Vacant).unwrap_err();
        assert_eq!(error.0, "post-Disable Slot controller status");
        assert!(matches!(error.1, BlockError::Device { status } if status & HCE != 0));
        assert_eq!(
            hardware.borrow().commands_seen.last(),
            Some(&(DISABLE_SLOT, slot, 0))
        );
        assert_eq!(addresses(&mut c, slot), original);
        assert_eq!(c.dcbaa.read::<u64>(8), original.output);
        assert!(c.current_slot(id).is_err());
        assert!(c.power.retained(slot));
        let seen = hardware.borrow().commands_seen.clone();
        assert!(c.address(attachment(HIGH_SPEED), &mut dma).is_err());
        assert_eq!(hardware.borrow().commands_seen, seen);
        assert_eq!(dma.remaining(), remaining);
    }
}
