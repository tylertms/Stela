// SPDX-License-Identifier: GPL-2.0-only
// Copyright (c) 2026 Stela contributors
//! Packet io cases.
use super::*;

/// An RTL8169S over `registers` and `memory`, its rings in `memory`.
fn modelled(registers: &mut [u32], memory: &mut [u64], signalled: bool) -> Realtek {
    use crate::bus::dma::RegionAllocator;
    let start = memory.as_mut_ptr() as u64;
    // SAFETY: the caller owns both allocations for the driver's lifetime.
    let (device, mut dma) = unsafe {
        (
            Mmio::new(registers.as_mut_ptr() as u64, size_of_val(&registers[..])),
            RegionAllocator::new(start..start + (memory.len() * 8) as u64),
        )
    };
    let revision = revision::detect(0x0080_0000).unwrap();
    Realtek {
        device,
        clock: crate::timing::model(Clock {
            ticks_per_microsecond: 1,
        }),
        receive: ManuallyDrop::new(Ring::new(&mut dma, true).unwrap()),
        transmit: ManuallyDrop::new(Ring::new(&mut dma, true).unwrap()),
        mac: Mac([2, 0, 0, 0, 0, 1]),
        link: RuntimeLink::default(),
        revision,
        warnings: Vec::new(),
        function: 0,
        location: "model".into(),
        signalled,
        receive_watch: recovery::ReceiveWatch::default(),
    }
}

/// The acknowledgement an empty receive makes writes back the whole status
/// it read, as Linux's `rtl_ack_events` does: the receive ring's exhaustion
/// (RxDescUnavail, bit 4) and the receive FIFO's overflow (RxFIFOOver,
/// bit 6) among the events, so a receiver that ran out starts again. The
/// 2.5G chips' status is 32 bits wide. Nothing is written when no event is
/// pending, or when the function has left the bus and reads all ones.
#[test]
fn acknowledge_writes_back_the_whole_status_at_its_width() {
    let time = Time::default();
    for (version, register, width, pending) in [
        (
            46,
            INTERRUPT_STATUS,
            2,
            u32::from(RX_OK | LINK_CHANGE | 1 << 2 | 1 << 4 | 1 << 6),
        ),
        (61, NBASE_INTERRUPT_STATUS, 4, 0x0021_0051),
    ] {
        let model = Model::ready(&time);
        let status = |value: u32| {
            let mut s = model.state.borrow_mut();
            s.bytes[register..register + width].copy_from_slice(&value.to_le_bytes()[..width]);
            s.writes.clear();
        };
        let hardware = Hardware::new(&model, &time, revision(version), 0);
        status(pending);
        assert_eq!(hardware.acknowledge(), pending as u16, "{version}");
        assert_eq!(
            model.state.borrow().writes,
            [(register, pending, width as u8)],
            "{version}"
        );
        for quiet in [0, u32::MAX] {
            status(quiet);
            assert_eq!(hardware.acknowledge(), 0, "{version} {quiet:#x}");
            assert!(
                model.state.borrow().writes.is_empty(),
                "{version} {quiet:#x}"
            );
        }
    }
}

#[test]
fn transmit_without_carrier_is_refused_and_leaves_the_ring_untouched() {
    let mut registers = vec![0u32; 64];
    let mut memory = vec![0u64; 262_144];
    let mut net = modelled(&mut registers, &mut memory, false);
    net.transmit.initialize(false);
    let descriptors = net.transmit.descriptors.bytes().to_vec();
    assert_eq!(net.transmit(&[0; 60]), Err(LinkError::NoCarrier));
    assert_eq!(net.transmit.descriptors.bytes(), descriptors);
    assert_eq!(registers[TX_POLL / 4], 0);
}

#[test]
fn h_receive_has_a_timer_even_when_msi_is_enabled_and_silent() {
    let mut registers = vec![0u32; 64];
    let mut memory = vec![0u64; 262_144];
    let mut net = modelled(&mut registers, &mut memory, true);
    assert_eq!(net.due_us(), None); // Other revisions retain their policy.
    net.revision = revision(46);
    for signalled in [true, false] {
        net.signalled = signalled;
        let before = net.clock.now_us();
        let due = net.due_us().unwrap();
        let after = net.clock.now_us();
        assert!(due >= before.saturating_add(10_000));
        assert!(due <= after.saturating_add(10_000));
    }
    net.receive.initialize(true);
    net.transmit.initialize(false);
    net.transmit.transmit(&[0; 60]).unwrap();
    assert_eq!(net.transmit.owned(), 1);
    net.receive.deliver(&[0x5a; 60]); // A packet with no interrupt raised.
    assert_eq!(net.receive.receive(), Some(vec![0x5a; 60]));
    let snapshot = net.diagnostics();
    assert!(
        snapshot.contains("tx=1 pending=1 rx=1 dropped=0"),
        "{snapshot}"
    );
    // A read-only snapshot must not release a queued TX descriptor.
    assert_eq!(net.transmit.owned(), 1);
    assert_eq!(net.device.read16(INTERRUPT_STATUS), 0);
}

/// A descriptor the controller still owns makes the next send busy at once,
/// and the ring is left as it was.
#[test]
fn a_descriptor_the_controller_still_owns_makes_a_send_busy() {
    let mut registers = vec![0u32; 64];
    let mut memory = vec![0u64; 262_144];
    let mut net = modelled(&mut registers, &mut memory, false);
    net.transmit.initialize(false);
    net.transmit.transmit(&[1; 60]).unwrap();
    let owned = net.transmit.descriptors.read::<u32>(0);
    net.transmit.descriptors.write(DESCRIPTOR, owned); // the next slot, after the one sent
    let descriptors = net.transmit.descriptors.bytes().to_vec();
    assert_eq!(net.transmit.transmit(&[2; 60]), Err(LinkError::Busy));
    assert_eq!(net.transmit.descriptors.bytes(), descriptors);
}

/// A dropped adapter closes its receive and asks for the MAC reset, as
/// `rtl8169_down` does; one whose function left the bus reads all ones and
/// gets no write.
#[test]
fn dropping_an_adapter_resets_it_and_a_lost_one_gets_no_write() {
    let mut registers = vec![0u32; 64];
    let mut memory = vec![0u64; 262_144];
    registers[RX_CONFIG / 4] = RX_FIFO_THRESHOLD | RX_ACCEPT_MASK;
    drop(modelled(&mut registers, &mut memory, false));
    assert_eq!(registers[RX_CONFIG / 4], RX_FIFO_THRESHOLD);
    assert_ne!(registers[COMMAND / 4].to_le_bytes()[COMMAND % 4] & RESET, 0);
    let mut registers = vec![u32::MAX; 64];
    drop(modelled(&mut registers, &mut memory, false));
    assert!(registers.iter().all(|&word| word == u32::MAX));
}

/// The stack's path refills its own buffer in place across the ring wrap,
/// and leaves it unchanged when nothing arrived or the link failed.
#[test]
fn receive_reusing_refills_the_callers_buffer_across_the_wrap() {
    let mut registers = vec![0u32; 64];
    let mut memory = vec![0u64; 262_144];
    let mut net = modelled(&mut registers, &mut memory, false);
    net.receive.initialize(true);
    let mut reused = Vec::with_capacity(MAX_FRAME);
    let storage = reused.as_ptr();
    let mut last = Vec::new();
    for sequence in 0..(DESCRIPTORS * 2 + 3) {
        let frame = vec![sequence as u8; 60 + sequence % 64];
        net.receive.deliver(&frame);
        if sequence % 2 == 0 {
            assert_eq!(net.receive(), Ok(Some(frame.clone())));
        } else {
            assert_eq!(net.receive_reusing(&mut reused), Ok(true));
            assert_eq!(reused, frame);
            assert_eq!(reused.as_ptr(), storage);
            last = frame;
        }
    }
    assert_eq!(net.receive_reusing(&mut reused), Ok(false));
    assert_eq!(reused, last);
    net.link.failure = Some(BlockError::Timeout);
    assert!(net.receive_reusing(&mut reused).is_err());
    assert_eq!(reused, last);
}
