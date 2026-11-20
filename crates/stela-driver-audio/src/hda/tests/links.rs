// SPDX-License-Identifier: GPL-2.0-only
//! links behavior tests.
use super::*;

#[test]
fn clock_selection_preserves_other_links() {
    for (frequencies, expected) in [
        (0x3f, 2u8),
        (0x3b, 3),
        (0x33, 1),
        (0x31, 4),
        (0x21, 5),
        (1, 0),
    ] {
        for initially_on in [false, true] {
            let mut fixture = Fixture::new();
            let model = extended_model();
            model.borrow_mut().set(0xc40, frequencies, 4);
            model.borrow_mut().set(
                0xc44,
                if initially_on {
                    (1 << 16) | (1 << 23)
                } else {
                    0
                },
                4,
            );
            let (mut function, state) = reset_pci(&model);
            let prepared = pci::tests::modern_prepared_model(function.address);
            let registers =
                // SAFETY: The prepared PCI fixture and ModelRegisters share the same BDF/BAR.
                // The Rc-backed model is driven synchronously and outlives this adapter.
                unsafe { prepared.bind_registers(&mut function, ModelRegisters(model.clone())) }
                    .unwrap();
            let timer = model.borrow().clock.clone();
            // SAFETY: The bound PCI fixture owns the register model exclusively. fixture
            // keeps the aligned DMA backing, and polling is the only model execution.
            let mut controller = unsafe {
                Controller::new(registers, timer, &mut fixture.allocator, modern_config())
                    .map_err(|failure| failure.error)
            }
            .unwrap();
            assert_eq!(
                controller.link_clock(),
                Ok(Some(LinkClock {
                    encoding: expected,
                    mhz: 6u16 << expected
                }))
            );
            assert_eq!(model.borrow().get(0xc44, 4) & 15, u32::from(expected));
            assert_eq!(
                model.borrow().get(0xc44, 4) & ((1 << 16) | (1 << 23)),
                (1 << 16) | (1 << 23)
            );
            assert_eq!(model.borrow().get(0xc84, 4), 3 | (1 << 16) | (1 << 23));
            assert!(
                !model
                    .borrow()
                    .writes
                    .iter()
                    .any(|(offset, _, _)| matches!(*offset, 0xc84 | 0x804 | 0x704 | 0x1004))
            );
            assert_eq!(state.borrow().writes.len(), 2);
            controller.reinitialize().unwrap();
            assert_eq!(controller.link_clock().unwrap().unwrap().encoding, expected);
            controller.verb(verb()).unwrap();
        }
    }
}

#[test]
fn raptor_lake_capability_layout_is_accepted() {
    // 8086:7a50: version-0 headers and an alternate second link. Only the
    // first link is clocked.
    let mut fixture = Fixture::new();
    let model = extended_model();
    {
        let mut m = model.borrow_mut();
        m.set(0x500, 0x0001_0800, 4);
        m.set(0xc00, 0x0002_0700, 4);
        m.set(0xc80, 1 << 28, 4);
    }
    let (mut function, _state) = reset_pci(&model);
    let prepared = pci::tests::modern_prepared_model(function.address);
    let registers =
        // SAFETY: The prepared PCI fixture and ModelRegisters share the same BDF/BAR.
        // The Rc-backed model is driven synchronously and outlives this adapter.
        unsafe { prepared.bind_registers(&mut function, ModelRegisters(model.clone())) }
            .unwrap();
    let timer = model.borrow().clock.clone();
    // SAFETY: The bound PCI fixture owns the register model exclusively. fixture
    // keeps the aligned DMA backing, and polling is the only model execution.
    let controller = unsafe {
        Controller::new(registers, timer, &mut fixture.allocator, modern_config())
            .map_err(|failure| failure.error)
    }
    .unwrap();
    assert_eq!(controller.link_clock().unwrap().unwrap().encoding, 2);
    assert_eq!(model.borrow().get(0xc84, 4), 3 | (1 << 16) | (1 << 23));
}

#[test]
fn invalid_link_preflight() {
    for variant in 0..17 {
        let mut fixture = Fixture::new();
        let model = extended_model();
        match variant {
            0 => model.borrow_mut().set(0x804, 1 << 30, 4),
            1 => model.borrow_mut().set(0x804, 1, 4),
            2 => model.borrow_mut().set(0x704, 1, 4),
            3 => model.borrow_mut().set(0x1004, 1, 4),
            4 => model.borrow_mut().set(0x14, 0x501, 2),
            5 => model.borrow_mut().set(0x14, 0x40, 2),
            6 => model.borrow_mut().set(0x1000, 0x1005_5000, 4),
            7 => model.borrow_mut().set(0x1000, 0x1005_0804, 4), //Back edge into PP body.
            8 => model.borrow_mut().set(0xc04, u32::MAX, 4),
            9 => model.borrow_mut().set(0xc40, (1 << 28) | 0x3f, 4),
            10 => model.borrow_mut().set(0xc40, 0, 4),
            11 => model.borrow_mut().set(0x500, 0x1006_0800, 4),
            12 => model.borrow_mut().set(0x500, 0x2001_0800, 4),
            13 => model.borrow_mut().set(0x804, u32::MAX, 4),
            14 => model.borrow_mut().set(0x1000, 0x1005_0700, 4), //Repeated SPB.
            15 => model.borrow_mut().bytes.truncate(0xc60),
            _ => model.borrow_mut().set(0x504, 1, 4), //Undecoded GTS geometry.
        }
        let (mut function, state) = reset_pci(&model);
        let prepared = pci::tests::modern_prepared_model(function.address);
        let registers =
            // SAFETY: The prepared PCI fixture and ModelRegisters share the same BDF/BAR.
            // The Rc-backed model is driven synchronously and outlives this adapter.
            unsafe { prepared.bind_registers(&mut function, ModelRegisters(model.clone())) }
                .unwrap();
        let timer = model.borrow().clock.clone();
        let remaining = fixture.allocator.remaining();
        assert!(
            // SAFETY: The bound PCI fixture owns the register model exclusively. fixture
            // keeps the aligned DMA backing, and polling is the only model execution.
            unsafe {
                Controller::new(registers, timer, &mut fixture.allocator, modern_config())
                    .map_err(|failure| failure.error)
            }
            .is_err(),
            "variant {variant}"
        );
        assert_eq!(
            fixture.allocator.remaining(),
            remaining,
            "variant {variant}"
        );
        assert!(model.borrow().writes.is_empty(), "variant {variant}");
        assert!(state.borrow().writes.is_empty(), "variant {variant}");
    }
}

#[test]
fn link_failure_restores_power() {
    for variant in 0..9 {
        let mut fixture = Fixture::new();
        let model = extended_model();
        match variant {
            0 => model.borrow_mut().link_down_stuck = true,
            1 => model.borrow_mut().link_up_stuck = true,
            2 => model.borrow_mut().reject_link_clock = true,
            3 => model.borrow_mut().set(0xc44, 1 << 16, 4),
            4 => model.borrow_mut().set(0xc44, 7 | (1 << 16) | (1 << 23), 4),
            5 => {
                let mut m = model.borrow_mut();
                m.control_failure_after_write = Some((0xc44, 2, 4));
                m.control_read_failure = Some((0xc44, 4));
            }
            6 => model.borrow_mut().link_clock_lost_on_power_up = true,
            7 => {
                let mut m = model.borrow_mut();
                m.set(0xc40, 4, 4);
                m.set(0xc44, 3 | (1 << 16) | (1 << 23), 4);
            }
            _ => {
                let mut m = model.borrow_mut();
                m.control_failure_after_write = Some((0xc44, 0x10002, 4));
                m.control_read_failure = Some((0xc44, 4));
            }
        }
        let (mut function, _state) = reset_pci(&model);
        let prepared = pci::tests::modern_prepared_model(function.address);
        let registers =
            // SAFETY: The prepared PCI fixture and ModelRegisters share the same BDF/BAR.
            // The Rc-backed model is driven synchronously and outlives this adapter.
            unsafe { prepared.bind_registers(&mut function, ModelRegisters(model.clone())) }
                .unwrap();
        let timer = model.borrow().clock.clone();
        let result =
            // SAFETY: The bound PCI fixture owns the register model exclusively. fixture
            // keeps the aligned DMA backing, and polling is the only model execution.
            unsafe { Controller::new(registers, timer, &mut fixture.allocator, modern_config()).map_err(|failure|failure.error) };
        let error = match result {
            Err(error) => error,
            Ok(_) => panic!("variant {variant} succeeded"),
        };
        assert_eq!(
            error,
            match variant {
                0 => Error::Timeout(Phase::LinkPowerDown),
                1 => Error::Timeout(Phase::LinkPowerUp),
                2 | 6 => Error::LinkClockRejected { offset: 0xc44 },
                3 => Error::LinkPowerUncertain { offset: 0xc44 },
                4 | 7 => Error::UnsupportedLink { offset: 0xc44 },
                _ => Error::RegisterUnavailable { offset: 0xc44 },
            }
        );
        assert!(model.borrow().commands.is_empty());
        assert_eq!(model.borrow().get(CORBCTL, 1) & u32::from(COMMAND_RUN), 0);
        assert_eq!(model.borrow().get(RIRBCTL, 1) & u32::from(COMMAND_RUN), 0);
        if variant == 1 {
            assert!(
                model
                    .borrow()
                    .writes
                    .iter()
                    .filter(|(offset, value, _)| *offset == 0xc44 && value & (1 << 16) != 0)
                    .count()
                    >= 2
            );
        }
        if variant == 5 || variant == 8 {
            assert!(model.borrow().control_failed_reads >= 2);
        }
        if variant == 0 || variant == 1 || variant == 2 {
            assert!(
                model
                    .borrow()
                    .writes
                    .iter()
                    .any(|(offset, value, _)| *offset == 0xc44 && value & (1 << 16) != 0)
            );
        }
    }
}

#[test]
fn dsp_ownership_blocks_reset_and_shutdown() {
    let mut fixture = Fixture::new();
    let model = extended_model();
    let (mut function, state) = reset_pci(&model);
    let prepared = pci::tests::modern_prepared_model(function.address);
    let registers =
        // SAFETY: The prepared PCI fixture and ModelRegisters share the same BDF/BAR.
        // The Rc-backed model is driven synchronously and outlives this adapter.
        unsafe { prepared.bind_registers(&mut function, ModelRegisters(model.clone())) }.unwrap();
    let timer = model.borrow().clock.clone();
    let mut controller =
        // SAFETY: The bound PCI fixture owns the register model exclusively. fixture
        // keeps the aligned DMA backing, and polling is the only model execution.
        unsafe { Controller::new(registers, timer, &mut fixture.allocator, modern_config()).map_err(|failure|failure.error) }
            .unwrap();
    let id = controller
        .allocate_stream(Direction::Playback, pcm(), 128, 2, &mut fixture.allocator)
        .unwrap();
    controller.start(id).unwrap();
    model.borrow_mut().set(0x804, 1 << 30, 4);
    let generation = controller.token;
    let writes = model.borrow().writes.len();
    let pci_writes = state.borrow().writes.len();
    assert_eq!(
        controller.reinitialize(),
        Err(Error::ProcessingOwned { offset: 0x804 })
    );
    assert_eq!(
        controller.shutdown(),
        Err(Error::ProcessingOwned { offset: 0x804 })
    );
    assert_eq!(model.borrow().writes.len(), writes);
    assert_eq!(state.borrow().writes.len(), pci_writes);
    assert_eq!(controller.token, generation);
    assert!(controller.streams[usize::from(id.descriptor_index())].is_some());
    assert_eq!(controller.link_clock(), Err(Error::NeedsReset));
    assert_eq!(controller.write_pcm(id, 0, &[1]), Err(Error::NeedsReset));
    assert_ne!(
        model
            .borrow()
            .get(stream_base(id.descriptor_index()) + SD_CTL, 1)
            & u32::from(SD_RUN),
        0
    );
}

#[test]
fn failed_reinitialize_stales_link_clock() {
    for initially_on in [false, true] {
        let mut fixture = Fixture::new();
        let model = extended_model();
        model.borrow_mut().set(
            0xc44,
            3 | if initially_on {
                (1 << 16) | (1 << 23)
            } else {
                0
            },
            4,
        );
        let (mut function, _state) = reset_pci(&model);
        let prepared = pci::tests::modern_prepared_model(function.address);
        let registers =
            // SAFETY: The prepared PCI fixture and ModelRegisters share the same BDF/BAR.
            // The Rc-backed model is driven synchronously and outlives this adapter.
            unsafe { prepared.bind_registers(&mut function, ModelRegisters(model.clone())) }
                .unwrap();
        let timer = model.borrow().clock.clone();
        let mut controller =
            // SAFETY: The bound PCI fixture owns the register model exclusively. fixture
            // keeps the aligned DMA backing, and polling is the only model execution.
            unsafe { Controller::new(registers, timer, &mut fixture.allocator, modern_config()).map_err(|failure|failure.error) }
                .unwrap();
        assert_eq!(
            controller.link_clock(),
            Ok(Some(LinkClock {
                encoding: 3,
                mhz: 48
            }))
        );
        assert_eq!(
            model
                .borrow()
                .writes
                .iter()
                .filter(|(offset, _, _)| *offset == 0xc44)
                .count(),
            usize::from(!initially_on)
        );
        let id = controller
            .allocate_stream(Direction::Playback, pcm(), 128, 2, &mut fixture.allocator)
            .unwrap();
        controller.start(id).unwrap();
        let remaining = fixture.allocator.remaining();
        model.borrow_mut().set(0xc44, (1 << 16) | (1 << 23), 4);
        model.borrow_mut().reject_link_clock = true;
        assert_eq!(
            controller.reinitialize(),
            Err(Error::LinkClockRejected { offset: 0xc44 })
        );
        assert_eq!(controller.link_clock(), Err(Error::NeedsReset));
        assert_eq!(controller.start(id), Err(Error::InvalidStream));
        assert_eq!(controller.verb(verb()), Err(Error::NeedsReset));
        assert_eq!(fixture.allocator.remaining(), remaining);
        model.borrow_mut().reject_link_clock = false;
        controller.reinitialize().unwrap();
        assert_eq!(
            controller.link_clock(),
            Ok(Some(LinkClock {
                encoding: 2,
                mhz: 24
            }))
        );
    }
}
