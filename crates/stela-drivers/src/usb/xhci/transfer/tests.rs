use super::*;
use crate::bus::dma::TestRegion;
use crate::usb::xhci::model::*;

#[test]
fn shared_preflight_samples_status_once_and_still_checks_every_endpoint() {
    let mut dma = TestRegion::new(64 * 1024 * 8);
    let (mut controller, model) = cancellation_controller(&mut dma);
    assert!(!controller.event_posted());
    controller.quiet_drains = 0;
    let before = model.borrow().status_reads;
    controller.endpoints_ready_owned(1, &[1, 3, 5]).unwrap();
    assert_eq!(model.borrow().status_reads - before, 1);

    model
        .borrow_mut()
        .event(0, INCOMPATIBLE_DEVICE, TRANSFER_EVENT, 5, 0);
    assert_eq!(
        controller.endpoints_ready_owned(1, &[1, 3, 5]),
        Err(BlockError::Device {
            status: INCOMPATIBLE_DEVICE,
        })
    );
    assert!(controller.endpoint_poisoned(1, 5));
    assert!(controller.endpoint_ready_owned(1, 3).is_ok());

    model.borrow_mut().fatal_status = HSE;
    assert_eq!(
        controller.endpoints_ready_owned(1, &[1, 3]),
        Err(BlockError::Device { status: HSE })
    );
}

#[test]
fn strings_use_the_first_listed_language_and_default_to_english() {
    let mut dma = TestRegion::new(32 * 1024 * 8);
    let (mut controller, model) = unconfigured_cancellation_controller(&mut dma);
    for (descriptor, language) in [
        (vec![6, STRING_DESCRIPTOR, 0x07, 0x04, 0x09, 0x04], 0x0407),
        (vec![2, STRING_DESCRIPTOR], LANGUAGE_EN_US),
        (vec![4, DEVICE_DESCRIPTOR, 0x07, 0x04], LANGUAGE_EN_US),
    ] {
        model.borrow_mut().descriptors = vec![(STRING_DESCRIPTOR, 0, descriptor)];
        assert_eq!(controller.language(1), Some(language));
    }
    // A descriptor too short to hold a language means no strings at all.
    model.borrow_mut().descriptors = vec![(STRING_DESCRIPTOR, 0, vec![1])];
    assert_eq!(controller.language(1), None);
}

#[test]
fn cancellation_stops_running_resets_halted_and_clears_late_events() {
    for initial in [1, 2, 3] {
        let mut dma = TestRegion::new(64 * 1024 * 8);
        let (mut controller, model) = cancellation_controller(&mut dma);
        let buffer = dma.allocate(512, 512).unwrap();
        let trb = controller.submit(1, 3, buffer.physical(), 512).unwrap();
        model.borrow_mut().set_state(3, initial);
        model.borrow_mut().stale = Some(trb);
        let pointer = controller.slots[1].as_ref().unwrap().endpoints[3]
            .as_ref()
            .unwrap()
            .ring
            .dequeue_pointer();
        controller.cancel(1, 3).unwrap();
        let seen = model.borrow().seen.clone();
        match initial {
            1 => assert_eq!(seen, [STOP_ENDPOINT, SET_DEQUEUE]),
            2 => assert_eq!(seen, [RESET_ENDPOINT, SET_DEQUEUE]),
            // Stopped since the submission's doorbell: Stop Endpoint, sent
            // again while it fails on Stopped, then taken as stopped.
            _ => {
                let (last, stops) = seen.split_last().unwrap();
                assert_eq!(*last, SET_DEQUEUE);
                assert!(stops.len() > 1 && stops.iter().all(|&kind| kind == STOP_ENDPOINT));
            }
        }
        assert_eq!(model.borrow().dequeues, vec![pointer]);
        assert_eq!(controller.completed(1, 3, trb, 512), None);
        assert!(controller.submit(1, 3, buffer.physical(), 512).is_ok());
    }
}

/// One look at a cancellation: its result, and that it took under 5 ms of
/// the controller's clock.
fn look(
    controller: &mut Controller,
    cancel: &mut Cancel,
) -> Option<Result<Vec<Event>, BlockError>> {
    let before = Clock::ticks();
    let result = controller.poll_cancel(cancel);
    assert!(Clock::ticks() - before < 5_000, "one look waited");
    result
}

/// Linux `xhci_handle_cmd_stop_ep`: a Stop Endpoint that fails with a
/// Context State Error on an endpoint still Running, or on one whose
/// context still says Stopped after a doorbell, is sent again, each time
/// from the next look.
#[test]
fn a_refused_stop_is_sent_again_from_the_next_look() {
    for (races, lag) in [(2, 0), (0, 2)] {
        let mut dma = TestRegion::new(64 * 1024 * 8);
        let (mut controller, model) = cancellation_controller(&mut dma);
        let buffer = dma.allocate(512, 512).unwrap();
        // A first cancellation leaves the endpoint Stopped.
        controller.submit(1, 3, buffer.physical(), 512).unwrap();
        controller.cancel(1, 3).unwrap();
        model.borrow_mut().stop_races = races;
        model.borrow_mut().context_lag = lag;
        model.borrow_mut().seen.clear();
        let trb = controller.submit(1, 3, buffer.physical(), 512).unwrap();
        let mut cancel = controller.start_cancel(1, 3).unwrap();
        let mut looks = 1;
        let result = loop {
            if let Some(result) = look(&mut controller, &mut cancel) {
                break result;
            }
            looks += 1;
        };
        assert!(result.is_ok());
        assert_eq!(looks, 3);
        assert_eq!(
            model.borrow().seen,
            [STOP_ENDPOINT, STOP_ENDPOINT, STOP_ENDPOINT, SET_DEQUEUE]
        );
        assert_eq!(controller.completed(1, 3, trb, 512), None);
        assert!(!controller.endpoint_poisoned(1, 3));
    }
}

/// Only a cancellation since the last doorbell makes the context's Stopped
/// current: then Set TR Dequeue comes without a Stop Endpoint.
#[test]
fn only_a_stop_since_the_last_doorbell_skips_stop_endpoint() {
    let mut dma = TestRegion::new(64 * 1024 * 8);
    let (mut controller, model) = cancellation_controller(&mut dma);
    let buffer = dma.allocate(512, 512).unwrap();
    controller.submit(1, 3, buffer.physical(), 512).unwrap();
    controller.cancel(1, 3).unwrap();
    model.borrow_mut().seen.clear();
    controller.cancel(1, 3).unwrap();
    assert_eq!(model.borrow().seen, [SET_DEQUEUE]);
    model.borrow_mut().seen.clear();
    model.borrow_mut().context_lag = 1;
    controller.submit(1, 3, buffer.physical(), 512).unwrap();
    assert_eq!(model.borrow().state(3), ENDPOINT_STOPPED);
    controller.cancel(1, 3).unwrap();
    assert_eq!(
        model.borrow().seen,
        [STOP_ENDPOINT, STOP_ENDPOINT, SET_DEQUEUE]
    );
}

/// A Running endpoint that refuses Stop Endpoint past 100 ms did not stop;
/// a device gone mid-sequence ends it at the next command. Either keeps
/// the endpoint poisoned with its TD recorded.
#[test]
fn a_stop_refused_past_its_window_or_cut_off_by_a_disconnect_ends_poisoned() {
    let mut dma = TestRegion::new(64 * 1024 * 8);
    let (mut controller, model) = cancellation_controller(&mut dma);
    let buffer = dma.allocate(512, 512).unwrap();
    model.borrow_mut().silence_transfer = Some(3);
    controller.submit(1, 3, buffer.physical(), 512).unwrap();
    model.borrow_mut().stop_races = usize::MAX;
    let mut cancel = controller.start_cancel(1, 3).unwrap();
    let start = Clock::ticks();
    let result = loop {
        if let Some(result) = look(&mut controller, &mut cancel) {
            break result;
        }
    };
    let elapsed = Clock::ticks() - start;
    assert_eq!(
        result.err(),
        Some(BlockError::Unsupported("xHCI endpoint did not stop"))
    );
    assert!((100_000..200_000).contains(&elapsed), "{elapsed} us");
    let seen = model.borrow().seen.clone();
    assert!(seen.len() > 2 && seen.iter().all(|&kind| kind == STOP_ENDPOINT));
    assert!(controller.endpoint_poisoned(1, 3));
    assert!(model.borrow().active[3].is_some());

    let mut dma = TestRegion::new(64 * 1024 * 8);
    let (mut controller, model) = cancellation_controller(&mut dma);
    let buffer = dma.allocate(512, 512).unwrap();
    model.borrow_mut().silence_transfer = Some(3);
    controller.submit(1, 3, buffer.physical(), 512).unwrap();
    model.borrow_mut().stop_races = usize::MAX;
    let mut cancel = controller.start_cancel(1, 3).unwrap();
    assert!(look(&mut controller, &mut cancel).is_none());
    model.borrow_mut().connected = false;
    model.borrow_mut().fail = Some((STOP_ENDPOINT, 11));
    assert_eq!(
        look(&mut controller, &mut cancel).map(Result::err),
        Some(Some(BlockError::Device { status: 11 }))
    );
    assert_eq!(model.borrow().seen, [STOP_ENDPOINT, STOP_ENDPOINT]);
    assert!(controller.endpoint_poisoned(1, 3));
    assert!(model.borrow().active[3].is_some());
}

#[test]
fn a_full_transfer_ring_refuses_submission_until_cancelled() {
    let mut dma = TestRegion::new(64 * 1024 * 8);
    let (mut controller, model) = cancellation_controller(&mut dma);
    let buffer = dma.allocate(512, 512).unwrap();
    for _ in 0..RING_TRBS - 2 {
        controller.submit(1, 3, buffer.physical(), 512).unwrap();
    }
    assert_eq!(
        controller.submit(1, 3, buffer.physical(), 512),
        Err(BlockError::Unsupported("xHCI transfer ring full"))
    );
    model.borrow_mut().set_state(3, 1);
    controller.cancel(1, 3).unwrap();
    assert!(controller.submit(1, 3, buffer.physical(), 512).is_ok());
}

#[test]
fn cancellation_failure_keeps_dma_recorded_and_blocks_reuse() {
    for failure in [
        Some((STOP_ENDPOINT, 11)),
        Some((STOP_ENDPOINT, 19)),
        Some((SET_DEQUEUE, 17)),
        None,
    ] {
        let mut dma = TestRegion::new(64 * 1024 * 8);
        let (mut controller, model) = cancellation_controller(&mut dma);
        let buffer = dma.allocate(512, 512).unwrap();
        controller.submit(1, 3, buffer.physical(), 512).unwrap();
        model.borrow_mut().fail = failure;
        if failure.is_none() {
            model.borrow_mut().silence = Some(STOP_ENDPOINT);
        }
        assert!(controller.cancel(1, 3).is_err());
        assert!(
            controller.slots[1].as_ref().unwrap().endpoints[3]
                .as_ref()
                .unwrap()
                .poisoned
        );
        let commands = model.borrow().seen.len();
        assert!(controller.submit(1, 3, buffer.physical(), 512).is_err());
        assert!(controller.cancel(1, 3).is_err());
        let endpoint = Endpoint {
            address: 0x81,
            attributes: 3,
            max_packet: 16,
            interval: 1,
            companion: None,
        };
        assert!(controller.clear(1, &endpoint).is_err());
        assert!(
            controller
                .configure(1, &[endpoint], None, &mut dma)
                .is_err()
        );
        assert_eq!(model.borrow().seen.len(), commands);
        if failure != Some((SET_DEQUEUE, 17)) {
            assert!(model.borrow().active[3].is_some());
            assert_eq!(model.borrow().state(3), 1);
        }
    }
}

#[test]
fn cancellation_handles_completion_and_halt_races_without_resetting_running() {
    let mut dma = TestRegion::new(64 * 1024 * 8);
    let (mut controller, model) = cancellation_controller(&mut dma);
    let buffer = dma.allocate(512, 512).unwrap();
    let trb = controller.submit(1, 3, buffer.physical(), 512).unwrap();
    model.borrow_mut().event(trb, SUCCESS, TRANSFER_EVENT, 3, 0);
    assert_eq!(controller.completed(1, 3, trb, 512), Some(Ok(512)));
    controller.cancel(1, 3).unwrap(); // Empty ring still Running.
    model.borrow_mut().seen.clear();
    controller.submit(1, 3, buffer.physical(), 512).unwrap();
    model.borrow_mut().race_halt = true;
    controller.cancel(1, 3).unwrap();
    assert_eq!(
        model.borrow().seen,
        vec![STOP_ENDPOINT, RESET_ENDPOINT, SET_DEQUEUE]
    );
}

#[test]
fn short_control_out_after_status_ack_fails_without_poisoning_completed_endpoint() {
    let mut dma = TestRegion::new(64 * 1024 * 8);
    let (mut controller, model) = unconfigured_cancellation_controller(&mut dma);
    model.borrow_mut().short_control_out = true;
    let request = setup(0x40, 5, 0xe810, 0x0188, 4);
    assert_eq!(
        controller.control(1, request, &[0; 4]),
        Err(BlockError::Unsupported("short USB control OUT transfer"))
    );
    assert!(controller.endpoint_ready(1, 1).is_ok());
    assert_eq!(model.borrow().indices[1], 3);
    assert!(model.borrow().seen.is_empty());
    model.borrow_mut().short_control_out = false;
    assert!(controller.control(1, request, &[0; 4]).is_ok());
    assert_eq!(model.borrow().indices[1], 6);
    assert!(model.borrow().active[1].is_none());
}

#[test]
fn control_timeout_stops_running_ep0_and_keeps_poison_if_stop_fails() {
    for fail_stop in [false, true] {
        let mut dma = TestRegion::new(64 * 1024 * 8);
        let (mut controller, model) = cancellation_controller(&mut dma);
        model.borrow_mut().silence_transfer = Some(1);
        if fail_stop {
            model.borrow_mut().fail = Some((STOP_ENDPOINT, CONTEXT_STATE_ERROR));
        }
        let result = controller.control(1, setup(0x20, 0, 0, 0, 3), &[3, 0x0c, 0]);
        if fail_stop {
            assert!(matches!(
                result,
                Err(BlockError::Unsupported("xHCI endpoint did not stop"))
            ));
            assert!(
                controller.slots[1].as_ref().unwrap().endpoints[1]
                    .as_ref()
                    .unwrap()
                    .poisoned
            );
            assert!(model.borrow().active[1].is_some());
            let old = controller.slots[1]
                .as_ref()
                .unwrap()
                .control
                .bytes()
                .to_vec();
            let count = model.borrow().setup.len();
            assert!(
                controller
                    .control(1, setup(0x20, 0, 0, 0, 3), &[9, 8, 7])
                    .is_err()
            );
            assert_eq!(controller.slots[1].as_ref().unwrap().control.bytes(), old);
            assert_eq!(model.borrow().setup.len(), count);
        } else {
            assert_eq!(result, Err(BlockError::Timeout));
            assert_eq!(model.borrow().seen, vec![STOP_ENDPOINT, SET_DEQUEUE]);
            assert!(model.borrow().active[1].is_none());
            model.borrow_mut().silence_transfer = None;
            assert!(
                controller
                    .control(1, setup(0x20, 0, 0, 0, 3), &[3, 0x0c, 0])
                    .is_ok()
            );
        }
        assert!(!model.borrow().seen.contains(&RESET_ENDPOINT));
    }
}

#[test]
fn bulk_timeout_stops_running_td_and_cannot_reuse_it_after_failed_cancel() {
    for fail_stop in [false, true] {
        let mut dma = TestRegion::new(64 * 1024 * 8);
        let (mut controller, model) = cancellation_controller(&mut dma);
        let buffer = dma.allocate(512, 512).unwrap();
        if fail_stop {
            model.borrow_mut().fail = Some((STOP_ENDPOINT, CONTEXT_STATE_ERROR));
        }
        // Keep a valid publication window in this shortened Clock1 model;
        // the held TD then consumes the budget and must stop.
        let result = controller.transfer(1, 5, buffer.physical(), 512, 3_000_000);
        if fail_stop {
            assert!(matches!(
                result,
                Err(BlockError::Unsupported("xHCI endpoint did not stop"))
            ));
            assert!(model.borrow().active[5].is_some());
            let old = controller.slots[1].as_ref().unwrap().endpoints[5]
                .as_ref()
                .unwrap()
                .ring
                .buffer
                .bytes()
                .to_vec();
            assert!(controller.endpoint_ready(1, 5).is_err());
            assert!(
                controller
                    .transfer(1, 5, buffer.physical(), 512, 1)
                    .is_err()
            );
            assert_eq!(
                controller.slots[1].as_ref().unwrap().endpoints[5]
                    .as_ref()
                    .unwrap()
                    .ring
                    .buffer
                    .bytes(),
                old
            );
        } else {
            assert_eq!(result, Err(BlockError::Timeout));
            assert_eq!(model.borrow().seen, vec![STOP_ENDPOINT, SET_DEQUEUE]);
            assert!(model.borrow().active[5].is_none());
            model.borrow_mut().push_acl(vec![1, 2, 3]);
            assert_eq!(
                controller.transfer(1, 5, buffer.physical(), 512, 50_000_000),
                Ok(3)
            );
            assert_eq!(&buffer.bytes()[..3], &[1, 2, 3]);
        }
        assert!(!model.borrow().seen.contains(&RESET_ENDPOINT));
    }
}

#[test]
fn halted_reset_with_failed_dequeue_refuses_endpoint_and_dma_reuse() {
    for dci in [1, 3, 4, 5] {
        let mut dma = TestRegion::new(64 * 1024 * 8);
        let buffer = dma.allocate(512, 512).unwrap();
        let (mut controller, model) =
            failed_stall_recovery(&mut dma, buffer.physical(), buffer.len(), dci);
        let slot = controller.slots[1].as_ref().unwrap();
        let control = slot.control.bytes().to_vec();
        let input = slot.input.bytes().to_vec();
        let transfer = slot.endpoints[usize::from(dci)].as_ref().unwrap();
        assert!(transfer.poisoned);
        let ring = transfer.ring.buffer.bytes().to_vec();
        let commands = controller.commands.buffer.bytes().to_vec();
        let seen = model.borrow().seen.len();
        let setup_count = model.borrow().setup.len();
        let old_index = model.borrow().indices[usize::from(dci)];
        assert!(controller.endpoint_ready(1, dci).is_err());
        assert!(controller.submit(1, dci, buffer.physical(), 512).is_err());
        assert!(
            controller
                .completed(1, dci, buffer.physical(), 512)
                .unwrap()
                .is_err()
        );
        assert!(controller.cancel(1, dci).is_err());
        if dci == 1 {
            assert!(
                controller
                    .control(1, setup(0x20, 0, 0, 0, 3), &[0; 3])
                    .is_err()
            );
            assert!(controller.recover(1, dci).is_err());
        } else {
            let endpoint = fixture_endpoint(dci);
            assert!(controller.clear(1, &endpoint).is_err());
            assert!(
                controller
                    .configure(1, &[endpoint], None, &mut dma)
                    .is_err()
            );
        }
        let slot = controller.slots[1].as_ref().unwrap();
        assert_eq!(slot.control.bytes(), control);
        assert_eq!(slot.input.bytes(), input);
        assert_eq!(
            slot.endpoints[usize::from(dci)]
                .as_ref()
                .unwrap()
                .ring
                .buffer
                .bytes(),
            ring
        );
        assert_eq!(controller.commands.buffer.bytes(), commands);
        assert_eq!(model.borrow().seen.len(), seen);
        assert_eq!(model.borrow().setup.len(), setup_count);
        assert_eq!(model.borrow().indices[usize::from(dci)], old_index);
        // An acknowledged Set Dequeue error poisons only its endpoint.
        let healthy = if dci == 5 { 3 } else { 5 };
        assert!(controller.endpoint_ready(1, healthy).is_ok());
        model.borrow_mut().fail = None;
        assert!(controller.command(0, 0, ENABLE_SLOT << 10).is_ok());
        assert!(!controller.command_poisoned);
    }
}
