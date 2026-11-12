use super::*;
use crate::usb::cdc;
use crate::usb::descriptor::*;
use crate::usb::xhci::{model, model::*, *};
use alloc::rc::Rc;
use core::cell::RefCell;

#[test]
fn mac_strings_parse_as_twelve_hex_digits() {
    assert_eq!(
        parse_mac("02535400AB01"),
        Some(Mac([2, 0x53, 0x54, 0, 0xab, 1]))
    );
    assert_eq!(parse_mac("0253540"), None);
    assert_eq!(parse_mac("02535400AB0G"), None);
}

#[test]
fn poisoned_output_refuses_to_rewrite_the_active_dma_buffer() {
    use crate::bus::dma::TestRegion;
    for halted in [false, true] {
        let mut dma = TestRegion::new(64 * 1024 * 8);
        let mut send = dma.allocate(FRAME_BYTES, FRAME_BYTES).unwrap();
        send.bytes_mut().fill(0xa5);
        let controller = super::super::xhci::model::controller_with_unresolved_td(
            &mut dma,
            send.physical(),
            send.len(),
            4,
            halted,
        );
        let input = Endpoint {
            address: 0x82,
            attributes: 2,
            max_packet: 64,
            interval: 0,
            companion: None,
        };
        let output = Endpoint {
            address: 0x02,
            attributes: 2,
            max_packet: 64,
            interval: 0,
            companion: None,
        };
        let id = controller.borrow().test_device_id(1).unwrap();
        let mut net = UsbNet {
            controller,
            id,
            binding: Binding {
                model: Model::Ecm,
                control: 0,
                data: 1,
                alternate: 0,
                mac_string: 1,
                input,
                output,
                notify: None,
            },
            framing: Framing::Ecm,
            mac: Mac([2, 0, 0, 0, 0, 1]),
            receive: dma.allocate(FRAME_BYTES, FRAME_BYTES).unwrap(),
            pending: VecDeque::new(),
            send,
            status: dma.allocate(64, 64).unwrap(),
            queued: None,
            sending: None,
            resetting: None,
            sent_error: None,
            notify_queued: None,
            connected: None,
            rx_dropped: 0,
            location: "poisoned model CDC".into(),
        };
        assert!(net.transmit(&[1, 2, 3]).is_err());
        // The model leaves this TD hardware-owned but performs no
        // DMA while the test reads its sentinel; production never reads it here.
        assert!(net.send.bytes().iter().all(|byte| *byte == 0xa5));
    }
}

pub(super) fn cdc_device(controller: &Shared, ncm: Option<u8>) -> Device {
    let mut bytes = model::ecm(42);
    if let Some(capabilities) = ncm {
        bytes[15] = cdc::NCM;
        bytes.splice(18..18, [6, 0x24, 0x1a, 0, 1, capabilities]);
    }
    let mut at = 0;
    while at < bytes.len() {
        let length = usize::from(bytes[at]);
        if bytes[at + 1] == ENDPOINT_DESCRIPTOR {
            // Reuse the model's event/ACL receive queues as the
            // CDC notification and data endpoints, respectively.
            if bytes[at + 3] & 3 == 3 {
                bytes[at + 2] = 0x81;
            } else if bytes[at + 2] & 0x80 != 0 {
                bytes[at + 2] = 0x82;
            }
        }
        at += length;
    }
    let total = bytes.len() as u16;
    bytes[2..4].copy_from_slice(&total.to_le_bytes());
    model::sanitize(&bytes, SUPER_SPEED).unwrap();
    let mut device = model::addressed_device(&controller.borrow(), vec![bytes]);
    device.language = Some(LANGUAGE_EN_US);
    model::select(&mut device).unwrap();
    device
}

#[test]
fn native_cdc_binding_requires_its_own_addressed_generation() {
    let mut dma = TestRegion::new(128 * 1024 * 8);
    let configurations = [vec![9, 2, 9, 0, 0, 42, 0, 0x80, 50]];
    let (first, hardware) =
        unconfigured_cancellation_controller_with_configurations(&mut dma, &configurations);
    let (second, other_hardware) =
        unconfigured_cancellation_controller_with_configurations(&mut dma, &configurations);
    hardware.borrow_mut().descriptors = vec![(STRING_DESCRIPTOR, 1, cdc_mac_descriptor())];
    let first = Rc::new(RefCell::new(first));
    let second = Rc::new(RefCell::new(second));
    let device = cdc_device(&first, None);
    let foreign = cdc_device(&second, None);
    assert_eq!(device.slot, foreign.slot);
    assert_ne!(device.id, foreign.id);
    for id in [SlotId::fixture(device.slot), foreign.id] {
        let mut rejected = device.clone();
        rejected.id = id;
        let mut found = Found::default();
        bind(&first, rejected, &mut dma, &mut found);
        assert!(found.links.is_empty());
        assert_eq!(found.failures.len(), 1);
        assert_eq!(
            found.failures[0].1,
            BlockError::Unsupported("xHCI device generation is permanently retired")
        );
        // Neither the class request path nor Configure Endpoint may be
        // entered by a descriptor-only or another controller's identity.
        assert!(hardware.borrow().setup.is_empty());
        assert!(hardware.borrow().seen.is_empty());
        assert!(
            hardware
                .borrow()
                .configuration_seen_before_endpoints
                .is_empty()
        );
        assert!(other_hardware.borrow().setup.is_empty());
        assert!(other_hardware.borrow().seen.is_empty());
    }
    // Refusal did not consume the native claim. The exact device
    // still binds through the real EP0 and endpoint DMA submission path.
    let mut found = Found::default();
    bind(&first, device, &mut dma, &mut found);
    assert!(found.failures.is_empty(), "{:?}", found.failures);
    assert_eq!(found.links.len(), 1);
    assert_eq!(hardware.borrow().configuration_seen_before_endpoints, [42]);
    assert_eq!(
        hardware.borrow().setup[0].0,
        setup(0, SET_CONFIGURATION, 42, 0, 0)
    );
    assert!(hardware.borrow().seen.contains(&CONFIGURE_ENDPOINT));
}

fn cdc_mac_descriptor() -> Vec<u8> {
    let mut bytes = vec![26, STRING_DESCRIPTOR];
    for unit in "025354000001".encode_utf16() {
        bytes.extend_from_slice(&unit.to_le_bytes());
    }
    bytes
}

fn ncm_parameters() -> Vec<u8> {
    let mut bytes = vec![0; 28];
    bytes[..2].copy_from_slice(&28u16.to_le_bytes());
    bytes[2..4].copy_from_slice(&1u16.to_le_bytes());
    bytes[4..8].copy_from_slice(&32768u32.to_le_bytes());
    bytes[16..20].copy_from_slice(&2048u32.to_le_bytes());
    bytes[20..22].copy_from_slice(&8u16.to_le_bytes());
    // Ethernet payload (14 bytes after its frame start) is 8-byte aligned.
    // The resulting frame start at 34 makes a 30-byte frame a 64-byte NTB.
    bytes[22..24].copy_from_slice(&0u16.to_le_bytes());
    bytes[24..26].copy_from_slice(&8u16.to_le_bytes());
    bytes
}

struct CdcDma<'a> {
    inner: &'a mut dyn DmaAllocator,
    fail: Option<(usize, usize)>,
    requests: Vec<(usize, usize)>,
}

impl DmaAllocator for CdcDma<'_> {
    fn allocate(&mut self, bytes: usize, align: usize) -> Result<DmaBuffer, BlockError> {
        self.requests.push((bytes, align));
        if self.fail == Some((bytes, align)) {
            return Err(BlockError::NoDmaMemory);
        }
        self.inner.allocate(bytes, align)
    }
}

#[test]
fn padded_ncm_full_frames_use_capacity_sized_non_crossing_real_tds() {
    use crate::net::{Link, MAX_FRAME};
    for (divisor, remainder, alignment, out_max, expected) in [
        (2048u16, 0u16, 4u16, 8192u32, 3548usize),
        (2048, 0, 2048, 8192, 5596),
        // An exact maximum block must not get an extra terminating ZLP.
        (2048, 548, 2048, 4096, 4096),
        (32768, 31267, 32768, 65535, 65535),
    ] {
        let mut dma = TestRegion::new(128 * 1024 * 8);
        let (controller, model) = unconfigured_cancellation_controller_with_configurations(
            &mut dma,
            &[vec![9, 2, 9, 0, 0, 42, 0, 0x80, 50]],
        );
        model.borrow_mut().descriptors = vec![(STRING_DESCRIPTOR, 1, cdc_mac_descriptor())];
        let mut parameters = ncm_parameters();
        parameters[16..20].copy_from_slice(&out_max.to_le_bytes());
        parameters[20..22].copy_from_slice(&divisor.to_le_bytes());
        parameters[22..24].copy_from_slice(&remainder.to_le_bytes());
        parameters[24..26].copy_from_slice(&alignment.to_le_bytes());
        model
            .borrow_mut()
            .control_replies
            .push_back((setup(0xa1, 0x80, 0, 0, 28), parameters));
        let shared = Rc::new(RefCell::new(controller));
        let mut found = Found::default();
        bind(&shared, cdc_device(&shared, Some(1)), &mut dma, &mut found);
        assert!(found.failures.is_empty(), "{:?}", found.failures);
        assert_eq!(found.links.len(), 1);
        let frame: Vec<_> = (0..MAX_FRAME).map(|byte| byte as u8).collect();
        let link = &mut found.links[0];
        link.transmit(&frame).unwrap();
        assert_eq!(model.borrow().outgoing.len(), 1);
        let model_ref = model.borrow();
        let block = &model_ref.outgoing[0];
        assert_eq!(block.len(), expected);
        assert_eq!(
            u16::from_le_bytes(block[8..10].try_into().unwrap()) as usize,
            expected
        );
        let table = usize::from(u16::from_le_bytes(block[10..12].try_into().unwrap()));
        assert_eq!(table % usize::from(alignment), 0);
        let datagram = usize::from(u16::from_le_bytes(
            block[table + 8..table + 10].try_into().unwrap(),
        ));
        assert_eq!(
            (datagram + 14) % usize::from(divisor),
            usize::from(remainder)
        );
        assert_eq!(&block[datagram..], frame);
        let trb = model_ref.rings[4];
        // SAFETY: the published OUT TRB remains in the live model
        // ring; its referenced allocation is still held by the link.
        let (physical, length) = unsafe {
            (
                (trb as *const u64).read_volatile(),
                ((trb + 8) as *const u32).read_volatile() as usize & 0x1ffff,
            )
        };
        assert_eq!(length, expected);
        assert_eq!(physical % expected.next_power_of_two() as u64, 0);
        assert!(physical % 65536 + length as u64 <= 65536);
        drop(model_ref);
        assert_eq!(link.receive().unwrap(), None);
        let (_, physical, length) = model.borrow().active[5].unwrap();
        assert_eq!(length, 16384);
        assert_eq!(physical % 16384, 0);
        assert!(physical % 65536 + length as u64 <= 65536);
    }
}

#[test]
fn ncm_dma_failures_and_unrepresentable_full_frames_never_activate_data() {
    for failure in 0..5 {
        let mut dma = TestRegion::new(128 * 1024 * 8);
        let (controller, model) = unconfigured_cancellation_controller_with_configurations(
            &mut dma,
            &[vec![9, 2, 9, 0, 0, 42, 0, 0x80, 50]],
        );
        model.borrow_mut().descriptors = vec![(STRING_DESCRIPTOR, 1, cdc_mac_descriptor())];
        let mut parameters = ncm_parameters();
        parameters[16..20].copy_from_slice(&8192u32.to_le_bytes());
        parameters[20..22].copy_from_slice(&2048u16.to_le_bytes());
        let fail = match failure {
            0 => Some((16384, 16384)),
            1 => Some((3548, 4096)),
            2 => Some((64, 64)),
            3 => {
                parameters[16..20].copy_from_slice(&u32::MAX.to_le_bytes());
                parameters[20..22].copy_from_slice(&32768u16.to_le_bytes());
                parameters[22..24].copy_from_slice(&31268u16.to_le_bytes());
                parameters[24..26].copy_from_slice(&32768u16.to_le_bytes());
                None
            }
            4 => {
                parameters[16..20].copy_from_slice(&3547u32.to_le_bytes());
                None
            }
            _ => unreachable!(),
        };
        model
            .borrow_mut()
            .control_replies
            .push_back((setup(0xa1, 0x80, 0, 0, 28), parameters));
        let shared = Rc::new(RefCell::new(controller));
        let mut found = Found::default();
        let mut checked = CdcDma {
            inner: &mut dma,
            fail,
            requests: Vec::new(),
        };
        bind(
            &shared,
            cdc_device(&shared, Some(1)),
            &mut checked,
            &mut found,
        );
        assert!(found.links.is_empty());
        assert_eq!(found.failures.len(), 1);
        if let Some(request) = fail {
            assert!(checked.requests.contains(&request));
            assert_eq!(found.failures[0].1, BlockError::NoDmaMemory);
        } else {
            assert_eq!(
                found.failures[0].1,
                BlockError::Unsupported("NCM OUT parameters cannot fit the maximum Ethernet frame")
            );
            assert!(
                checked
                    .requests
                    .iter()
                    .all(|request| *request == (RING_TRBS * TRB, RING_TRBS * TRB))
            );
        }
        assert!(
            !model
                .borrow()
                .setup
                .iter()
                .any(|(request, _)| matches!(request[1], SET_INTERFACE | 0x43))
        );
        // Every failure must reach the NCM response; an earlier
        // binding refusal is not evidence for DMA or frame-size handling.
        assert!(model.borrow().control_replies.is_empty());
        assert!(
            model
                .borrow()
                .setup
                .iter()
                .any(|(request, _)| *request == setup(0xa1, 0x80, 0, 0, 28))
        );
        assert!(model.borrow().outgoing.is_empty());
        assert!(model.borrow().active[5].is_none());
    }
}

#[test]
fn cdc_short_bulk_out_is_an_error_and_never_sends_a_terminating_zlp() {
    use crate::net::Link;
    for ncm in [None, Some(1)] {
        let mut dma = TestRegion::new(128 * 1024 * 8);
        let (controller, model) = unconfigured_cancellation_controller_with_configurations(
            &mut dma,
            &[vec![9, 2, 9, 0, 0, 42, 0, 0x80, 50]],
        );
        model.borrow_mut().descriptors = vec![(STRING_DESCRIPTOR, 1, cdc_mac_descriptor())];
        if ncm.is_some() {
            model
                .borrow_mut()
                .control_replies
                .push_back((setup(0xa1, 0x80, 0, 0, 28), ncm_parameters()));
        }
        let shared = Rc::new(RefCell::new(controller));
        let mut found = Found::default();
        bind(&shared, cdc_device(&shared, ncm), &mut dma, &mut found);
        assert!(found.failures.is_empty(), "{:?}", found.failures);
        assert_eq!(found.links.len(), 1);
        let link = &mut found.links[0];
        let frame = vec![0x5a; if ncm.is_some() { 30 } else { 64 }];
        model.borrow_mut().short_bulk_out = true;
        assert!(link.transmit(&frame).is_err());
        assert_eq!(model.borrow().outgoing.len(), 1);
        assert_eq!(model.borrow().outgoing[0].len(), 64);
        model.borrow_mut().short_bulk_out = false;
        link.transmit(&frame).unwrap();
        assert_eq!(model.borrow().outgoing.len(), 3);
        assert_eq!(model.borrow().outgoing[1].len(), 64);
        assert!(model.borrow().outgoing[2].is_empty());
    }
}

#[test]
fn ncm_binding_negotiates_before_alt_setting_and_moves_real_transfer_blocks() {
    use crate::net::{Link, Mac};
    let cases = [0, 1, 0x10, 0x11, 0x20, 0x21, 0x30, 0x31]
        .into_iter()
        .flat_map(|capabilities| {
            [1u16, 3].into_iter().flat_map(move |formats| {
                [16384u32, 32768]
                    .into_iter()
                    .map(move |in_max| (capabilities, formats, in_max))
            })
        });
    for (capabilities, formats, in_max) in cases {
        let mut dma = TestRegion::new(128 * 1024 * 8);
        let (controller, model) = unconfigured_cancellation_controller_with_configurations(
            &mut dma,
            &[vec![9, 2, 9, 0, 0, 42, 0, 0x80, 50]],
        );
        model.borrow_mut().descriptors = vec![(STRING_DESCRIPTOR, 1, cdc_mac_descriptor())];
        let mut parameters = ncm_parameters();
        parameters[2..4].copy_from_slice(&formats.to_le_bytes());
        parameters[4..8].copy_from_slice(&in_max.to_le_bytes());
        model
            .borrow_mut()
            .control_replies
            .push_back((setup(0xa1, 0x80, 0, 0, 28), parameters));
        let shared = Rc::new(RefCell::new(controller));
        let mut found = Found::default();
        bind(
            &shared,
            cdc_device(&shared, Some(capabilities)),
            &mut dma,
            &mut found,
        );
        assert!(found.failures.is_empty(), "{:?}", found.failures);
        assert_eq!(found.links.len(), 1);
        let mut expected = vec![
            setup(0, SET_CONFIGURATION, 42, 0, 0),
            setup(0x80, GET_DESCRIPTOR, 0x301, LANGUAGE_EN_US, 255),
            setup(0xa1, 0x80, 0, 0, 28),
        ];
        if capabilities & 0x10 != 0 {
            expected.push(setup(0x21, 0x8a, 0, 0, 0));
        }
        if formats & 2 != 0 {
            expected.push(setup(0x21, 0x84, 0, 0, 0));
        }
        if in_max > 16384 {
            expected.push(setup(
                0x21,
                0x86,
                0,
                0,
                if capabilities & 0x20 != 0 { 8 } else { 4 },
            ));
        }
        expected.push(setup(1, SET_INTERFACE, 3, 1, 0));
        if capabilities & 1 != 0 {
            expected.push(setup(0x21, 0x43, 0x0e, 0, 0));
        }
        assert_eq!(
            model
                .borrow()
                .setup
                .iter()
                .map(|(request, _)| *request)
                .collect::<Vec<_>>(),
            expected
        );
        assert!(model.borrow().control_replies.is_empty());
        let mut size = 16384u32.to_le_bytes().to_vec();
        if capabilities & 0x20 != 0 {
            size.extend_from_slice(&[0; 4]);
        }
        let model_ref = model.borrow();
        let size_command = model_ref
            .setup
            .iter()
            .find(|(request, _)| request[1] == 0x86);
        if in_max > 16384 {
            assert_eq!(size_command.unwrap().1, size);
        } else {
            assert!(size_command.is_none());
        }
        drop(model_ref);
        let link = &mut found.links[0];
        assert_eq!(link.mac(), Mac([2, 0x53, 0x54, 0, 0, 1]));
        let frame: Vec<_> = (0..30).collect();
        link.transmit(&frame).unwrap();
        let mut block = b"NCMH\x0c\x00\x00\x00\x40\x00\x10\x00\x00\x00\x00\x00NCM0\x10\x00\x00\x00\x22\x00\x1e\x00\x00\x00\x00\x00\x00\x00".to_vec();
        block.extend_from_slice(&frame);
        assert_eq!(model.borrow().outgoing, [block.clone(), Vec::new()]);
        model.borrow_mut().push_acl(block.clone());
        assert_eq!(link.receive().unwrap(), Some(frame.clone()));
        assert_eq!(link.receive().unwrap(), None);
        // The next transfer is malformed and is dropped whole.
        model.borrow_mut().push_acl(b"NCMX".to_vec());
        assert_eq!(link.receive().unwrap(), None);
        // usbnet's rx_complete: a STALLed transfer is counted and dropped
        // once its endpoint is reset, and the link keeps receiving.
        model.borrow_mut().halt_transfer = Some(5);
        assert_eq!(link.receive().unwrap(), None);
        model.borrow_mut().halt_transfer = None;
        model.borrow_mut().push_acl(block);
        let received = (0..2).find_map(|_| link.receive().unwrap());
        assert_eq!(received, Some(frame));
        assert_eq!(link.rx_dropped(), 2);
        for connected in [false, true] {
            model
                .borrow_mut()
                .push_event(vec![0xa1, 0, u8::from(connected), 0, 0, 0, 0, 0]);
            assert_eq!(link.carrier(), connected);
        }
    }
}

#[test]
fn ncm_mode_changes_require_real_status_acknowledgements_before_data_activation() {
    for request in [0x8a, 0x84] {
        for stall in [false, true] {
            let mut dma = TestRegion::new(128 * 1024 * 8);
            let (controller, model) = unconfigured_cancellation_controller_with_configurations(
                &mut dma,
                &[vec![9, 2, 9, 0, 0, 42, 0, 0x80, 50]],
            );
            model.borrow_mut().descriptors = vec![(STRING_DESCRIPTOR, 1, cdc_mac_descriptor())];
            let mut parameters = ncm_parameters();
            parameters[2] = 3;
            model
                .borrow_mut()
                .control_replies
                .push_back((setup(0xa1, 0x80, 0, 0, 28), parameters));
            if stall {
                model.borrow_mut().stall_control = Some(request);
            } else {
                model.borrow_mut().silence_control = Some(request);
            }
            let shared = Rc::new(RefCell::new(controller));
            let mut found = Found::default();
            bind(
                &shared,
                cdc_device(&shared, Some(0x31)),
                &mut dma,
                &mut found,
            );
            assert!(found.links.is_empty());
            assert_eq!(found.failures.len(), 1);
            assert!(match &found.failures[0].1 {
                BlockError::Device { status } => stall && *status == STALL,
                BlockError::Timeout => !stall,
                _ => false,
            });
            let model = model.borrow();
            assert_eq!(model.setup.last().unwrap().0, setup(0x21, request, 0, 0, 0));
            assert!(
                !model
                    .setup
                    .iter()
                    .any(|(setup, _)| matches!(setup[1], SET_INTERFACE | 0x43 | 0x86))
            );
            assert!(model.outgoing.is_empty());
            assert!(model.active[1].is_none(), "failed EP0 TD must be cancelled");
            assert!(model.seen.contains(&SET_DEQUEUE));
        }
    }
}

#[test]
fn malformed_parameters_and_short_size_commands_abort_before_ncm_data_activation() {
    for failure in 0..3 {
        let mut dma = TestRegion::new(128 * 1024 * 8);
        let (controller, model) = unconfigured_cancellation_controller_with_configurations(
            &mut dma,
            &[vec![9, 2, 9, 0, 0, 42, 0, 0x80, 50]],
        );
        model.borrow_mut().descriptors = vec![(STRING_DESCRIPTOR, 1, cdc_mac_descriptor())];
        let mut parameters = ncm_parameters();
        match failure {
            0 => parameters.truncate(22),
            1 => parameters[2] = 2,
            2 => model.borrow_mut().short_control_out = true,
            _ => unreachable!(),
        }
        model
            .borrow_mut()
            .control_replies
            .push_back((setup(0xa1, 0x80, 0, 0, 28), parameters));
        let shared = Rc::new(RefCell::new(controller));
        let mut found = Found::default();
        bind(
            &shared,
            cdc_device(&shared, Some(0x21)),
            &mut dma,
            &mut found,
        );
        assert_eq!(found.failures.len(), 1);
        assert!(found.links.is_empty());
        assert_eq!(
            found.failures[0].1,
            BlockError::Unsupported(match failure {
                0 => "short NTB parameters",
                1 => "NCM function without NTB16",
                2 => "short USB control OUT transfer",
                _ => unreachable!(),
            })
        );
        assert!(model.borrow().control_replies.is_empty());
        assert!(
            model
                .borrow()
                .setup
                .iter()
                .any(|(request, _)| *request == setup(0xa1, 0x80, 0, 0, 28))
        );
        if failure == 2 {
            assert_eq!(
                model.borrow().setup.last().unwrap().0,
                setup(0x21, 0x86, 0, 0, 8)
            );
        }
        assert!(
            !model
                .borrow()
                .setup
                .iter()
                .any(|(request, _)| request[1] == SET_INTERFACE || request[1] == 0x43)
        );
        assert!(model.borrow().outgoing.is_empty());
    }
}

/// An ECM link sending frames, one of whole packets with the zero-length
/// packet after it, and recovering a STALLed IN transfer, a STALLed OUT
/// transfer and one past its deadline, its calls bounded by
/// [`crate::usb::ethernet::tests::stepped::Calls`]; what the device saw and
/// the transfers dropped.
fn ecm_link_through_failures(late: bool) -> (crate::usb::ethernet::tests::stepped::Effects, u64) {
    use crate::net::{Link, LinkError};
    use crate::usb::ethernet::tests::stepped::{Calls, effects};
    let mut dma = TestRegion::new(128 * 1024 * 8);
    let (controller, model) = unconfigured_cancellation_controller_with_configurations(
        &mut dma,
        &[vec![9, 2, 9, 0, 0, 42, 0, 0x80, 50]],
    );
    model.borrow_mut().descriptors = vec![(STRING_DESCRIPTOR, 1, cdc_mac_descriptor())];
    let shared = Rc::new(RefCell::new(controller));
    let mut found = Found::default();
    bind(&shared, cdc_device(&shared, None), &mut dma, &mut found);
    assert!(found.failures.is_empty(), "{:?}", found.failures);
    let link = &mut found.links[0];
    let clock = shared.borrow().clock;
    let mut calls = Calls::new(model.clone(), clock, late);
    let not_busy = |result: &Result<(), LinkError>| result != &Err(LinkError::Busy);

    calls
        .until(|| link.transmit(&[0x5a; 60]), not_busy)
        .unwrap();
    // A frame of whole packets, then its zero-length packet.
    calls
        .until(|| link.transmit(&[0x6a; 64]), not_busy)
        .unwrap();
    calls
        .until(|| link.transmit(&[0x7a; 60]), not_busy)
        .unwrap();

    // usbnet's rx_complete: a STALLed IN transfer is dropped once its
    // endpoint is reset, a command or poll per call.
    model.borrow_mut().halt_transfer = Some(5);
    calls.call(|| link.receive().unwrap());
    model.borrow_mut().halt_transfer = None;
    calls.until(
        || (link.receive().unwrap(), link.rx_dropped()),
        |(_, dropped)| *dropped == 1,
    );
    model.borrow_mut().push_acl(vec![0x8a; 60]);
    let frame = calls.until(|| link.receive().unwrap(), Option::is_some);
    assert_eq!(frame, Some(vec![0x8a; 60]));

    // A STALLed OUT transfer is reported once its TDs are cancelled.
    model.borrow_mut().halt_transfer = Some(4);
    let failed = calls.until(
        || link.transmit(&[0x9a; 60]),
        |result| result.is_err() && not_busy(result),
    );
    assert!(matches!(failed, Err(LinkError::Adapter(_))));
    model.borrow_mut().halt_transfer = None;
    calls
        .until(|| link.transmit(&[0xaa; 60]), not_busy)
        .unwrap();

    // So is one past its deadline.
    model.borrow_mut().silence_transfer = Some(4);
    calls
        .until(|| link.transmit(&[0xba; 60]), not_busy)
        .unwrap();
    calls.wait(SEND_US);
    let failed = calls.until(|| link.transmit(&[0xca; 60]), not_busy);
    assert!(matches!(
        failed,
        Err(LinkError::Adapter(message)) if message.ends_with("device timeout")
    ));
    model.borrow_mut().silence_transfer = None;
    calls
        .until(|| link.transmit(&[0xda; 60]), not_busy)
        .unwrap();
    assert_eq!(calls.answered_late > 0, late);
    let seen = (effects(&model.borrow(), |_| false), link.rx_dropped());
    calls.answer_at_once();
    seen
}

/// A device that answers each transfer and command only after the link
/// call that asked returned: no transmit waits for its OUT transfer, a
/// frame sent while one is in flight is busy, and endpoint resets run a
/// command or poll per call, no call holding the processor beyond its
/// bound; the device sees what one that answers at once sees.
#[test]
fn ecm_link_calls_never_wait_when_the_device_answers_late() {
    assert_eq!(
        ecm_link_through_failures(true),
        ecm_link_through_failures(false)
    );
}

pub(crate) mod generation;
