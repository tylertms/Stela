use super::super::super::scanout::{Mode, PixelFormat};
use super::super::swap::tests::{fenced_submission, submission};
use super::*;
use crate::Result;
use alloc::rc::Rc;
use core::cell::RefCell;
use stela_contracts::refuse;

const MODE: Mode = Mode {
    width: 4,
    height: 2,
    format: PixelFormat::Bgra8888,
};
const MODE_QHD: Mode = Mode {
    width: 2560,
    height: 1440,
    format: PixelFormat::Bgra8888,
};
const FULL_HD: Mode = Mode {
    width: 1920,
    height: 1080,
    format: PixelFormat::Bgra8888,
};

#[derive(Clone, Default)]
struct TestScreen(Rc<RefCell<Vec<Vec<u8>>>>, Option<Mode>);
impl Scanout for TestScreen {
    fn mode(&self) -> Mode {
        self.1.unwrap_or(FULL_HD)
    }
    fn present(&mut self, rows: &[u8]) -> Result<()> {
        self.0.borrow_mut().push(rows.to_vec());
        Ok(())
    }
}

/// The 26A434 interface, whose tags these tests name.
fn interface() -> &'static abi::Interface {
    abi::interface(211).unwrap()
}

/// The 26A434 row for method `tag`.
fn row(tag: &[u8; 4]) -> &'static abi::Method {
    interface().method(tag).unwrap()
}

fn methods() -> Methods {
    Methods::new(Box::new(TestScreen::default()), interface())
}

fn returned(step: Result<Step<Methods>>) -> Vec<u8> {
    match step {
        Ok(Step::Return(bytes)) => bytes,
        Ok(Step::Call { .. }) => panic!("expected a return, got a callback"),
        Err(reason) => panic!("refused: {reason}"),
    }
}

/// Start the display: A401, nesting D120, which the AP answers true.
fn started(methods: &mut Methods) {
    let step = methods.call(*b"A401", &[], 4).unwrap();
    let Step::Call { then, .. } = step else {
        panic!("A401 must nest D120");
    };
    returned(then(methods, &1u32.to_le_bytes()));
}

/// Make every queued callback, answering each with `answer`; returns
/// their tags and inputs.
fn drain(methods: &mut Methods, answer: &[u8]) -> Vec<([u8; 4], Vec<u8>)> {
    let mut made = Vec::new();
    while let Some(callback) = methods.take_callback() {
        assert!(methods.take_callback().is_none(), "one callback at a time");
        made.push((callback.tag, callback.input.clone()));
        let mut output = alloc::vec![0u8; callback.output as usize];
        let n = output.len().min(answer.len());
        output[..n].copy_from_slice(&answer[..n]);
        (callback.then)(methods, &output).unwrap();
    }
    made
}

#[test]
fn display_size_tags_report_the_fixed_mode_at_verified_sizes() {
    assert_eq!(
        answer(row(b"A451"), &[], 4, FULL_HD).unwrap().unwrap(),
        1920u32.to_le_bytes()
    );
    assert_eq!(
        answer(row(b"A452"), &[], 4, FULL_HD).unwrap().unwrap(),
        1080u32.to_le_bytes()
    );
    assert_eq!(
        answer(row(b"A351"), &[], 4, FULL_HD).unwrap().unwrap(),
        1920u32.to_le_bytes()
    );
    assert_eq!(
        answer(row(b"A350"), &[], 4, FULL_HD).unwrap().unwrap(),
        1080u32.to_le_bytes()
    );
    let size = answer(row(b"A453"), &[0; 4], 8, FULL_HD).unwrap().unwrap();
    assert_eq!(&size[0..4], &1920u32.to_le_bytes());
    assert_eq!(&size[4..8], &1080u32.to_le_bytes());
}

#[test]
fn connection_tags_describe_one_external_main_display() {
    assert_eq!(
        answer(row(b"A379"), &[], 4, FULL_HD).unwrap().unwrap(),
        1u32.to_le_bytes()
    );
    assert_eq!(
        answer(row(b"A384"), &[], 4, FULL_HD).unwrap().unwrap(),
        0u32.to_le_bytes()
    );
    assert_eq!(
        answer(row(b"A412"), &[], 4, FULL_HD).unwrap().unwrap(),
        1u32.to_le_bytes()
    );
}

#[test]
fn status_methods_succeed_under_the_ap_link_status_contract() {
    // The original AP maps link statuses 1..=18 to IOReturn errors
    // (0x3d9cf30, table 0xaf72f0). In particular a boolean true is
    // kIOReturnError, not success. Keep the consumer independent of
    // the reply builder so accidentally changing zero to true fails.
    fn ap_success(bytes: &[u8], offset: usize) -> bool {
        let status = u32::from_le_bytes(bytes[offset..offset + 4].try_into().unwrap());
        !(1..=18).contains(&status)
    }
    assert!(!ap_success(&1u32.to_le_bytes(), 0));
    let mut methods = methods();
    let mut parameter = [0; 0x28];
    parameter[..4].copy_from_slice(&14u32.to_le_bytes());
    let mut power = [0; 0x10];
    power[..8].copy_from_slice(&1u64.to_le_bytes());
    let mode = [
        modes::COLOR_ID.to_le_bytes(),
        modes::TIMING_ID.to_le_bytes(),
    ]
    .concat();
    for (tag, input, output, status_offset) in [
        (*b"A411", 2u32.to_le_bytes().as_slice(), 4, 0),
        (*b"A450", parameter.as_slice(), 4, 0),
        (*b"A484", power.as_slice(), 8, 4),
        (*b"A413", mode.as_slice(), 4, 0),
        (*b"A414", [0; 8].as_slice(), 8, 4),
        (*b"A406", [0; 0x10].as_slice(), 8, 4),
        (*b"A479", [0; 8].as_slice(), 4, 0),
    ] {
        let reply = returned(methods.call(tag, input, output));
        assert!(ap_success(&reply, status_offset), "{tag:?}: {reply:?}");
        // Use canonical zero rather than an out-of-range value the
        // AP happens to accept.
        assert_eq!(&reply[status_offset..status_offset + 4], &[0; 4]);
    }
    let id = start_swap(&mut methods);
    let reply = returned(methods.call(*b"A407", &submission(id, &[]), 0xc));
    assert!(ap_success(&reply, 8));
    assert_eq!(&reply[8..12], &[0; 4]);
    // setDisplayRefreshProperties is boolean: the AP at
    // 0x3d7a4e4 reads its low bit instead of mapping a link status.
    assert_eq!(returned(methods.call(*b"A475", &[], 4)), [1, 0, 0, 0]);
}

#[test]
fn the_boot_framebuffer_is_inherited_not_recreated() {
    assert_eq!(
        answer(row(b"A454"), &[], 4, FULL_HD).unwrap().unwrap(),
        0u32.to_le_bytes()
    );
    assert!(
        answer(row(b"A389"), &[], 0, FULL_HD)
            .unwrap()
            .unwrap()
            .is_empty()
    );
    assert!(
        answer(row(b"A478"), &[0; 4], 0, FULL_HD)
            .unwrap()
            .unwrap()
            .is_empty()
    );
    assert_eq!(
        answer(row(b"A475"), &[], 4, FULL_HD).unwrap().unwrap(),
        1u32.to_le_bytes()
    );
}

#[test]
fn the_config_frame_size_starts_unset_and_programs_only_the_mode() {
    let mut methods = methods();
    // Values 0, both pointers present, the AP's 0xaa padding.
    let query = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0xaa, 0xaa];
    assert_eq!(returned(methods.call(*b"A103", &query, 8)), [0; 8]);
    let mut size = 1920u32.to_le_bytes().to_vec();
    size.extend_from_slice(&1080u32.to_le_bytes());
    assert!(returned(methods.call(*b"A104", &size, 0)).is_empty());
    assert_eq!(returned(methods.call(*b"A103", &query, 8)), size);
    assert!(returned(methods.call(*b"A105", &[], 0)).is_empty());
    let mut other = 1280u32.to_le_bytes().to_vec();
    other.extend_from_slice(&720u32.to_le_bytes());
    returned(methods.call(*b"A104", &other, 0));
    assert!(methods.call(*b"A105", &[], 0).is_err());
    assert!(methods.last_refusal().unwrap().contains("A105"));
}

#[test]
fn the_pipe_start_up_queries_answer_as_the_j473_firmware_does() {
    let mut methods = methods();
    // apt_supported, isKeepOnScreen, get_shared_disp_state: false.
    for tag in [*b"A035", *b"A485", *b"A489"] {
        assert_eq!(returned(methods.call(tag, &[], 4)), [0; 4]);
    }
    // setSharedEventMemBlockSize(u32, u32): no output.
    let sizes = Packed::new().u32(0x4000).u32(0x40).finish(8).unwrap();
    assert!(returned(methods.call(*b"A465", &sizes, 0)).is_empty());
    assert!(methods.last_refusal().is_none());
}

#[test]
fn answers_fill_their_whole_output_past_a_callbacks_stale_bytes() {
    // A callback's output room keeps whatever the stream held before
    // (the firmware's rpc_caller_gated writes only the header and
    // input), and the AP writes just its bool. A401's answer, built
    // from that bool, still zeroes the rest of its own output.
    let mut methods = methods();
    let Step::Call { then, .. } = methods.call(*b"A401", &[], 4).unwrap() else {
        panic!("A401 must nest D120");
    };
    assert_eq!(
        returned(then(&mut methods, &[1, 0x3e, 0x49, 0x44])),
        [1, 0, 0, 0]
    );
    let made = drain(&mut methods, &[1, 0x6b, 0x65, 0x79]);
    assert_eq!(made.last().unwrap().0, HOTPLUG);
    assert!(methods.last_refusal().is_none());
}

#[test]
fn link_shutdown_stops_the_pipe_and_hibernation_refuses() {
    // Hibernation (first byte 1) refuses and leaves the link up.
    let mut hibernating = methods();
    assert!(hibernating.call(*b"A500", &[1, 1, 0xaa, 0xaa], 0).is_err());
    assert_eq!(returned(hibernating.call(*b"A035", &[], 4)), [0; 4]);
    // set_power_state's link_shutdown_signal(false, true), padded 0xaaaa,
    // with the published mode's callbacks still queued.
    let mut methods = methods();
    started(&mut methods);
    assert!(methods.callback_pending());
    assert!(returned(methods.call(*b"A500", &[0, 1, 0xaa, 0xaa], 0)).is_empty());
    assert!(!methods.callback_pending() && methods.take_callback().is_none());
    // The link is down: nothing more is served.
    assert!(methods.call(*b"A035", &[], 4).is_err());
    assert!(
        methods
            .last_refusal()
            .unwrap()
            .contains("shut the DCP link down")
    );
    methods.resume();
    assert_eq!(returned(methods.call(*b"A035", &[], 4)), [0; 4]);
    assert!(!methods.callback_pending() && methods.take_callback().is_none());
}

#[test]
fn the_rest_of_asahis_boot_sequence_is_served() {
    let mut methods = methods();
    // late_init_signal(true) -> true; setup_video_limits() -> void.
    assert_eq!(
        returned(methods.call(*b"A000", &[1, 0, 0, 0], 4)),
        [1, 0, 0, 0]
    );
    assert!(returned(methods.call(*b"A027", &[], 0)).is_empty());
}

#[test]
fn removing_gain_maps_returns_the_clients_record_with_success() {
    let input = [7, 0, 0, 0, 1, 1, 0xaa, 0xaa, 0, 0xaa, 0xaa, 0xaa];
    let reply = returned(methods().call(*b"A480", &input, 0xc));
    assert_eq!(reply[..8], input[..8]);
    assert_eq!(reply[8..], IO_RETURN_SUCCESS.to_le_bytes());
}

#[test]
fn calls_off_the_interface_or_its_sizes_refuse_by_name() {
    let mut methods = methods();
    assert!(methods.call(*b"A999", &[], 4).is_err());
    assert!(methods.last_refusal().unwrap().contains("A999"));
    // display_width is (in 0, out 4): a different output refuses.
    assert!(methods.call(*b"A451", &[], 8).is_err());
    let refusal = methods.last_refusal().unwrap();
    assert!(refusal.contains("display_width") && refusal.contains("out 0x4"));
    // Each call and refusal is in the display log.
    assert!(matches!(methods.take_log(), Some(Log::Call { tag, .. }) if &tag == b"A999"));
    assert!(matches!(methods.take_log(), Some(Log::Refused(_))));
}

#[test]
fn a_method_without_a_reply_layout_answers_zeros_and_is_reported_once() {
    let mut methods = methods();
    let logged = |methods: &mut Methods| {
        let mut reported = 0;
        while let Some(entry) = methods.take_log() {
            reported += usize::from(matches!(
                entry,
                Log::Unmodeled { tag, answer: Degraded::Zeros, .. } if &tag == b"A123"
            ));
        }
        reported
    };
    assert_eq!(returned(methods.call(*b"A123", &[0; 0x18], 4)), [0; 4]);
    assert_eq!(logged(&mut methods), 1);
    let reported = methods.take_unmodeled();
    assert_eq!(reported.len(), 1);
    assert!(reported[0].starts_with("IOMFB unmodeled A123 start_task_dcp: answered with zeros"));
    assert_eq!(returned(methods.call(*b"A123", &[0; 0x18], 4)), [0; 4]);
    assert_eq!(logged(&mut methods), 0);
    assert!(methods.take_unmodeled().is_empty());
    // Reported once, counted per call.
    assert_eq!(methods.pacing.unmodeled, 2);
    assert!(methods.last_refusal().is_none());
}

/// The link status at `offset` as the AP reads it (`iomfb_status_from_link`).
fn status(bytes: &[u8], offset: usize) -> u32 {
    u32::from_le_bytes(bytes[offset..offset + 4].try_into().unwrap())
}

#[test]
fn an_unmodeled_status_method_answers_unsupported_and_keeps_its_argument() {
    // The 26B123 interface, whose tags the latest boots call.
    let mut methods = Methods::new(
        Box::new(TestScreen::default()),
        abi::interface(213).unwrap(),
    );
    // get_gamma_table(IOMFBGammaTable*): the caller's table returns as it
    // came, with kIOReturnUnsupported, never a zeroed table and success.
    let mut table = alloc::vec![0x11; 0xc10];
    table[0xc0c] = 0;
    let reply = returned(methods.call(*b"A470", &table, 0xc10));
    assert_eq!(reply[..0xc0c], table[..0xc0c]);
    assert_eq!(status(&reply, 0xc0c), LINK_UNSUPPORTED);
    let reply = returned(methods.call(*b"A471", &table, 0xc10));
    assert_eq!(status(&reply, 0xc0c), LINK_UNSUPPORTED);
    // set_contrast(float*): the float returns.
    let contrast = [0, 0, 0x80, 0x3f, 0, 0xaa, 0xaa, 0xaa];
    let reply = returned(methods.call(*b"A474", &contrast, 8));
    assert_eq!(reply[..4], contrast[..4]);
    assert_eq!(status(&reply, 4), LINK_UNSUPPORTED);
    // get_block_dcp: no data, the status after it.
    let reply = returned(methods.call(*b"A492", &[0; 0x5c], 0x1004));
    assert!(reply[..0x1000].iter().all(|&byte| byte == 0));
    assert_eq!(status(&reply, 0x1000), LINK_UNSUPPORTED);
    for (tag, input, output) in [(*b"A491", 0x106c, 4), (*b"A499", 0x84, 0x7c)] {
        let reply = returned(methods.call(tag, &alloc::vec![0; input], output));
        assert_eq!(status(&reply, output as usize - 4), LINK_UNSUPPORTED);
    }
    let reported = methods.take_unmodeled();
    assert_eq!(reported.len(), 6, "{reported:?}");
    assert!(
        reported
            .iter()
            .all(|line| line.contains("answered kIOReturnUnsupported"))
    );
    // Each is reported once.
    returned(methods.call(*b"A470", &table, 0xc10));
    assert!(methods.take_unmodeled().is_empty());
    assert!(methods.last_refusal().is_none());
}

#[test]
fn identity_color_requests_are_carried_out_and_others_are_unsupported() {
    let mut methods = Methods::new(
        Box::new(TestScreen::default()),
        abi::interface(213).unwrap(),
    );
    // set_matrix(9, identity) as the AP sends it at boot.
    let matrix = |diagonal: u64| {
        let mut packed = Packed::new().u32(9);
        for index in 0..9 {
            packed = packed.u64(if index % 4 == 0 { diagonal } else { 0 });
        }
        packed.bool(false).finish(0x50).unwrap()
    };
    assert_eq!(
        returned(methods.call(*b"A473", &matrix(MATRIX_ONE), 4)),
        [0; 4]
    );
    let reply = returned(methods.call(*b"A473", &matrix(MATRIX_ONE / 2), 4));
    assert_eq!(status(&reply, 0), LINK_UNSUPPORTED);
    // setBrightnessCorrection(1.0) then (0.5).
    let correction = |value: u32| value.to_le_bytes();
    assert_eq!(
        returned(methods.call(*b"A478", &correction(BRIGHTNESS_UNCORRECTED), 4)),
        [0; 4]
    );
    let reply = returned(methods.call(*b"A478", &correction(0x8000), 4));
    assert_eq!(status(&reply, 0), LINK_UNSUPPORTED);
    // Video power savings: disabling is what the display does already.
    assert_eq!(returned(methods.call(*b"A508", &[0; 4], 4)), [0; 4]);
    let reply = returned(methods.call(*b"A508", &[1, 0, 0, 0], 4));
    assert_eq!(status(&reply, 0), LINK_UNSUPPORTED);
    assert_eq!(methods.take_unmodeled().len(), 3);
}

#[test]
fn an_identity_swap_color_matrix_succeeds_and_another_is_reported_once_with_its_matrix() {
    // The 26B123 interface, whose tags the latest boots call.
    let mut methods = Methods::new(
        Box::new(TestScreen::default()),
        abi::interface(213).unwrap(),
    );
    // IOMFBColorFixedMatrix: offsets, 3x3 coefficients, offsets; then the
    // function, a u32 and the null flag, as Core Animation's A499 sends it.
    let call = |scale: u64, function: u32, null: bool| {
        let mut packed = Packed::new();
        for index in 0..15 {
            let one = (3..12).contains(&index) && (index - 3) % 4 == 0;
            packed = packed.u64(if one { scale } else { 0 });
        }
        packed.u32(function).u32(0).bool(null).finish(0x84).unwrap()
    };
    let identity = call(MATRIX_ONE, 2, false);
    for _ in 0..3 {
        let reply = returned(methods.call(*b"A499", &identity, 0x7c));
        assert_eq!(reply[..0x78], identity[..0x78]);
        assert_eq!(status(&reply, 0x78), IO_RETURN_SUCCESS);
    }
    // A null matrix sets nothing.
    let reply = returned(methods.call(*b"A499", &call(0, 2, true), 0x7c));
    assert_eq!(status(&reply, 0x78), IO_RETURN_SUCCESS);
    assert!(methods.take_unmodeled().is_empty());
    // Half intensity (an accessibility filter) is not applied: unsupported,
    // reported once with the whole matrix. So is a function with no location.
    let dimmed = call(MATRIX_ONE / 2, 2, false);
    for _ in 0..2 {
        let reply = returned(methods.call(*b"A499", &dimmed, 0x7c));
        assert_eq!(reply[..0x78], dimmed[..0x78]);
        assert_eq!(status(&reply, 0x78), LINK_UNSUPPORTED);
    }
    let reply = returned(methods.call(*b"A499", &call(MATRIX_ONE, 3, false), 0x7c));
    assert_eq!(status(&reply, 0x78), LINK_UNSUPPORTED);
    assert_eq!(methods.take_unmodeled().len(), 2);
    let logs: Vec<_> = core::iter::from_fn(|| methods.take_log()).collect();
    let payloads: Vec<_> = logs
        .iter()
        .filter_map(|entry| match entry {
            Log::Payload {
                tag: [b'A', b'4', b'9', b'9'],
                bytes,
                ..
            } => Some(bytes.clone()),
            _ => None,
        })
        .collect();
    // The first call's whole input, then each reported matrix.
    assert_eq!(payloads, [identity, dimmed, call(MATRIX_ONE, 3, false)]);
    assert!(logs.iter().any(|entry| matches!(
        entry,
        Log::Unmodeled {
            answer: Degraded::Unsupported,
            detail: Some(2),
            ..
        }
    )));
    assert_eq!(methods.pacing.color_matrix, 7);
    assert_eq!(methods.pacing.unmodeled, 3);
    let line = methods.pacing.to_string();
    assert!(line.contains(" unmodeled=3 color_matrix=7 "), "{line}");
    assert!(methods.last_refusal().is_none());
}

#[test]
fn the_color_remap_mode_reads_back_and_a_mode_the_scanout_lacks_is_unsupported() {
    let mut methods = methods();
    let query = [6, 0, 0, 0, 0, 0xaa, 0xaa, 0xaa];
    let expected = [0u8; 8];
    assert_eq!(returned(methods.call(*b"A427", &query, 8)), expected);
    assert_eq!(
        returned(methods.call(*b"A426", &0u32.to_le_bytes(), 4)),
        [0; 4]
    );
    // A mode the scanout lacks is unsupported and does not read back.
    let reply = returned(methods.call(*b"A426", &2u32.to_le_bytes(), 4));
    assert_eq!(status(&reply, 0), LINK_UNSUPPORTED);
    assert_eq!(returned(methods.call(*b"A427", &query, 8)), expected);
    let reported = core::iter::from_fn(|| methods.take_log()).any(|entry| {
        matches!(
            entry,
            Log::Unmodeled { tag, answer: Degraded::Unsupported, detail: Some(2), .. }
                if &tag == b"A426"
        )
    });
    assert!(reported);
}

#[test]
fn start_signal_nests_start_hardware_boot_and_reports_its_result() {
    let mut methods = methods();
    let step = methods.call(*b"A401", &[], 4).unwrap();
    let then = match step {
        Step::Call {
            tag, output, then, ..
        } => {
            assert_eq!(&tag, b"D120");
            assert_eq!(output, 4);
            then
        }
        Step::Return(_) => panic!("A401 must nest D120, not answer directly"),
    };
    let step = then(&mut methods, &1u32.to_le_bytes()).unwrap();
    match step {
        Step::Return(bytes) => assert_eq!(bytes, 1u32.to_le_bytes()),
        Step::Call { .. } => panic!("the continuation returns start_signal's bool"),
    }
    assert!(methods.call(*b"A401", &[], 8).is_err());
    assert!(methods.last_refusal().unwrap().contains("A401"));
}

#[test]
fn a_failed_hardware_boot_publishes_nothing() {
    let mut methods = methods();
    let Ok(Step::Call { then, .. }) = methods.call(*b"A401", &[], 4) else {
        panic!("A401 must nest D120");
    };
    assert_eq!(returned(then(&mut methods, &[0; 4])), [0; 4]);
    assert!(!methods.callback_pending());
}

#[test]
fn start_publishes_the_mode_then_reports_the_monitor_plugged() {
    let mut methods = methods();
    started(&mut methods);
    assert!(methods.callback_pending());
    let made = drain(&mut methods, &[1, 0, 0, 0]);
    let tags: Vec<&[u8; 4]> = made.iter().map(|(tag, _)| tag).collect();
    // Start, the chunks, End per key; then the hotplug.
    assert_eq!(tags.first(), Some(&&PROP_START));
    assert_eq!(tags.last(), Some(&&HOTPLUG));
    assert_eq!(tags.iter().filter(|tag| ***tag == PROP_END).count(), 5);
    let mut keys = Vec::new();
    let mut buffer = Vec::new();
    let mut expected = 0usize;
    for (tag, input) in &made {
        match tag {
            &PROP_START => {
                assert_eq!(input.len(), 4);
                expected = u32::from_le_bytes(input[0..4].try_into().unwrap()) as usize;
                buffer = alloc::vec![0u8; expected];
            }
            &PROP_CHUNK => {
                assert_eq!(input.len(), 0x1008);
                let offset = u32::from_le_bytes(input[0x1000..0x1004].try_into().unwrap());
                let length = u32::from_le_bytes(input[0x1004..0x1008].try_into().unwrap());
                let (offset, length) = (offset as usize, length as usize);
                assert!(length <= CHUNK_BYTES && offset + length <= expected);
                buffer[offset..offset + length].copy_from_slice(&input[..length]);
            }
            &PROP_END => {
                assert_eq!(input.len(), 0x40);
                let key = input.split(|&byte| byte == 0).next().unwrap();
                keys.push(String::from_utf8(key.to_vec()).unwrap());
                // The buffer ends in the NUL setDCPAVPropEnd writes.
                assert_eq!(buffer.last(), Some(&0));
                let text = core::str::from_utf8(&buffer[..buffer.len() - 1]).unwrap();
                assert!(
                    ["<array><dict>", "<dict>", "<string>"]
                        .iter()
                        .any(|start| text.starts_with(start))
                );
                if key == b"DisplayAttributes" {
                    assert!(text.contains("<key>ProductName</key><string>Stela</string>"));
                }
            }
            &HOTPLUG => {
                assert_eq!(input.len(), 0x58);
                assert_eq!(&input[0..8], &1u64.to_le_bytes());
                // The tiled-display info is present (null flag clear).
                assert_eq!(input[0x54], 0);
            }
            other => panic!("unexpected callback {other:?}"),
        }
    }
    assert_eq!(
        keys,
        [
            "TimingElements",
            "ColorElements",
            "DisplayAttributes",
            "Transport",
            "IOMFBUUID"
        ]
    );
    assert!(!methods.callback_pending());
}

/// The properties `made` published, as `(key, text)` in order, each
/// reassembled from its `D126`..`D128` callbacks.
fn published(made: &[([u8; 4], Vec<u8>)]) -> Vec<(String, String)> {
    let mut properties = Vec::new();
    let mut buffer = Vec::new();
    for (tag, input) in made {
        match *tag {
            PROP_START => {
                buffer =
                    alloc::vec![0u8; u32::from_le_bytes(input[0..4].try_into().unwrap()) as usize];
            }
            PROP_CHUNK => {
                let offset = u32::from_le_bytes(input[0x1000..0x1004].try_into().unwrap()) as usize;
                let length = u32::from_le_bytes(input[0x1004..0x1008].try_into().unwrap()) as usize;
                buffer[offset..offset + length].copy_from_slice(&input[..length]);
            }
            PROP_END => {
                let key = input.split(|&byte| byte == 0).next().unwrap();
                let text = core::str::from_utf8(&buffer[..buffer.len() - 1]).unwrap();
                properties.push((String::from_utf8(key.to_vec()).unwrap(), String::from(text)));
            }
            _ => {}
        }
    }
    properties
}

#[test]
fn the_display_is_described_once_before_the_hotplug_and_the_same_after_a_reset() {
    // WindowServer builds its mode list, default mode and physical size from
    // these properties when the monitor is reported plugged. A guest that
    // read them at its first query and again later, after a coprocessor
    // reset, must see the same monitor: nothing published depends on time.
    let screen = TestScreen(Rc::default(), Some(MODE_QHD));
    let mut methods = Methods::new(Box::new(screen), interface());
    started(&mut methods);
    let first = drain(&mut methods, &[1, 0, 0, 0]);
    assert_eq!(first.last().unwrap().0, HOTPLUG);
    assert_eq!(first.iter().filter(|(tag, _)| *tag == HOTPLUG).count(), 1);
    let properties = published(&first);
    let attributes = &properties
        .iter()
        .find(|(key, _)| key == "DisplayAttributes")
        .unwrap()
        .1;
    for key in [
        "<key>NativeFormatHorizontalPixels</key><integer size=\"32\">2560</integer>",
        "<key>NativeFormatVerticalPixels</key><integer size=\"32\">1440</integer>",
        // The assumed 27-inch monitor: 597 x 336 mm.
        "<key>MaxHorizontalImageSize</key><integer size=\"32\">60</integer>",
        "<key>MaxVerticalImageSize</key><integer size=\"32\">34</integer>",
    ] {
        assert!(attributes.contains(key), "{key}");
    }
    // The display size the AP reads is the native size the attributes name.
    let size = returned(methods.call(*b"A453", &[0; 4], 8));
    assert_eq!(&size[0..4], &2560u32.to_le_bytes());
    assert_eq!(&size[4..8], &1440u32.to_le_bytes());
    // No later callback re-describes the monitor.
    assert!(methods.take_callback().is_none());

    methods.check_reset().unwrap();
    methods.reset();
    started(&mut methods);
    let again = drain(&mut methods, &[1, 0, 0, 0]);
    assert_eq!(published(&again), properties);
    assert_eq!(again, first);
}

#[test]
fn a_refused_property_stops_publishing_by_name() {
    let mut methods = methods();
    started(&mut methods);
    let callback = methods.take_callback().unwrap();
    assert_eq!(callback.tag, PROP_START);
    assert!((callback.then)(&mut methods, &[0; 4]).is_err());
    assert!(
        methods
            .last_refusal()
            .unwrap()
            .contains("setDCPAVPropStart")
    );
    assert!(!methods.callback_pending());
}

#[test]
fn the_mode_set_uses_the_published_ids() {
    let mut methods = methods();
    started(&mut methods);
    drain(&mut methods, &[1, 0, 0, 0]);
    // get_digital_out_state: connected, success.
    let state = Packed::new().u32(0).bool(false).finish(8).unwrap();
    assert_eq!(
        returned(methods.call(*b"A414", &state, 8)),
        [1, 0, 0, 0, 0, 0, 0, 0]
    );
    // getVideoDataForIDs(timing, color): both records, no dual pipe, true.
    let query = |timing: u32, color: u32| {
        Packed::new()
            .bytes(&[0; modes::TIMING_BYTES])
            .bytes(&[0; modes::COLOR_BYTES])
            .u32(timing)
            .u32(color)
            .bool(false)
            .finish(0x7c)
            .unwrap()
    };
    let data = returned(methods.call(*b"A364", &query(modes::TIMING_ID, modes::COLOR_ID), 0x74));
    let advertised = Advertised::new(FULL_HD).unwrap();
    assert_eq!(&data[..0x50], &advertised.timing[..]);
    assert_eq!(&data[0x50..0x70], &advertised.color[..]);
    assert_eq!((data[0x70], data[0x71]), (0, 1));
    // Unknown IDs answer false.
    let data = returned(methods.call(*b"A364", &query(7, 7), 0x74));
    assert_eq!(data[0x71], 0);
    // set_digital_out_mode(color, timing).
    let set = Packed::new()
        .u32(modes::COLOR_ID)
        .u32(modes::TIMING_ID)
        .finish(8)
        .unwrap();
    assert_eq!(returned(methods.call(*b"A413", &set, 4)), [0; 4]);
    assert_eq!(
        methods.digital_out(),
        Some((modes::COLOR_ID, modes::TIMING_ID))
    );
    let swapped = Packed::new()
        .u32(modes::TIMING_ID)
        .u32(modes::COLOR_ID)
        .finish(8)
        .unwrap();
    assert!(methods.call(*b"A413", &swapped, 4).is_err());
    // manage_properties(true) keeps them; (false) refuses.
    assert!(returned(methods.call(*b"A363", &[1, 0, 0, 0], 0)).is_empty());
    assert!(methods.call(*b"A363", &[0; 4], 0).is_err());
}

#[test]
fn power_and_parameters_follow_asahis_power_on() {
    let mut methods = methods();
    // set_parameter_dcp(ADAPTIVE_SYNC, 0, count 3) as Asahi sends it.
    let mut param = Packed::new().u32(14);
    for _ in 0..4 {
        param = param.u64(0);
    }
    let param = param.u32(3).finish(0x28).unwrap();
    assert_eq!(returned(methods.call(*b"A450", &param, 4)), [0; 4]);
    // A value the fixed mode cannot honor is a recorded gap, as WindowServer
    // retries a failure forever.
    let mut vrr = param.clone();
    vrr[4] = 1;
    assert_eq!(returned(methods.call(*b"A450", &vrr, 4)), [0; 4]);
    // WindowServer zeroes others as it opens the display, leaving its stack in
    // the values past `count`.
    let mut zeroed = param.clone();
    zeroed[0] = 31;
    zeroed[0x1c..0x24].fill(0xaa);
    assert_eq!(returned(methods.call(*b"A450", &zeroed, 4)), [0; 4]);
    // setPowerState(1) then (0).
    let on = Packed::new().u64(1).finish(0x10).unwrap();
    assert_eq!(returned(methods.call(*b"A484", &on, 8)), [0; 8]);
    assert_eq!(methods.powered(), Some(true));
    let off = Packed::new().u64(0).finish(0x10).unwrap();
    returned(methods.call(*b"A484", &off, 8));
    assert_eq!(methods.powered(), Some(false));
    let other = Packed::new().u64(2).finish(0x10).unwrap();
    assert!(methods.call(*b"A484", &other, 8).is_err());
    // first_client_open / last_client_close_dcp.
    assert!(returned(methods.call(*b"A466", &[], 0)).is_empty());
    assert_eq!(returned(methods.call(*b"A467", &[0; 4], 4)), [0; 4]);
}

#[test]
fn the_controller_power_state_follows_power_and_the_link() {
    let mut methods = methods();
    // The configured monitor runs before any request.
    assert_eq!(methods.shared_variables(), [1, 0, 0]);
    let off = Packed::new().u64(0).finish(0x10).unwrap();
    returned(methods.call(*b"A484", &off, 8));
    assert_eq!(methods.shared_variables(), [0, 0, 0]);
    let on = Packed::new().u64(1).finish(0x10).unwrap();
    returned(methods.call(*b"A484", &on, 8));
    assert_eq!(methods.shared_variables(), [1, 0, 0]);
    returned(methods.call(*b"A500", &[0, 1, 0xaa, 0xaa], 0));
    assert_eq!(methods.shared_variables(), [0, 0, 0]);
}

#[test]
fn power_and_sleep_gate_scanout() {
    use super::super::avipc::{COMMAND_PACKET_TYPE, Controller};
    let display = Rc::new(RefCell::new(DisplayState::new(true)));
    let screen = TestScreen(Rc::default(), Some(MODE));
    let frames = screen.0.clone();
    let mut methods =
        Methods::with_display_state(Box::new(screen), Rc::clone(&display), interface());
    let mut controller = Controller::new(display);
    let mut command = |index: u32, value: Option<u64>| {
        let size: u32 = if value.is_some() { 0x20 } else { 0x10 };
        let mut bytes = alloc::vec![0; 0x40 + size as usize];
        bytes[4..8].copy_from_slice(&index.to_le_bytes());
        bytes[8..12].copy_from_slice(&size.to_le_bytes());
        bytes[12..16].copy_from_slice(&0x6970_6378u32.to_le_bytes());
        if let Some(value) = value {
            bytes[0x50..0x58].copy_from_slice(&value.to_le_bytes());
        }
        let (status, reply) = controller.handle(COMMAND_PACKET_TYPE, &bytes).unwrap();
        assert_eq!(status, 0);
        assert_eq!(&reply[0x40..0x44], &[0; 4]);
        reply
    };
    start_swap(&mut methods);
    returned(methods.call(*b"A407", &submission(1, &[]), 0xc));
    command(6, Some(0));
    methods
        .present(0, |_, _| panic!("powered-off scanout reads no pixels"))
        .unwrap();
    assert!(methods.frame_pending() && !methods.callback_pending());
    assert!(frames.borrow().is_empty());
    command(6, Some(1));
    command(9, None);
    methods
        .present(0, |_, _| panic!("asleep scanout reads no pixels"))
        .unwrap();
    assert!(methods.frame_pending() && frames.borrow().is_empty());
    command(8, None);
    methods
        .present(0, |_, _| panic!("background clear has no surface"))
        .unwrap();
    assert_eq!(frames.borrow().len(), 1);
    assert!(!methods.frame_pending() && methods.callback_pending());
    let callback = methods.take_callback().unwrap();
    assert_eq!(callback.tag, SWAP_COMPLETE);
    (callback.then)(&mut methods, &[]).unwrap();
    // The opposite API observes the same power change.
    returned(methods.call(*b"A484", &[0; 0x10], 8));
    let power = command(7, Some(0));
    assert_eq!(&power[0x50..0x58], &[0; 8]);
    command(10, None);
    assert!(methods.callback_pending());
    let callback = methods.take_callback().unwrap();
    assert_eq!(callback.tag, HOTPLUG);
    assert_eq!(&callback.input[..8], &PLUGGED.to_le_bytes());
}

fn started_ids(methods: &Methods) -> Vec<u32> {
    methods.started.iter().map(|&(id, _)| id).collect()
}

fn start_swap(methods: &mut Methods) -> u32 {
    start_client_swap(methods, 0xfeed)
}

fn start_client_swap(methods: &mut Methods, client: u64) -> u32 {
    let start = Packed::new()
        .u32(0)
        .u64(client)
        .bool(false)
        .finish(0x10)
        .unwrap();
    let started = returned(methods.call(*b"A406", &start, 8));
    u32::from_le_bytes(started[0..4].try_into().unwrap())
}

fn submit_refusal_keeps_accepted_swap(mutate: impl FnOnce(&mut [u8]), reason: &str) {
    let screen = TestScreen(Rc::default(), Some(MODE));
    let displayed = screen.0.clone();
    let mut methods = Methods::new(Box::new(screen), interface());
    started(&mut methods);
    let completed = start_swap(&mut methods);
    returned(methods.call(*b"A407", &submission(completed, &[]), 0xc));
    methods
        .present(0, |_, _| panic!("background clear reads no surface"))
        .unwrap();
    let queued = start_swap(&mut methods);
    returned(methods.call(*b"A407", &submission(queued, &[(0, 4, 2, 0x1000)]), 0xc));
    let id = start_swap(&mut methods);
    let valid = submission(id, &[(0, 4, 2, 0x2000)]);
    let mut refused = valid.clone();
    mutate(&mut refused);
    let planes = methods.planes.to_vec();
    let frames = methods.frames.clone();
    let completions = methods.completed.clone();
    let error = methods
        .call(*b"A407", &refused, 0xc)
        .err()
        .expect("unsupported submission refused");
    assert!(error.to_string().contains(reason), "{error}");
    assert_eq!(started_ids(&methods), [id]);
    assert_eq!(methods.planes.as_slice(), planes);
    assert_eq!(methods.frames, frames);
    assert_eq!(methods.completed, completions);
    assert_eq!(displayed.borrow().len(), 1);
    assert_eq!(methods.call(*b"A407", &refused, 0xc).err(), Some(error));
    returned(methods.call(*b"A407", &valid, 0xc));
    assert!(started_ids(&methods).is_empty());
    assert_eq!(methods.frames.len(), frames.len() + 1);
    assert_eq!(methods.frames.back().unwrap().swap_id, id);
    assert_eq!(methods.completed, completions);
}

#[test]
fn refused_swap_parse_preserves_started_swap_and_prior_frames() {
    submit_refusal_keeps_accepted_swap(
        |input| input[swap::SECONDARY_NULL] = 0,
        "secondary surface",
    );
}

#[test]
fn refused_tiled_surface_preserves_started_swap_and_prior_frames() {
    submit_refusal_keeps_accepted_swap(|input| input[swap::SURFACES] = 1, "tiled");
}

#[test]
fn clients_are_told_of_each_refresh_and_a_late_one_replaces_the_last() {
    // 60 Hz on the 24 MHz counter.
    const REFRESH_TICKS: u64 = 400_000;
    let mut methods = methods();
    methods.refresh(1000);
    assert!(methods.take_callback().is_none());
    // With no client, no swap and nothing queued, nothing has to wake the
    // display.
    assert_eq!(methods.refresh_deadline(), None);
    let mut clients = [0; 0x6c];
    clients[24] = 1;
    assert!(returned(methods.call(*b"A033", &clients, 0)).is_empty());
    // The grid the first look started goes on.
    assert_eq!(methods.refresh_deadline(), Some(1000 + REFRESH_TICKS));
    // Within the refresh period nothing new is due, and a later one wins.
    methods.refresh(1000 + REFRESH_TICKS - 1);
    assert!(methods.take_callback().is_none());
    methods.refresh(1000 + REFRESH_TICKS);
    methods.refresh(1000 + 2 * REFRESH_TICKS);
    let callback = methods.take_callback().unwrap();
    assert_eq!(callback.tag, SWAP_NOTIFY);
    assert_eq!(callback.input.len(), 0x18);
    assert_eq!(
        callback.input[..8],
        (1000 + 2 * REFRESH_TICKS).to_le_bytes()
    );
    assert_eq!(callback.output, 0);
}

#[test]
fn a_late_look_sends_the_vblanks_own_time_and_keeps_the_grid() {
    const REFRESH_TICKS: u64 = 400_000;
    // 5 ms of the 24 MHz counter.
    const LATE: u64 = 120_000;
    let mut methods = methods();
    let mut clients = [0; 0x6c];
    clients[24] = 1;
    returned(methods.call(*b"A033", &clients, 0));
    methods.refresh(1000);
    let first = methods.take_callback().unwrap();
    (first.then)(&mut methods, &[]).unwrap();
    methods.refresh(1000 + REFRESH_TICKS + LATE);
    let callback = methods.take_callback().unwrap();
    assert_eq!(callback.tag, SWAP_NOTIFY);
    assert_eq!(callback.input[..8], (1000 + REFRESH_TICKS).to_le_bytes());
    (callback.then)(&mut methods, &[]).unwrap();
    assert_eq!(methods.refresh_deadline(), Some(1000 + 2 * REFRESH_TICKS));
    // Three and a half periods late: the last vblank passed is sent, and
    // the grid's phase holds.
    methods.refresh(1000 + 5 * REFRESH_TICKS + REFRESH_TICKS / 2);
    let callback = methods.take_callback().unwrap();
    assert_eq!(
        callback.input[..8],
        (1000 + 5 * REFRESH_TICKS).to_le_bytes()
    );
    assert_eq!(methods.refresh_deadline(), Some(1000 + 6 * REFRESH_TICKS));
}

/// A screen of the default mode at a given refresh.
struct FastScreen(u32);
impl Scanout for FastScreen {
    fn mode(&self) -> Mode {
        FULL_HD
    }
    fn refresh_millihertz(&self) -> u32 {
        self.0
    }
    fn present(&mut self, _: &[u8]) -> Result<()> {
        Ok(())
    }
}

#[test]
fn refreshes_keep_the_monitors_rate_and_phase_when_looked_at_late() {
    let mut methods = Methods::new(Box::new(FastScreen(144_000)), interface());
    let mut clients = [0; 0x6c];
    clients[24] = 1;
    assert!(returned(methods.call(*b"A033", &clients, 0)).is_empty());
    // 144 Hz: 166_667 ticks of the 24 MHz counter.
    let period = 166_667;
    methods.refresh(10);
    assert_eq!(methods.refresh_deadline(), Some(10 + period));
    // A look 5000 ticks late keeps the grid rather than starting it again.
    methods.refresh(10 + period + 5000);
    assert_eq!(methods.refresh_deadline(), Some(10 + 2 * period));
    // So does one more than a period late: the vblanks between are skipped.
    methods.refresh(10 + 4 * period + 7);
    assert_eq!(methods.refresh_deadline(), Some(10 + 5 * period));
    // The advertised timing is the monitor's rate.
    let advertised = methods.advertised.as_ref().unwrap();
    let rate = u32::from_le_bytes(advertised.timing[0x38..0x3c].try_into().unwrap());
    assert_eq!((rate + 0x8000) >> 16, 144);
}

/// Present a swap on the 4x2 test screen whose one layer, a single pixel at
/// (0, 0) read as 0x10 bytes, leaves the rest uncovered, with `background`.
fn present_uncovered(background: u32) -> (Methods, Vec<u8>) {
    let screen = TestScreen(Rc::default(), Some(MODE));
    let frames = screen.0.clone();
    let mut methods = Methods::new(Box::new(screen), interface());
    let id = start_swap(&mut methods);
    let mut input = submission(id, &[(0, 1, 1, 0x1000)]);
    input[swap::BACKGROUND..swap::BACKGROUND + 4].copy_from_slice(&background.to_le_bytes());
    returned(methods.call(*b"A407", &input, 0xc));
    methods
        .present(0, |_, bytes| {
            bytes.fill(0x10);
            true
        })
        .unwrap();
    let frame = frames.borrow()[0].clone();
    (methods, frame)
}

#[test]
fn uncovered_regions_show_a_black_background_exactly() {
    // Every recorded WindowServer swap sends 0 or 0xff000000: black, with
    // the high byte alpha or unused, and black is what scanout composes.
    for background in [0, 0xff00_0000, 0x8000_0000] {
        let (mut methods, frame) = present_uncovered(background);
        let mut want = [0, 0, 0, 0xff].repeat(8);
        want[0..4].copy_from_slice(&[0x10, 0x10, 0x10, 0xff]);
        assert_eq!(frame, want, "{background:#x}");
        assert!(methods.unmodeled.is_empty(), "{background:#x}");
        let reported = methods.take_unmodeled();
        assert!(
            !reported.iter().any(|line| line.contains("unmodeled")),
            "{reported:?}"
        );
    }
}

#[test]
fn a_background_color_is_a_recorded_gap_shown_black() {
    // The channel order of a non-black background is not recorded: the
    // uncovered pixels stay black and the swap is reported once.
    let (mut methods, frame) = present_uncovered(0xff);
    assert_eq!(&frame[4..], [0, 0, 0, 0xff].repeat(7));
    assert!(methods.unmodeled.contains(&(*b"A407", None)));
    let reported = methods.take_unmodeled();
    let unmodeled: Vec<_> = reported
        .iter()
        .filter(|line| line.contains("unmodeled"))
        .collect();
    assert_eq!(unmodeled.len(), 1, "{reported:?}");
    assert!(
        unmodeled[0].contains("answered success, value ignored")
            && unmodeled[0].contains("background is shown black"),
        "{reported:?}"
    );
    let id = start_swap(&mut methods);
    let mut again = submission(id, &[]);
    again[swap::BACKGROUND] = 0xff;
    returned(methods.call(*b"A407", &again, 0xc));
    assert!(methods.take_unmodeled().is_empty());
}

#[test]
fn refused_submit_reply_preflight_preserves_started_swap() {
    let mut methods = methods();
    let id = start_swap(&mut methods);
    let input = submission(id, &[]);
    // Public RPC validates ABI output size before dispatch. The
    // submit method also prepares its fallible reply before committing.
    assert!(methods.submit(*b"A407", &input, 0).is_err());
    assert_eq!(started_ids(&methods), [id]);
    assert!(methods.frames.is_empty());
    returned(methods.call(*b"A407", &input, 0xc));
    assert_eq!(methods.frames.back().unwrap().swap_id, id);
}

#[test]
fn swaps_started_together_submit_by_the_id_their_record_names() {
    let mut methods = methods();
    let first = start_swap(&mut methods);
    let second = start_swap(&mut methods);
    assert_ne!(first, second);
    returned(methods.call(*b"A407", &submission(second, &[]), 0xc));
    assert_eq!(started_ids(&methods), [first]);
    let error = methods
        .call(*b"A407", &submission(second, &[]), 0xc)
        .err()
        .expect("a submitted swap is not started");
    assert!(error.to_string().contains("not started"), "{error}");
    returned(methods.call(*b"A407", &submission(first, &[]), 0xc));
    let order: Vec<u32> = methods.frames.iter().map(|frame| frame.swap_id).collect();
    assert_eq!(order, [second, first]);
}

#[test]
fn an_abandoned_start_is_forgotten_past_the_limit() {
    let mut methods = methods();
    let abandoned = start_swap(&mut methods);
    for _ in 0..STARTED_SWAPS {
        start_swap(&mut methods);
    }
    assert_eq!(methods.started.len(), STARTED_SWAPS);
    assert!(!started_ids(&methods).contains(&abandoned));
    assert!(
        methods
            .call(*b"A407", &submission(abandoned, &[]), 0xc)
            .is_err()
    );
}

#[test]
fn failed_presents_keep_swap_order_until_abort() {
    struct Sink {
        attempts: Rc<RefCell<usize>>,
        frames: Rc<RefCell<Vec<Vec<u8>>>>,
    }
    impl Scanout for Sink {
        fn mode(&self) -> Mode {
            MODE
        }
        fn present(&mut self, rows: &[u8]) -> Result<()> {
            let mut attempts = self.attempts.borrow_mut();
            *attempts += 1;
            if *attempts == 1 {
                return Err(refuse!("sink temporarily unavailable"));
            }
            self.frames.borrow_mut().push(rows.to_vec());
            Ok(())
        }
    }
    let attempts = Rc::new(RefCell::new(0));
    let frames = Rc::new(RefCell::new(Vec::new()));
    let mut methods = Methods::new(
        Box::new(Sink {
            attempts: attempts.clone(),
            frames: frames.clone(),
        }),
        interface(),
    );
    assert_eq!(start_swap(&mut methods), 1);
    returned(methods.call(*b"A407", &submission(1, &[(0, 4, 2, 0x1000)]), 0xc));
    let mut reads = 0;
    assert!(
        methods
            .present(0, |dva, bytes| {
                reads += 1;
                assert!((0x1000..0x1020).contains(&dva));
                bytes.fill(0x11);
                reads != 2
            })
            .is_err()
    );
    assert_eq!(*attempts.borrow(), 0);
    assert!(methods.frame_pending() && !methods.callback_pending());
    let read = |dva, bytes: &mut [u8]| {
        assert!((0x1000..0x1020).contains(&dva));
        bytes.fill(0x11);
        true
    };
    // A failure keeps the vblank's latch: the retry needs no new vblank.
    assert_eq!(
        methods.present(1, read),
        Err(refuse!("sink temporarily unavailable"))
    );
    assert!(methods.frame_pending() && !methods.callback_pending());
    assert!(frames.borrow().is_empty());
    methods.present(2, read).unwrap();
    assert_eq!(*attempts.borrow(), 2);
    assert_eq!(frames.borrow().len(), 1);
    // Presented, so the AP keeps its surfaces as the displayed ones.
    complete(&mut methods, 1, true);
    // The next submitted swap waits for its vblank, and an AP abort owns
    // its cancellation. It emits no invented successful completion.
    assert_eq!(start_swap(&mut methods), 2);
    returned(methods.call(*b"A407", &submission(2, &[(0, 4, 2, 0x2000)]), 0xc));
    methods
        .present(3, |_, _| panic!("no vblank passed"))
        .unwrap();
    assert!(methods.frame_pending());
    // Another client's abort leaves it queued; its own client's drops it.
    returned(methods.call(*b"A479", &[0; 8], 4));
    assert!(methods.frame_pending());
    returned(methods.call(*b"A479", &0xfeed_u64.to_le_bytes(), 4));
    assert!(!methods.frame_pending() && !methods.callback_pending());
    methods
        .present(methods.refresh_ticks, |_, _| {
            panic!("aborted swap has no DMA")
        })
        .unwrap();
    assert_eq!(*attempts.borrow(), 2);
    // The aborted swap set layer 0: the next present composes its rows
    // again, whatever the slot shows.
    assert_eq!(methods.unshown, 1);
}

#[test]
fn a_swap_is_started_submitted_presented_then_completed() {
    let screen = TestScreen(Rc::default(), Some(MODE));
    let frames = screen.0.clone();
    let mut methods = Methods::new(Box::new(screen), interface());
    assert!(returned(methods.call(*b"A403", &[], 0)).is_empty());
    assert_eq!(start_swap(&mut methods), 1);
    let submitted = returned(methods.call(*b"A407", &submission(1, &[(0, 4, 2, 0x1000)]), 0xc));
    assert_eq!(submitted, [0; 12]);
    // Not complete until presented.
    assert!(!methods.callback_pending() && methods.frame_pending());
    methods
        .present(0, |dva, bytes| {
            assert!((0x1000..0x1020).contains(&dva));
            bytes.fill(0x5a);
            true
        })
        .unwrap();
    assert_eq!(frames.borrow()[0], [0x5a, 0x5a, 0x5a, 0xff].repeat(8));
    let callback = methods.take_callback().unwrap();
    assert_eq!(&callback.tag, b"D594");
    assert_eq!(callback.input.len(), 0x730);
    assert_eq!(&callback.input[0..4], &1u32.to_le_bytes());
    (callback.then)(&mut methods, &[]).unwrap();
    // The record is logged whole the first times.
    let logged = core::iter::from_fn(|| methods.take_log())
        .any(|entry| matches!(entry, Log::Payload { bytes, .. } if bytes.len() == 0x1d30));
    assert!(logged);
}

#[test]
fn bounded_swap_logs_keep_boot_samples_failures_and_guest_completions() {
    let screen = TestScreen(Rc::default(), Some(MODE));
    let presented = screen.0.clone();
    let mut methods = Methods::new(Box::new(screen), interface());
    methods.set_swap_tracing(false);
    let period = methods.refresh_ticks;
    let mut now = 0;
    let mut swap = |methods: &mut Methods| {
        let id = start_swap(methods);
        let input = submission(id, &[]);
        assert_eq!(returned(methods.call(*b"A407", &input, 0xc)), [0; 12]);
        methods
            .present(now, |_, _| panic!("a clear has no surface DMA"))
            .unwrap();
        now += period;
        let callback = methods.take_callback().unwrap();
        assert_eq!(callback.tag, SWAP_COMPLETE);
        assert_eq!(&callback.input[..4], &id.to_le_bytes());
        (callback.then)(methods, &[]).unwrap();
        input
    };
    let first = swap(&mut methods);
    let second = swap(&mut methods);
    for _ in 2..=PRESENT_SUMMARY_INTERVAL {
        swap(&mut methods);
    }
    // Every swap after the first clears a screen already clear: the
    // scanout is given the first frame only.
    assert_eq!(presented.borrow().len(), 1);
    assert!(methods.call(*b"A407", &first, 0xc).is_err());

    let records: Vec<_> = core::iter::from_fn(|| methods.take_log()).collect();
    assert_eq!(records.len(), 15);
    for (index, input) in [first, second].iter().enumerate() {
        let logs = &records[6 * index..][..6];
        let id = index as u32 + 1;
        assert!(matches!(&logs[0], Log::Call { tag, .. } if tag == b"A406"));
        assert!(matches!(&logs[1], Log::Call { tag, .. } if tag == b"A407"));
        assert!(
            matches!(&logs[2], Log::Payload { tag, bytes, .. } if tag == b"A407" && bytes == input)
        );
        assert!(matches!(&logs[3], Log::Submitted { swap_id, .. } if *swap_id == id));
        assert!(matches!(&logs[4], Log::Presented { swap_id, .. } if *swap_id == id));
        assert!(matches!(&logs[5], Log::Callback { tag, .. } if *tag == SWAP_COMPLETE));
    }
    assert_eq!(
        records[12].to_string(),
        "DCP present summary total=256 last_swap=256"
    );
    // A refused call is logged with its refusal even when its kind is not.
    assert!(matches!(&records[13], Log::Call { tag, .. } if tag == b"A407"));
    assert!(
        matches!(&records[14], Log::Refused(reason) if reason.contains("names a swap that is not started"))
    );

    methods.set_swap_tracing(true);
    swap(&mut methods);
    let exact: Vec<_> = core::iter::from_fn(|| methods.take_log()).collect();
    assert_eq!(exact.len(), 5);
    assert!(matches!(&exact[0], Log::Call { tag, .. } if tag == b"A406"));
    assert!(matches!(&exact[1], Log::Call { tag, .. } if tag == b"A407"));
    assert!(matches!(&exact[2], Log::Submitted { swap_id: 258, .. }));
    assert!(matches!(&exact[3], Log::Presented { swap_id: 258, .. }));
    assert!(matches!(&exact[4], Log::Callback { tag, .. } if *tag == SWAP_COMPLETE));
}

#[test]
fn a_layer_left_out_of_the_mask_keeps_showing() {
    let screen = TestScreen(Rc::default(), Some(MODE));
    let frames = screen.0.clone();
    let mut methods = Methods::new(Box::new(screen), interface());
    start_swap(&mut methods);
    returned(methods.call(*b"A407", &submission(1, &[(2, 4, 2, 0x1000)]), 0xc));
    methods
        .present(0, |_, bytes| {
            bytes.fill(1);
            true
        })
        .unwrap();
    complete(&mut methods, 1, true);
    // A swap that enables no layer keeps layer 2: nothing on screen
    // changes, so nothing is read or copied again, and the swap completes
    // as displayed at its vblank.
    start_swap(&mut methods);
    returned(methods.call(*b"A407", &submission(2, &[]), 0xc));
    let period = methods.refresh_ticks;
    methods
        .present(period, |_, _| panic!("an unchanged screen reads nothing"))
        .unwrap();
    complete(&mut methods, 2, true);
    assert_eq!(frames.borrow().len(), 1);
    assert_eq!(frames.borrow()[0], [1, 1, 1, 0xff].repeat(8));
    assert_eq!(methods.shown.unwrap()[2].unwrap().surface.iova, 0x1000);
}

/// Take the next callback, a completion of `swap_id` telling the AP
/// whether it was `displayed`, and answer it.
fn complete(methods: &mut Methods, swap_id: u32, displayed: bool) {
    let callback = methods.take_callback().expect("a completion is due");
    assert_eq!(callback.tag, SWAP_COMPLETE);
    assert_eq!(&callback.input[..4], &swap_id.to_le_bytes());
    assert_eq!(callback.input[SWAP_COMPLETE_DISPLAYED], u8::from(displayed));
    (callback.then)(methods, &[]).unwrap();
}

#[test]
fn several_layers_are_composed_bottom_to_top() {
    let screen = TestScreen(Rc::default(), Some(MODE));
    let frames = screen.0.clone();
    let mut methods = Methods::new(Box::new(screen), interface());
    start_swap(&mut methods);
    // Layer 0 covers the screen; layer 1, one pixel, sits at (3, 1).
    let mut two = submission(1, &[(0, 4, 2, 0x1000), (1, 1, 1, 0x2000)]);
    let at = swap::DESTINATION_RECTS + 16;
    two[at..at + 8].copy_from_slice(&[3, 0, 0, 0, 1, 0, 0, 0]);
    assert_eq!(returned(methods.call(*b"A407", &two, 0xc)), [0; 12]);
    methods
        .present(0, |dva, bytes| {
            bytes.fill(if dva >= 0x2000 { 0xff } else { 0x10 });
            true
        })
        .unwrap();
    let mut want = [0x10, 0x10, 0x10, 0xff].repeat(8);
    want[28..32].fill(0xff);
    assert_eq!(frames.borrow()[0], want);
}

#[test]
fn a_background_swap_clears_and_bad_swaps_refuse() {
    let screen = TestScreen(Rc::default(), Some(MODE));
    let frames = screen.0.clone();
    let mut methods = Methods::new(Box::new(screen), interface());
    // A submit without a start refuses.
    assert!(methods.call(*b"A407", &submission(0, &[]), 0xc).is_err());
    start_swap(&mut methods);
    returned(methods.call(*b"A407", &submission(1, &[]), 0xc));
    methods
        .present(0, |_, _| panic!("a clear reads no surface"))
        .unwrap();
    assert_eq!(frames.borrow()[0], [0, 0, 0, 0xff].repeat(8));
    // A source rectangle outside its surface refuses.
    start_swap(&mut methods);
    let mut outside = submission(2, &[(0, 4, 2, 0x1000)]);
    outside[swap::SOURCE_RECTS] = 1;
    assert!(methods.call(*b"A407", &outside, 0xc).is_err());
    assert!(
        methods
            .last_refusal()
            .unwrap()
            .contains("outside its surface")
    );
    // The refused submission keeps its accepted start until explicit abort.
    returned(methods.call(*b"A479", &[0; 8], 4));
}

#[test]
fn pacing_reports_rate_gaps_stalls_lateness_and_queue_depth() {
    let screen = TestScreen(Rc::default(), Some(MODE));
    let mut methods = Methods::new(Box::new(screen), interface());
    methods.set_swap_tracing(false);
    let refresh = methods.refresh_ticks;
    let submit = |methods: &mut Methods| {
        let id = start_swap(methods);
        returned(methods.call(*b"A407", &submission(id, &[]), 0xc));
    };
    let present = |methods: &mut Methods, now: u64| {
        methods
            .present(now, |_, _| panic!("a clear has no surface DMA"))
            .unwrap();
        while let Some(callback) = methods.take_callback() {
            (callback.then)(methods, &[]).unwrap();
        }
    };
    // One swap a refresh for ten seconds, each looked at 100 ticks after its
    // vblank, but two wait at once at the start (one present, one merged)
    // and one present comes three refreshes late.
    let mut now = 0;
    submit(&mut methods);
    submit(&mut methods);
    present(&mut methods, now);
    while now < PACING_WINDOW - refresh {
        now += if now == 100 * refresh {
            3 * refresh
        } else {
            refresh
        };
        submit(&mut methods);
        present(&mut methods, now + 100);
    }
    let pacing: Vec<_> = core::iter::from_fn(|| methods.take_log())
        .filter_map(|log| match log {
            Log::Pacing(pacing) => Some(pacing),
            _ => None,
        })
        .collect();
    assert!(pacing.is_empty());
    // The window closes at the first vblank ten seconds after it opened,
    // whether or not anything is presented there.
    methods.refresh(PACING_WINDOW + 50);
    let pacing: Vec<_> = core::iter::from_fn(|| methods.take_log())
        .filter_map(|log| match log {
            Log::Pacing(pacing) => Some(pacing),
            _ => None,
        })
        .collect();
    assert_eq!(pacing.len(), 1);
    let pacing = &pacing[0];
    assert_eq!(pacing.ticks, PACING_WINDOW / refresh * refresh);
    assert_eq!(pacing.stalls, 1);
    assert_eq!(pacing.deepest, 2);
    assert_eq!(pacing.merged, 1);
    assert_eq!(pacing.reordered, 0);
    // Gaps are between vblanks: whole periods, however late each look.
    assert_eq!(pacing.shortest, refresh);
    assert_eq!(pacing.longest, 3 * refresh);
    assert_eq!(pacing.total_gap, now);
    assert_eq!(pacing.gaps + 1, pacing.presented);
    assert_eq!(pacing.late_longest, 100);
    assert_eq!(pacing.late_total, 100 * u64::from(pacing.presented - 1));
    assert_eq!(pacing.submitted, pacing.presented + 1);
    let line = pacing.to_string();
    assert!(
        line.starts_with("DCP PACING seconds=10.0 presented=") && line.contains(" merged=1 "),
        "{line}"
    );
    // A window in which nothing was submitted or shown is not reported.
    methods.refresh(2 * PACING_WINDOW + refresh);
    assert!(core::iter::from_fn(|| methods.take_log()).next().is_none());
}

#[test]
fn ready_swaps_latch_at_one_vblank_and_the_newest_is_shown() {
    let screen = TestScreen(Rc::default(), Some(MODE));
    let frames = screen.0.clone();
    let mut methods = Methods::new(Box::new(screen), interface());
    let period = methods.refresh_ticks;
    let fill = |dva: u64, bytes: &mut [u8]| {
        bytes.fill((dva >> 12) as u8);
        true
    };
    // The first look starts the grid.
    methods.refresh(0);
    // Three swaps, each replacing layer 0 with its own surface, all ready
    // before the next vblank.
    for dva in [0x1000, 0x2000, 0x3000] {
        let id = start_swap(&mut methods);
        returned(methods.call(*b"A407", &submission(id, &[(0, 4, 2, dva)]), 0xc));
    }
    // Between vblanks nothing is shown.
    assert!(!methods.frame_ready(period - 1));
    methods.present(period - 1, fill).unwrap();
    assert!(frames.borrow().is_empty() && !methods.callback_pending());
    // At the vblank, one present: the newest. All three complete in order;
    // the two it replaced in every slot they set were not displayed.
    methods.present(period + 7, fill).unwrap();
    assert_eq!(frames.borrow().len(), 1);
    assert_eq!(frames.borrow()[0], [3, 3, 3, 0xff].repeat(8));
    complete(&mut methods, 1, false);
    complete(&mut methods, 2, false);
    complete(&mut methods, 3, true);
    assert!(methods.take_callback().is_none());
    assert_eq!(methods.pacing.merged, 2);
    // A swap submitted between ticks shows at the next tick, not before.
    let id = start_swap(&mut methods);
    returned(methods.call(*b"A407", &submission(id, &[(0, 4, 2, 0x4000)]), 0xc));
    methods.present(period + 100, fill).unwrap();
    methods.present(2 * period - 1, fill).unwrap();
    assert_eq!(frames.borrow().len(), 1);
    methods.present(2 * period, fill).unwrap();
    assert_eq!(frames.borrow().len(), 2);
    complete(&mut methods, id, true);
    // At most one present a vblank, however many looks fall in it.
    let id = start_swap(&mut methods);
    returned(methods.call(*b"A407", &submission(id, &[(0, 4, 2, 0x5000)]), 0xc));
    methods.present(2 * period + 1, fill).unwrap();
    assert_eq!(frames.borrow().len(), 2);
    assert!(methods.frame_pending());
}

#[test]
fn a_merged_swap_whose_surface_stays_on_screen_completes_as_displayed() {
    let screen = TestScreen(Rc::default(), Some(MODE));
    let mut methods = Methods::new(Box::new(screen), interface());
    let fill = |_: u64, bytes: &mut [u8]| {
        bytes.fill(0x40);
        true
    };
    let id = start_swap(&mut methods);
    returned(methods.call(*b"A407", &submission(id, &[(0, 4, 2, 0x1000)]), 0xc));
    methods.present(0, fill).unwrap();
    complete(&mut methods, id, true);
    // Cursor swaps: layer 1, the same surface each time, moved. The first
    // two are merged into the third, and as their surface is still on
    // screen they complete as displayed.
    let cursor = |methods: &mut Methods| {
        let id = start_swap(methods);
        let input = submission(id, &[(1, 1, 1, 0x8000)]);
        let mut input = input;
        input[swap::ENABLED..swap::ENABLED + 4].copy_from_slice(&2u32.to_le_bytes());
        returned(methods.call(*b"A407", &input, 0xc));
        id
    };
    let ids = [
        cursor(&mut methods),
        cursor(&mut methods),
        cursor(&mut methods),
    ];
    methods.present(methods.refresh_ticks, fill).unwrap();
    for id in ids {
        complete(&mut methods, id, true);
    }
}

#[test]
fn a_cursor_only_swap_presents_only_the_rows_its_rectangles_cover() {
    #[derive(Clone, Default)]
    struct Damage(Rc<RefCell<Vec<Option<core::ops::Range<usize>>>>>);
    impl Scanout for Damage {
        fn mode(&self) -> Mode {
            Mode {
                width: 4,
                height: 8,
                format: PixelFormat::Bgra8888,
            }
        }
        fn present(&mut self, _: &[u8]) -> Result<()> {
            unreachable!("rows are taken")
        }
        fn try_present_rows(
            &mut self,
            damage: Option<core::ops::Range<usize>>,
            fill: &mut dyn FnMut(usize, &mut [u8]) -> Result<()>,
        ) -> Result<bool> {
            let mut row = [0; 16];
            for y in damage.clone().unwrap_or(0..8) {
                fill(y, &mut row)?;
            }
            self.0.borrow_mut().push(damage);
            Ok(true)
        }
    }
    let sink = Damage::default();
    let presented = sink.0.clone();
    let mut methods = Methods::new(Box::new(sink), interface());
    let period = methods.refresh_ticks;
    let cursor_at = |methods: &mut Methods, y: u8, base: bool| {
        let id = start_swap(methods);
        let mut layers = alloc::vec![(1, 2, 2, 0x8000)];
        if base {
            layers.insert(0, (0, 4, 8, 0x1000));
        }
        let mut input = submission(id, &layers);
        let at = swap::DESTINATION_RECTS + 16;
        input[at..at + 8].copy_from_slice(&[1, 0, 0, 0, y, 0, 0, 0]);
        if !base {
            input[swap::ENABLED..swap::ENABLED + 4].copy_from_slice(&2u32.to_le_bytes());
        }
        returned(methods.call(*b"A407", &input, 0xc));
    };
    cursor_at(&mut methods, 1, true);
    methods.present(0, |_, _| true).unwrap();
    // The cursor moves from rows 1..3 to rows 5..7: those rows only.
    cursor_at(&mut methods, 5, false);
    methods.present(period, |_, _| true).unwrap();
    // Two moves merged into one vblank: the rows of where it was shown and
    // where it is shown now.
    cursor_at(&mut methods, 3, false);
    cursor_at(&mut methods, 4, false);
    methods.present(2 * period, |_, _| true).unwrap();
    assert_eq!(*presented.borrow(), [None, Some(1..7), Some(4..7)]);
}

#[test]
fn a_swap_waits_for_the_gpu_fences_of_the_surfaces_it_shows() {
    let screen = TestScreen(Rc::default(), Some(MODE));
    let frames = screen.0.clone();
    let mut methods = Methods::new(Box::new(screen), interface());
    methods.set_swap_tracing(false);
    assert_eq!(row(b"A470").name, "io_fence_notify");
    let notify = |methods: &mut Methods, swap: u32, surface: u32| {
        let input = Packed::new()
            .u32(swap)
            .u32(surface)
            .u64(0x1234)
            .u32(0)
            .finish(0x14)
            .unwrap();
        assert_eq!(returned(methods.call(*b"A470", &input, 4)), [0; 4]);
    };
    let layer = [(0, 4, 2, 0x1000)];
    // Each frame's pixels differ: a frame of the same pixels changes no row.
    let fill = |value: u8| {
        move |_: u64, bytes: &mut [u8]| {
            bytes.fill(value);
            true
        }
    };
    let complete = |methods: &mut Methods| {
        let callback = methods.take_callback().unwrap();
        (callback.then)(methods, &[]).unwrap();
    };

    let period = methods.refresh_ticks;
    // Submitted while the GPU still draws surface 5: not shown until the AP
    // names it, and another surface's fence does not count.
    let id = start_swap(&mut methods);
    returned(methods.call(
        *b"A407",
        &fenced_submission(id, &layer, 1, [5, 0, 0, 0]),
        0xc,
    ));
    methods.refresh(0);
    assert!(methods.frame_pending() && !methods.frame_ready(0));
    methods
        .present(0, |_, _| panic!("a fenced surface is not read"))
        .unwrap();
    assert!(frames.borrow().is_empty() && !methods.callback_pending());
    notify(&mut methods, id, 9);
    methods.refresh(period);
    assert!(!methods.frame_ready(period));
    // Ready between vblanks: shown at the next one.
    notify(&mut methods, id, 5);
    assert!(!methods.frame_ready(period + 1));
    methods.refresh(2 * period);
    assert!(methods.frame_ready(2 * period));
    methods.present(2 * period, fill(0x5a)).unwrap();
    assert_eq!(frames.borrow().len(), 1);
    complete(&mut methods);

    // A fence that signals before its swap is submitted is kept for it.
    let id = start_swap(&mut methods);
    notify(&mut methods, id, 5);
    returned(methods.call(
        *b"A407",
        &fenced_submission(id, &layer, 1, [5, 0, 0, 0]),
        0xc,
    ));
    methods.refresh(3 * period);
    assert!(methods.frame_ready(3 * period));
    methods.present(3 * period, fill(0x5b)).unwrap();
    complete(&mut methods);

    // Swaps show in order: a ready swap waits behind a fenced one. Without
    // its notice, the fenced one is shown after the timeout, reported once,
    // and the ready one behind it is latched with it.
    let fenced = start_swap(&mut methods);
    returned(methods.call(
        *b"A407",
        &fenced_submission(fenced, &layer, 1, [5, 0, 0, 0]),
        0xc,
    ));
    let ready = start_swap(&mut methods);
    returned(methods.call(*b"A407", &submission(ready, &layer), 0xc));
    let since = 4 * period;
    methods.refresh(since);
    assert!(!methods.frame_ready(since));
    let timeout = since + FENCE_TIMEOUT.next_multiple_of(period);
    methods.refresh(timeout - period);
    assert!(!methods.frame_ready(timeout - period));
    methods.refresh(timeout);
    assert!(methods.frame_ready(timeout));
    methods.present(timeout, fill(0x5c)).unwrap();
    complete(&mut methods);
    complete(&mut methods);
    assert!(!methods.frame_pending());
    assert_eq!(frames.borrow().len(), 3);
    let timeouts: Vec<_> = core::iter::from_fn(|| methods.take_log())
        .filter(
            |log| matches!(log, Log::FenceTimeout { swap_id, fenced: 1, .. } if *swap_id == fenced),
        )
        .collect();
    assert_eq!(timeouts.len(), 1);
    assert_eq!(
        (methods.pacing.fence_waits, methods.pacing.fence_timeouts),
        (2, 1)
    );

    // A notice for a swap that is gone changes nothing.
    notify(&mut methods, fenced, 5);
    assert!(!methods.frame_pending());
}

#[test]
fn each_set_parameter_dcp_parameter_is_reported_with_its_answer() {
    let mut methods = methods();
    let parameter = |id: u32| {
        let mut packed = Packed::new().u32(id).u64(1);
        for _ in 0..3 {
            packed = packed.u64(0);
        }
        packed.u32(1).finish(0x28).unwrap()
    };
    // WindowServer retries a failure forever: success, value ignored.
    assert_eq!(returned(methods.call(*b"A450", &parameter(6), 4)), [0; 4]);
    assert_eq!(returned(methods.call(*b"A450", &parameter(6), 4)), [0; 4]);
    assert_eq!(returned(methods.call(*b"A450", &parameter(15), 4)), [0; 4]);
    let reported = methods.take_unmodeled();
    assert_eq!(reported.len(), 2, "{reported:?}");
    assert!(reported[0].contains("set_parameter_dcp (0x6): answered success, value ignored"));
    assert!(reported[1].contains("(0xf)"));
    let entry = core::iter::from_fn(|| methods.take_log())
        .find(|entry| matches!(entry, Log::Unmodeled { .. }))
        .unwrap();
    assert_eq!(
        entry.to_string(),
        "DCP unmodeled A450 set_parameter_dcp (0x6): answered success, value ignored \
         (the caller retries a failure forever): the fixed mode runs as it is"
    );
}

#[test]
fn an_abort_drops_only_its_clients_swaps() {
    let screen = TestScreen(Rc::default(), Some(MODE));
    let mut methods = Methods::new(Box::new(screen), interface());
    let kept = start_client_swap(&mut methods, 0x1000);
    let aborted = start_client_swap(&mut methods, 0x3000);
    returned(methods.call(*b"A407", &submission(kept, &[]), 0xc));
    returned(methods.call(*b"A407", &submission(aborted, &[]), 0xc));
    let open_kept = start_client_swap(&mut methods, 0x1000);
    let open_aborted = start_client_swap(&mut methods, 0x3000);
    // abort_swaps_dcp(0x3000).
    let abort = Packed::new().u64(0x3000).finish(8).unwrap();
    assert_eq!(returned(methods.call(*b"A479", &abort, 4)), [0; 4]);
    assert_eq!(started_ids(&methods), [open_kept]);
    assert!(!started_ids(&methods).contains(&open_aborted));
    let queued: Vec<u32> = methods.frames.iter().map(|frame| frame.swap_id).collect();
    assert_eq!(queued, [kept]);
    // The other client's swap is still presented and completed.
    methods.present(0, |_, _| true).unwrap();
    let callback = methods.take_callback().unwrap();
    assert_eq!(callback.tag, SWAP_COMPLETE);
    assert_eq!(&callback.input[..4], &kept.to_le_bytes());
}

#[test]
fn without_tracing_each_method_logs_its_first_call_and_every_refusal() {
    let mut methods = methods();
    methods.set_swap_tracing(false);
    for _ in 0..3 {
        returned(methods.call(*b"A451", &[], 4));
    }
    assert!(methods.call(*b"A451", &[], 8).is_err());
    let calls: Vec<_> = core::iter::from_fn(|| methods.take_log()).collect();
    assert_eq!(calls.len(), 3, "{calls:?}");
    assert!(matches!(&calls[0], Log::Call { tag, output: 4, .. } if tag == b"A451"));
    // The refused call is logged before its refusal.
    assert!(matches!(&calls[1], Log::Call { tag, output: 8, .. } if tag == b"A451"));
    assert!(matches!(&calls[2], Log::Refused(_)));
    // Tracing logs every call.
    methods.set_swap_tracing(true);
    returned(methods.call(*b"A451", &[], 4));
    returned(methods.call(*b"A451", &[], 4));
    assert_eq!(core::iter::from_fn(|| methods.take_log()).count(), 2);
    // The pacing summary counts every call.
    assert_eq!(methods.pacing.calls, 6);
}

#[test]
fn a_waiting_swap_or_a_queued_flip_wakes_the_display_at_its_next_vblank() {
    /// A sink whose frames wait for a flush, as a page flip requested
    /// only on the boot CPU does.
    #[derive(Clone, Default)]
    struct Queued(Rc<RefCell<(bool, usize)>>);
    impl Scanout for Queued {
        fn mode(&self) -> Mode {
            MODE
        }
        fn present(&mut self, _: &[u8]) -> Result<()> {
            self.0.borrow_mut().0 = true;
            Ok(())
        }
        fn queued(&self) -> bool {
            self.0.borrow().0
        }
        fn flush(&mut self) -> Result<()> {
            let mut state = self.0.borrow_mut();
            state.1 += 1;
            state.0 = state.1 < 2;
            Ok(())
        }
    }
    let sink = Queued::default();
    let state = sink.0.clone();
    let mut methods = Methods::new(Box::new(sink), interface());
    let period = methods.refresh_ticks;
    methods.refresh(0);
    assert_eq!(methods.refresh_deadline(), None);
    // No client listens, but a submitted swap needs the next vblank.
    let id = start_swap(&mut methods);
    returned(methods.call(*b"A407", &submission(id, &[(0, 4, 2, 0x1000)]), 0xc));
    assert_eq!(methods.refresh_deadline(), Some(period));
    methods.present(period, |_, _| true).unwrap();
    // The frame's flip is queued: the display needs the next vblank for it,
    // and each look flushes until the sink has nothing queued.
    assert!(!methods.frame_pending());
    assert_eq!(methods.refresh_deadline(), Some(2 * period));
    methods.refresh(period + 1);
    assert_eq!(*state.borrow(), (true, 1));
    methods.refresh(2 * period);
    assert_eq!(*state.borrow(), (false, 2));
    assert_eq!(methods.refresh_deadline(), None);
}

#[test]
fn a_present_that_moves_a_layer_whole_counts_as_moved() {
    let layer = |x, y, width| {
        Some(scanout::Layer {
            surface: scanout::Surface {
                width: 32,
                height: 32,
                stride: 128,
                format: PixelFormat::Bgra8888,
                iova: 0x1000,
            },
            source: scanout::Rect {
                x: 0,
                y: 0,
                width: 32,
                height: 32,
            },
            destination: scanout::Rect {
                x,
                y,
                width,
                height: 32,
            },
            premultiplied: true,
        })
    };
    let shown = [layer(0, 0, 640), layer(10, 20, 32), None, None];
    // The pointer's layer at another place: moved.
    assert!(moved(
        &shown,
        &[layer(0, 0, 640), layer(11, 20, 32), None, None]
    ));
    // Unchanged, resized in place, or shown in a slot empty before: not.
    assert!(!moved(&shown, &shown));
    assert!(!moved(
        &shown,
        &[layer(0, 0, 640), layer(10, 20, 64), None, None]
    ));
    assert!(!moved(
        &shown,
        &[layer(0, 0, 640), layer(10, 20, 32), layer(5, 5, 32), None]
    ));
}

/// Set layer `layer`'s source and destination rectangles in `input`.
fn place(input: &mut [u8], layer: usize, source: swap::Rect, destination: swap::Rect) {
    for (base, rect) in [
        (swap::SOURCE_RECTS, source),
        (swap::DESTINATION_RECTS, destination),
    ] {
        let at = base + layer * 16;
        for (index, value) in [rect.x, rect.y, rect.width, rect.height].iter().enumerate() {
            input[at + index * 4..at + index * 4 + 4].copy_from_slice(&value.to_le_bytes());
        }
    }
}

fn rect(x: u32, y: u32, width: u32, height: u32) -> swap::Rect {
    swap::Rect {
        x,
        y,
        width,
        height,
    }
}

/// The opaque BGRA pixel a pattern surface holds at `(x, y)`.
fn pattern_pixel(x: usize, y: usize) -> [u8; 4] {
    [x as u8, y as u8, ((x >> 8) | (y >> 8) << 4) as u8, 0xff]
}

/// A reader of pattern surfaces `width` pixels wide whose rows are `width *
/// 4` bytes apart, at `bases`.
fn pattern(bases: &[(u64, usize)]) -> impl FnMut(u64, &mut [u8]) -> bool + '_ {
    move |dva, bytes| {
        let Some(&(base, width)) = bases.iter().rev().find(|&&(base, _)| dva >= base) else {
            return false;
        };
        let first = ((dva - base) / 4) as usize;
        for (index, pixel) in bytes.as_chunks_mut::<4>().0.iter_mut().enumerate() {
            let at = first + index;
            *pixel = pattern_pixel(at % width, at / width);
        }
        true
    }
}

#[test]
fn a_half_size_source_fills_the_screen_with_exact_2x_pixels() {
    let mode = Mode {
        width: 2560,
        height: 1440,
        format: PixelFormat::Bgra8888,
    };
    let screen = TestScreen(Rc::default(), Some(mode));
    let frames = screen.0.clone();
    let mut methods = Methods::new(Box::new(screen), interface());
    start_swap(&mut methods);
    // A 2560x1440 surface whose top-left 1280x720 the AP shows on the whole
    // screen: the DCP scales it 2x.
    let mut input = submission(1, &[(0, 2560, 1440, 0x1000_0000)]);
    place(&mut input, 0, rect(0, 0, 1280, 720), rect(0, 0, 2560, 1440));
    returned(methods.call(*b"A407", &input, 0xc));
    methods.present(0, pattern(&[(0x1000_0000, 2560)])).unwrap();
    let frames = frames.borrow();
    let frame = &frames[0];
    for y in 0..1440 {
        for x in 0..2560 {
            let at = (y * 2560 + x) * 4;
            assert_eq!(frame[at..at + 4], pattern_pixel(x / 2, y / 2), "({x}, {y})");
        }
    }
    // The guest's shape is logged, once.
    let placed: Vec<String> = core::iter::from_fn(|| methods.take_log())
        .filter(|entry| matches!(entry, Log::Placement { .. }))
        .map(|entry| entry.to_string())
        .collect();
    assert_eq!(
        placed,
        [
            "DCP LAYER swap 1 layer 0 surface=2560x1440 source=0,0+1280x720 destination=0,0+2560x1440"
        ]
    );
}

#[test]
fn a_cropped_offset_source_and_an_unscaled_layer_compose_together() {
    let mode = Mode {
        width: 8,
        height: 4,
        format: PixelFormat::Bgra8888,
    };
    let screen = TestScreen(Rc::default(), Some(mode));
    let frames = screen.0.clone();
    let mut methods = Methods::new(Box::new(screen), interface());
    start_swap(&mut methods);
    // Layer 0: the 4x2 at (2, 1) of an 8x4 surface, scaled 2x over the
    // screen. Layer 1: a 2x1 surface unscaled at (5, 2).
    let mut input = submission(1, &[(0, 8, 4, 0x1000), (1, 2, 1, 0x8000)]);
    place(&mut input, 0, rect(2, 1, 4, 2), rect(0, 0, 8, 4));
    place(&mut input, 1, rect(0, 0, 2, 1), rect(5, 2, 2, 1));
    returned(methods.call(*b"A407", &input, 0xc));
    methods
        .present(0, pattern(&[(0x1000, 8), (0x8000, 2)]))
        .unwrap();
    let frames = frames.borrow();
    for y in 0..4 {
        for x in 0..8 {
            let want = if y == 2 && (5..7).contains(&x) {
                pattern_pixel(x - 5, 0)
            } else {
                pattern_pixel(2 + x / 2, 1 + y / 2)
            };
            let at = (y * 8 + x) * 4;
            assert_eq!(frames[0][at..at + 4], want, "({x}, {y})");
        }
    }
}

#[test]
fn a_scaled_layer_damages_the_screen_rows_its_destination_covers() {
    #[derive(Clone, Default)]
    struct Damage(Rc<RefCell<Vec<Option<core::ops::Range<usize>>>>>);
    impl Scanout for Damage {
        fn mode(&self) -> Mode {
            Mode {
                width: 8,
                height: 16,
                format: PixelFormat::Bgra8888,
            }
        }
        fn present(&mut self, _: &[u8]) -> Result<()> {
            unreachable!("rows are taken")
        }
        fn try_present_rows(
            &mut self,
            damage: Option<core::ops::Range<usize>>,
            fill: &mut dyn FnMut(usize, &mut [u8]) -> Result<()>,
        ) -> Result<bool> {
            let mut row = [0; 32];
            for y in damage.clone().unwrap_or(0..16) {
                fill(y, &mut row)?;
            }
            self.0.borrow_mut().push(damage);
            Ok(true)
        }
    }
    let sink = Damage::default();
    let presented = sink.0.clone();
    let mut methods = Methods::new(Box::new(sink), interface());
    let period = methods.refresh_ticks;
    // A full-screen base, then layer 1: a 2x2 source shown 2x at row `y`.
    let id = start_swap(&mut methods);
    let mut input = submission(id, &[(0, 8, 16, 0x1000), (1, 2, 2, 0x8000)]);
    place(&mut input, 1, rect(0, 0, 2, 2), rect(1, 3, 4, 4));
    returned(methods.call(*b"A407", &input, 0xc));
    methods.present(0, |_, _| true).unwrap();
    // Moved down: the rows it covered (3..7) and covers (9..13), in source
    // rows 0..2 scaled to four screen rows each time.
    let id = start_swap(&mut methods);
    let mut input = submission(id, &[(1, 2, 2, 0x8000)]);
    place(&mut input, 1, rect(0, 0, 2, 2), rect(1, 9, 4, 4));
    returned(methods.call(*b"A407", &input, 0xc));
    methods.present(period, |_, _| true).unwrap();
    // Shown at its own size in place: the old scaled rows and the new ones.
    let id = start_swap(&mut methods);
    let mut input = submission(id, &[(1, 2, 2, 0x8000)]);
    place(&mut input, 1, rect(0, 0, 2, 2), rect(1, 9, 2, 2));
    returned(methods.call(*b"A407", &input, 0xc));
    methods.present(2 * period, |_, _| true).unwrap();
    assert_eq!(*presented.borrow(), [None, Some(3..13), Some(9..13)]);
}

#[test]
fn surface_layouts_are_reported_once_up_to_a_bound() {
    let mut methods = methods();
    let mut surfaces = Vec::new();
    for width in (1..=LAYOUTS as u32 + 6).chain([1, 2]) {
        let id = start_swap(&mut methods);
        returned(methods.call(*b"A407", &submission(id, &[(0, width, 1, 0x1000)]), 0xc));
        surfaces.extend(
            methods
                .take_unmodeled()
                .into_iter()
                .filter(|line| line.contains("SURFACE")),
        );
    }
    assert_eq!(surfaces.len(), LAYOUTS + 1, "{surfaces:?}");
    assert!(
        surfaces[LAYOUTS - 1].starts_with(&format!("SURFACE format=BGRA (0x42475241) {LAYOUTS}x1"))
    );
    assert_eq!(
        surfaces[LAYOUTS],
        format!("SURFACE layouts after the first {LAYOUTS} are not listed")
    );
    assert_eq!(methods.layouts.len(), LAYOUTS);
}

#[test]
fn degraded_answers_are_reported_once_up_to_a_bound() {
    let mut methods = methods();
    let mut lines = Vec::new();
    for mode in (1..=UNMODELED_REPORTS as u32 + 3).chain([1]) {
        let reply = returned(methods.call(*b"A426", &mode.to_le_bytes(), 4));
        assert_eq!(status(&reply, 0), LINK_UNSUPPORTED);
        lines.extend(methods.take_unmodeled());
    }
    assert_eq!(lines.len(), UNMODELED_REPORTS + 1);
    assert_eq!(
        lines[UNMODELED_REPORTS],
        format!("IOMFB unmodeled answers after the first {UNMODELED_REPORTS} are not listed")
    );
    assert_eq!(methods.unmodeled.len(), UNMODELED_REPORTS);
}

#[test]
fn a_submit_past_the_held_swaps_refuses_and_keeps_its_swap_started() {
    let mut methods = methods();
    for _ in 0..QUEUED_SWAPS {
        let id = start_swap(&mut methods);
        returned(methods.call(*b"A407", &submission(id, &[]), 0xc));
    }
    let id = start_swap(&mut methods);
    assert!(methods.call(*b"A407", &submission(id, &[]), 0xc).is_err());
    assert_eq!(methods.frames.len(), QUEUED_SWAPS);
    assert!(methods.started.iter().any(|&(started, _)| started == id));
}

#[test]
fn pacing_times_each_stage_of_a_swap_and_the_vsyncs() {
    let screen = TestScreen(Rc::default(), Some(MODE));
    let mut methods = Methods::new(Box::new(screen), interface());
    methods.set_swap_tracing(false);
    methods.notified = true;
    let period = methods.refresh_ticks;
    // Answer every callback the AP is sent, at counter `at`.
    let answer_all = |methods: &mut Methods, at: u64| {
        methods.observe(at);
        while let Some(callback) = methods.take_callback() {
            (callback.then)(methods, &[]).unwrap();
        }
    };
    // The grid starts at 0; its vsync is answered 100 ticks later.
    methods.refresh(0);
    answer_all(&mut methods, 100);
    assert_eq!(methods.pacing.vsync.count, 1);
    assert_eq!(methods.pacing.vsync.longest, 100);

    // Submitted at 1000 while the GPU draws surface 5; the AP names it at
    // half a period, 30 ticks after its own time of the signal.
    methods.observe(1000);
    let id = start_swap(&mut methods);
    returned(methods.call(
        *b"A407",
        &fenced_submission(id, &[(0, 4, 2, 0x1000)], 1, [5, 0, 0, 0]),
        0xc,
    ));
    let notice = period / 2;
    methods.observe(notice);
    let input = Packed::new()
        .u32(id)
        .u32(5)
        .u64(notice - 30)
        .u32(0)
        .finish(0x14)
        .unwrap();
    returned(methods.call(*b"A470", &input, 4));
    // A time the AP's clock puts ahead of the model is not counted.
    let ahead = Packed::new()
        .u32(id)
        .u32(5)
        .u64(notice + 1)
        .u32(0)
        .finish(0x14)
        .unwrap();
    returned(methods.call(*b"A470", &ahead, 4));
    assert_eq!(methods.pacing.fence.count, 1);
    assert_eq!(methods.pacing.fence.total, notice - 1000);
    assert_eq!(methods.pacing.notice.count, 1);
    assert_eq!(methods.pacing.notice.total, 30);

    // Latched at the next vblank, presented 5 ticks after it; its vsync and
    // completion are answered 50 ticks after the vblank.
    methods
        .present(period + 5, |_, bytes: &mut [u8]| {
            bytes.fill(0x5a);
            true
        })
        .unwrap();
    assert_eq!(methods.pacing.latch.count, 1);
    assert_eq!(methods.pacing.latch.total, period + 5 - notice);
    answer_all(&mut methods, period + 50);
    assert_eq!(methods.pacing.vsync.count, 2);
    assert_eq!(methods.pacing.vsync.total, 150);
    assert_eq!(methods.pacing.complete.count, 1);
    assert_eq!(methods.pacing.complete.total, 45);
    assert_eq!(methods.pacing.total.total, period + 50 - 1000);

    // An unfenced swap is ready when submitted.
    methods.observe(period + 60);
    let id = start_swap(&mut methods);
    returned(methods.call(*b"A407", &submission(id, &[(0, 4, 2, 0x1000)]), 0xc));
    methods
        .present(2 * period, |_, bytes: &mut [u8]| {
            bytes.fill(0x5b);
            true
        })
        .unwrap();
    assert_eq!(methods.pacing.fence.count, 1);
    assert_eq!(methods.pacing.latch.longest, period - 60);
    let line = methods.pacing.to_string();
    assert!(
        line.contains(" swap_ms=n:avg/max fence=1:") && line.contains(" vsync=2:"),
        "{line}"
    );
}

/// A screen whose frames another thread can push, as a compose outside the
/// devices' lock does.
#[derive(Clone, Default)]
struct SharedScreen(alloc::sync::Arc<std::sync::Mutex<Vec<Vec<u8>>>>);
impl Scanout for SharedScreen {
    fn mode(&self) -> Mode {
        MODE
    }
    fn present(&mut self, rows: &[u8]) -> Result<()> {
        self.0.lock().unwrap().push(rows.to_vec());
        Ok(())
    }
}

/// Guest RAM holding the DART's pages for test surfaces: I/O address
/// `dva` is at `GUEST + dva`.
const GUEST: u64 = 0x8_0000_0000;
const DART_PAGE: u64 = 0x4000;

fn guest_ram() -> stela_contracts::dma::test_support::Ram {
    let mut ram = stela_contracts::dma::test_support::Ram::new(GUEST, 0x10000);
    for (index, byte) in ram.bytes.iter_mut().enumerate() {
        *byte = (index * 7 + index / 64) as u8;
    }
    ram
}

fn resolve(dva: u64) -> core::result::Result<u64, stela_contracts::dma::DmaError> {
    if dva < 0x10000 {
        Ok(GUEST + dva)
    } else {
        Err(stela_contracts::dma::DmaError::Fault(1))
    }
}

/// The swaps of a two-layer frame, then one moving its small layer.
fn compose_swaps(methods: &mut Methods) -> [Vec<u8>; 2] {
    let mut submits = [Vec::new(), Vec::new()];
    for (index, x) in [(0, 1u32), (1, 3)] {
        let id = start_swap(methods);
        let mut input = submission(id, &[(0, 4, 2, 0x1000), (1, 2, 2, 0x5000)]);
        place(&mut input, 1, rect(0, 0, 1, 1), rect(x, 1, 1, 1));
        submits[index] = input;
    }
    submits
}

/// Each present of [`compose_swaps`], in place and composed outside the
/// lock, shows the same frames and completes the same swaps, and a
/// compose's swaps complete only once it is finished.
#[test]
fn a_compose_outside_the_lock_presents_as_in_place() {
    let mut shown = Vec::new();
    for deferred in [false, true] {
        let screen = SharedScreen::default();
        let frames = screen.0.clone();
        let mut methods = Methods::new(Box::new(screen), interface());
        let mut ram = guest_ram();
        let submits = compose_swaps(&mut methods);
        let mut completed = Vec::new();
        for (step, input) in submits.iter().enumerate() {
            assert_eq!(returned(methods.call(*b"A407", input, 0xc)), [0; 12]);
            let now = step as u64 * 1_000_000;
            if deferred {
                let compose = methods
                    .compose(now, DART_PAGE, resolve)
                    .expect("pages resolve");
                assert!(methods.composing());
                assert!(
                    methods.compose(now, DART_PAGE, resolve).is_none(),
                    "one at a time"
                );
                methods
                    .present(now, |_, _| panic!("no present in place while composing"))
                    .unwrap();
                assert!(
                    methods.take_callback().is_none(),
                    "not complete before the compose"
                );
                assert!(!methods.finish_compose(0).unwrap(), "not run yet");
                compose.run(&mut ram);
                assert!(methods.finish_compose(0).unwrap());
                assert!(!methods.composing());
            } else {
                methods
                    .present(now, |dva, bytes| {
                        use stela_contracts::dma::Memory;
                        ram.read(GUEST + dva, bytes)
                    })
                    .unwrap();
            }
            while let Some(callback) = methods.take_callback() {
                completed.push((callback.tag, callback.input[..4].to_vec()));
                (callback.then)(&mut methods, &[]).unwrap();
            }
        }
        shown.push((frames.lock().unwrap().clone(), completed));
    }
    assert_eq!(shown[0].0.len(), 2);
    assert_eq!(shown[0], shown[1]);
}

/// A compose that reaches a page the DART does not map is not made: the
/// present happens in place, where the fault has its effects.
#[test]
fn a_layer_with_an_unresolved_page_presents_in_place() {
    let mut methods = Methods::new(Box::new(SharedScreen::default()), interface());
    let id = start_swap(&mut methods);
    let input = submission(id, &[(0, 4, 2, 0x10000)]);
    assert_eq!(returned(methods.call(*b"A407", &input, 0xc)), [0; 12]);
    assert!(methods.compose(0, DART_PAGE, resolve).is_none());
    assert!(!methods.composing());
    methods
        .present(0, |_, bytes| {
            bytes.fill(1);
            true
        })
        .unwrap();
    assert_eq!(methods.take_callback().unwrap().tag, SWAP_COMPLETE);
}

/// An abort waits for a compose in flight on another PE, whose swaps then
/// complete before the abort drops the rest.
#[test]
fn an_abort_waits_for_the_compose_in_flight() {
    struct Sent(
        super::super::compose::Compose,
        stela_contracts::dma::test_support::Ram,
    );
    // SAFETY: the test's screen is shared through a mutex, and nothing
    // else in the compose is touched by the main thread meanwhile.
    unsafe impl Send for Sent {}
    impl Sent {
        fn run(self) {
            let Self(compose, mut ram) = self;
            compose.run(&mut ram);
        }
    }
    let screen = SharedScreen::default();
    let frames = screen.0.clone();
    let mut methods = Methods::new(Box::new(screen), interface());
    let shown = start_client_swap(&mut methods, 0x1000);
    returned(methods.call(*b"A407", &submission(shown, &[(0, 4, 2, 0x1000)]), 0xc));
    let compose = methods
        .compose(0, DART_PAGE, resolve)
        .expect("pages resolve");
    let dropped = start_client_swap(&mut methods, 0x1000);
    returned(methods.call(*b"A407", &submission(dropped, &[(0, 4, 2, 0x1000)]), 0xc));
    let sent = Sent(compose, guest_ram());
    let thread = std::thread::spawn(move || {
        std::thread::sleep(std::time::Duration::from_millis(5));
        sent.run();
    });
    let abort = Packed::new().u64(0x1000).finish(8).unwrap();
    assert_eq!(returned(methods.call(*b"A479", &abort, 4)), [0; 4]);
    thread.join().unwrap();
    assert!(!methods.composing());
    assert_eq!(frames.lock().unwrap().len(), 1);
    assert!(methods.frames.is_empty());
    let callback = methods.take_callback().unwrap();
    assert_eq!(callback.tag, SWAP_COMPLETE);
    assert_eq!(&callback.input[..4], &shown.to_le_bytes());
}

/// Host micro-benchmark: the time a full-HD present holds the devices'
/// lock in place, against a compose's resolve and finish under it and its
/// compose outside. `cargo test --release -p stela-soc --lib
/// compose_lock_costs -- --ignored --nocapture`.
#[test]
#[ignore = "manual: opt-in"]
fn compose_lock_costs() {
    #[derive(Clone, Default)]
    struct Hd(alloc::sync::Arc<std::sync::Mutex<Vec<u8>>>);
    impl Scanout for Hd {
        fn mode(&self) -> Mode {
            FULL_HD
        }
        fn present(&mut self, rows: &[u8]) -> Result<()> {
            self.0.lock().unwrap().push(rows[0]);
            Ok(())
        }
    }
    let bytes = 1920 * 1080 * 4;
    let mut ram = stela_contracts::dma::test_support::Ram::new(GUEST, bytes + 0x10000);
    let resolve =
        |dva: u64| -> core::result::Result<u64, stela_contracts::dma::DmaError> { Ok(GUEST + dva) };
    const ROUNDS: u32 = 20;
    let mut times = [0u128; 4];
    for deferred in [false, true] {
        let mut methods = Methods::new(Box::new(Hd::default()), interface());
        for round in 0..ROUNDS {
            // Every frame changes every row.
            for (index, byte) in ram.bytes.iter_mut().enumerate().step_by(1920 * 4) {
                *byte = (index as u32 + round) as u8;
            }
            let id = start_swap(&mut methods);
            let input = submission(id, &[(0, 1920, 1080, 0)]);
            returned(methods.call(*b"A407", &input, 0xc));
            let now = u64::from(round) * 1_000_000;
            if deferred {
                let started = std::time::Instant::now();
                let compose = methods.compose(now, DART_PAGE, resolve).unwrap();
                times[1] += started.elapsed().as_nanos();
                let started = std::time::Instant::now();
                compose.run(&mut ram);
                times[2] += started.elapsed().as_nanos();
                let started = std::time::Instant::now();
                assert!(methods.finish_compose(0).unwrap());
                times[3] += started.elapsed().as_nanos();
            } else {
                let started = std::time::Instant::now();
                methods
                    .present(now, |dva, bytes| {
                        use stela_contracts::dma::Memory;
                        ram.read(GUEST + dva, bytes)
                    })
                    .unwrap();
                times[0] += started.elapsed().as_nanos();
            }
            while let Some(callback) = methods.take_callback() {
                (callback.then)(&mut methods, &[]).unwrap();
            }
        }
    }
    let us = |nanos: u128| nanos / u128::from(ROUNDS) / 1000;
    std::println!(
        "full-HD present: in place {} us under the lock; deferred {} us resolve + {} us finish under it, {} us compose outside",
        us(times[0]),
        us(times[1]),
        us(times[3]),
        us(times[2]),
    );
}

/// An abort while a compose that never runs is in flight is refused after a
/// bounded wait, keeps every swap and is reported once; once the compose
/// runs, the next abort settles it and proceeds.
#[test]
fn an_abort_never_waits_for_good_for_a_compose_that_does_not_run() {
    let mut methods = Methods::new(Box::new(SharedScreen::default()), interface());
    let shown = start_client_swap(&mut methods, 0x1000);
    returned(methods.call(*b"A407", &submission(shown, &[(0, 4, 2, 0x1000)]), 0xc));
    let compose = methods
        .compose(0, DART_PAGE, resolve)
        .expect("pages resolve");
    assert!(methods.settle_compose_within(1000).is_err());
    assert!(methods.settle_compose_within(1000).is_err());
    let reported = core::iter::from_fn(|| methods.take_log())
        .filter(|entry| matches!(entry, Log::Refused(text) if text.contains("did not end in time")))
        .count();
    assert_eq!(reported, 1, "reported once");
    assert!(
        methods.composing() && methods.frame_pending(),
        "the swap is kept"
    );
    compose.run(&mut guest_ram());
    let abort = Packed::new().u64(0x1000).finish(8).unwrap();
    assert_eq!(returned(methods.call(*b"A479", &abort, 4)), [0; 4]);
    assert!(!methods.composing());
    assert_eq!(methods.take_callback().unwrap().tag, SWAP_COMPLETE);
}

#[test]
fn a_refresh_that_cannot_be_advertised_is_published_at_the_usual_rate_and_said() {
    // 1920x1080 at 2000 Hz has no detailed timing.
    assert!(Advertised::at(FULL_HD, 2_000_000).is_err());
    let methods = Methods::new(Box::new(FastScreen(2_000_000)), interface());
    let advertised = methods.advertised.as_ref().unwrap();
    assert_eq!(advertised.edid, Advertised::new(FULL_HD).unwrap().edid);
    assert_eq!(methods.reported.len(), 1);
    assert!(methods.reported[0].contains("2000000 mHz display is advertised at 60000 mHz"));
    // One that can be is published as it is, with nothing said.
    let methods = Methods::new(Box::new(FastScreen(240_000)), interface());
    assert_eq!(
        methods.advertised.as_ref().unwrap().edid,
        Advertised::at(FULL_HD, 240_000).unwrap().edid
    );
    assert!(methods.reported.is_empty());
}

/// A screen that records each power change and each present, in order.
#[derive(Clone, Default)]
struct PoweredScreen(Rc<RefCell<Vec<&'static str>>>, bool);
impl Scanout for PoweredScreen {
    fn mode(&self) -> Mode {
        MODE
    }
    fn present(&mut self, _: &[u8]) -> Result<()> {
        self.0.borrow_mut().push("present");
        Ok(())
    }
    fn set_power(&mut self, on: bool) -> Result<()> {
        if self.1 {
            return Err(refuse!("no power control"));
        }
        self.0.borrow_mut().push(if on { "on" } else { "off" });
        Ok(())
    }
}

#[test]
fn display_sleep_turns_the_monitor_off_and_a_wake_lights_it_before_the_next_swap() {
    let screen = PoweredScreen::default();
    let events = screen.0.clone();
    let mut methods = Methods::new(Box::new(screen), interface());
    let period = methods.refresh_ticks;
    let fill = |_: u64, bytes: &mut [u8]| {
        bytes.fill(1);
        true
    };
    let on = Packed::new().u64(1).finish(0x10).unwrap();
    let off = Packed::new().u64(0).finish(0x10).unwrap();
    // The monitor is lit from the start: turning it on changes nothing.
    returned(methods.call(*b"A484", &on, 8));
    assert!(events.borrow().is_empty());
    returned(methods.call(*b"A484", &off, 8));
    returned(methods.call(*b"A484", &off, 8));
    assert_eq!(*events.borrow(), ["off"]);
    // A swap while the display sleeps is not shown.
    methods.refresh(0);
    let id = start_swap(&mut methods);
    returned(methods.call(*b"A407", &submission(id, &[(0, 4, 2, 0x1000)]), 0xc));
    methods.present(period, fill).unwrap();
    assert_eq!(*events.borrow(), ["off"]);
    // The wake lights the monitor, then the next vblank shows the swap.
    returned(methods.call(*b"A484", &on, 8));
    methods.present(2 * period, fill).unwrap();
    assert_eq!(*events.borrow(), ["off", "on", "present"]);
    let reported = methods.take_unmodeled();
    assert!(
        reported
            .iter()
            .all(|line| !line.starts_with("IOMFB display")),
        "{reported:?}"
    );
}

#[test]
fn display_sleep_follows_the_controllers_sleep_and_a_refusing_sink_is_reported_once() {
    let screen = PoweredScreen(Rc::default(), true);
    let events = screen.0.clone();
    let mut methods = Methods::new(Box::new(screen), interface());
    let display = methods.display_state();
    // The IOAV controller's display sleep reaches the scanout at the next look.
    display.borrow_mut().asleep = true;
    assert!(methods.refresh_deadline().is_some());
    methods.refresh(0);
    display.borrow_mut().asleep = false;
    methods.refresh(1);
    assert!(events.borrow().is_empty());
    let reported = methods.take_unmodeled();
    assert_eq!(reported.len(), 1, "{reported:?}");
    assert!(reported[0].contains("no power control"), "{reported:?}");
}
