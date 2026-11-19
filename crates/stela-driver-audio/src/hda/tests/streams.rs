// SPDX-License-Identifier: GPL-2.0-only
//! streams behavior tests.
use super::*;

#[test]
fn pcm_format_encoding() {
    for rate in [
        8000, 11025, 16000, 22050, 32000, 44100, 48000, 88200, 96000, 176400, 192000,
    ] {
        for bits in [8, 16, 20, 24, 32] {
            let format = PcmFormat {
                rate,
                bits,
                channels: 2,
            };
            assert!(format.encode().is_ok());
            assert_eq!(
                format.frame_bytes().unwrap(),
                if bits == 8 {
                    2
                } else if bits == 16 {
                    4
                } else {
                    8
                }
            );
        }
    }
    assert_eq!(
        PcmFormat {
            rate: 12_345,
            ..pcm()
        }
        .encode(),
        Err(Error::InvalidFormat)
    );
    assert_eq!(
        PcmFormat {
            channels: 0,
            ..pcm()
        }
        .encode(),
        Err(Error::InvalidFormat)
    );
}

#[test]
fn verb_field_bounds() {
    assert_eq!(Verb::short(15, 0, 0xf00, 0), Err(Error::InvalidVerb));
    assert_eq!(Verb::short(0, 128, 0xf00, 0), Err(Error::InvalidVerb));
    assert_eq!(Verb::long(0, 2, 2, 0x4011).unwrap().raw(), 0x0022_4011);
}

#[test]
fn stream_buffer_preflight() {
    let mut fixture = Fixture::new();
    let model = Model::new(16);
    let mut controller = create(&model, &mut fixture, config()).unwrap();
    let count = model.borrow().writes.len();
    for (period, periods) in [(0, 2), (127, 2), (128, 1), (4096, 257), (128 * 33, 256)] {
        assert_eq!(
            controller.allocate_stream(
                Direction::Playback,
                pcm(),
                period,
                periods,
                &mut fixture.allocator
            ),
            Err(Error::InvalidBuffer)
        );
    }
    assert_eq!(model.borrow().writes.len(), count);
    let id = controller
        .allocate_stream(Direction::Capture, pcm(), 128, 2, &mut fixture.allocator)
        .unwrap();
    assert_eq!(id.descriptor_index(), 0);
    assert_eq!(controller.write_pcm(id, 0, &[0]), Err(Error::InvalidBuffer));
}

#[test]
fn split_bdl_streaming() {
    let mut fixture = Fixture::new();
    let model = Model::new(16);
    let mut controller = create(&model, &mut fixture, config()).unwrap();
    let playback = controller
        .allocate_stream(Direction::Playback, pcm(), 1536, 4, &mut fixture.allocator)
        .unwrap();
    let capture = controller
        .allocate_stream(Direction::Capture, pcm(), 128, 2, &mut fixture.allocator)
        .unwrap();
    assert_eq!(playback.descriptor_index(), 2);
    assert_ne!(playback.tag(), capture.tag());
    let samples: Vec<u8> = (0..6144).map(|index| index as u8).collect();
    controller.write_pcm(playback, 0, &samples).unwrap();
    controller.start(playback).unwrap();
    controller.start(capture).unwrap();
    assert_eq!(
        controller.write_pcm(playback, 0, &[1]),
        Err(Error::StreamRunning)
    );
    assert_eq!(
        controller.read_pcm(capture, 0, &mut [0]),
        Err(Error::StreamRunning)
    );
    model.borrow_mut().dma_step_bytes = 128;
    advance(&model, 100);
    let status = controller.status(playback).unwrap();
    assert!(status.running && status.period_completed);
    assert!(status.position_bytes < 6144);
    assert!(!controller.status(playback).unwrap().period_completed);
    controller.stop(playback).unwrap();
    controller.stop(capture).unwrap();
    let m = model.borrow();
    assert!(m.stream_descriptors_seen > 5);
    assert!(m.playback.len() > 6144);
    for (index, byte) in m.playback.iter().enumerate() {
        assert_eq!(*byte, samples[index % samples.len()]);
    }
    drop(m);
    let mut captured = [0; 256];
    controller.read_pcm(capture, 0, &mut captured).unwrap();
    assert_eq!(captured, core::array::from_fn(|index| index as u8));
    assert_eq!(
        controller.read_pcm(capture, usize::MAX, &mut [0]),
        Err(Error::InvalidBuffer)
    );
    controller.shutdown().unwrap();
    assert_eq!(controller.start(playback), Err(Error::Shutdown));
}

#[test]
fn failed_stop_keeps_buffer_ownership() {
    for fault in [SD_FIFO_ERROR, SD_DESC_ERROR] {
        let mut fixture = Fixture::new();
        let model = Model::new(16);
        let mut controller = create(&model, &mut fixture, config()).unwrap();
        let id = controller
            .allocate_stream(Direction::Playback, pcm(), 128, 2, &mut fixture.allocator)
            .unwrap();
        controller.start(id).unwrap();
        model.borrow_mut().bytes[stream_base(id.descriptor_index()) + SD_STS] = 0x80 | fault;
        assert_eq!(
            controller.status(id),
            Err(Error::StreamDma {
                status: 0x80 | fault
            })
        );
        assert_eq!(
            model.borrow().bytes[stream_base(id.descriptor_index()) + SD_STS],
            0x80
        );
        assert_eq!(controller.write_pcm(id, 0, &[0]), Err(Error::NeedsReset));
    }
    let mut fixture = Fixture::new();
    let model = Model::new(16);
    let mut controller = create(&model, &mut fixture, config()).unwrap();
    let id = controller
        .allocate_stream(Direction::Playback, pcm(), 128, 2, &mut fixture.allocator)
        .unwrap();
    controller.start(id).unwrap();
    model.borrow_mut().stream_stop_stuck = true;
    assert_eq!(controller.stop(id), Err(Error::Timeout(Phase::StreamStop)));
    assert_eq!(controller.write_pcm(id, 0, &[0]), Err(Error::NeedsReset));
    assert_eq!(
        controller.shutdown(),
        Err(Error::Timeout(Phase::StreamStop))
    );
    assert_ne!(
        model.borrow().get(stream_base(id.descriptor_index()), 1) & u32::from(SD_RUN),
        0
    );
}

#[test]
fn reset_start_and_position_failures() {
    let mut fixture = Fixture::new();
    let model = Model::new(16);
    let mut controller = create(&model, &mut fixture, config()).unwrap();
    model.borrow_mut().stream_reset_stuck = true;
    assert_eq!(
        controller.allocate_stream(Direction::Playback, pcm(), 128, 2, &mut fixture.allocator),
        Err(Error::Timeout(Phase::StreamResetEnter))
    );
    model.borrow_mut().stream_reset_stuck = false;
    let id = controller
        .allocate_stream(Direction::Playback, pcm(), 128, 2, &mut fixture.allocator)
        .unwrap();
    model.borrow_mut().stream_start_stuck = true;
    assert_eq!(
        controller.start(id),
        Err(Error::Timeout(Phase::StreamStart))
    );
    assert_eq!(controller.write_pcm(id, 0, &[0]), Err(Error::NeedsReset));

    let model = Model::new(16);
    let mut controller = create(&model, &mut fixture, config()).unwrap();
    let id = controller
        .allocate_stream(Direction::Playback, pcm(), 128, 2, &mut fixture.allocator)
        .unwrap();
    model
        .borrow_mut()
        .set(stream_base(id.descriptor_index()) + SD_LPIB, 256, 4);
    assert_eq!(controller.position(id), Ok(0));
    assert_eq!(controller.status(id).unwrap().position_bytes, 0);
    controller.start(id).unwrap(); // Terminal/wrap position is valid here too.
    controller.stop(id).unwrap();
    model
        .borrow_mut()
        .set(stream_base(id.descriptor_index()) + SD_LPIB, 257, 4);
    assert_eq!(controller.position(id), Err(Error::InvalidPosition(257)));
}

#[test]
fn converter_graph_and_readback() {
    let mut fixture = Fixture::new();
    let model = graph_model();
    let mut controller = create(&model, &mut fixture, config()).unwrap();
    let graph = controller.enumerate_codec(0).unwrap();
    let id = controller
        .allocate_stream(Direction::Playback, pcm(), 128, 2, &mut fixture.allocator)
        .unwrap();
    controller.bind_converter(&graph, 2, id, 0).unwrap();
    assert_eq!(model.borrow().formats.get(&2), Some(&0x11));
    assert_eq!(model.borrow().bindings.get(&2), Some(&(id.tag() << 4)));
    assert_eq!(
        controller.bind_converter(&graph, 3, id, 0),
        Err(Error::UnsupportedTopology)
    );
    let unsupported = controller
        .allocate_stream(
            Direction::Playback,
            PcmFormat {
                rate: 96_000,
                ..pcm()
            },
            128,
            2,
            &mut fixture.allocator,
        )
        .unwrap();
    assert_eq!(
        controller.bind_converter(&graph, 2, unsupported, 0),
        Err(Error::InvalidFormat)
    );
}

#[test]
fn dma_ring_and_mmio_bounds() {
    let mut fixture = Fixture::new();
    let model = Model::new(16);
    model.borrow_mut().bytes[GCAP] &= !1;
    assert!(matches!(
        create(&model, &mut fixture, config()),
        Err(Error::DmaAddress)
    ));
    let model = Model::new(16);
    model.borrow_mut().bytes[CORBSIZE] = 0;
    assert!(matches!(
        create(&model, &mut fixture, config()),
        Err(Error::UnsupportedRingSize)
    ));
}

#[test]
fn mmio_claim_bounds() {
    // SAFETY: these invalid ranges are rejected before any MMIO is touched.
    assert!(unsafe { MmioRegisters::claim(0, 0x1000) }.is_err());
    // SAFETY: the undersized extent is rejected before an MMIO pointer is made.
    assert!(unsafe { MmioRegisters::claim(0x1000, 0x400) }.is_err());
}

#[test]
fn converter_channel_preflight() {
    for (widget_capabilities, channels, channel, expected) in [
        (1, 2, 1, Err(Error::InvalidFormat)),
        (1, 1, 0, Err(Error::InvalidFormat)),
        (1 | (1 << 13), 4, 0, Err(Error::UnsupportedTopology)),
        (0, 2, 1, Ok(())), // mono converter consumes only channel1.
        (1, 4, 2, Ok(())), // stereo converter consumes channels2 and3.
    ] {
        let mut fixture = Fixture::new();
        let model = graph_model();
        model
            .borrow_mut()
            .parameters
            .insert((2, 9), widget_capabilities);
        let mut controller = create(&model, &mut fixture, config()).unwrap();
        let graph = controller.enumerate_codec(0).unwrap();
        let id = controller
            .allocate_stream(
                Direction::Playback,
                PcmFormat { channels, ..pcm() },
                128,
                2,
                &mut fixture.allocator,
            )
            .unwrap();
        let commands = model.borrow().commands.len();
        let writes = model.borrow().writes.len();
        assert_eq!(controller.bind_converter(&graph, 2, id, channel), expected);
        if expected.is_err() {
            assert_eq!(model.borrow().commands.len(), commands);
            assert_eq!(model.borrow().writes.len(), writes);
        }
    }
}

#[test]
fn cyclic_packet_alignment() {
    for rate in [176_400, 192_000] {
        let mut fixture = Fixture::new();
        let model = Model::new(16);
        let mut controller = create(&model, &mut fixture, config()).unwrap();
        let format = PcmFormat {
            rate,
            bits: 32,
            channels: 16,
        };
        let writes = model.borrow().writes.len();
        let remaining = fixture.allocator.remaining();
        // Six 64-byte frames are not an integer number of four-frame packets.
        assert_eq!(
            controller.allocate_stream(Direction::Playback, format, 128, 3, &mut fixture.allocator),
            Err(Error::InvalidBuffer)
        );
        assert_eq!(model.borrow().writes.len(), writes);
        assert_eq!(fixture.allocator.remaining(), remaining);
        // A smaller supported stream has complete packets at the boundary.
        let format = PcmFormat {
            channels: 2,
            ..format
        };
        assert!(
            controller
                .allocate_stream(Direction::Playback, format, 128, 3, &mut fixture.allocator)
                .is_ok()
        );
    }
    // 32kHz transmits two blocks every third link frame, not one block.
    let format = PcmFormat {
        rate: 32_000,
        bits: 32,
        channels: 16,
    };
    assert_eq!(format.packet_bytes(), Ok(128));
}

#[test]
fn bandwidth_reserved_before_allocation() {
    for rate in [176_400, 192_000] {
        let mut fixture = Fixture::new();
        let model = Model::new(16);
        let mut controller = create(&model, &mut fixture, config()).unwrap();
        let writes = model.borrow().writes.len();
        let remaining = fixture.allocator.remaining();
        // Complete packets, but 128 words cannot fit the 60-word SDO0.
        let format = PcmFormat {
            rate,
            bits: 32,
            channels: 16,
        };
        assert_eq!(
            controller.allocate_stream(Direction::Playback, format, 128, 2, &mut fixture.allocator),
            Err(Error::LinkBandwidth {
                required_words: 128,
                available_words: 60
            })
        );
        assert_eq!(model.borrow().writes.len(), writes);
        assert_eq!(fixture.allocator.remaining(), remaining);
    }

    let mut fixture = Fixture::new();
    let model = Model::new(16);
    model.borrow_mut().set(OUTSTRMPAY, 3, 2);
    let mut controller = create(&model, &mut fixture, config()).unwrap();
    assert_eq!(
        controller.allocate_stream(
            Direction::Playback,
            PcmFormat { bits: 32, ..pcm() },
            128,
            2,
            &mut fixture.allocator
        ),
        Err(Error::LinkBandwidth {
            required_words: 4,
            available_words: 3
        })
    );

    let model = Model::new(16);
    model.borrow_mut().set(OUTPAY, 8, 2);
    let mut controller = create(&model, &mut fixture, config()).unwrap();
    let format = PcmFormat {
        channels: 6,
        ..pcm()
    };
    controller
        .allocate_stream(Direction::Playback, format, 384, 2, &mut fixture.allocator)
        .unwrap();
    let remaining = fixture.allocator.remaining();
    let writes = model.borrow().writes.len();
    assert_eq!(
        controller.allocate_stream(Direction::Playback, format, 384, 2, &mut fixture.allocator),
        Err(Error::LinkBandwidth {
            required_words: 6,
            available_words: 2
        })
    );
    assert_eq!(fixture.allocator.remaining(), remaining);
    assert_eq!(model.borrow().writes.len(), writes);

    let format = PcmFormat {
        rate: 192_000,
        bits: 32,
        channels: 6,
    };
    assert_eq!(
        controller.allocate_stream(Direction::Capture, format, 384, 2, &mut fixture.allocator),
        Err(Error::LinkBandwidth {
            required_words: 49,
            available_words: 29
        })
    );
}

#[test]
fn write_live_pcm_refills_running_playback_stream_with_aligned_words() {
    let mut fixture = Fixture::new();
    let model = Model::new(16);
    let mut controller = create(&model, &mut fixture, config()).unwrap();
    let playback = controller
        .allocate_stream(Direction::Playback, pcm(), 4096, 2, &mut fixture.allocator)
        .unwrap();
    let capture = controller
        .allocate_stream(Direction::Capture, pcm(), 4096, 2, &mut fixture.allocator)
        .unwrap();
    controller.start(playback).unwrap();
    // Standard write_pcm still refuses while running; write_live_pcm succeeds.
    assert_eq!(
        controller.write_pcm(playback, 0, &[1, 2, 3, 4]),
        Err(Error::StreamRunning)
    );
    controller
        .write_live_pcm(playback, 16, &[0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88])
        .unwrap();
    // Unaligned offset, unaligned length, out-of-bounds, and capture streams are rejected.
    assert_eq!(
        controller.write_live_pcm(playback, 2, &[0; 4]),
        Err(Error::InvalidBuffer)
    );
    assert_eq!(
        controller.write_live_pcm(playback, 0, &[0; 2]),
        Err(Error::InvalidBuffer)
    );
    assert_eq!(
        controller.write_live_pcm(playback, 8192, &[0; 4]),
        Err(Error::InvalidBuffer)
    );
    assert_eq!(
        controller.write_live_pcm(capture, 0, &[0; 4]),
        Err(Error::InvalidBuffer)
    );
    controller.stop(playback).unwrap();
    assert_eq!(
        &controller.stream(playback).unwrap().buffer.bytes()[16..24],
        &[0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88]
    );
}
