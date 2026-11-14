use super::*;
use stela_contracts::interrupt::InterruptLatch;
use stela_contracts::refuse;

// The Gen1 and Gen2 offsets and status bits.
const STATUS: u64 = 0x00;
const REQUEST: u64 = 0x04;
const RESPONSE: u64 = 0x08;
const ENABLE: u64 = 0x20;
const PENDING: u64 = 0x60;
const CONTROL: u64 = 0xa0;
const QUEUE_RESET: u64 = 0xa4;
const FAULT_LATCHES: u64 = BANK_B + 0x110;
const FAULT_STATUS: u64 = BANK_B;
const FAULT_RESPONSE: u64 = BANK_B + 8;
const COUNTERS: u64 = BANK_C;
const RESPONSE_EMPTY: u32 = 1 << 24;
const RESPONSE_FULL: u32 = 1 << 25;
const REQUEST_EMPTY: u32 = 1 << 8;

/// A byte register file of 256 registers; address 0xff NACKs.
struct Bytes([u8; 0x100]);
impl SpmiSlave for Bytes {
    fn read(&mut self, address: u16, data: &mut [u8], _: u64) -> Result<(), SlaveError> {
        let at = address as usize;
        if at + data.len() > 0xff {
            return Err(SlaveError::Nack);
        }
        data.copy_from_slice(&self.0[at..at + data.len()]);
        Ok(())
    }
    fn write(&mut self, address: u16, data: &[u8], _: u64) -> Result<(), SlaveError> {
        let at = address as usize;
        if at + data.len() > 0xff {
            return Err(SlaveError::Nack);
        }
        self.0[at..at + data.len()].copy_from_slice(data);
        Ok(())
    }
}

fn wr(c: &mut Spmi, a: &mut Access<'_>, offset: u64, value: u32) {
    c.write(offset, Width::Word, u64::from(value), a).unwrap();
}
fn rd(c: &mut Spmi, a: &mut Access<'_>, offset: u64) -> u32 {
    c.read(offset, Width::Word, a).unwrap() as u32
}
fn asserted(c: &Spmi) -> bool {
    c.enable.iter().zip(&c.pending).any(|(e, p)| e & p != 0)
}
fn controller() -> Spmi {
    let mut c = Spmi::new(AicInput::new(0, 0x180));
    c.attach(0xe, Box::new(Bytes([0; 0x100]))).unwrap();
    c
}

#[test]
fn payloadless_commands_never_ack_unqualified_slaves_or_reserved_packet_fields() {
    let mut c = controller();
    let mut latch = InterruptLatch::new();
    let mut access = Access::new(0, 0, &mut latch);
    for command in [
        Command::Reset,
        Command::Sleep,
        Command::Shutdown,
        Command::Wakeup,
    ] {
        let packet = LAST | 14 << 8 | command as u32;
        assert!(
            c.write(REQUEST, Width::Word, packet.into(), &mut access)
                .is_err()
        );
        assert!(c.responses.is_empty());
        assert!(c.partial.is_none());
        assert!(c.pending.iter().all(|&pending| pending == 0));
        for bits in [1 << 16, 1 << 24, 1 << 12] {
            assert!(Chunk::decode(packet | bits).is_err());
        }
        // An absent slave gives a real NACK, not a successful command.
        wr(&mut c, &mut access, REQUEST, LAST | 3 << 8 | command as u32);
        assert_eq!(rd(&mut c, &mut access, RESPONSE), 3 << 8 | command as u32);
        wr(&mut c, &mut access, PENDING + 8 * 4, 1);
    }
}

#[test]
fn extended_long_write_then_read_round_trips_with_ack_parity_and_vector() {
    let mut latch = InterruptLatch::new();
    let mut a = Access::new(0, 0, &mut latch);
    let mut c = controller();
    assert_eq!(rd(&mut c, &mut a, STATUS), 0x0100_0100);
    wr(&mut c, &mut a, ENABLE + 4 * 8, 1);
    // Write 5 bytes at 0x10 (ext-long write, len-1 = 4), last chunk.
    let write = 0x10 << 16 | LAST | 0xe << 8 | 0x30 | 4;
    wr(&mut c, &mut a, REQUEST, write);
    assert_eq!(rd(&mut c, &mut a, STATUS), RESPONSE_EMPTY);
    wr(&mut c, &mut a, REQUEST, 0x4433_2211);
    wr(&mut c, &mut a, REQUEST, 0x55);
    assert!(asserted(&c));
    assert_eq!(rd(&mut c, &mut a, STATUS), REQUEST_EMPTY);
    assert_eq!(
        rd(&mut c, &mut a, RESPONSE) & 0xffff,
        (write & 0x7fff) | ACK
    );
    wr(&mut c, &mut a, PENDING + 4 * 8, 1);
    assert!(!asserted(&c));
    // Read 6 bytes at 0x10 (ext-long read, len-1 = 5).
    let read = 0x10 << 16 | LAST | 0xe << 8 | 0x38 | 5;
    wr(&mut c, &mut a, REQUEST, read);
    let header = rd(&mut c, &mut a, RESPONSE);
    assert_eq!(header & 0xfff, read & 0xfff);
    assert_eq!(header >> 16, 0x3f);
    assert_eq!(rd(&mut c, &mut a, RESPONSE), 0x4433_2211);
    assert_eq!(rd(&mut c, &mut a, RESPONSE), 0x55);
    assert_eq!(rd(&mut c, &mut a, STATUS), 0x0100_0100);
    // The read's last chunk raised the enabled response vector again.
    assert!(latch.is_asserted(Wire::Aic(AicInput::new(0, 0x180))));
}

#[test]
fn refused_transfers_nack_or_fail_parity() {
    let mut latch = InterruptLatch::new();
    let mut a = Access::new(0, 0, &mut latch);
    let mut c = controller();
    let write = 3 << 8 | LAST | 0x30;
    wr(&mut c, &mut a, REQUEST, write);
    wr(&mut c, &mut a, REQUEST, 7);
    assert_eq!(rd(&mut c, &mut a, RESPONSE) & ACK, 0);
    let read = 0xff << 16 | 0xe << 8 | LAST | 0x38 | 1;
    wr(&mut c, &mut a, REQUEST, read);
    assert_eq!(rd(&mut c, &mut a, RESPONSE) >> 15, 0);
    assert_eq!(rd(&mut c, &mut a, RESPONSE), 0);
    // An 8-bit extended access cannot carry a 16-bit address.
    assert!(
        c.write(
            REQUEST,
            Width::Word,
            u64::from(0x100u32 << 16 | 0xe << 8 | 0x20),
            &mut a
        )
        .is_err()
    );
    assert!(c.read(RESPONSE, Width::Word, &mut a).is_err());
}

#[test]
#[ignore = "oracle: STELA_TEST_SPMI_SCALAR_KC"]
fn original_ap_scalar_command_encoding_is_source_pinned() {
    use sha2::Digest;
    let data = std::fs::read(std::env::var("STELA_TEST_SPMI_SCALAR_KC").unwrap()).unwrap();
    let digest = sha2::Sha256::digest(&data);
    assert_eq!(
        format!("{digest:x}"),
        "8210f3645ea16c2481bcaf285ead56eaf2e591bb0d124fe2c01f596c6361ded3"
    );
    // This pinned decoded fileset's VA/file relation is a research
    // fixture fact, not a production image-placement assumption.
    const BASE: u64 = 0xffff_fe00_0700_4000;
    let word = |va| {
        let at = (va - BASE) as usize;
        u32::from_le_bytes(data[at..at + 4].try_into().unwrap())
    };
    let table = (0xffff_fe00_0772_ff48 - BASE) as usize;
    assert_eq!(data[table + 5], REGISTER_WRITE as u8);
    assert_eq!(data[table + 6], REGISTER_READ as u8);
    assert_eq!(data[table + 7], REGISTER_ZERO_WRITE as u8);
    for (index, command) in [
        (8, Command::Reset),
        (9, Command::Sleep),
        (10, Command::Shutdown),
        (11, Command::Wakeup),
    ] {
        assert_eq!(data[table + index], command as u8);
    }
    for (va, encoding) in [
        (0xffff_fe00_09d5_c8a0, 0x7100_7f9f), // CMP address, #31
        (0xffff_fe00_09d5_c9ac, 0x2a1c_4129), // ORR address, LSL #16
        (0xffff_fe00_09d5_c9cc, 0x2a16_6129), // ORR write byte, LSL #24
        (0xffff_fe00_09d5_d0a8, 0x2a08_6128), // ORR Register0 byte, LSL #24
        (0xffff_fe00_09d5_d0b4, 0x3219_0116), // ORR opcode, #0x80
    ] {
        assert_eq!(word(va), encoding, "original producer at {va:#x}");
    }
    let masks = 0xffff_fe00_0772_ff5c;
    assert_eq!(word(masks + 4 * 4), 0xfe0);
    assert_eq!(word(masks + 4 * 5), 0xfe0);
    assert_eq!(word(masks + 4 * 6), 0xf80);
    for index in 7..=10 {
        assert_eq!(word(masks + 4 * index), 0xfff);
    }
}

#[test]
fn scalar_register_commands_own_inline_bytes_and_normal_fifo_replies_on_every_generation() {
    for profile in [Profile::Gen1, Profile::Gen2, Profile::Gen3, Profile::Gen4] {
        let mut c = configured(profile, 0x100, 256);
        let mut latch = InterruptLatch::new();
        let mut access = Access::new(0, 0, &mut latch);
        let layout = profile.layout();
        wr(&mut c, &mut access, layout.enable + 8 * 4, 1);
        // Original AP packet: one word with a five-bit address and
        // an inline high byte, not a second request FIFO write.
        let write = 0xa5 << 24 | 0x1f << 16 | LAST | 0xe << 8 | REGISTER_WRITE;
        wr(&mut c, &mut access, layout.request, write);
        assert!(c.partial.is_none());
        assert_eq!(c.responses.len(), 1);
        assert_eq!(rd(&mut c, &mut access, layout.response), ACK | 0xe5f);
        assert!(asserted(&c));
        wr(&mut c, &mut access, layout.pending + 8 * 4, 1);
        let read = 0x1f << 16 | LAST | 0xe << 8 | REGISTER_READ;
        wr(&mut c, &mut access, layout.request, read);
        assert_eq!(c.responses.len(), 2);
        let reply = rd(&mut c, &mut access, layout.response);
        assert_eq!(reply & 0xfe0, read & 0xfe0);
        assert_eq!(reply & (ACK | 1 << 16), ACK | 1 << 16);
        assert_eq!(rd(&mut c, &mut access, layout.response), 0xa5);
        // Register0 Write is also payloadless at the controller FIFO,
        // but it performs a real one-byte write to the slave register.
        let zero = 0x6d << 24 | LAST | 0xe << 8 | REGISTER_ZERO_WRITE;
        wr(&mut c, &mut access, layout.request, zero);
        assert!(c.partial.is_none());
        assert_eq!(rd(&mut c, &mut access, layout.response), ACK | 0xeed);
        wr(
            &mut c,
            &mut access,
            layout.request,
            LAST | 0xe << 8 | REGISTER_READ,
        );
        assert_eq!(rd(&mut c, &mut access, layout.response) >> 16, 1);
        assert_eq!(rd(&mut c, &mut access, layout.response), 0x6d);
        assert_eq!(c.status(), layout.response_empty | layout.request_empty);
        // An absent target stays a hardware NACK, never a successful
        // empty reply that could complete an HPM mailbox transaction.
        wr(
            &mut c,
            &mut access,
            layout.request,
            LAST | 0xd << 8 | REGISTER_READ,
        );
        assert_eq!(
            rd(&mut c, &mut access, layout.response) & (ACK | 1 << 16),
            0
        );
        assert_eq!(rd(&mut c, &mut access, layout.response), 0);
    }
}

#[test]
fn scalar_reserved_fields_and_fifo_pressure_refuse_before_slave_or_queue_effects() {
    let mut c = configured(Profile::Gen3, 0x100, 20);
    let mut latch = InterruptLatch::new();
    let mut access = Access::new(0, 0, &mut latch);
    for packet in [
        0x20 << 16 | 0xe << 8 | REGISTER_READ,
        1 << 24 | 0xe << 8 | REGISTER_READ,
        0x20 << 16 | 0xe << 8 | REGISTER_WRITE,
        1 << 16 | 0xe << 8 | REGISTER_ZERO_WRITE,
        0x80 << 24 | 0xe << 8 | REGISTER_ZERO_WRITE,
        1 << 12 | 0xe << 8 | REGISTER_WRITE,
    ] {
        assert!(
            c.write(REQUEST, Width::Word, u64::from(packet), &mut access)
                .is_err()
        );
        assert!(c.partial.is_none());
        assert!(c.responses.is_empty());
        assert!(c.pending.iter().all(|&bits| bits == 0));
    }
    // Four accepted writes occupy four of five response words.
    // A read needs two; its rejected header keeps the entire prefix.
    for byte in 1..=4 {
        wr(
            &mut c,
            &mut access,
            REQUEST,
            byte << 24 | LAST | 0xe << 8 | REGISTER_ZERO_WRITE,
        );
    }
    let responses = c.responses.clone();
    let pending = c.pending;
    assert!(
        c.write(
            REQUEST,
            Width::Word,
            u64::from(LAST | 0xe << 8 | REGISTER_READ),
            &mut access
        )
        .is_err()
    );
    assert_eq!(c.responses, responses);
    assert_eq!(c.pending, pending);
    assert!(c.partial.is_none());
    while !c.responses.is_empty() {
        rd(&mut c, &mut access, RESPONSE);
    }
    wr(
        &mut c,
        &mut access,
        REQUEST,
        LAST | 0xe << 8 | REGISTER_READ,
    );
    assert_eq!(rd(&mut c, &mut access, RESPONSE) >> 16, 1);
    assert_eq!(rd(&mut c, &mut access, RESPONSE), 4);
}

#[test]
fn queue_reset_flushes_a_partial_command_and_unknown_commands_refuse() {
    let mut latch = InterruptLatch::new();
    let mut a = Access::new(0, 0, &mut latch);
    let mut c = controller();
    wr(&mut c, &mut a, REQUEST, 0xe << 8 | 0x30 | 7);
    assert_eq!(rd(&mut c, &mut a, STATUS), RESPONSE_EMPTY);
    wr(&mut c, &mut a, CONTROL, 0x8000_0001);
    wr(&mut c, &mut a, CONTROL, 0);
    wr(&mut c, &mut a, QUEUE_RESET, 1);
    assert_eq!(rd(&mut c, &mut a, QUEUE_RESET), 0);
    assert_eq!(rd(&mut c, &mut a, STATUS), 0x0100_0100);
    // Payloadless reset/sleep commands have no checked slave lifecycle.
    for opcode in [0x10, 0x11] {
        assert!(
            c.write(
                REQUEST,
                Width::Word,
                u64::from(0xe << 8 | LAST | opcode),
                &mut a
            )
            .is_err()
        );
    }
    assert_eq!(rd(&mut c, &mut a, BANK_B + 0x200), 0);
    assert_eq!(rd(&mut c, &mut a, BANK_B + 0x274), 0);
    assert!(c.read(0xb0, Width::Word, &mut a).is_err());
    assert!(c.read(STATUS, Width::Byte, &mut a).is_err());
}
#[test]
fn kept_pauses_keep_masked_response_ownership_and_independent_inputs() {
    let mut c = controller();
    let mut latch = InterruptLatch::new();
    let mut a = Access::new(0, 0, &mut latch);
    let command = 0x10 << 16 | LAST | 0xe << 8 | 0x33;
    wr(&mut c, &mut a, REQUEST, command);
    wr(&mut c, &mut a, REQUEST, 0x4433_2211);
    let responses = c.responses.clone();
    let pending = c.pending;
    let running = c.lifecycle;
    let paused = LifecycleState {
        clocked: false,
        quiesced: true,
        ..running
    };
    c.check_lifecycle(paused).unwrap();
    c.set_lifecycle(paused, &mut a);
    assert!(c.read(RESPONSE, Width::Word, &mut a).is_err());
    assert!(c.write(QUEUE_RESET, Width::Word, 1, &mut a).is_err());
    assert_eq!(c.responses, responses);
    assert_eq!(c.pending, pending);
    // Clock restart cannot clear an independent quiesce input or pop a reply.
    let quiesced = LifecycleState {
        clocked: true,
        ..paused
    };
    c.check_lifecycle(quiesced).unwrap();
    c.set_lifecycle(quiesced, &mut a);
    assert_eq!(rd(&mut c, &mut a, STATUS), REQUEST_EMPTY);
    assert!(c.read(RESPONSE, Width::Word, &mut a).is_err());
    assert!(c.write(REQUEST, Width::Word, 0, &mut a).is_err());
    assert_eq!(c.responses, responses);
    c.check_lifecycle(running).unwrap();
    c.set_lifecycle(running, &mut a);
    assert_eq!(rd(&mut c, &mut a, RESPONSE), responses[0]);
    assert_eq!(c.pending, pending); // FIFO pop does not acknowledge the vector.
    assert!(c.read(RESPONSE, Width::Word, &mut a).is_err());
    wr(&mut c, &mut a, REQUEST, 0x10 << 16 | LAST | 0xe << 8 | 0x3b);
    assert_eq!(rd(&mut c, &mut a, RESPONSE) >> 16, 0xf);
    assert_eq!(rd(&mut c, &mut a, RESPONSE), 0x4433_2211);
}
#[test]
fn partial_requests_armed_vectors_and_destructive_edges_refuse_before_effects() {
    let mut c = controller();
    let mut latch = InterruptLatch::new();
    let mut a = Access::new(0, 0, &mut latch);
    let running = c.lifecycle;
    wr(&mut c, &mut a, REQUEST, 0x10 << 16 | LAST | 0xe << 8 | 0x37);
    wr(&mut c, &mut a, REQUEST, 0x4433_2211);
    for proposed in [
        LifecycleState {
            clocked: false,
            ..running
        },
        LifecycleState {
            quiesced: true,
            ..running
        },
        LifecycleState {
            reset: true,
            ..running
        },
        LifecycleState {
            powered: false,
            retention: true,
            ..running
        },
        LifecycleState {
            powered: false,
            retention: false,
            ..running
        },
    ] {
        assert!(c.check_lifecycle(proposed).is_err());
        assert_eq!(c.lifecycle, running);
        assert_eq!(c.partial.as_ref().unwrap().1, [0x4433_2211]);
        assert!(c.responses.is_empty());
    }
    wr(&mut c, &mut a, REQUEST, 0x8877_6655);
    assert_eq!(c.responses.len(), 1);
    wr(&mut c, &mut a, ENABLE + 4 * 8, 1);
    assert!(
        c.check_lifecycle(LifecycleState {
            clocked: false,
            ..running
        })
        .is_err()
    );
    assert!(
        c.check_lifecycle(LifecycleState {
            quiesced: true,
            ..running
        })
        .is_err()
    );
    assert!(asserted(&c));
    assert_eq!(c.responses.len(), 1);
    assert!(latch.is_asserted(Wire::Aic(AicInput::new(0, 0x180))));
}
#[test]
fn pmu_output_edges_latch_local_vector_independently_of_mask_response_and_ack() {
    use {
        crate::counter::FREQUENCY_HZ, crate::pmu::Pmu, stela_soc_data::pmu::ClockOffset,
        stela_soc_data::pmu::Rtc,
    };
    let rtc = Rtc {
        upcount: 0x1406,
        clock_offset: ClockOffset::Register(0x1411),
        alarm_control: 0x1401,
        alarm_enable: 0x40,
        alarm_offset: 0x140c,
        alarm_event: 0x14c0,
        irq_mask: 0x14c2,
        event_mask: 1,
        interrupt_mask: 1,
    };
    let model = Pmu::new(Vec::new(), Some(rtc), None);
    let output = model.interrupt();
    let mut c = Spmi::with_configuration(
        AicInput::new(0, 0x180),
        Configuration::new(Profile::Gen1, 192, [256, 256]).unwrap(),
    );
    c.attach_with_interrupt(15, 2, Box::new(model)).unwrap();
    struct Sink(alloc::rc::Rc<core::cell::Cell<bool>>, Wire);
    impl stela_contracts::interrupt::InterruptSink for Sink {
        fn set_level(&mut self, wire: Wire, asserted: bool) {
            assert_eq!(wire, self.1);
            self.0.set(asserted);
        }
    }
    let level = alloc::rc::Rc::new(core::cell::Cell::new(false));
    let mut sink = Sink(level.clone(), Wire::Aic(c.irq));
    let mut a = Access::new(0, 0, &mut sink);
    let write = |c: &mut Spmi, a: &mut Access<'_>, address: u16, opcode: u32, value: u32| {
        wr(
            c,
            a,
            REQUEST,
            u32::from(address) << 16 | LAST | 15 << 8 | opcode,
        );
        wr(c, a, REQUEST, value);
        assert_ne!(rd(c, a, RESPONSE) & ACK, 0);
    };
    write(&mut c, &mut a, rtc.alarm_offset, 0x33, 30);
    write(&mut c, &mut a, rtc.alarm_control, 0x30, 0x41);
    assert_eq!(c.pending[6], 1); // response192, not slave2
    assert_eq!(c.pending[0], 0);
    let stopped = LifecycleState {
        clocked: false,
        ..c.lifecycle
    };
    // A masked controller vector cannot justify capturing an autonomous edge
    // through a clock/power transition. Preflight leaves all owners intact.
    assert!(c.check_lifecycle(stopped).is_err());
    a.now = 30 * FREQUENCY_HZ;
    assert_eq!(c.advance(&mut a), Ok(None));
    assert!(output.asserted());
    assert_eq!(c.pending[0], 4);
    assert!(!level.get());
    wr(&mut c, &mut a, ENABLE, 4);
    assert!(level.get());
    wr(&mut c, &mut a, PENDING, 4);
    assert_eq!(c.advance(&mut a), Ok(None));
    assert_eq!(c.pending[0], 0); // still-high input is not a second edge
    assert!(output.asserted());
    assert!(!level.get());
    write(&mut c, &mut a, rtc.irq_mask, 0x30, 1);
    assert!(!output.asserted());
    write(&mut c, &mut a, rtc.irq_mask, 0x30, 0);
    assert_eq!(c.pending[0], 4); // wire fell and rose
    assert!(output.asserted());
    write(&mut c, &mut a, rtc.alarm_event, 0x30, 1);
    assert!(!output.asserted());
    assert_eq!(c.pending[0], 4); // PMU W1C is distinct from controller W1C
    wr(&mut c, &mut a, PENDING, 4);
    assert!(!level.get());
    wr(&mut c, &mut a, ENABLE, 0);
    c.check_lifecycle(stopped).unwrap();
    c.set_lifecycle(stopped, &mut a);
    assert_eq!(c.advance(&mut a), Ok(None));
    assert_eq!(c.pending[0], 0);
}

#[test]
fn autonomous_preflight_refusal_does_not_advance_any_peer() {
    use {
        crate::counter::FREQUENCY_HZ, crate::pmu::Pmu, crate::pmu::RTC_HZ,
        stela_soc_data::pmu::ClockOffset, stela_soc_data::pmu::Rtc,
    };
    struct Peer(alloc::rc::Rc<core::cell::Cell<u32>>);
    impl SpmiSlave for Peer {
        fn read(&mut self, _: u16, _: &mut [u8], _: u64) -> Result<(), SlaveError> {
            Ok(())
        }
        fn write(&mut self, _: u16, _: &[u8], _: u64) -> Result<(), SlaveError> {
            Ok(())
        }
        fn advance(&mut self, _: u64) -> Option<u64> {
            self.0.set(self.0.get() + 1);
            None
        }
    }
    let count = alloc::rc::Rc::new(core::cell::Cell::new(0));
    let mut c = Spmi::new(AicInput::new(0, 1));
    c.attach(1, Box::new(Peer(count.clone()))).unwrap();
    c.attach(
        2,
        Box::new(Pmu::new(
            Vec::new(),
            Some(Rtc {
                upcount: 0x1406,
                clock_offset: ClockOffset::Register(0x1411),
                alarm_control: 0x1401,
                alarm_enable: 0x40,
                alarm_offset: 0x140c,
                alarm_event: 0x14c0,
                irq_mask: 0x14c2,
                event_mask: 1,
                interrupt_mask: 1,
            }),
            None,
        )),
    )
    .unwrap();
    let mut latch = InterruptLatch::new();
    let now = ((1u128 << 47) * u128::from(FREQUENCY_HZ) / u128::from(RTC_HZ)) as u64;
    let mut access = Access::new(0, now, &mut latch);
    // No guest transfer was made: the refusal names the board's count,
    // which lies 2^32 seconds of upcount past the epoch.
    match c.advance(&mut access) {
        Err(MmioError::Advance {
            device: NAME,
            now: refused,
            reason,
        }) => {
            assert_eq!(refused, now);
            assert!(
                alloc::format!("{reason}").contains("past its 47-bit range"),
                "{reason}"
            );
        }
        other => panic!("expected the slave's time refused, got {other:?}"),
    }
    assert_eq!(count.get(), 0);
    access.now = 0;
    assert_eq!(c.advance(&mut access), Ok(None));
    assert_eq!(count.get(), 1);
}

#[test]
fn routed_outputs_require_unique_real_deasserted_owners_and_valid_vectors() {
    use {crate::pmu::Pmu, stela_soc_data::pmu::ClockOffset, stela_soc_data::pmu::Rtc};
    let mut c = Spmi::with_configuration(
        AicInput::new(0, 1),
        Configuration::new(Profile::Gen1, 192, [256, 256]).unwrap(),
    );
    let model = || {
        Box::new(Pmu::new(
            Vec::new(),
            Some(Rtc {
                upcount: 0x1406,
                clock_offset: ClockOffset::Register(0x1411),
                alarm_control: 0x1401,
                alarm_enable: 0x40,
                alarm_offset: 0x140c,
                alarm_event: 0x14c0,
                irq_mask: 0x14c2,
                event_mask: 1,
                interrupt_mask: 1,
            }),
            None,
        ))
    };
    for vector in [192, 288, u16::MAX] {
        assert!(c.attach_with_interrupt(1, vector, model()).is_err());
        assert!(c.slaves.is_empty());
    }
    assert!(
        c.attach_with_interrupt(1, 2, Box::new(Bytes([0; 256])))
            .is_err()
    );
    assert!(c.slaves.is_empty());
    c.attach_with_interrupt(1, 2, model()).unwrap();
    assert!(c.attach_with_interrupt(2, 2, model()).is_err());
    assert!(c.attach_with_interrupt(1, 3, model()).is_err());
    assert_eq!((c.slaves.len(), c.inputs.len()), (1, 1));
}

#[test]
fn pmic_alarm_time_and_output_continue_while_spmi_master_clock_is_paused() {
    use {
        crate::counter::FREQUENCY_HZ, crate::pmu::Pmu, stela_soc_data::pmu::ClockOffset,
        stela_soc_data::pmu::Rtc,
    };
    let rtc = Rtc {
        upcount: 0xf802,
        clock_offset: ClockOffset::Register(0xf900),
        alarm_control: 0xf800,
        alarm_enable: 0x40,
        alarm_offset: 0xf808,
        alarm_event: 0xf80c,
        irq_mask: 0xf80e,
        event_mask: 1,
        interrupt_mask: 1,
    };
    let pmu = Pmu::new(Vec::new(), Some(rtc), None);
    let alarm = pmu.interrupt();
    let mut c = Spmi::new(AicInput::new(0, 0x180));
    c.attach(0xe, Box::new(pmu)).unwrap();
    let mut latch = InterruptLatch::new();
    let mut a = Access::new(0, 0, &mut latch);
    for (address, opcode, value) in [
        (rtc.alarm_offset, 0x33, 30),
        (rtc.alarm_control, 0x30, 0x41),
    ] {
        wr(
            &mut c,
            &mut a,
            REQUEST,
            u32::from(address) << 16 | LAST | 0xe << 8 | opcode,
        );
        wr(&mut c, &mut a, REQUEST, value);
        assert_ne!(rd(&mut c, &mut a, RESPONSE) & ACK, 0);
    }
    assert_eq!(c.advance(&mut a), Ok(Some(30 * FREQUENCY_HZ)));
    let running = c.lifecycle;
    let paused = LifecycleState {
        clocked: false,
        quiesced: true,
        ..running
    };
    c.check_lifecycle(paused).unwrap();
    c.set_lifecycle(paused, &mut a);
    a.now = 30 * FREQUENCY_HZ;
    assert_eq!(c.advance(&mut a), Ok(None));
    assert!(alarm.asserted()); // PMIC's external output is not the SPMI response IRQ.
    assert!(c.read(RESPONSE, Width::Word, &mut a).is_err());
    assert!(
        c.check_lifecycle(LifecycleState {
            reset: true,
            ..running
        })
        .is_err()
    );
    assert!(alarm.asserted());
    c.check_lifecycle(running).unwrap();
    c.set_lifecycle(running, &mut a);
    wr(
        &mut c,
        &mut a,
        REQUEST,
        u32::from(rtc.alarm_event) << 16 | LAST | 0xe << 8 | 0x38,
    );
    assert_eq!(rd(&mut c, &mut a, RESPONSE) >> 16, 1);
    assert_eq!(rd(&mut c, &mut a, RESPONSE), 1);
    assert!(alarm.asserted()); // Reading does not acknowledge the PMIC event.
}
#[test]
fn mounted_spmi_aliases_share_one_checked_controller_capability() {
    use stela_contracts::bus::Bus;
    let mut bus = Bus::new();
    let device = bus.add(Box::new(controller()));
    bus.map(device, 0x10000, BANK_BYTES, 0).unwrap();
    bus.map(device, 0x20000, BANK_BYTES, BANK_B).unwrap();
    let mut latch = InterruptLatch::new();
    let mut a = Access::new(0, 0, &mut latch);
    let paused = LifecycleState {
        powered: true,
        clocked: false,
        quiesced: false,
        reset: false,
        retention: true,
    };
    let lifecycle = bus.lifecycle(device).unwrap();
    lifecycle.check_lifecycle(paused).unwrap();
    lifecycle.set_lifecycle(paused, &mut a);
    assert!(bus.read(0x10000 + STATUS, Width::Word, &mut a).is_err());
    assert!(bus.read(0x20000 + 0x110, Width::Word, &mut a).is_err());
    let running = LifecycleState {
        clocked: true,
        ..paused
    };
    let lifecycle = bus.lifecycle(device).unwrap();
    lifecycle.check_lifecycle(running).unwrap();
    lifecycle.set_lifecycle(running, &mut a);
    assert_eq!(
        bus.read(0x10000 + STATUS, Width::Word, &mut a),
        Ok(u64::from(REQUEST_EMPTY | RESPONSE_EMPTY))
    );
    assert_eq!(bus.read(0x20000 + 0x110, Width::Word, &mut a), Ok(0));
}
#[cfg(feature = "std")]
#[test]
#[ignore = "oracle: STELA_TEST_PMGR_KERNELCACHE"]
fn original_gen2_fifo_reset_is_separate_from_physical_controller_lifecycle() {
    use sha2::{Digest, Sha256};
    let path = std::env::var_os("STELA_TEST_PMGR_KERNELCACHE").expect("original KernelCache path");
    let bytes = std::fs::read(path).unwrap();
    assert_eq!(
        hex::encode(Sha256::digest(&bytes)),
        "06846c718a67715f540c7f5a5d5a183da6185d94b6fc01f1ce57a30b601be44d"
    );
    for (offset, opcode) in [
        (0x2dc35fc, 0x91029048u32),
        (0x2dc3668, 0xf9406408u32),
        (0x2dc366c, 0xb9400114u32),
        (0x2dc3670, 0xb900011fu32),
        (0x2dc3678, 0x94001d36u32),
        (0x2dc3680, 0xb9000114u32),
        (0x2dcabe8, 0x710006b5u32),
        (0x2dcac00, 0x2a080128u32),
    ] {
        assert_eq!(
            u32::from_le_bytes(bytes[offset..offset + 4].try_into().unwrap()),
            opcode,
            "original opcode at {offset:#x}"
        );
    }
}
#[cfg(feature = "std")]
#[test]
#[ignore = "oracle: STELA_TEST_PMGR_M1_KERNELCACHE"]
fn original_m1_gen2_queue_reset_preserves_separate_controller_control() {
    use sha2::{Digest, Sha256};
    let path = std::env::var_os("STELA_TEST_PMGR_M1_KERNELCACHE")
        .expect("original M1 Pro KernelCache path");
    let bytes = std::fs::read(path).unwrap();
    assert_eq!(
        hex::encode(Sha256::digest(&bytes)),
        "5109fca5209bdbb89cec29e3241cc42a90418cb737d65e31962b059b9879af90"
    );
    // Original Gen2 init records the software queue-reset word at A+a4;
    // resetQueue saves control, zeros it, resets the queue, then restores it.
    for (offset, opcode) in [
        (0x2d4600c, 0x91029048u32),
        (0x2d46078, 0xf9406408u32),
        (0x2d4607c, 0xb9400114u32),
        (0x2d46080, 0xb900011fu32),
        (0x2d46088, 0x94001d36u32),
        (0x2d46090, 0xb9000114u32),
    ] {
        assert_eq!(
            u32::from_le_bytes(bytes[offset..offset + 4].try_into().unwrap()),
            opcode,
            "original M1 opcode at {offset:#x}"
        );
    }
}
fn configured(profile: Profile, vector: u32, response_bytes: u32) -> Spmi {
    let mut c = Spmi::with_configuration(
        AicInput::new(0, 636),
        Configuration::new(profile, vector, [256, response_bytes]).unwrap(),
    );
    c.attach(0xe, Box::new(Bytes([0; 0x100]))).unwrap();
    c
}

#[test]
fn gen1_vector_and_counter_view_follow_separate_source_resources() {
    let mut c = configured(Profile::Gen1, 192, 256);
    let mut latch = InterruptLatch::new();
    let mut a = Access::new(0, 0, &mut latch);
    wr(&mut c, &mut a, ENABLE + 6 * 4, 1);
    wr(&mut c, &mut a, REQUEST, 0xe << 8 | LAST | 0x30);
    wr(&mut c, &mut a, REQUEST, 0x7b);
    assert!(asserted(&c));
    assert_eq!(rd(&mut c, &mut a, PENDING + 6 * 4), 1);
    assert_eq!(rd(&mut c, &mut a, PENDING + 8 * 4), 0);
    assert_eq!(rd(&mut c, &mut a, RESPONSE), 0x8e30);
    // Popping the response cannot acknowledge its separately latched vector.
    assert_eq!(rd(&mut c, &mut a, PENDING + 6 * 4), 1);
    wr(&mut c, &mut a, PENDING + 6 * 4, 1);
    assert!(!asserted(&c));
    for offset in [
        0, 4, 8, 0x10, 0x14, 0x18, 0x1c, 0x20, 0x24, 0x5c, 0x60, 0x64, 0x68, 0x6c, 0x70, 0x74,
    ] {
        assert_eq!(rd(&mut c, &mut a, COUNTERS + offset), 0);
    }
    for offset in [
        COUNTERS + 0xc,
        COUNTERS + 0x28,
        COUNTERS + 0x78,
        BANK_B + 0x200,
        FAULT_LATCHES,
    ] {
        assert!(c.read(offset, Width::Word, &mut a).is_err());
        assert!(c.write(offset, Width::Word, 0, &mut a).is_err());
    }
    assert_eq!(rd(&mut c, &mut a, FAULT_STATUS), RESPONSE_EMPTY);
    assert!(c.read(FAULT_RESPONSE, Width::Word, &mut a).is_err());
    assert_eq!(rd(&mut c, &mut a, STATUS), RESPONSE_EMPTY | REQUEST_EMPTY);
}

#[test]
fn complete_extended_modes_have_identical_payload_effects_on_both_layouts() {
    for (profile, vector) in [(Profile::Gen1, 192), (Profile::Gen2, 256)] {
        for long in [false, true] {
            for bytes in 1..=if long { 8 } else { 16 } {
                let mut c = configured(profile, vector, 256);
                let mut latch = InterruptLatch::new();
                let mut a = Access::new(0, 0, &mut latch);
                let command = 0x20 << 16 | 0xe << 8 | if long { 0x30 } else { 0 } | (bytes - 1);
                let data: Vec<u8> = (1..=bytes as u8).collect();
                wr(&mut c, &mut a, REQUEST, command);
                for word in data.chunks(4) {
                    let mut value = [0x6a; 4];
                    value[..word.len()].copy_from_slice(word);
                    wr(&mut c, &mut a, REQUEST, u32::from_le_bytes(value));
                }
                assert_eq!(rd(&mut c, &mut a, RESPONSE), (command & 0x7fff) | ACK);
                assert!(c.pending.iter().all(|&bits| bits == 0));
                let read = command | LAST | if long { 8 } else { 0x20 };
                wr(&mut c, &mut a, REQUEST, read);
                assert_eq!(
                    rd(&mut c, &mut a, RESPONSE),
                    (read & 0xfff) | ACK | ((1 << bytes) - 1) << 16
                );
                let mut actual = Vec::new();
                for _ in 0..bytes.div_ceil(4) {
                    actual.extend_from_slice(&rd(&mut c, &mut a, RESPONSE).to_le_bytes());
                }
                assert_eq!(&actual[..bytes as usize], data);
                assert!(actual[bytes as usize..].iter().all(|&b| b == 0));
                assert_eq!(c.pending[(vector / 32) as usize], 1 << (vector % 32));
                assert_eq!(rd(&mut c, &mut a, STATUS), RESPONSE_EMPTY | REQUEST_EMPTY);
            }
        }
    }
}

#[test]
fn fifo_byte_capacity_and_refused_final_payload_preserve_transaction_ownership() {
    for (profile, vector) in [(Profile::Gen1, 192), (Profile::Gen2, 256)] {
        let mut c = configured(profile, vector, 256);
        let mut latch = InterruptLatch::new();
        let mut a = Access::new(0, 0, &mut latch);
        // 32 one-byte reads produce exactly64 words, not256 words.
        for _ in 0..32 {
            wr(&mut c, &mut a, REQUEST, 0xe << 8 | 0x38);
        }
        assert_eq!(rd(&mut c, &mut a, STATUS), RESPONSE_FULL | REQUEST_EMPTY);
        let before = c.responses.clone();
        let pending = c.pending;
        assert!(
            c.write(
                REQUEST,
                Width::Word,
                u64::from(0xe << 8 | LAST | 0x38),
                &mut a
            )
            .is_err()
        );
        assert_eq!(c.responses, before);
        assert_eq!(c.pending, pending);
        let write = 0x10 << 16 | 0xe << 8 | LAST | 0x37;
        wr(&mut c, &mut a, REQUEST, write);
        wr(&mut c, &mut a, REQUEST, 0x4433_2211);
        assert!(c.write(REQUEST, Width::Word, 0x8877_6655, &mut a).is_err());
        assert_eq!(c.partial.as_ref().unwrap().1, [0x4433_2211]);
        assert_eq!(c.responses, before);
        // Free space and retry exactly the previously unconsumed MMIO word.
        for _ in 0..64 {
            rd(&mut c, &mut a, RESPONSE);
        }
        wr(&mut c, &mut a, REQUEST, 0x8877_6655);
        assert!(c.partial.is_none());
        assert_eq!(c.responses.len(), 1);
        assert_eq!(rd(&mut c, &mut a, RESPONSE), (write & 0x7fff) | ACK);
        wr(&mut c, &mut a, REQUEST, 0x10 << 16 | 0xe << 8 | LAST | 0x3f);
        rd(&mut c, &mut a, RESPONSE);
        assert_eq!(rd(&mut c, &mut a, RESPONSE), 0x4433_2211);
        assert_eq!(rd(&mut c, &mut a, RESPONSE), 0x8877_6655);
        // Misaligned register words cannot alias another register or consume it.
        for offset in [RESPONSE + 1, ENABLE + 1, PENDING + 2] {
            let before = (c.responses.clone(), c.enable, c.pending);
            assert!(c.read(offset, Width::Word, &mut a).is_err());
            assert!(c.write(offset, Width::Word, 1, &mut a).is_err());
            assert_eq!((c.responses.clone(), c.enable, c.pending), before);
        }
    }
}

#[test]
#[ignore = "oracle: STELA_TEST_SPMI_DT"]
fn original_j314s_all_seven_gen1_resource_views_match_the_checked_contract() {
    use stela_contracts::adt::Node;
    let data = std::fs::read(std::env::var("STELA_TEST_SPMI_DT").unwrap()).unwrap();
    assert_eq!(
        stela_formats::sha256(&data),
        "44fa41bfd0fe0027fd862b8615ae956885614636625d44a59cab226d682558ad"
    );
    let root = Node::root(&data).unwrap();
    let io = root.child("arm-io").unwrap();
    let mut controllers = 0;
    for node in io.children().unwrap() {
        if !node.compatible("aapl,spmi") {
            continue;
        }
        controllers += 1;
        assert_eq!(node.first_u32("gen"), Some(1));
        let depths: [u32; 2] = node.cells32("queue-depth").unwrap().try_into().unwrap();
        let vectors = node.cells32("interrupts").unwrap();
        let cfg = Configuration::new(Profile::Gen1, vectors[0], depths).unwrap();
        assert_eq!(cfg.response_vector(), 192);
        assert_eq!(cfg.queue_bytes(), [256, 256]);
        let banks = node.pairs("reg").unwrap();
        assert_eq!(
            banks.iter().map(|&(_, size)| size).collect::<Vec<_>>(),
            [256, 256, 120]
        );
        assert_eq!(node.first_u32("fault-counters-enabled"), Some(1));
        let parents = node.cells32("interrupt-parent").unwrap();
        assert_eq!(parents[1], 130);
        assert_eq!(parents[0], node.first_u32("AAPL,phandle").unwrap());
        assert_eq!(parents[2], parents[0]);
        let mut c = Spmi::with_configuration(AicInput::new(0, vectors[1] as u16), cfg);
        let mut latch = InterruptLatch::new();
        let mut a = Access::new(0, 0, &mut latch);
        assert_eq!(rd(&mut c, &mut a, STATUS), 0x01000100);
        assert_eq!(rd(&mut c, &mut a, FAULT_STATUS), 0x01000000);
        assert_eq!(rd(&mut c, &mut a, COUNTERS + 0x74), 0);
        assert!(c.read(FAULT_RESPONSE, Width::Word, &mut a).is_err());
    }
    assert_eq!(controllers, 7);
}
#[test]
fn unsupported_slave_preserves_unconsumed_payload_and_never_invents_ack() {
    use alloc::rc::Rc;
    use core::cell::Cell;
    struct Refused {
        allow: Rc<Cell<bool>>,
        writes: Rc<Cell<u32>>,
    }
    impl SpmiSlave for Refused {
        fn read(&mut self, _: u16, _: &mut [u8], _: u64) -> Result<(), SlaveError> {
            Err(SlaveError::Unsupported(refuse!("unqualified slave read")))
        }
        fn write(&mut self, _: u16, _: &[u8], _: u64) -> Result<(), SlaveError> {
            if !self.allow.get() {
                return Err(SlaveError::Unsupported(refuse!("unqualified slave write")));
            }
            self.writes.set(self.writes.get() + 1);
            Ok(())
        }
    }
    let allow = Rc::new(Cell::new(false));
    let writes = Rc::new(Cell::new(0));
    let mut c = configured(Profile::Gen1, 192, 256);
    c.attach(
        1,
        Box::new(Refused {
            allow: allow.clone(),
            writes: writes.clone(),
        }),
    )
    .unwrap();
    let mut latch = InterruptLatch::new();
    let mut a = Access::new(0, 0, &mut latch);
    wr(&mut c, &mut a, REQUEST, 1 << 8 | LAST | 0x37);
    wr(&mut c, &mut a, REQUEST, 0x44332211);
    assert!(c.write(REQUEST, Width::Word, 0x88776655, &mut a).is_err());
    assert_eq!(c.partial.as_ref().unwrap().1, [0x44332211]);
    assert_eq!(writes.get(), 0);
    assert!(c.responses.is_empty());
    assert!(c.pending.iter().all(|&word| word == 0));
    allow.set(true);
    wr(&mut c, &mut a, REQUEST, 0x88776655);
    assert_eq!(writes.get(), 1);
    assert!(c.partial.is_none());
    assert_eq!(rd(&mut c, &mut a, RESPONSE), 0x8137);
}

#[cfg(feature = "std")]
#[test]
#[ignore = "oracle: STELA_TEST_PMGR_M1_KERNELCACHE"]
fn original_gen1_layout_vector_depth_and_counter_tables_are_source_pinned() {
    let bytes = std::fs::read(std::env::var("STELA_TEST_PMGR_M1_KERNELCACHE").unwrap()).unwrap();
    assert_eq!(
        stela_formats::sha256(&bytes),
        "5109fca5209bdbb89cec29e3241cc42a90418cb737d65e31962b059b9879af90"
    );
    let word = |at: usize| u32::from_le_bytes(bytes[at..at + 4].try_into().unwrap());
    // reg1 fault response; group6 sleep mask; independent reset/control;
    // map reg2 counters; DT response-vector destination; byte->word depths.
    for (offset, opcode) in [
        (0x2d45a1c, 0x91002068),
        (0x2d45a48, 0x528000c8),
        (0x2d45a58, 0x91029049),
        (0x2d45afc, 0x52800041),
        (0x2d45b94, 0xf9406408),
        (0x2d45b9c, 0xb900011f),
        (0x2d45bac, 0xb9000114),
        (0x2d45e84, 0xb8294901),
        (0x2d48f30, 0x91062263),
        (0x2d4e698, 0x53027d6b),
        (0x2d4e69c, 0x53027d4a),
    ] {
        assert_eq!(word(offset), opcode, "original opcode {offset:#x}");
    }
    let values: Vec<u64> = (0..10)
        .map(|i| {
            u64::from_le_bytes(
                bytes[0x71f750 + i * 8..0x71f758 + i * 8]
                    .try_into()
                    .unwrap(),
            )
        })
        .collect();
    assert_eq!(
        values,
        [
            4,
            8,
            0x10000000200,
            0x100000002000000,
            0x20,
            0x60,
            0x100000002000000,
            0,
            0xb8,
            0xa0
        ]
    );
    let offsets: Vec<u64> = (0..16)
        .map(|i| {
            u64::from_le_bytes(
                bytes[0x12a4f80 + i * 16..0x12a4f88 + i * 16]
                    .try_into()
                    .unwrap(),
            )
        })
        .collect();
    assert_eq!(
        offsets,
        [
            0, 4, 8, 0x10, 0x14, 0x18, 0x1c, 0x20, 0x24, 0x5c, 0x60, 0x64, 0x68, 0x6c, 0x70, 0x74
        ]
    );
}
#[test]
fn gen1_only_zero_control_reset_is_checked_and_kept_pauses_keep_owners() {
    let mut c = configured(Profile::Gen1, 192, 256);
    let mut latch = InterruptLatch::new();
    let mut a = Access::new(0, 0, &mut latch);
    wr(&mut c, &mut a, REQUEST, 0xe << 8 | LAST | 0x37);
    wr(&mut c, &mut a, REQUEST, 0x44332211);
    let before = c.partial.clone();
    assert!(c.write(CONTROL, Width::Word, 0x80000001, &mut a).is_err());
    assert_eq!(rd(&mut c, &mut a, CONTROL), 0);
    assert_eq!(c.partial.as_ref().unwrap().1, before.unwrap().1);
    assert!(c.write(QUEUE_RESET, Width::Word, 0, &mut a).is_err());
    assert!(c.partial.is_some());
    wr(&mut c, &mut a, CONTROL, 0);
    wr(&mut c, &mut a, QUEUE_RESET, 1);
    assert!(c.partial.is_none());
    assert_eq!(rd(&mut c, &mut a, STATUS), 0x01000100);
    wr(&mut c, &mut a, REQUEST, 0xe << 8 | LAST | 0x30);
    wr(&mut c, &mut a, REQUEST, 0x7b);
    let responses = c.responses.clone();
    let pending = c.pending;
    let live = c.lifecycle;
    let paused = LifecycleState {
        clocked: false,
        quiesced: true,
        ..live
    };
    c.check_lifecycle(paused).unwrap();
    c.set_lifecycle(paused, &mut a);
    assert!(c.read(RESPONSE, Width::Word, &mut a).is_err());
    assert!(c.write(QUEUE_RESET, Width::Word, 1, &mut a).is_err());
    assert_eq!(c.responses, responses);
    assert_eq!(c.pending, pending);
    c.check_lifecycle(live).unwrap();
    c.set_lifecycle(live, &mut a);
    assert_eq!(rd(&mut c, &mut a, RESPONSE), 0x8e30);
    assert_eq!(c.configuration.response_vector(), 192);
    assert_eq!(c.configuration.queue_bytes(), [256, 256]);
}

#[test]
fn gen3_moves_fault_latches_and_counters() {
    let mut c = configured(Profile::Gen3, 0x100, 256);
    let mut latch = InterruptLatch::new();
    let mut a = Access::new(0, 0, &mut latch);
    assert_eq!(rd(&mut c, &mut a, STATUS), RESPONSE_EMPTY | REQUEST_EMPTY);
    wr(&mut c, &mut a, REQUEST, 0xe << 8 | LAST | 0x30);
    wr(&mut c, &mut a, REQUEST, 0x7b);
    assert_eq!(rd(&mut c, &mut a, PENDING + 8 * 4), 1);
    assert_eq!(rd(&mut c, &mut a, RESPONSE), 0x8e30);
    // The driver's resetQueue: save control, zero it, reset, restore.
    wr(&mut c, &mut a, CONTROL, 0);
    wr(&mut c, &mut a, QUEUE_RESET, 1);
    assert_eq!(rd(&mut c, &mut a, QUEUE_RESET), 0);
    assert!(c.write(CONTROL, Width::Word, 1, &mut a).is_err());
    assert_eq!(rd(&mut c, &mut a, FAULT_STATUS), RESPONSE_EMPTY);
    for latch in (0x200..0x218).step_by(4) {
        assert_eq!(rd(&mut c, &mut a, BANK_B + latch), 0);
        wr(&mut c, &mut a, BANK_B + latch, 0);
    }
    assert_eq!(rd(&mut c, &mut a, BANK_B + 0x4d8), 0);
    // Gen2's latches and counters, the arbiter and the debug windows.
    for offset in [FAULT_LATCHES, BANK_B + 0x25c, BANK_C + 0x20, 0xbc] {
        assert!(c.read(offset, Width::Word, &mut a).is_err(), "{offset:#x}");
    }
}

#[test]
fn gen4_moves_the_queue_bank_and_reports_its_fifo_depths() {
    let mut c = configured(Profile::Gen4, 0x100, 256);
    let mut latch = InterruptLatch::new();
    let mut a = Access::new(0, 0, &mut latch);
    let empty = 1 << 30 | 1 << 14;
    assert_eq!(rd(&mut c, &mut a, 0x200), empty);
    assert_eq!(rd(&mut c, &mut a, 0x48), 64 << 16 | 64);
    wr(&mut c, &mut a, 0x400 + 8 * 4, 1);
    let read = 0x10 << 16 | LAST | 0xe << 8 | 0x38 | 1;
    wr(&mut c, &mut a, 0x210, read);
    assert_eq!(rd(&mut c, &mut a, 0x200), 1 << 14);
    assert!(asserted(&c));
    assert_eq!(rd(&mut c, &mut a, 0x220) & 0xfff, read & 0xfff);
    assert_eq!(rd(&mut c, &mut a, 0x220), 0);
    wr(&mut c, &mut a, 0x600 + 8 * 4, 1);
    assert!(!asserted(&c));
    // resetQueue: control A+0 zeroed around the self-clearing A+4 reset.
    wr(&mut c, &mut a, 0x210, 0xe << 8 | 0x30 | 7);
    assert_eq!(rd(&mut c, &mut a, 0), 0);
    wr(&mut c, &mut a, 0, 0);
    wr(&mut c, &mut a, 4, 1);
    assert_eq!(rd(&mut c, &mut a, 4), 0);
    assert_eq!(rd(&mut c, &mut a, 0x200), empty);
    assert_eq!(rd(&mut c, &mut a, BANK_B + 0x200), 1 << 30);
    assert!(c.read(BANK_B + 0x220, Width::Word, &mut a).is_err());
    assert_eq!(rd(&mut c, &mut a, BANK_B + 0x1214), 0);
    assert_eq!(rd(&mut c, &mut a, BANK_B + 0x1400), 0);
    // The Gen2 offsets are other registers or none.
    for offset in [STATUS + 8, CONTROL, BANK_B + 8, BANK_B + 0x400, 0x800] {
        assert!(c.read(offset, Width::Word, &mut a).is_err(), "{offset:#x}");
    }
}
