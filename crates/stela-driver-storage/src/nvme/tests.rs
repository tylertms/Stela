// SPDX-License-Identifier: GPL-2.0-only
// Copyright (c) 2011-2014, Intel Corporation.
use super::*;
use crate::bus::{dma::RegionAllocator, mmio::Mmio};
use alloc::boxed::Box;

struct Fixture {
    disk: Namespace,
    // The real driver uses volatile MMIO/DMA accesses to these owned,
    // stable allocations. This fixture advances completions explicitly;
    // it is not a physical NVMe model or a reset validation.
    _registers: Box<[u64]>,
    _memory: Vec<u64>,
}

impl Fixture {
    fn new() -> Self {
        let mut registers = vec![0u64; 1024].into_boxed_slice();
        let mut memory = vec![0u64; 196608];
        let start = memory.as_mut_ptr() as u64;
        // SAFETY: this fixture owns both stable allocations until after
        // all controller operations; no device accesses them.
        let (mmio, mut dma) = unsafe {
            (
                Mmio::new(registers.as_mut_ptr() as u64, size_of_val(&registers[..])),
                RegionAllocator::new(start..start + (memory.len() * 8) as u64),
            )
        };
        let controller = Controller {
            registers: RegisterIo::Legacy(mmio),
            doorbell_stride: 4,
            admin: Queue::new(0, ADMIN_ENTRIES, &mut dma).unwrap(),
            io: Queue::new(1, IO_ENTRIES, &mut dma).unwrap(),
            admin_data: Buffer::Legacy(dma.allocate(PAGE, PAGE).unwrap()),
            pool: Pool::legacy(&mut dma).unwrap(),
            held: (0..DEPTH).map(|_| Vec::new()).collect(),
            tags: (0..DEPTH).map(|_| Tag::Free).collect(),
            recovery: None,
            signalled: false,
            owed: false,
            max_transfer: BOUNCE_BYTES,
            max_command_bytes: MAX_COMMAND_BYTES,
            shutdown_timeout_us: 1000,
            shut_down: false,
            command_id: 0,
            // Exercise a real finite deadline with a short captured test
            // policy; native attachment keeps its thirty-second policy.
            clock: Clock {
                ticks_per_microsecond: 1000,
            },
            command_timeout_us: 1000,
            admin_timeout_us: 1000,
            fault: None,
            owned: None,
            teardown: owned::Teardown::Unprepared,
            ready_timeout_us: 500_000,
            configuration: CONFIGURATION_VALUE & !ENABLE,
            independent_identify: false,
            fatal_standing: false,
            version: 0,
            commands: 0,
            scans: 0,
            retry_delays: [0; 3],
            host_memory: None,
            host_memory_enabled: false,
            events: Events::default(),
            aborts: Vec::new(),
            limits: Limits::default(),
            quirks: Quirks::NONE,
        };
        Self {
            _registers: registers,
            _memory: memory,
            disk: Namespace {
                controller: Rc::new(RefCell::new(controller)),
                namespace: 1,
                ids: Ids::default(),
                scanned: 0,
                removed: None,
                identity: Identity {
                    transport: Transport::Nvme,
                    location: "fixture".into(),
                    model: "fixture".into(),
                    serial: "fixture".into(),
                    blocks: 1024,
                    block_size: 512,
                    read_only: false,
                    features: Features {
                        write_cache: true,
                        fua: true,
                        max_transfer: BOUNCE_BYTES,
                        ..Features::default()
                    },
                },
            },
        }
    }
}

#[test]
fn a_timeout_aborts_then_a_controller_that_stays_down_poisons_its_siblings() {
    let _clock = crate::test_clock::virtualize();
    let mut fixture = Fixture::new();
    fixture.disk.controller.borrow_mut().clock = Clock {
        ticks_per_microsecond: 1,
    };
    // Nothing answers: the write times out, its abort goes unanswered,
    // and the controller never reports ready again after its reset.
    assert_eq!(
        fixture.disk.write(0, &[0x31; 512]),
        Err(BlockError::Timeout)
    );
    let mut sibling = Namespace {
        controller: fixture.disk.controller.clone(),
        namespace: 2,
        identity: fixture.disk.identity.clone(),
        ids: Ids::default(),
        scanned: 0,
        removed: None,
    };
    {
        let c = fixture.disk.controller.borrow();
        // The abort names the write's command and queue.
        let abort = &c.admin.submissions.bytes()[..64];
        assert_eq!(abort[0], ADMIN_ABORT);
        assert_eq!(
            u32::from_le_bytes(abort[40..44].try_into().unwrap()),
            1 << 16 | 1
        );
        // The reset re-enabled the controller with the admin queue.
        assert_eq!(
            c.registers.read32(CONFIGURATION).unwrap(),
            CONFIGURATION_VALUE
        );
        assert_eq!(c.fault, Some(BlockError::Timeout));
        assert_eq!(c.command_id, 2);
    }
    let mut output = vec![0x55; 512];
    assert_eq!(sibling.read(0, &mut output), Err(BlockError::Timeout));
    assert!(output.iter().all(|byte| *byte == 0x55));
    assert_eq!(sibling.flush(), Err(BlockError::Timeout));
    assert_eq!(fixture.disk.controller.borrow().command_id, 2);
}

#[test]
fn unrelated_or_malformed_completions_do_not_retire_our_command() {
    for (cid, sqid, head) in [
        (2u32, 1u32, 1u32),
        (1, 2, 1),
        (1, 1, 16),
        (1, 1, 0),
        (1, 1, 2),
    ] {
        let mut fixture = Fixture::new();
        {
            let mut c = fixture.disk.controller.borrow_mut();
            c.io.completions.write(8, sqid << 16 | head);
            c.io.completions.write(12, 1u32 << 16 | cid);
        }
        let error = BlockError::Device { status: 0xffff };
        assert_eq!(fixture.disk.flush(), Err(error.clone()));
        assert_eq!(fixture.disk.write(0, &[0x71; 512]), Err(error));
        let c = fixture.disk.controller.borrow();
        assert_eq!(c.io.head, 0);
        assert_eq!(c.registers.read32(DOORBELLS + 12).unwrap(), 0);
        assert!(pool_untouched(&c));
    }
}

#[test]
fn sq_head_validation_preserves_wrap_and_rejects_stale_or_ahead_heads() {
    let stale = u32::from(IO_ENTRIES - 1);
    for invalid_head in [None, Some(stale), Some(1)] {
        let mut fixture = Fixture::new();
        for next in 1..IO_ENTRIES {
            {
                let mut c = fixture.disk.controller.borrow_mut();
                let offset = usize::from(next - 1) * 16;
                c.io.completions
                    .write(offset + 8, 1u32 << 16 | u32::from(next));
                c.io.completions
                    .write(offset + 12, 1u32 << 16 | u32::from(next));
            }
            fixture.disk.flush().unwrap();
        }
        {
            let mut c = fixture.disk.controller.borrow_mut();
            let last = usize::from(IO_ENTRIES - 1) * 16;
            c.io.completions
                .write(last + 8, 1u32 << 16 | invalid_head.unwrap_or(0));
            c.io.completions
                .write(last + 12, 1u32 << 16 | u32::from(IO_ENTRIES));
        }
        let result = fixture.disk.flush();
        let mut c = fixture.disk.controller.borrow_mut();
        assert_eq!(c.io.tail, 0);
        match invalid_head {
            Some(_) => {
                assert_eq!(result, Err(BlockError::Device { status: 0xffff }));
                assert_eq!(c.io.head, IO_ENTRIES - 1);
                assert!(c.io.phase);
            }
            None => {
                assert_eq!(result, Ok(()));
                assert_eq!(c.io.head, 0);
                assert!(!c.io.phase);
                c.io.completions.write(8, 1u32 << 16 | 1);
                c.io.completions.write(12, u32::from(IO_ENTRIES) + 1); // New phase is zero.
                drop(c);
                fixture.disk.flush().unwrap();
            }
        }
    }
}

#[test]
fn completed_command_error_is_distinct_from_unresolved_dma() {
    let mut fixture = Fixture::new();
    {
        let mut c = fixture.disk.controller.borrow_mut();
        c.io.completions.write(8, 1u32 << 16 | 1);
        // Invalid Field, with Do Not Retry.
        c.io.completions
            .write(12, (STATUS_DO_NOT_RETRY | 2) << 17 | 1 << 16 | 1);
    }
    assert_eq!(
        fixture.disk.flush(),
        Err(BlockError::Device {
            status: STATUS_DO_NOT_RETRY | 2
        })
    );
    {
        let mut c = fixture.disk.controller.borrow_mut();
        assert!(c.fault.is_none());
        assert_eq!(c.io.head, 1);
        c.io.completions.write(16 + 8, 1u32 << 16 | 2);
        c.io.completions.write(16 + 12, 1u32 << 16 | 2);
    }
    assert_eq!(fixture.disk.flush(), Ok(()));
    assert_eq!(fixture.disk.controller.borrow().io.head, 2);
}

#[test]
fn synchronous_read_changes_the_caller_only_after_success() {
    let _clock = crate::test_clock::virtualize();
    let mut fixture = Fixture::new();
    let disk = &mut fixture.disk;
    let mut data = [0x11; 512];
    {
        let mut controller = disk.controller.borrow_mut();
        next_page(&mut controller).bytes_mut()[..512].fill(0x42);
        controller.io.completions.write(8, 1u32 << 16 | 1);
        controller.io.completions.write(12, 1u32 << 16 | 1);
    }
    disk.read(0, &mut data).unwrap();
    assert_eq!(data, [0x42; 512]);
    {
        let mut controller = disk.controller.borrow_mut();
        next_page(&mut controller).bytes_mut()[..512].fill(0x87);
        controller.io.completions.write(16 + 8, 1u32 << 16 | 2);
        controller
            .io
            .completions
            .write(16 + 12, (STATUS_DO_NOT_RETRY | 2) << 17 | 1 << 16 | 2);
    }
    assert_eq!(
        disk.read(1, &mut data),
        Err(BlockError::Device {
            status: STATUS_DO_NOT_RETRY | 2
        })
    );
    assert_eq!(data, [0x42; 512]);
}

#[test]
fn a_fatal_controller_is_reset_first_and_poisons_when_it_stays_fatal() {
    let mut fixture = Fixture::new();
    fixture
        .disk
        .controller
        .borrow()
        .registers
        .write32(STATUS, FATAL)
        .unwrap();
    let error = BlockError::Device { status: FATAL };
    assert_eq!(fixture.disk.write(0, &[0xa9; 512]), Err(error.clone()));
    fixture
        .disk
        .controller
        .borrow()
        .registers
        .write32(STATUS, 0)
        .unwrap();
    assert_eq!(fixture.disk.flush(), Err(error));
    let c = fixture.disk.controller.borrow();
    // Nothing was submitted to the fatal controller, whose reset found
    // it fatal again.
    assert_eq!(
        c.registers.read32(CONFIGURATION).unwrap(),
        CONFIGURATION_VALUE
    );
    assert_eq!((c.command_id, c.io.tail, c.io.head), (0, 0, 0));
    assert!(c.io.submissions.bytes().iter().all(|byte| *byte == 0));
    assert!(pool_untouched(&c));
    assert_eq!(c.registers.read32(DOORBELLS + 8).unwrap(), 0);
}

/// INTMS masks pin and MSI interrupts. A controller configured for MSI-X
/// is never written there, at attach or at a reset.
#[test]
fn the_interrupt_mask_register_is_left_alone_under_msix() {
    for (signalled, mask) in [(false, u32::MAX), (true, 0)] {
        let fixture = Fixture::new();
        let mut c = fixture.disk.controller.borrow_mut();
        c.signalled = signalled;
        c.configure_admin().unwrap();
        assert_eq!(c.registers.read32(INTERRUPT_MASK_SET).unwrap(), mask);
        assert_eq!(c.registers.read32(ADMIN_ATTRIBUTES).unwrap(), 15 << 16 | 15);
        assert_ne!(c.registers.read64(ADMIN_SUBMISSION).unwrap(), 0);
        assert_ne!(c.registers.read64(ADMIN_COMPLETION).unwrap(), 0);
    }
}

fn namespace(blocks: u64, flbas: u8, formats: &[(usize, u32)], attributes: u8) -> Vec<u8> {
    let mut page = vec![0u8; PAGE];
    page[NAMESPACE_SIZE].copy_from_slice(&blocks.to_le_bytes());
    page[NAMESPACE_FORMAT] = flbas;
    page[NAMESPACE_ATTRIBUTES] = attributes;
    for &(index, descriptor) in formats {
        page[NAMESPACE_FORMATS + 4 * index..][..4].copy_from_slice(&descriptor.to_le_bytes());
    }
    page
}

#[test]
fn namespace_formats_use_both_flbas_index_fields_and_write_protection() {
    // Format 17 (index bits 0001 and 01) holds 4 KiB blocks.
    let page = namespace(64, 1 | 1 << 5, &[(17, 12 << 16)], NAMESPACE_WRITE_PROTECTED);
    let format = plain_format(&page, BOUNCE_BYTES).unwrap().unwrap();
    assert_eq!((format.blocks, format.block_size), (64, 4096));
    assert!(format.read_only);
    assert!(
        plain_format(&namespace(0, 0, &[], 0), BOUNCE_BYTES)
            .unwrap()
            .is_none()
    );
    assert_eq!(
        plain_format(&namespace(8, 0, &[(0, 8 | 9 << 16)], 0), BOUNCE_BYTES).err(),
        Some(BlockError::Unsupported("NVMe format with metadata"))
    );
    // A 64 KiB block cannot fit an 8 KiB maximum transfer.
    assert_eq!(
        plain_format(&namespace(8, 0, &[(0, 16 << 16)], 0), 2 * PAGE).err(),
        Some(BlockError::Unsupported("NVMe logical block size"))
    );
}

#[test]
fn shutdown_waits_follow_the_rtd3_entry_latency_within_linux_bounds() {
    assert_eq!(shutdown_timeout_us(0), SHUTDOWN_MINIMUM_US);
    assert_eq!(shutdown_timeout_us(8_000_001), 9 * MICROSECONDS_PER_SECOND);
    assert_eq!(shutdown_timeout_us(u64::MAX), SHUTDOWN_MAXIMUM_US);
}

/// Post, at the I/O queue's head, the completion of the command `id`,
/// with every submission up to the tail consumed.
fn post(controller: &mut Controller, id: u16, status: u32) {
    let entry = usize::from(controller.io.head) * 16;
    let tail = u32::from(controller.io.tail);
    let phase = u32::from(controller.io.phase) << 16;
    controller
        .io
        .completions
        .write(entry + 8, 1u32 << 16 | tail);
    controller
        .io
        .completions
        .write(entry + 12, status << 17 | phase | u32::from(id));
}

/// Post, at the admin queue's head, the completion of the command `id`.
fn post_admin(controller: &mut Controller, id: u16, status: u32) {
    let entry = usize::from(controller.admin.head) * 16;
    let tail = u32::from(controller.admin.tail);
    let phase = u32::from(controller.admin.phase) << 16;
    controller.admin.completions.write(entry + 8, tail);
    controller
        .admin
        .completions
        .write(entry + 12, status << 17 | phase | u32::from(id));
}

fn running_id(controller: &Controller, tag: u32) -> u16 {
    match controller.tags[tag as usize] {
        Tag::Running { id, .. } => id,
        _ => panic!("tag {tag} runs nothing"),
    }
}

/// Post, before it is sent, the successful completion of the next admin
/// command, reporting it fetched.
fn answer_next_admin(controller: &mut Controller) {
    let id = controller.command_id.wrapping_add(1);
    let entry = usize::from(controller.admin.head) * 16;
    let head = u32::from(controller.admin.next(controller.admin.tail));
    let phase = u32::from(controller.admin.phase) << 16;
    controller.admin.completions.write(entry + 8, head);
    controller
        .admin
        .completions
        .write(entry + 12, phase | u32::from(id));
}

/// Linux `nvme_wait_ready` ends at CSTS.RDY and reads no more of CSTS
/// before its first command, whose completion it takes: a CSTS.CFS still
/// set once the controller is ready stands, at attach and after a reset,
/// and fails no command the controller answers. A CFS that rises after a
/// clean start fails the next command before it is sent.
#[test]
fn a_fatal_status_standing_at_ready_fails_no_command_the_controller_answers() {
    let _clock = crate::test_clock::virtualize();
    let fixture = Fixture::new();
    let mut c = fixture.disk.controller.borrow_mut();
    c.registers.write32(STATUS, READY | FATAL).unwrap();
    c.wait_enabled(1000).unwrap();
    assert!(c.fatal_standing);
    answer_next_admin(&mut c);
    let identify = c.identify_command(IDENTIFY_CONTROLLER, 0).unwrap();
    assert_eq!(c.admin(identify), Ok(0));
    assert_eq!(c.fault, None);

    // A clean start: the CFS that rises later is the controller's failure.
    c.registers.write32(STATUS, READY).unwrap();
    c.wait_enabled(1000).unwrap();
    assert!(!c.fatal_standing);
    c.registers.write32(STATUS, READY | FATAL).unwrap();
    let tail = c.admin.tail;
    assert!(matches!(
        c.admin_wait(identify, true),
        Err(Lost::Reset(BlockError::Device { status: 3 }))
    ));
    assert_eq!(c.admin.tail, tail, "nothing sent to the failed controller");

    // A reset's enable keeps a CFS standing at ready the same way and
    // goes on to Identify Controller.
    c.fatal_standing = false;
    let deadline = c.deadline(1000).unwrap();
    c.recovery = Some(Recovery::Enabling(deadline));
    c.recover().unwrap();
    assert!(c.fatal_standing);
    assert!(matches!(
        c.recovery,
        Some(Recovery::Admin {
            step: Step::Identify,
            ..
        })
    ));
}

/// CAP.MQES bounds the I/O queues alone, as Linux shortens its I/O queue
/// depth to it: a controller with four-entry queues is started with a
/// four-entry I/O queue and three tags. Only the reserved MQES 0, one
/// entry that can hold no command, is refused.
#[test]
fn a_short_mqes_shortens_the_io_queue_and_only_one_entry_is_refused() {
    let _clock = crate::test_clock::virtualize();
    let fixture = Fixture::new();
    let mut c = fixture.disk.controller.borrow_mut();
    c.clock = Clock {
        ticks_per_microsecond: 1,
    };
    // MQES 3, CAP.TO one half second, the NVM command set. The fixture's
    // CSTS never shows RDY, so the start ends waiting for it.
    c.registers
        .write64(CAPABILITIES, 3 | 1 << 24 | 1 << 37)
        .unwrap();
    assert_eq!(c.initialize(), Err(BlockError::Timeout.at(ENABLE_WAIT)));
    assert_eq!(c.io.entries, 4);
    assert_eq!(c.tags.len(), 3);
    c.registers
        .write64(CAPABILITIES, 1 << 24 | 1 << 37)
        .unwrap();
    assert!(matches!(c.initialize(), Err(BlockError::Unsupported(_))));
}

/// `NVME_QUIRK_QDEPTH_ONE` runs one command on a two-entry queue whatever
/// CAP.MQES says, and the PM1725 rule reads its MQES 0 as 64 entries
/// instead of refusing it.
#[test]
fn queue_depth_quirks_set_the_io_queue_whatever_mqes_says() {
    let _clock = crate::test_clock::virtualize();
    let fixture = Fixture::new();
    let mut c = fixture.disk.controller.borrow_mut();
    c.clock = Clock {
        ticks_per_microsecond: 1,
    };
    c.quirks = Quirks::QUEUE_DEPTH_ONE;
    c.registers
        .write64(CAPABILITIES, 63 | 1 << 24 | 1 << 37)
        .unwrap();
    assert_eq!(c.initialize(), Err(BlockError::Timeout.at(ENABLE_WAIT)));
    assert_eq!((c.io.entries, c.tags.len()), (2, 1));
    assert_eq!(c.create_queue(true).unwrap()[10] >> 16, 1);

    let fixture = Fixture::new();
    let mut c = fixture.disk.controller.borrow_mut();
    c.clock = Clock {
        ticks_per_microsecond: 1,
    };
    assert!(Quirks::for_device(0x144d, 0xa821).contains(Quirks::QUEUE_DEPTH_64));
    c.quirks = Quirks::QUEUE_DEPTH_64;
    c.registers
        .write64(CAPABILITIES, 1 << 24 | 1 << 37)
        .unwrap();
    assert_eq!(c.initialize(), Err(BlockError::Timeout.at(ENABLE_WAIT)));
    assert_eq!((c.io.entries, c.tags.len()), (64, DEPTH));
}

/// A reset of a controller with `NVME_QUIRK_DELAY_BEFORE_CHK_RDY` reads
/// CSTS only 2.3 s after clearing CC.EN, and each look meanwhile returns
/// at once: CSTS reading all ones in that time is not a removal.
#[test]
fn a_reset_reads_csts_only_after_the_ready_delay_under_its_quirk() {
    let _clock = crate::test_clock::virtualize();
    let fixture = Fixture::new();
    let mut c = fixture.disk.controller.borrow_mut();
    c.quirks = Quirks::READY_DELAY;
    c.disable().unwrap();
    assert_eq!(c.registers.read32(CONFIGURATION).unwrap(), 0);
    c.registers.write32(STATUS, REMOVED).unwrap();
    for _ in 0..3 {
        let started = Clock::ticks();
        assert_eq!(c.recover(), Ok(()));
        assert!(Clock::ticks() - started < 1000 * 1000, "a step took 1 ms");
        assert!(matches!(c.recovery, Some(Recovery::Disabling(_, Some(_)))));
    }
    crate::test_clock::advance(c.clock, quirks::READY_CHECK_DELAY_US);
    c.registers.write32(STATUS, 0).unwrap();
    assert_eq!(c.recover(), Ok(()));
    assert!(matches!(c.recovery, Some(Recovery::Enabling(_))));

    // Without it, the same CSTS is a removal at the first look.
    let fixture = Fixture::new();
    let mut c = fixture.disk.controller.borrow_mut();
    c.disable().unwrap();
    assert!(matches!(c.recovery, Some(Recovery::Disabling(_, None))));
    c.registers.write32(STATUS, REMOVED).unwrap();
    assert_eq!(c.recover(), Err(removed()));
}

/// `NVME_QUIRK_NO_FUA` and `NVME_QUIRK_DISABLE_WRITE_ZEROES` leave FUA and
/// Write Zeroes out of a namespace's features; other quirks change neither.
#[test]
fn feature_quirks_leave_fua_and_write_zeroes_out() {
    let format = Format {
        blocks: 1024,
        block_size: 512,
        read_only: false,
        deallocate: 0,
    };
    let transfer = TransferLimits {
        data: BOUNCE_BYTES,
        command: MAX_COMMAND_BYTES,
    };
    let commands = COMMANDS_DATASET_MANAGEMENT | COMMANDS_WRITE_ZEROES;
    let of = |quirks| features(true, commands, Limits::default(), &format, transfer, quirks);
    let plain = of(Quirks::NONE);
    assert!(plain.fua && plain.write_zeroes.is_some());
    assert_eq!(
        of(Quirks::NO_DEEPEST_POWER_STATE | Quirks::BOGUS_IDS),
        plain
    );
    let no_fua = of(Quirks::NO_FUA);
    assert!(no_fua.write_cache && !no_fua.fua);
    assert_eq!(no_fua.write_zeroes, plain.write_zeroes);
    let no_zeroes = of(Quirks::NO_WRITE_ZEROES);
    assert_eq!(no_zeroes.write_zeroes, None);
    assert!(no_zeroes.fua && no_zeroes.discard.is_some());
}

/// Bus mastering comes between the stop and the start: CC is cleared and
/// CSTS.RDY seen clear first, CC.EN set only after; a controller that never
/// stops is never given it.
#[test]
fn bus_mastering_is_enabled_only_once_the_controller_stopped() {
    let _clock = crate::test_clock::virtualize();
    let fixture = Fixture::new();
    let mut c = fixture.disk.controller.borrow_mut();
    c.clock = Clock {
        ticks_per_microsecond: 1,
    };
    c.registers
        .write64(CAPABILITIES, 15 | 1 << 24 | 1 << 37)
        .unwrap();
    // As the firmware left it: enabled and ready.
    c.registers
        .write32(CONFIGURATION, CONFIGURATION_VALUE)
        .unwrap();
    c.registers.write32(STATUS, READY).unwrap();
    let mut seen = None;
    let observe = |c: &Controller| {
        (
            c.registers.read32(CONFIGURATION).unwrap(),
            c.registers.read32(STATUS).unwrap(),
        )
    };
    let started = start_after_stop(&mut c, |c| {
        seen = Some(observe(c));
        Ok(false)
    });
    assert_eq!(seen, None, "a controller that never stopped got DMA");
    assert_eq!(started, Err(BlockError::Timeout.at(DISABLE_WAIT)));

    c.registers.write32(STATUS, 0).unwrap();
    let started = start_after_stop(&mut c, |c| {
        seen = Some(observe(c));
        Ok(true)
    });
    assert_eq!(seen, Some((0, 0)));
    // The fixture never sets CSTS.RDY again.
    assert_eq!(started, Err(BlockError::Timeout.at(ENABLE_WAIT)));
    assert_eq!(c.registers.read32(CONFIGURATION).unwrap() & ENABLE, ENABLE);
    assert!(c.signalled);
}

#[test]
fn submission_publication_keeps_the_whole_wire_entry_and_rings_its_queue() {
    let fixture = Fixture::new();
    let mut controller = fixture.disk.controller.borrow_mut();
    for io in [false, true] {
        let mut words = core::array::from_fn(|index| 0x1020_3040 + index as u32 * 0x0101_0101);
        words[0] &= 0xffff;
        let id = controller.publish(io, words, Data::None).unwrap();
        words[0] |= u32::from(id) << 16;
        let queue = if io {
            &controller.io
        } else {
            &controller.admin
        };
        let expected = words.map(u32::to_le_bytes).concat();
        assert_eq!(&queue.submissions.bytes()[..64], &expected);
        assert_eq!(&queue.submissions.bytes()[64..128], &[0; 64]);
        let doorbell = DOORBELLS + 2 * usize::from(queue.id) * controller.doorbell_stride;
        assert_eq!(controller.registers.read32(doorbell), Ok(1));
    }
}

/// The pool page the next command takes first.
fn next_page(controller: &mut Controller) -> &mut Buffer {
    let page = *controller.pool.free.last().unwrap();
    &mut controller.pool.pages[usize::from(page)]
}

/// No pool page holds data.
fn pool_untouched(controller: &Controller) -> bool {
    controller
        .pool
        .pages
        .iter()
        .all(|page| page.bytes().iter().all(|byte| *byte == 0))
}

/// The submission entry of tag `tag`'s command: its PRP1 and PRP2.
fn prps(controller: &Controller, tag: u32) -> (u64, u64) {
    let Tag::Running { slot, .. } = controller.tags[tag as usize] else {
        panic!("tag {tag} runs nothing");
    };
    let entry = &controller.io.submissions.bytes()[usize::from(slot) * 64..][..64];
    let word = |at: usize| u64::from_le_bytes(entry[at..at + 8].try_into().unwrap());
    (word(24), word(32))
}

#[test]
fn prp_lists_name_every_page_a_command_holds_across_page_boundaries() {
    let mut fixture = Fixture::new();
    let disk = &mut fixture.disk;
    // One, two, three and thirty-two pages: PRP2 unused, a page, a list.
    let sizes = [512, PAGE + 512, 2 * PAGE + 512, BOUNCE_BYTES];
    let tags: Vec<u32> = sizes
        .iter()
        .map(|&bytes| disk.start(Command::Read { lba: 0, bytes }).unwrap())
        .collect();
    let mut controller = disk.controller.borrow_mut();
    let mut seen = Vec::new();
    for (&bytes, &tag) in sizes.iter().zip(&tags) {
        let pages = bytes.div_ceil(PAGE);
        let held = controller.held[tag as usize].clone();
        assert_eq!(held.len(), pages_for(bytes));
        seen.extend(held.iter().copied());
        let address = |controller: &mut Controller, index| {
            controller.tag_page(tag as usize, index).address().unwrap()
        };
        let (prp1, prp2) = prps(&controller, tag);
        assert_eq!(prp1, address(&mut controller, 0));
        assert!(prp1.is_multiple_of(PAGE as u64));
        match pages {
            1 => assert_eq!(prp2, 0),
            2 => assert_eq!(prp2, address(&mut controller, 1)),
            _ => {
                assert_eq!(prp2, address(&mut controller, pages));
                let list =
                    controller.tag_page(tag as usize, pages).bytes()[..8 * (pages - 1)].to_vec();
                for (index, entry) in list.as_chunks::<8>().0.iter().enumerate() {
                    assert_eq!(
                        u64::from_le_bytes(*entry),
                        address(&mut controller, index + 1)
                    );
                }
            }
        }
    }
    // No page is held twice.
    seen.sort_unstable();
    seen.dedup();
    assert_eq!(
        seen.len(),
        sizes.iter().map(|&b| pages_for(b)).sum::<usize>()
    );
    assert_eq!(controller.pool.free.len(), POOL_PAGES - seen.len());
}

#[test]
fn a_read_across_pages_gathers_each_page_and_returns_them_to_the_pool() {
    let mut fixture = Fixture::new();
    let disk = &mut fixture.disk;
    let bytes = 3 * PAGE;
    let read = disk.start(Command::Read { lba: 0, bytes }).unwrap();
    {
        let mut controller = disk.controller.borrow_mut();
        for index in 0..3 {
            controller
                .tag_page(read as usize, index)
                .bytes_mut()
                .fill(index as u8 + 1);
        }
        let id = running_id(&controller, read);
        post(&mut controller, id, 0);
    }
    let mut expected = vec![1; PAGE];
    expected.extend([2; PAGE]);
    expected.extend([3; PAGE]);
    assert_eq!(disk.finish(read), Some(Ok(Some(expected))));
    let controller = disk.controller.borrow();
    assert!(controller.held[read as usize].is_empty());
    assert_eq!(controller.pool.free.len(), POOL_PAGES);
}

#[test]
fn a_write_across_pages_copies_each_page_and_keeps_force_unit_access() {
    let mut fixture = Fixture::new();
    let disk = &mut fixture.disk;
    let data: Vec<u8> = (0..2 * PAGE + 1024).map(|at| (at / 512) as u8).collect();
    let write = disk
        .start(Command::Write {
            lba: 4,
            data: data.clone(),
            fua: true,
        })
        .unwrap();
    let mut controller = disk.controller.borrow_mut();
    let mut written = Vec::new();
    for (index, part) in data.chunks(PAGE).enumerate() {
        written
            .extend_from_slice(&controller.tag_page(write as usize, index).bytes()[..part.len()]);
    }
    assert_eq!(written, data);
    let Tag::Running { slot, .. } = controller.tags[write as usize] else {
        panic!("the write runs");
    };
    let entry = &controller.io.submissions.bytes()[usize::from(slot) * 64..][..64];
    let cdw12 = u32::from_le_bytes(entry[48..52].try_into().unwrap());
    assert_eq!(cdw12, FORCE_UNIT_ACCESS | (data.len() / 512 - 1) as u32);
}

#[test]
fn small_commands_run_past_eight_until_the_pool_cannot_take_the_largest() {
    let mut fixture = Fixture::new();
    let disk = &mut fixture.disk;
    let largest = pages_for(BOUNCE_BYTES);
    let mut running = Vec::new();
    while disk.room() {
        running.push(
            disk.start(Command::Read {
                lba: 0,
                bytes: 4 * 512,
            })
            .unwrap(),
        );
    }
    // One page each: every tag runs while the pool keeps the largest.
    assert_eq!(running.len(), DEPTH.min(POOL_PAGES - largest + 1));
    assert!(running.len() > 8);
    // Nine pages each: the pool, not the tags, is what runs out.
    let mut fixture = Fixture::new();
    let disk = &mut fixture.disk;
    let mut running = 0;
    while disk.room() {
        disk.start(Command::Read {
            lba: 0,
            bytes: 8 * PAGE,
        })
        .unwrap();
        running += 1;
    }
    assert_eq!(running, (POOL_PAGES - largest) / pages_for(8 * PAGE) + 1);
    assert!(running < DEPTH);
    assert!(disk.controller.borrow().pool.free.len() < largest);
}

#[test]
fn a_retried_command_keeps_its_pages_and_a_failed_one_returns_them() {
    let _clock = crate::test_clock::virtualize();
    let mut fixture = Fixture::new();
    let disk = &mut fixture.disk;
    let write = disk
        .start(Command::Write {
            lba: 0,
            data: vec![0x3c; 3 * PAGE],
            fua: false,
        })
        .unwrap();
    let held = disk.controller.borrow().held[write as usize].clone();
    {
        let mut controller = disk.controller.borrow_mut();
        let id = running_id(&controller, write);
        // Namespace Not Ready, retried at once.
        post(&mut controller, id, 0x82);
    }
    assert_eq!(disk.finish(write), None);
    {
        let controller = disk.controller.borrow();
        assert_eq!(controller.held[write as usize], held);
        assert!(matches!(
            controller.tags[write as usize],
            Tag::Running { retries: 1, .. }
        ));
        assert_eq!(
            prps(&controller, write).0,
            controller.pool.pages[usize::from(held[0])]
                .address()
                .unwrap()
        );
    }
    {
        let mut controller = disk.controller.borrow_mut();
        let id = running_id(&controller, write);
        post(&mut controller, id, STATUS_DO_NOT_RETRY | 0x80);
    }
    assert_eq!(
        disk.finish(write),
        Some(Err(BlockError::Device {
            status: STATUS_DO_NOT_RETRY | 0x80
        }))
    );
    let controller = disk.controller.borrow();
    assert!(controller.held[write as usize].is_empty());
    assert_eq!(controller.pool.free.len(), POOL_PAGES);
}

#[test]
fn queued_commands_run_together_and_complete_in_any_order() {
    let mut fixture = Fixture::new();
    let disk = &mut fixture.disk;
    assert_eq!(disk.depth(), DEPTH);
    let read = disk.start(Command::Read { lba: 0, bytes: 512 }).unwrap();
    let write = disk
        .start(Command::Write {
            lba: 1,
            data: vec![7; 512],
            fua: true,
        })
        .unwrap();
    assert_ne!(read, write);
    assert_eq!(
        disk.controller.borrow().io.tail,
        2,
        "both submitted at once"
    );
    assert_eq!(disk.finish(read), None, "not complete yet");
    {
        let mut controller = disk.controller.borrow_mut();
        // The write carries Force Unit Access.
        let words = controller.io.submissions.bytes()[64 + 48..64 + 52].to_vec();
        assert_eq!(
            u32::from_le_bytes(words.try_into().unwrap()),
            FORCE_UNIT_ACCESS
        );
        // The write completes first.
        let id = running_id(&controller, write);
        post(&mut controller, id, 0);
    }
    assert_eq!(disk.finish(write), Some(Ok(None)));
    assert_eq!(disk.finish(read), None);
    {
        let mut controller = disk.controller.borrow_mut();
        let tag = read as usize;
        controller.tag_page(tag, 0).bytes_mut()[..512].fill(0x42);
        let id = running_id(&controller, read);
        post(&mut controller, id, 0);
    }
    assert_eq!(disk.finish(read), Some(Ok(Some(vec![0x42; 512]))));
    assert_eq!(disk.controller.borrow().io.head, 2);
    // Every tag in use leaves no room.
    for _ in 0..DEPTH {
        disk.start(Command::Flush).unwrap();
    }
    assert!(!disk.room());
    assert!(disk.start(Command::Flush).is_err());
}

#[test]
fn borrowed_read_does_not_consume_another_tags_owned_result() {
    let _clock = crate::test_clock::virtualize();
    let mut fixture = Fixture::new();
    let disk = &mut fixture.disk;
    let first = disk.start(Command::Read { lba: 0, bytes: 512 }).unwrap();
    let second = disk.start(Command::Read { lba: 1, bytes: 512 }).unwrap();
    let mut controller = disk.controller.borrow_mut();
    let mut data = [0x11; 512];
    let id = running_id(&controller, first);
    controller.tag_page(first as usize, 0).bytes_mut()[..512].fill(0x42);
    post(&mut controller, id, 0);
    assert_eq!(
        controller.finish_into(second as usize, Some(&mut data)),
        None
    );
    assert_eq!(data, [0x11; 512]);
    // The completed read's pages went back to the pool with its data
    // copied out: reusing them leaves its result as it was.
    next_page(&mut controller).bytes_mut()[..512].fill(0x87);
    let id = running_id(&controller, second);
    controller.tag_page(second as usize, 0).bytes_mut()[..512].fill(0x5a);
    post(&mut controller, id, 0);
    assert_eq!(
        controller.finish_into(second as usize, Some(&mut data)),
        Some(Ok(None))
    );
    assert_eq!(data, [0x5a; 512]);
    assert_eq!(
        controller.finish(first as usize),
        Some(Ok(Some(vec![0x42; 512])))
    );
}

#[test]
fn a_completed_borrowed_read_survives_a_later_cq_fault() {
    let _clock = crate::test_clock::virtualize();
    let mut fixture = Fixture::new();
    let disk = &mut fixture.disk;
    let read = disk.start(Command::Read { lba: 0, bytes: 512 }).unwrap();
    let other = disk.start(Command::Flush).unwrap();
    let mut controller = disk.controller.borrow_mut();
    let id = running_id(&controller, read);
    controller.tag_page(read as usize, 0).bytes_mut()[..512].fill(0x5a);
    post(&mut controller, id, 0);
    let other_id = running_id(&controller, other);
    controller.io.completions.write(16 + 8, 2u32 << 16 | 2);
    controller
        .io
        .completions
        .write(16 + 12, 1u32 << 16 | u32::from(other_id));
    let mut data = [0x11; 512];
    assert_eq!(
        controller.finish_into(read as usize, Some(&mut data)),
        Some(Ok(None))
    );
    assert_eq!(data, [0x5a; 512]);
    assert_eq!(controller.io.head, 1);
    assert_eq!(
        controller.finish(other as usize),
        Some(Err(BlockError::Device { status: 0xffff }))
    );
}

#[test]
fn taking_a_ready_result_leaves_live_deadlines_to_their_poll() {
    let _clock = crate::test_clock::virtualize();
    let mut fixture = Fixture::new();
    let disk = &mut fixture.disk;
    let done = disk.start(Command::Flush).unwrap();
    let live = disk.start(Command::Flush).unwrap();
    let clock = {
        let mut controller = disk.controller.borrow_mut();
        let id = running_id(&controller, done);
        post(&mut controller, id, 0);
        controller.advance();
        controller.clock
    };
    crate::test_clock::advance(clock, 1000);

    assert_eq!(disk.finish(done), Some(Ok(None)));
    assert!(disk.controller.borrow().aborts.is_empty());
    assert_eq!(disk.finish(live), None);
    assert_eq!(disk.controller.borrow().aborts.len(), 1);
}

#[test]
fn delayed_read_completes_after_submission_ring_wraps() {
    let _clock = crate::test_clock::virtualize();
    for turns in [1, 2] {
        let mut fixture = Fixture::new();
        let disk = &mut fixture.disk;
        let delayed = disk.start(Command::Read { lba: 0, bytes: 512 }).unwrap();
        let delayed_id = running_id(&disk.controller.borrow(), delayed);
        for _ in 1..usize::from(IO_ENTRIES) * turns {
            let read = disk.start(Command::Read { lba: 1, bytes: 512 }).unwrap();
            {
                let mut controller = disk.controller.borrow_mut();
                controller.tag_page(read as usize, 0).bytes_mut()[..512].fill(0x71);
                let id = running_id(&controller, read);
                post(&mut controller, id, 0);
            }
            assert_eq!(disk.finish(read), Some(Ok(Some(vec![0x71; 512]))));
            assert_eq!(disk.finish(delayed), None);
        }
        {
            let mut controller = disk.controller.borrow_mut();
            assert_eq!(controller.io.tail, 0);
            assert_eq!(controller.io.consumed, 0);
            controller.tag_page(delayed as usize, 0).bytes_mut()[..512].fill(0x42);
            post(&mut controller, delayed_id, 0);
        }
        assert_eq!(disk.finish(delayed), Some(Ok(Some(vec![0x42; 512]))));
        assert!(disk.controller.borrow().fault.is_none());
    }
}

#[test]
fn reused_tag_requires_its_new_submission_to_be_fetched() {
    let _clock = crate::test_clock::virtualize();
    let mut fixture = Fixture::new();
    let disk = &mut fixture.disk;
    let first = disk.start(Command::Read { lba: 0, bytes: 512 }).unwrap();
    {
        let mut controller = disk.controller.borrow_mut();
        let id = running_id(&controller, first);
        post(&mut controller, id, 0);
    }
    assert!(disk.finish(first).unwrap().is_ok());
    let second = disk.start(Command::Read { lba: 1, bytes: 512 }).unwrap();
    assert_eq!(first, second);
    {
        let mut controller = disk.controller.borrow_mut();
        let id = running_id(&controller, second);
        let entry = usize::from(controller.io.head) * 16;
        let consumed = u32::from(controller.io.consumed);
        let phase = u32::from(controller.io.phase) << 16;
        controller
            .io
            .completions
            .write(entry + 8, 1u32 << 16 | consumed);
        controller
            .io
            .completions
            .write(entry + 12, phase | u32::from(id));
    }
    assert_eq!(
        disk.finish(second),
        Some(Err(BlockError::Device { status: 0xffff }))
    );
    assert_eq!(disk.controller.borrow().io.head, 1);
}

#[test]
fn deallocate_and_write_zeroes_carry_their_ranges() {
    let mut fixture = Fixture::new();
    let disk = &mut fixture.disk;
    let discard = disk
        .start(Command::Discard {
            extents: vec![
                crate::block::Extent { lba: 9, blocks: 3 },
                crate::block::Extent { lba: 40, blocks: 2 },
            ],
        })
        .unwrap();
    let zeroes = disk
        .start(Command::WriteZeroes {
            lba: 20,
            blocks: 4,
            unmap: true,
        })
        .unwrap();
    let mut controller = disk.controller.borrow_mut();
    let word = |controller: &Controller, entry: usize, index: usize| {
        let bytes = &controller.io.submissions.bytes()[entry * 64 + 4 * index..][..4];
        u32::from_le_bytes(bytes.try_into().unwrap())
    };
    assert_eq!(
        word(&controller, 0, 0) & 0xff,
        u32::from(IO_DATASET_MANAGEMENT)
    );
    assert_eq!(word(&controller, 0, 10), 1);
    assert_eq!(word(&controller, 0, 11), ATTRIBUTE_DEALLOCATE);
    let range = controller.tag_page(discard as usize, 0).bytes()[..32].to_vec();
    assert_eq!(range[4..8], 3u32.to_le_bytes());
    assert_eq!(range[8..16], 9u64.to_le_bytes());
    assert_eq!(range[20..24], 2u32.to_le_bytes());
    assert_eq!(range[24..32], 40u64.to_le_bytes());
    assert_eq!(word(&controller, 1, 0) & 0xff, u32::from(IO_WRITE_ZEROES));
    assert_eq!(word(&controller, 1, 10), 20);
    assert_eq!(word(&controller, 1, 12), 3 | DEALLOCATE);
    let _ = zeroes;
}

#[test]
fn a_command_past_its_deadline_is_aborted_then_resets_the_controller_and_runs_again() {
    let _clock = crate::test_clock::virtualize();
    let mut fixture = Fixture::new();
    let disk = &mut fixture.disk;
    let read = disk.start(Command::Read { lba: 0, bytes: 512 }).unwrap();
    let read_id = running_id(&disk.controller.borrow(), read);
    let wait = |disk: &mut Namespace| {
        let clock = disk.controller.borrow().clock;
        let started = clock.try_start().unwrap();
        while clock.try_elapsed_us(started).unwrap() < 1000 {
            core::hint::spin_loop();
        }
    };
    // The deadline passes: the read is aborted and given another.
    wait(disk);
    assert_eq!(disk.finish(read), None);
    {
        let mut controller = disk.controller.borrow_mut();
        let abort = &controller.admin.submissions.bytes()[..64];
        assert_eq!(abort[0], ADMIN_ABORT);
        assert_eq!(abort[40..44], (u32::from(read_id) << 16 | 1).to_le_bytes());
        // The controller reports it not aborted.
        let id = controller.aborts[0];
        post_admin(&mut controller, id, 0);
    }
    assert_eq!(disk.finish(read), None);
    assert!(disk.controller.borrow().aborts.is_empty());
    // The second deadline passes: the controller, not ready, is disabled.
    wait(disk);
    assert_eq!(disk.finish(read), None);
    {
        let controller = disk.controller.borrow();
        assert!(matches!(controller.recovery, Some(Recovery::Disabling(..))));
        assert_eq!(controller.registers.read32(CONFIGURATION).unwrap(), 0);
        assert!(!disk.room());
    }
    // CSTS.RDY is clear: it is enabled again.
    assert_eq!(disk.finish(read), None);
    {
        let controller = disk.controller.borrow();
        assert!(matches!(controller.recovery, Some(Recovery::Enabling(_))));
        controller.registers.write32(STATUS, READY).unwrap();
    }
    // Ready: identified, then the queue count set and the I/O queues
    // created again, completion queue first.
    for opcode in [
        ADMIN_IDENTIFY,
        ADMIN_SET_FEATURES,
        ADMIN_CREATE_COMPLETION,
        ADMIN_CREATE_SUBMISSION,
    ] {
        assert_eq!(disk.finish(read), None);
        let mut controller = disk.controller.borrow_mut();
        let Some(Recovery::Admin { id, .. }) = controller.recovery else {
            panic!("no bring-up command");
        };
        let offset = usize::from(controller.admin.tail - 1) * 64;
        assert_eq!(controller.admin.submissions.bytes()[offset], opcode);
        post_admin(&mut controller, id, 0);
    }
    // The interrupted read runs again on the empty queue.
    assert_eq!(disk.finish(read), None);
    let mut controller = disk.controller.borrow_mut();
    assert!(controller.recovery.is_none());
    assert_eq!(controller.scans, 1);
    assert_eq!((controller.io.tail, controller.io.head), (1, 0));
    let id = running_id(&controller, read);
    controller.tag_page(read as usize, 0).bytes_mut()[..512].fill(0x5a);
    post(&mut controller, id, 0);
    drop(controller);
    assert_eq!(disk.finish(read), Some(Ok(Some(vec![0x5a; 512]))));
}

#[test]
fn errors_without_do_not_retry_run_again_after_their_retry_delay() {
    let _clock = crate::test_clock::virtualize();
    let mut fixture = Fixture::new();
    let disk = &mut fixture.disk;
    disk.controller.borrow_mut().clock = Clock {
        ticks_per_microsecond: 1,
    };
    disk.controller.borrow_mut().retry_delays = [1, 0, 0];
    let flush = disk.start(Command::Flush).unwrap();
    {
        let mut controller = disk.controller.borrow_mut();
        let id = running_id(&controller, flush);
        // Namespace Not Ready with CRD 1: 100 ms.
        post(&mut controller, id, 0x82 | 1 << 11);
    }
    assert_eq!(disk.finish(flush), None);
    assert!(matches!(
        disk.controller.borrow().tags[flush as usize],
        Tag::Delayed { retries: 1, .. }
    ));
    let clock = disk.controller.borrow().clock;
    let started = clock.try_start().unwrap();
    while disk.controller.borrow().io.tail == 1 {
        assert_eq!(disk.finish(flush), None);
    }
    assert!(clock.try_elapsed_us(started).unwrap() >= 100_000);
    {
        let mut controller = disk.controller.borrow_mut();
        let id = running_id(&controller, flush);
        post(&mut controller, id, 0);
    }
    assert_eq!(disk.finish(flush), Some(Ok(None)));
}

#[test]
fn due_retries_publish_before_expired_commands_abort_in_tag_order() {
    let _clock = crate::test_clock::virtualize();
    let mut fixture = Fixture::new();
    let disk = &mut fixture.disk;
    let first = disk.start(Command::Flush).unwrap();
    let ready = disk.start(Command::Flush).unwrap();
    let pending = disk.start(Command::Flush).unwrap();
    let last = disk.start(Command::Flush).unwrap();
    let mut controller = disk.controller.borrow_mut();
    controller.retry_delays = [1, 0, 0];
    let first_id = running_id(&controller, first);
    let ready_id = running_id(&controller, ready);
    let pending_id = running_id(&controller, pending);
    let last_id = running_id(&controller, last);
    post(&mut controller, pending_id, 0x82 | 1 << 11);
    controller.advance();
    assert!(controller.aborts.is_empty());
    assert!(matches!(
        controller.tags[pending as usize],
        Tag::Delayed { .. }
    ));

    crate::test_clock::advance(controller.clock, controller.command_timeout_us);
    post(&mut controller, ready_id, 0x82);
    controller.advance();
    let retry_id = running_id(&controller, ready);
    assert_eq!(retry_id, last_id + 1);
    assert_eq!(controller.aborts, [retry_id + 1, retry_id + 2]);
    for (entry, command) in controller
        .admin
        .submissions
        .bytes()
        .as_chunks::<64>()
        .0
        .iter()
        .zip([first_id, last_id])
    {
        assert_eq!(entry[0], ADMIN_ABORT);
        assert_eq!(entry[40..44], (u32::from(command) << 16 | 1).to_le_bytes());
    }
    assert!(matches!(
        controller.tags[pending as usize],
        Tag::Delayed { .. }
    ));
    assert!(matches!(
        controller.tags[ready as usize],
        Tag::Running { aborted: false, .. }
    ));
}

/// An Identify Controller page with APSTA and NPSS 5, whose (state, entry
/// us, exit us) non-operational states are `states`.
fn apst_identify(states: &[(usize, u32, u32)]) -> Vec<u8> {
    let mut identify = vec![0u8; PAGE];
    identify[CONTROLLER_APST] = 1;
    identify[CONTROLLER_POWER_STATES] = 5;
    for &(state, entry, exit) in states {
        let descriptor = &mut identify[CONTROLLER_POWER_DESCRIPTORS + 32 * state..][..32];
        descriptor[3] = POWER_NON_OPERATIONAL;
        descriptor[4..8].copy_from_slice(&u32::to_le_bytes(entry));
        descriptor[8..12].copy_from_slice(&u32::to_le_bytes(exit));
    }
    identify
}

/// Each state idles into the deepest usable non-operational state below
/// it, one per latency tolerance, as Linux `nvme_configure_apst`; a state
/// whose exit latency is past `default_ps_max_latency_us` is never a target.
#[test]
fn apst_follows_linux_tolerances_from_the_deepest_state_up() {
    let mut identify = vec![0u8; PAGE];
    assert_eq!(apst_table(&identify, Quirks::NONE), None, "no APST support");
    identify[CONTROLLER_POWER_STATES] = 5;
    assert_eq!(apst_table(&identify, Quirks::NO_APST), None);
    // PS5's exit is past the latency limit.
    let identify = apst_identify(&[
        (2, 500, 500),
        (3, 2_000, 2_000),
        (4, 4_000, 30_000),
        (5, 0, 200_000),
    ]);
    let (table, enable) = apst_table(&identify, Quirks::NONE).unwrap();
    assert!(enable);
    let primary = 3 << 3 | APST_PRIMARY_TIMEOUT_MS << 8;
    let secondary = 4 << 3 | APST_SECONDARY_TIMEOUT_MS << 8;
    assert_eq!(table[..6], [primary, primary, primary, secondary, 0, 0]);
    assert!(table[6..].iter().all(|&entry| entry == 0));
}

/// `NVME_QUIRK_NO_DEEPEST_PS`: no entry targets NPSS; quirks of other
/// paths leave the table as it was.
#[test]
fn apst_never_targets_the_deepest_state_under_its_quirk() {
    // Non-operational PS3 to PS5, exit latencies 2, 10 and 22 ms.
    let identify = apst_identify(&[(3, 0, 2_000), (4, 0, 10_000), (5, 0, 22_000)]);
    let primary = |state: u64| state << 3 | APST_PRIMARY_TIMEOUT_MS << 8;
    let secondary = |state: u64| state << 3 | APST_SECONDARY_TIMEOUT_MS << 8;
    // PS4 is the primary target; NPSS the secondary one, from PS4 alone.
    let (plain, _) = apst_table(&identify, Quirks::NONE).unwrap();
    assert_eq!(
        plain[..6],
        [
            primary(4),
            primary(4),
            primary(4),
            primary(4),
            secondary(5),
            0
        ]
    );
    assert_eq!(
        apst_table(&identify, Quirks::NO_WRITE_ZEROES | Quirks::BOGUS_IDS),
        Some((plain, true))
    );
    let (table, enable) = apst_table(&identify, Quirks::NO_DEEPEST_POWER_STATE).unwrap();
    assert!(enable);
    assert!(table.iter().all(|entry| entry >> 3 & 0x1f != 5));
    assert_eq!(
        table[..6],
        [primary(4), primary(4), primary(4), primary(4), 0, 0]
    );
}

/// `NVME_QUIRK_NO_APST` sends an empty table with APSTE clear, so APST the
/// firmware or the drive left on is turned off.
#[test]
fn apst_is_turned_off_under_its_quirk() {
    let identify = apst_identify(&[(3, 0, 2_000), (4, 0, 10_000)]);
    assert_eq!(
        apst_table(&identify, Quirks::NO_APST | Quirks::NO_DEEPEST_POWER_STATE),
        Some(([0; 32], false))
    );
    let fixture = Fixture::new();
    let mut c = fixture.disk.controller.borrow_mut();
    let command = c.apst_command(&[0; 32], false).unwrap();
    assert_eq!((command[10], command[11]), (FEATURE_APST, 0));
    assert!(c.admin_data.bytes()[..256].iter().all(|byte| *byte == 0));
}

/// A signalled controller's I/O completion queue raises vector 0, and
/// a posted completion shows in memory before the namespace takes it.
#[test]
fn completions_are_signalled_and_posted_in_memory() {
    let mut fixture = Fixture::new();
    let disk = &mut fixture.disk;
    assert_eq!(disk.controller.borrow().create_queue(false).unwrap()[11], 1);
    disk.controller.borrow_mut().signalled = true;
    assert!(disk.signals());
    assert_eq!(
        disk.controller.borrow().create_queue(false).unwrap()[11],
        0b11
    );
    assert!(!disk.posted(), "nothing runs");
    let read = disk.start(Command::Read { lba: 0, bytes: 512 }).unwrap();
    assert!(!disk.posted());
    {
        let mut controller = disk.controller.borrow_mut();
        let id = running_id(&controller, read);
        post(&mut controller, id, 0);
    }
    assert!(disk.posted());
    assert!(disk.finish(read).is_some());
    assert!(!disk.posted());
}

#[test]
fn a_removed_controller_fails_its_commands_without_a_reset() {
    let mut fixture = Fixture::new();
    let disk = &mut fixture.disk;
    let read = disk.start(Command::Read { lba: 0, bytes: 512 }).unwrap();
    disk.controller
        .borrow()
        .registers
        .write32(STATUS, REMOVED)
        .unwrap();
    disk.controller.borrow_mut().reset();
    assert_eq!(disk.finish(read), Some(Err(removed())));
    assert!(disk.controller.borrow().recovery.is_none());
    assert_eq!(disk.read(0, &mut [0; 512]), Err(removed()));
}

#[test]
fn only_a_controller_with_a_volatile_write_cache_is_sent_flushes() {
    let mut fixture = Fixture::new();
    fixture.disk.identity.features.write_cache = false;
    assert_eq!(fixture.disk.flush(), Ok(()));
    assert_eq!(fixture.disk.controller.borrow().io.tail, 0);
    // With a cache the flush is submitted; this fixture never completes
    // it, nor its abort.
    fixture.disk.identity.features.write_cache = true;
    assert_eq!(fixture.disk.flush(), Err(BlockError::Timeout));
    let controller = fixture.disk.controller.borrow();
    assert_eq!(controller.io.submissions.bytes()[0], IO_FLUSH);
    assert_eq!(controller.admin.submissions.bytes()[0], ADMIN_ABORT);
}

#[test]
fn identification_descriptors_parse_like_nvme_identify_ns_descs() {
    let mut page = vec![0u8; PAGE];
    let mut at = 0;
    for (kind, value) in [
        (DESCRIPTOR_EUI64, &[1u8; 8][..]),
        (9, &[7; 5][..]),
        (DESCRIPTOR_UUID, &[3; 16][..]),
        (DESCRIPTOR_CSI, &[2][..]),
    ] {
        page[at] = kind;
        page[at + 1] = value.len() as u8;
        page[at + 4..at + 4 + value.len()].copy_from_slice(value);
        at += 4 + value.len();
    }
    let ids = Ids::from_descriptors(&page);
    assert_eq!(
        ids,
        Ids {
            eui64: [1; 8],
            nguid: [0; 16],
            uuid: [3; 16],
            csi: 2
        }
    );
    // A wrong length ends the list; Identify Namespace then fills EUI64
    // from 1.1 and NGUID from 1.2.
    page[1] = 9;
    let mut ids = Ids::from_descriptors(&page);
    assert_eq!(ids, Ids::default());
    let mut data = vec![0u8; PAGE];
    data[NAMESPACE_EUI64].fill(5);
    data[NAMESPACE_NGUID].fill(6);
    ids.fill(VERSION_1_1, &data);
    assert_eq!((ids.eui64, ids.nguid), ([5; 8], [0; 16]));
    ids.fill(VERSION_1_2, &data);
    assert_eq!(ids.nguid, [6; 16]);
}

/// An active namespace list is increasing and within NN; `NVME_QUIRK_BOGUS_NID`
/// keeps only the descriptors' command set.
#[test]
fn active_lists_must_increase_within_nn_and_bogus_ids_keep_the_command_set() {
    assert!(active_list_valid(&[], 0));
    assert!(active_list_valid(&[1, 2, 7], 7));
    assert!(!active_list_valid(&[1, 1], 7));
    assert!(!active_list_valid(&[3, 2], 7));
    assert!(!active_list_valid(&[1, 8], 7));
    let ids = Ids {
        eui64: [1; 8],
        nguid: [2; 16],
        uuid: [3; 16],
        csi: CSI_ZONED,
    };
    assert_eq!(
        ids.without_identifiers(),
        Ids {
            csi: CSI_ZONED,
            ..Ids::default()
        }
    );
}

#[test]
fn a_controller_shuts_down_once_and_then_refuses_commands() {
    let fixture = Fixture::new();
    let mut controller = fixture.disk.controller.borrow_mut();
    controller
        .registers
        .write32(CONFIGURATION, CONFIGURATION_VALUE)
        .unwrap();
    controller
        .registers
        .write32(STATUS, READY | SHUTDOWN_COMPLETE)
        .unwrap();
    // The I/O submission queue's deletion is refused (Invalid Queue
    // Identifier); its completion queue's succeeds.
    let completion = |c: &mut Controller, slot: usize, id: u32, status: u32| {
        c.admin.completions.write(16 * slot + 8, id);
        c.admin
            .completions
            .write(16 * slot + 12, status << 17 | 1 << 16 | id);
    };
    completion(&mut controller, 0, 1, 1);
    completion(&mut controller, 1, 2, 0);
    controller.shutdown().unwrap();
    let submitted = controller.admin.submissions.bytes();
    for (slot, opcode) in [(0, ADMIN_DELETE_SUBMISSION), (1, ADMIN_DELETE_COMPLETION)] {
        let entry = &submitted[64 * slot..64 * slot + 64];
        assert_eq!(entry[0], opcode);
        assert_eq!(u32::from_le_bytes(entry[40..44].try_into().unwrap()), 1);
    }
    assert_eq!(
        controller.registers.read32(CONFIGURATION).unwrap(),
        CONFIGURATION_VALUE | SHUTDOWN_NORMAL
    );
    controller.registers.write32(CONFIGURATION, 0).unwrap();
    controller.shutdown().unwrap();
    assert_eq!(controller.registers.read32(CONFIGURATION).unwrap(), 0);
    assert_eq!(
        controller.ready(),
        Err(BlockError::Unsupported("NVMe controller is shut down"))
    );
}

#[test]
fn only_the_first_namespace_carries_the_bare_controller_serial() {
    assert_eq!(super::namespace_serial("S1", 1, Some(1)), "S1");
    assert_eq!(super::namespace_serial("S1", 2, Some(1)), "S1/n2");
    // A controller whose only active namespace is not the first ID.
    assert_eq!(super::namespace_serial("S1", 3, Some(3)), "S1");
}

/// A command's start and finish count in their parts: the start and its
/// doorbell, and the look at the completion queue with its status read and
/// completion doorbell. Other tests run commands too, so counts only grow.
#[test]
fn a_command_counts_its_start_and_finish_parts() {
    let mut fixture = Fixture::new();
    let disk = &mut fixture.disk;
    parts::take();
    let read = disk.start(Command::Read { lba: 0, bytes: 512 }).unwrap();
    {
        let mut controller = disk.controller.borrow_mut();
        let id = running_id(&controller, read);
        post(&mut controller, id, 0);
    }
    assert!(matches!(disk.finish(read), Some(Ok(Some(_)))));
    let counted = parts::take();
    let calls = |part: parts::Part| counted[part as usize][0];
    for part in [
        parts::Part::Start,
        parts::Part::Doorbell,
        parts::Part::Advance,
        parts::Part::Ring,
    ] {
        assert!(calls(part) >= 1, "{part:?}");
    }
}

/// A valid completion is taken with no status read, as Linux takes it, even
/// while the controller reports a fatal status; an invalid one reads the
/// status and fails the controller with it.
#[test]
fn completions_are_trusted_without_a_status_read_unless_they_look_invalid() {
    let mut fixture = Fixture::new();
    let disk = &mut fixture.disk;
    let read = disk.start(Command::Read { lba: 0, bytes: 512 }).unwrap();
    {
        let mut controller = disk.controller.borrow_mut();
        controller.registers.write32(STATUS, FATAL).unwrap();
        let id = running_id(&controller, read);
        post(&mut controller, id, 0);
    }
    assert!(matches!(disk.finish(read), Some(Ok(Some(_)))));
    let read = disk.start(Command::Read { lba: 0, bytes: 512 }).unwrap();
    {
        let mut controller = disk.controller.borrow_mut();
        let id = running_id(&controller, read);
        post(&mut controller, id.wrapping_add(7), 0);
    }
    let error = BlockError::Device { status: FATAL };
    assert_eq!(disk.finish(read), Some(Err(error.clone())));
    assert_eq!(disk.controller.borrow().fault, Some(error));
}

/// An invalid completion from a controller whose status is good poisons it
/// as a corrupt queue.
#[test]
fn an_invalid_completion_from_a_healthy_controller_is_a_corrupt_queue() {
    let mut fixture = Fixture::new();
    let disk = &mut fixture.disk;
    let read = disk.start(Command::Read { lba: 0, bytes: 512 }).unwrap();
    {
        let mut controller = disk.controller.borrow_mut();
        controller.registers.write32(STATUS, READY).unwrap();
        let id = running_id(&controller, read);
        post(&mut controller, id.wrapping_add(7), 0);
    }
    let corrupt = BlockError::Device { status: 0xffff };
    assert_eq!(disk.finish(read), Some(Err(corrupt)));
}

/// A dead controller posts nothing: its commands' timeout reads its status,
/// within the command deadline, and resets one that failed fatally and
/// fails the commands of one that reads all ones.
#[test]
fn a_dead_controller_is_found_at_the_command_timeout() {
    let _clock = crate::test_clock::virtualize();
    let wait = |disk: &mut Namespace| {
        let clock = disk.controller.borrow().clock;
        let started = clock.try_start().unwrap();
        while clock.try_elapsed_us(started).unwrap() < 1000 {
            core::hint::spin_loop();
        }
    };
    let mut fixture = Fixture::new();
    let disk = &mut fixture.disk;
    let read = disk.start(Command::Read { lba: 0, bytes: 512 }).unwrap();
    disk.controller
        .borrow()
        .registers
        .write32(STATUS, FATAL)
        .unwrap();
    assert_eq!(disk.finish(read), None);
    wait(disk);
    assert_eq!(disk.finish(read), None);
    {
        let controller = disk.controller.borrow();
        assert!(
            controller.aborts.is_empty(),
            "no abort to a failed controller"
        );
        assert!(controller.recovery.is_some(), "reset at the first deadline");
    }
    let mut fixture = Fixture::new();
    let disk = &mut fixture.disk;
    let read = disk.start(Command::Read { lba: 0, bytes: 512 }).unwrap();
    disk.controller
        .borrow()
        .registers
        .write32(STATUS, REMOVED)
        .unwrap();
    assert_eq!(disk.finish(read), None);
    wait(disk);
    assert_eq!(disk.finish(read), Some(Err(removed())));
}

/// Commands started with more to follow share one submission doorbell,
/// rung at the commit with the tail past the last; a look at the queue
/// rings one a caller left owed, and a plain start rings at once.
#[test]
fn a_batch_of_starts_rings_the_submission_doorbell_once() {
    let mut fixture = Fixture::new();
    let disk = &mut fixture.disk;
    let doorbell = |disk: &Namespace| {
        let controller = disk.controller.borrow();
        let at = DOORBELLS + 2 * usize::from(controller.io.id) * controller.doorbell_stride;
        controller.registers.read32(at).unwrap()
    };
    let first = disk
        .start_more(Command::Read { lba: 0, bytes: 512 })
        .unwrap();
    let second = disk
        .start_more(Command::Read { lba: 8, bytes: 512 })
        .unwrap();
    assert_eq!(doorbell(disk), 0, "owed, not rung");
    disk.commit();
    assert_eq!(doorbell(disk), 2, "one ring past both");
    disk.commit();
    assert_eq!(doorbell(disk), 2);
    for tag in [first, second] {
        {
            let mut controller = disk.controller.borrow_mut();
            let id = running_id(&controller, tag);
            post(&mut controller, id, 0);
        }
        assert!(matches!(disk.finish(tag), Some(Ok(Some(_)))));
    }
    // Left owed: the next look at the queue rings it.
    let third = disk
        .start_more(Command::Read { lba: 0, bytes: 512 })
        .unwrap();
    assert_eq!(doorbell(disk), 2);
    assert_eq!(disk.finish(third), None);
    assert_eq!(doorbell(disk), 3);
    // A plain start rings at once.
    let fourth = disk.start(Command::Read { lba: 0, bytes: 512 }).unwrap();
    assert_eq!(doorbell(disk), 4);
    let _ = fourth;
}
