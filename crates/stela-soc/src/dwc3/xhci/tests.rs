use super::*;
use {
    crate::dart::Dart, stela_contracts::bus::Access, stela_contracts::bus::MmioDevice,
    stela_contracts::bus::Width, stela_contracts::interrupt::InterruptLatch,
};
use {stela_contracts::dma::Memory, stela_contracts::dma::test_support::Ram};

const STOPPED_CODE: u32 = 26;

const RAM: u64 = 0x1_0000_0000;
const ERST: u64 = RAM;
const EVENTS: u64 = RAM + 0x1000;
const COMMANDS: u64 = RAM + 0x2000;

fn rd(x: &mut Xhci, at: u64) -> u32 {
    x.read(at, 4, 0).unwrap() as u32
}
fn wr(x: &mut Xhci, at: u64, value: u32) {
    x.write(at, 4, u64::from(value), 0).unwrap();
}
fn service(x: &mut Xhci, ram: &mut Ram) {
    service_at(x, ram, 0);
}
fn service_at(x: &mut Xhci, ram: &mut Ram, now: u64) {
    service_with(x, ram, now, None);
}
fn service_with(x: &mut Xhci, ram: &mut Ram, now: u64, function: Option<&mut (dyn Function + '_)>) {
    let mut dart = Dart::new(16, None);
    let mut latch = InterruptLatch::new();
    let mut access = Access::new(0, now, &mut latch);
    // Stream 0 bypasses translation and the filter: IOVAs are physical.
    dart.write(0xc00, Width::Word, 1, &mut access).unwrap();
    dart.write(0x1000, Width::Word, 6, &mut access).unwrap();
    let mut port = Port {
        iommu: &mut dart,
        stream: 0,
        memory: ram,
        access: &mut access,
    };
    x.service(&mut port, function).unwrap();
    assert!(x.live_endpoints_match());
}
fn trb(ram: &mut Ram, at: u64, words: [u32; 4]) {
    ram.put_u64(at, u64::from(words[0]) | u64::from(words[1]) << 32);
    ram.put_u64(at + 8, u64::from(words[2]) | u64::from(words[3]) << 32);
}
fn event(ram: &mut Ram, n: u64) -> [u32; 4] {
    let mut bytes = [0u8; 16];
    assert!(ram.read(EVENTS + n * TRB_BYTES, &mut bytes));
    core::array::from_fn(|w| u32::from_le_bytes(bytes[w * 4..w * 4 + 4].try_into().unwrap()))
}

#[test]
fn capabilities_describe_a_two_port_root_hub_and_its_protocols() {
    let mut x = Xhci::new();
    assert_eq!(x.read(0, 1, 0), Ok(0x20));
    assert_eq!(x.read(2, 2, 0), Ok(0x0110));
    assert_eq!(rd(&mut x, 4), 64 | 4 << 8 | 2 << 24);
    let ecp = u64::from(rd(&mut x, 0x10) >> 16) * 4;
    assert_eq!(rd(&mut x, ecp) & 0xff, 2);
    assert_eq!(rd(&mut x, ecp + 4), u32::from_le_bytes(*b"USB "));
    assert_eq!(rd(&mut x, ecp + 8) & 0xffff, 0x0101);
    let next = ecp + u64::from(rd(&mut x, ecp) >> 8 & 0xff) * 4;
    assert_eq!(rd(&mut x, next) >> 24, 3);
    assert_eq!(rd(&mut x, next + 8) & 0xffff, 0x0102);
    assert!(x.write(0, 4, 0, 0).is_err());
    assert!(x.read(1, 4, 0).is_err());
}

#[test]
fn held_events_wait_per_interrupter_and_overflow_reports_event_ring_full() {
    const RING1: u64 = RAM + 0x3000;
    let mut ram = Ram::new(RAM, 0x4000);
    let mut x = Xhci::new();
    wr(&mut x, PORT_BASE, PP);
    wr(&mut x, OPERATIONAL, RUN);
    for _ in 0..MAX_PENDING_EVENTS + 4 {
        x.post(0, Xhci::port_status_change(0));
    }
    assert_eq!(x.pending.len(), MAX_PENDING_EVENTS);
    assert_eq!(x.interrupters[0].lost, 4);
    // A register write that posts an event lands though its event is lost.
    x.attach(Some(Usb2Speed::High));
    assert_eq!(rd(&mut x, PORT_BASE) & (CCS | CSC), CCS | CSC);
    assert_eq!(x.interrupters[0].lost, 5);
    // Interrupter 0 has no ring; interrupter 1's event is not held behind it.
    let i1 = INTERRUPTER_BASE + INTERRUPTER_STRIDE;
    ram.put_u64(ERST + 0x40, RING1);
    ram.put_u64(ERST + 0x48, 16);
    wr(&mut x, i1 + 8, 1);
    x.write(i1 + 0x18, 8, RING1, 0).unwrap();
    x.write(i1 + 0x10, 8, ERST + 0x40, 0).unwrap();
    x.queue(Event {
        interrupter: 1,
        trb: Xhci::port_status_change(0),
        interrupt: true,
    });
    service(&mut x, &mut ram);
    let mut first = [0u8; 16];
    assert!(ram.read(RING1, &mut first));
    assert_eq!(u32::from(first[12]) & CYCLE, 1);
    assert_eq!(x.pending.len(), MAX_PENDING_EVENTS);
    // Once interrupter 0 has a ring, its held events and then the error.
    let i0 = INTERRUPTER_BASE;
    ram.put_u64(ERST, EVENTS);
    ram.put_u64(ERST + 8, 512);
    wr(&mut x, i0 + 8, 1);
    x.write(i0 + 0x18, 8, EVENTS, 0).unwrap();
    x.write(i0 + 0x10, 8, ERST, 0).unwrap();
    service(&mut x, &mut ram);
    assert!(x.pending.is_empty());
    let error = event(&mut ram, MAX_PENDING_EVENTS as u64);
    assert_eq!(error[2] >> 24, EVENT_RING_FULL);
    assert_eq!(error[3] >> 10 & 0x3f, HOST_CONTROLLER);
    assert_eq!(x.interrupters[0].lost, 0);
    let trace: alloc::vec::Vec<String> = core::iter::from_fn(|| x.take_trace()).collect();
    assert!(
        trace
            .iter()
            .any(|l| l.starts_with("interrupter 0 loses events"))
    );
    assert!(
        trace
            .iter()
            .any(|l| l.contains("Event Ring Full Error: 5 events lost"))
    );
}

#[test]
fn run_halt_reset_and_mfindex_follow_usbcmd() {
    let mut x = Xhci::new();
    let sts = OPERATIONAL + 4;
    assert_eq!(rd(&mut x, sts), HCH);
    x.write(OPERATIONAL, 4, u64::from(RUN | INTE), 3000)
        .unwrap();
    assert_eq!(rd(&mut x, sts) & HCH, 0);
    assert_eq!(x.read(RUNTIME, 4, 3000 * 11), Ok(10));
    assert!(x.write(OPERATIONAL, 4, u64::from(HCRST), 0).is_err());
    x.write(OPERATIONAL, 4, 0, 3000 * 21).unwrap();
    assert_eq!(x.read(RUNTIME, 4, 1 << 40), Ok(20));
    wr(&mut x, OPERATIONAL + 0x38, 8);
    wr(&mut x, OPERATIONAL, CSS);
    wr(&mut x, OPERATIONAL, CRS);
    assert_eq!(rd(&mut x, sts) & SRE, 0);
    wr(&mut x, OPERATIONAL, HCRST);
    assert_eq!(rd(&mut x, OPERATIONAL + 0x38), 0);
    assert_eq!(rd(&mut x, OPERATIONAL), 0);
    assert!(x.write(OPERATIONAL, 4, u64::from(LHCRST), 0).is_err());
    assert!(x.write(OPERATIONAL + 0x38, 4, 65, 0).is_err());
}

#[test]
fn ports_power_and_stay_disconnected() {
    let mut x = Xhci::new();
    let (usb2, usb3) = (PORT_BASE, PORT_BASE + PORT_STRIDE);
    assert_eq!(rd(&mut x, usb2), DISABLED << PLS_SHIFT);
    wr(&mut x, usb2, PP | PR);
    wr(&mut x, usb3, PP | WAKE);
    assert_eq!(rd(&mut x, usb2), PP | RX_DETECT << PLS_SHIFT);
    assert_eq!(rd(&mut x, usb3), PP | WAKE | RX_DETECT << PLS_SHIFT);
    wr(&mut x, usb3, PP | LWS | DISABLED << PLS_SHIFT);
    assert_eq!(rd(&mut x, usb3) & PLS, DISABLED << PLS_SHIFT);
    wr(&mut x, usb2, 0);
    assert_eq!(rd(&mut x, usb2), DISABLED << PLS_SHIFT);
    assert!(x.write(usb2, 4, u64::from(PP | 1 << 14), 0).is_err());
    assert!(x.write(usb2 + 4, 4, 1 << 28, 0).is_err());
    // WPR is RsvdZ on the USB2 port; halted, a change posts no event.
    wr(&mut x, usb2, PP | WPR);
    assert_eq!(rd(&mut x, usb2), PP | RX_DETECT << PLS_SHIFT);
    wr(&mut x, usb3, PP | WPR);
    assert_eq!(rd(&mut x, usb3), PP | PRC | WRC | RX_DETECT << PLS_SHIFT);
    assert_ne!(rd(&mut x, OPERATIONAL + 4) & PCD, 0);
    assert!(x.pending.is_empty());
    // The driver's write-back keeps WPR clear and clears the changes.
    wr(&mut x, usb3, PP | PRC | WRC);
    assert_eq!(rd(&mut x, usb3), PP | RX_DETECT << PLS_SHIFT);
}

#[test]
fn a_running_controller_posts_port_changes_and_mfindex_wraps() {
    let mut ram = Ram::new(RAM, 0x4000);
    let mut x = Xhci::new();
    ram.put_u64(ERST, EVENTS);
    ram.put_u64(ERST + 8, 16);
    let i0 = INTERRUPTER_BASE;
    wr(&mut x, i0 + 8, 1);
    x.write(i0 + 0x18, 8, EVENTS, 0).unwrap();
    x.write(i0 + 0x10, 8, ERST, 0).unwrap();
    wr(&mut x, i0, IE);
    let period = MFINDEX_PERIOD * MICROFRAME;
    x.write(OPERATIONAL, 4, u64::from(RUN | INTE | EWE), 100)
        .unwrap();
    assert_eq!(x.deadline(None), Some(100 + period));
    wr(&mut x, PORT_BASE + PORT_STRIDE, PP | WPR);
    service_at(&mut x, &mut ram, period);
    assert_eq!(event(&mut ram, 0)[0] >> 24, 2);
    assert_eq!(event(&mut ram, 0)[3] >> 10 & 0x3f, PORT_STATUS_CHANGE);
    assert_eq!(event(&mut ram, 1), [0; 4]);
    service_at(&mut x, &mut ram, 100 + period);
    assert_eq!(event(&mut ram, 1)[2] >> 24, SUCCESS);
    assert_eq!(event(&mut ram, 1)[3] >> 10 & 0x3f, MFINDEX_WRAP);
    assert_eq!(x.read(RUNTIME, 4, 100 + period), Ok(0));
    assert_eq!(x.deadline(None), Some(100 + 2 * period));
    assert!(x.interrupt(0));
    // Halting stops the count and the wraps.
    x.write(OPERATIONAL, 4, u64::from(EWE), 100 + period + 3000)
        .unwrap();
    assert_eq!(x.deadline(None), None);
    assert_eq!(x.read(RUNTIME, 4, u64::MAX), Ok(1));
}

#[test]
fn a_split_erstba_write_loads_the_segment_table_with_its_high_dword() {
    // RAM sits above 4 GiB, so the low dword alone names no segment table.
    let mut ram = Ram::new(RAM, 0x4000);
    let mut x = Xhci::new();
    ram.put_u64(ERST, EVENTS);
    ram.put_u64(ERST + 8, 16);
    let i0 = INTERRUPTER_BASE;
    wr(&mut x, i0 + 8, 1);
    x.write(i0 + 0x18, 8, EVENTS, 0).unwrap();
    wr(&mut x, i0 + 0x10, ERST as u32);
    service(&mut x, &mut ram);
    wr(&mut x, i0 + 0x14, (ERST >> 32) as u32);
    service(&mut x, &mut ram);
    assert_eq!(
        u64::from(rd(&mut x, i0 + 0x10)) | u64::from(rd(&mut x, i0 + 0x14)) << 32,
        ERST
    );
}

#[test]
fn commands_complete_through_the_event_ring_and_raise_the_interrupter() {
    let mut ram = Ram::new(RAM, 0x4000);
    let mut x = Xhci::new();
    ram.put_u64(ERST, EVENTS);
    ram.put_u64(ERST + 8, 16);
    let i0 = INTERRUPTER_BASE;
    wr(&mut x, i0 + 8, 1);
    x.write(i0 + 0x18, 8, EVENTS, 0).unwrap();
    x.write(i0 + 0x10, 8, ERST, 0).unwrap();
    wr(&mut x, i0, IE);
    wr(&mut x, OPERATIONAL + 0x38, 2);
    x.write(OPERATIONAL + 0x18, 8, COMMANDS | RCS, 0).unwrap();
    wr(&mut x, OPERATIONAL, RUN | INTE);
    let cmd = |kind: u32, slot: u32| [0, 0, 0, kind << 10 | slot << 24 | CYCLE];
    trb(&mut ram, COMMANDS, cmd(NO_OP, 0));
    trb(&mut ram, COMMANDS + 16, cmd(ENABLE_SLOT, 0));
    // An input context adding the slot and EP0 on port 1, which has no
    // device.
    let input = RAM + 0x3000;
    ram.put_u64(input, 3 << 32);
    ram.put_u64(input + 0x20 + 4, 1 << 16);
    ram.put_u64(input + 0x40 + 4, u64::from(4u32 << 3 | 64 << 16));
    trb(
        &mut ram,
        COMMANDS + 32,
        [
            input as u32,
            (input >> 32) as u32,
            0,
            ADDRESS_DEVICE << 10 | 1 << 24 | CYCLE,
        ],
    );
    trb(&mut ram, COMMANDS + 48, cmd(12, 1));
    trb(&mut ram, COMMANDS + 64, cmd(DISABLE_SLOT, 1));
    trb(&mut ram, COMMANDS + 80, cmd(DISABLE_SLOT, 1));
    // A Link back to the start toggles the cycle, ending the ring.
    trb(
        &mut ram,
        COMMANDS + 96,
        [
            COMMANDS as u32,
            (COMMANDS >> 32) as u32,
            0,
            LINK << 10 | TOGGLE | CYCLE,
        ],
    );
    wr(&mut x, DOORBELLS, 0);
    assert!(x.pending());
    assert!(!x.interrupt(0));
    service(&mut x, &mut ram);
    let codes: alloc::vec::Vec<_> = (0..6)
        .map(|n| (event(&mut ram, n)[2] >> 24, event(&mut ram, n)[3] >> 24))
        .collect();
    assert_eq!(
        codes,
        [
            (SUCCESS, 0),
            (SUCCESS, 1),
            (TRANSACTION_ERROR, 1),
            (CONTEXT_STATE_ERROR, 1),
            (SUCCESS, 1),
            (SLOT_NOT_ENABLED, 1),
        ]
    );
    assert_eq!(event(&mut ram, 0)[0], COMMANDS as u32);
    assert_eq!(event(&mut ram, 0)[3] >> 10 & 0x3f, COMMAND_COMPLETION);
    assert_eq!(event(&mut ram, 0)[3] & CYCLE, 1);
    assert!(x.interrupt(0));
    assert_ne!(rd(&mut x, OPERATIONAL + 4) & EINT, 0);
    assert_ne!(rd(&mut x, i0 + 0x18) & EHB as u32, 0);
    assert_eq!(rd(&mut x, OPERATIONAL + 0x18), CRR as u32);
    // Acknowledge, then stop the ring: a Command Ring Stopped event.
    wr(&mut x, i0, IP | IE);
    x.write(i0 + 0x18, 8, (EVENTS + 6 * TRB_BYTES) | EHB, 0)
        .unwrap();
    assert!(!x.interrupt(0));
    wr(&mut x, OPERATIONAL + 0x18, CS as u32);
    service(&mut x, &mut ram);
    assert_eq!(event(&mut ram, 6)[2] >> 24, COMMAND_RING_STOPPED);
    assert_eq!(rd(&mut x, OPERATIONAL + 0x18), 0);
    assert!(x.write(DOORBELLS + 4, 4, 0, 0).is_err());
}

#[test]
fn a_full_event_ring_holds_events_until_software_dequeues() {
    let mut ram = Ram::new(RAM, 0x4000);
    let mut x = Xhci::new();
    ram.put_u64(ERST, EVENTS);
    ram.put_u64(ERST + 8, 16);
    let i0 = INTERRUPTER_BASE;
    wr(&mut x, i0 + 8, 1);
    x.write(i0 + 0x18, 8, EVENTS, 0).unwrap();
    x.write(i0 + 0x10, 8, ERST, 0).unwrap();
    x.write(OPERATIONAL + 0x18, 8, COMMANDS | RCS, 0).unwrap();
    wr(&mut x, OPERATIONAL, RUN);
    for n in 0..20 {
        trb(&mut ram, COMMANDS + n * 16, [0, 0, 0, NO_OP << 10 | CYCLE]);
    }
    wr(&mut x, DOORBELLS, 0);
    service(&mut x, &mut ram);
    // 15 of a 16-TRB segment; the rest wait.
    assert_eq!(x.pending.len(), 5);
    x.write(i0 + 0x18, 8, EVENTS + 15 * TRB_BYTES, 0).unwrap();
    service(&mut x, &mut ram);
    assert_eq!(x.pending.len(), 0);
    // Wrapping to the segment start flips the producer cycle.
    assert_eq!(event(&mut ram, 15)[3] & CYCLE, 1);
    assert_eq!(event(&mut ram, 0)[3] & CYCLE, 0);
}

/// A controller running with one 16-TRB event segment, eight slots,
/// a DCBAA and a command ring, its USB2 port powered.
struct Bench {
    x: Xhci,
    ram: Ram,
    hid: crate::usb::hid::Peripheral,
    now: u64,
    commands: u64,
    events: u64,
}

const DCBAA: u64 = RAM + 0x4000;
const OUTPUT: u64 = RAM + 0x5000;
const INPUT: u64 = RAM + 0x6000;
const EP0_RING: u64 = RAM + 0x7000;
const EP1_RING: u64 = RAM + 0x8000;
const BUFFER: u64 = RAM + 0x9000;
const PORTSC1: u64 = PORT_BASE;

impl Bench {
    fn new() -> Self {
        let mut ram = Ram::new(RAM, 0x10000);
        let mut x = Xhci::new();
        ram.put_u64(ERST, EVENTS);
        ram.put_u64(ERST + 8, 64);
        ram.put_u64(DCBAA + 8, OUTPUT);
        let i0 = INTERRUPTER_BASE;
        wr(&mut x, i0 + 8, 1);
        x.write(i0 + 0x18, 8, EVENTS, 0).unwrap();
        x.write(i0 + 0x10, 8, ERST, 0).unwrap();
        wr(&mut x, i0, IE);
        wr(&mut x, OPERATIONAL + 0x38, 8);
        x.write(OPERATIONAL + 0x30, 8, DCBAA, 0).unwrap();
        x.write(OPERATIONAL + 0x18, 8, COMMANDS | RCS, 0).unwrap();
        wr(&mut x, OPERATIONAL, RUN | INTE);
        wr(&mut x, PORTSC1, PP);
        Self {
            x,
            ram,
            hid: crate::usb::hid::Peripheral::new(),
            now: 0,
            commands: COMMANDS,
            events: 0,
        }
    }

    fn service(&mut self) {
        service_with(&mut self.x, &mut self.ram, self.now, Some(&mut self.hid));
    }

    /// The next event, which must exist.
    fn event(&mut self) -> [u32; 4] {
        let e = event(&mut self.ram, self.events);
        assert_eq!(e[3] & CYCLE, 1, "no event {}", self.events);
        self.events += 1;
        e
    }

    fn command(&mut self, words: [u32; 4]) -> [u32; 4] {
        trb(&mut self.ram, self.commands, words);
        self.commands += 16;
        wr(&mut self.x, DOORBELLS, 0);
        self.service();
        let e = self.event();
        assert_eq!(e[3] >> 10 & 0x3f, COMMAND_COMPLETION);
        e
    }

    fn context(&mut self, at: u64) -> [u32; 8] {
        let mut bytes = [0u8; 32];
        assert!(self.ram.read(at, &mut bytes));
        core::array::from_fn(|n| u32::from_le_bytes(bytes[n * 4..n * 4 + 4].try_into().unwrap()))
    }

    fn put_context(&mut self, at: u64, words: [u32; 8]) {
        for (n, pair) in words.chunks(2).enumerate() {
            self.ram.put_u64(
                at + 8 * n as u64,
                u64::from(pair[0]) | u64::from(pair[1]) << 32,
            );
        }
    }
}

const NORMAL_TRB: u32 = 1 << 10;
const SETUP_TRB: u32 = 2 << 10;
const DATA_TRB: u32 = 3 << 10;
const STATUS_TRB: u32 = 4 << 10;
const TRB_IOC: u32 = 1 << 5;
const TRB_IDT: u32 = 1 << 6;
const TRB_ISP: u32 = 1 << 2;
const TRB_IN: u32 = 1 << 16;
const TRANSFER_EVENT: u32 = 32;

fn setup_trb(packet: [u8; 8], transfer_type: u32) -> [u32; 4] {
    let p = u64::from_le_bytes(packet);
    [
        p as u32,
        (p >> 32) as u32,
        8,
        SETUP_TRB | TRB_IDT | transfer_type << 16 | CYCLE,
    ]
}

/// Attaches, resets and addresses the keyboard as slot 1, then runs
/// SET_CONFIGURATION and configures its keyboard endpoint (DCI 3).
fn enumerated() -> Bench {
    let mut b = Bench::new();
    b.x.attach(Some(Usb2Speed::Full));
    let portsc = rd(&mut b.x, PORTSC1);
    assert_eq!(portsc & (CCS | CSC | PED), CCS | CSC);
    assert_eq!(portsc & PLS, POLLING << PLS_SHIFT);
    b.service();
    assert_eq!(b.event()[3] >> 10 & 0x3f, PORT_STATUS_CHANGE);
    wr(&mut b.x, PORTSC1, PP | CSC | PR);
    let portsc = rd(&mut b.x, PORTSC1);
    assert_eq!(portsc & (CCS | PED | PRC | PR), CCS | PED | PRC);
    assert_eq!(portsc & SPEED, FULL_SPEED << SPEED_SHIFT);
    assert_eq!(portsc & PLS, U0 << PLS_SHIFT);
    b.service();
    assert_eq!(b.event()[0] >> 24, 1);
    wr(&mut b.x, PORTSC1, PP | PRC);
    let e = b.command([0, 0, 0, ENABLE_SLOT << 10 | CYCLE]);
    assert_eq!((e[2] >> 24, e[3] >> 24), (SUCCESS, 1));
    // Input context: add the slot and EP0; root port 1, one entry.
    b.put_context(INPUT, [0, 3, 0, 0, 0, 0, 0, 0]);
    b.put_context(INPUT + 0x20, [1 << 27 | 1 << 20, 1 << 16, 0, 0, 0, 0, 0, 0]);
    b.put_context(
        INPUT + 0x40,
        [
            0,
            3 << 1 | 4 << 3 | 64 << 16,
            EP0_RING as u32 | 1,
            (EP0_RING >> 32) as u32,
            8,
            0,
            0,
            0,
        ],
    );
    let input = INPUT as u32;
    let e = b.command([
        input,
        (INPUT >> 32) as u32,
        0,
        ADDRESS_DEVICE << 10 | 1 << 24 | CYCLE,
    ]);
    assert_eq!(e[2] >> 24, SUCCESS);
    let slot = b.context(OUTPUT);
    assert_eq!((slot[3] >> 27, slot[3] & 0xff), (2, 1));
    assert_eq!(b.context(OUTPUT + 0x20)[0] & 7, 1);
    // SET_CONFIGURATION 1: no data stage, status IN.
    trb(&mut b.ram, EP0_RING, setup_trb([0, 9, 1, 0, 0, 0, 0, 0], 0));
    trb(
        &mut b.ram,
        EP0_RING + 16,
        [0, 0, 0, STATUS_TRB | TRB_IN | TRB_IOC | CYCLE],
    );
    wr(&mut b.x, DOORBELLS + 4, 1);
    assert!(b.x.pending());
    b.service();
    let e = b.event();
    assert_eq!(e[3] >> 10 & 0x3f, TRANSFER_EVENT);
    assert_eq!(
        (e[0], e[2] >> 24, e[3] >> 16 & 0x1f),
        ((EP0_RING + 16) as u32, SUCCESS, 1)
    );
    // Configure the interrupt IN endpoint 0x81 (DCI 3), 1 ms interval.
    b.put_context(INPUT, [0, 1 | 1 << 3, 0, 0, 0, 0, 0, 0]);
    b.put_context(INPUT + 0x20, [3 << 27, 1 << 16, 0, 0, 0, 0, 0, 0]);
    b.put_context(
        INPUT + 0x20 * 4,
        [
            3 << 16,
            3 << 1 | 7 << 3 | 8 << 16,
            EP1_RING as u32 | 1,
            (EP1_RING >> 32) as u32,
            8,
            0,
            0,
            0,
        ],
    );
    let e = b.command([
        input,
        (INPUT >> 32) as u32,
        0,
        CONFIGURE_ENDPOINT << 10 | 1 << 24 | CYCLE,
    ]);
    assert_eq!(e[2] >> 24, SUCCESS);
    assert_eq!(b.context(OUTPUT)[3] >> 27, 3);
    assert_eq!(b.context(OUTPUT + 0x20 * 3)[0] & 7, 1);
    b
}

#[test]
fn an_attached_keyboard_enumerates_and_reports_keys_through_its_rings() {
    let mut b = enumerated();
    // GET_DESCRIPTOR(device) with a 64-byte IN data stage: short.
    trb(
        &mut b.ram,
        EP0_RING + 32,
        setup_trb([0x80, 6, 0, 1, 0, 0, 64, 0], 3),
    );
    trb(
        &mut b.ram,
        EP0_RING + 48,
        [
            BUFFER as u32,
            (BUFFER >> 32) as u32,
            64,
            DATA_TRB | TRB_IN | TRB_ISP | CYCLE,
        ],
    );
    trb(
        &mut b.ram,
        EP0_RING + 64,
        [0, 0, 0, STATUS_TRB | TRB_IOC | CYCLE],
    );
    wr(&mut b.x, DOORBELLS + 4, 1);
    b.service();
    let short = b.event();
    assert_eq!((short[0], short[2] >> 24), ((EP0_RING + 48) as u32, 13));
    assert_eq!(short[2] & 0xff_ffff, 64 - 18);
    assert_eq!(b.event()[2] >> 24, SUCCESS);
    let mut device = [0u8; 18];
    assert!(b.ram.read(BUFFER, &mut device));
    assert_eq!(device[..2], [18, 1]);
    assert_eq!(device[8..12], [0xac, 0x05, 0x04, 0x02]);
    // The keyboard endpoint NAKs until a key goes down, and is polled
    // again only then, at its next interval.
    trb(
        &mut b.ram,
        EP1_RING,
        [
            BUFFER as u32,
            (BUFFER >> 32) as u32,
            8,
            NORMAL_TRB | TRB_IOC | TRB_ISP | CYCLE,
        ],
    );
    wr(&mut b.x, DOORBELLS + 4, 3);
    b.now = 1000;
    b.service();
    assert_eq!(event(&mut b.ram, b.events)[3] & CYCLE, 0);
    assert!(!b.x.pending());
    // The keyboard reports changes alone, so nothing is due.
    assert_eq!(b.x.deadline(Some(&b.hid)), None);
    b.hid
        .input(stela_drivers::input::Input::Key {
            usage: 0x04,
            pressed: true,
        })
        .unwrap();
    assert_eq!(b.x.deadline(Some(&b.hid)), Some(1000 + 8 * MICROFRAME));
    b.now = 1000 + 8 * MICROFRAME;
    b.service();
    let e = b.event();
    assert_eq!(
        (e[0], e[2], e[3] >> 16 & 0x1f),
        (EP1_RING as u32, SUCCESS << 24, 3)
    );
    let mut report = [0u8; 8];
    assert!(b.ram.read(BUFFER, &mut report));
    assert_eq!(report, [0, 0, 4, 0, 0, 0, 0, 0]);
}

#[test]
fn a_stall_halts_ep0_until_reset_and_a_new_dequeue_pointer() {
    let mut b = enumerated();
    // GET_DESCRIPTOR(device qualifier): a full-speed device STALLs.
    trb(
        &mut b.ram,
        EP0_RING + 32,
        setup_trb([0x80, 6, 0, 6, 0, 0, 10, 0], 3),
    );
    trb(
        &mut b.ram,
        EP0_RING + 48,
        [
            BUFFER as u32,
            (BUFFER >> 32) as u32,
            10,
            DATA_TRB | TRB_IN | CYCLE,
        ],
    );
    trb(
        &mut b.ram,
        EP0_RING + 64,
        [0, 0, 0, STATUS_TRB | TRB_IOC | CYCLE],
    );
    wr(&mut b.x, DOORBELLS + 4, 1);
    b.service();
    let e = b.event();
    assert_eq!(
        (e[0], e[2] >> 24, e[2] & 0xff_ffff),
        ((EP0_RING + 48) as u32, 6, 10)
    );
    let ep0 = b.context(OUTPUT + 0x20);
    assert_eq!((ep0[0] & 7, ep0[2]), (2, (EP0_RING + 48) as u32 | 1));
    // A halted endpoint's doorbell does nothing; Set TR Dequeue needs
    // Stopped.
    wr(&mut b.x, DOORBELLS + 4, 1);
    assert!(!b.x.pending());
    let next = (EP0_RING + 80) as u32 | 1;
    let e = b.command([
        next,
        (EP0_RING >> 32) as u32,
        0,
        SET_TR_DEQUEUE << 10 | 1 << 16 | 1 << 24 | CYCLE,
    ]);
    assert_eq!(e[2] >> 24, CONTEXT_STATE_ERROR);
    let e = b.command([0, 0, 0, RESET_ENDPOINT << 10 | 1 << 16 | 1 << 24 | CYCLE]);
    assert_eq!(e[2] >> 24, SUCCESS);
    let e = b.command([
        next,
        (EP0_RING >> 32) as u32,
        0,
        SET_TR_DEQUEUE << 10 | 1 << 16 | 1 << 24 | CYCLE,
    ]);
    assert_eq!(e[2] >> 24, SUCCESS);
    assert_eq!(b.context(OUTPUT + 0x20)[0] & 7, 3);
    // The next request runs from the new dequeue pointer.
    trb(
        &mut b.ram,
        EP0_RING + 80,
        setup_trb([0x80, 8, 0, 0, 0, 0, 1, 0], 3),
    );
    trb(
        &mut b.ram,
        EP0_RING + 96,
        [
            BUFFER as u32,
            (BUFFER >> 32) as u32,
            1,
            DATA_TRB | TRB_IN | CYCLE,
        ],
    );
    trb(
        &mut b.ram,
        EP0_RING + 112,
        [0, 0, 0, STATUS_TRB | TRB_IOC | CYCLE],
    );
    wr(&mut b.x, DOORBELLS + 4, 1);
    b.service();
    assert_eq!(b.event()[2] >> 24, SUCCESS);
    let mut configuration = [0u8; 1];
    assert!(b.ram.read(BUFFER, &mut configuration));
    assert_eq!(configuration, [1]);
}

#[test]
fn stopping_a_polled_endpoint_reports_its_started_td_first() {
    let mut b = enumerated();
    trb(
        &mut b.ram,
        EP1_RING,
        [
            BUFFER as u32,
            (BUFFER >> 32) as u32,
            8,
            NORMAL_TRB | TRB_IOC | CYCLE,
        ],
    );
    wr(&mut b.x, DOORBELLS + 4, 3);
    b.service();
    trb(
        &mut b.ram,
        b.commands,
        [0, 0, 0, STOP_ENDPOINT << 10 | 3 << 16 | 1 << 24 | CYCLE],
    );
    b.commands += 16;
    wr(&mut b.x, DOORBELLS, 0);
    b.service();
    // The Stopped event precedes the command's completion.
    let e = b.event();
    assert_eq!(
        (e[0], e[2] >> 24, e[2] & 0xff_ffff),
        (EP1_RING as u32, STOPPED_CODE, 8)
    );
    let stopped = b.event();
    assert_eq!(
        (stopped[2] >> 24, stopped[3] >> 10 & 0x3f),
        (SUCCESS, COMMAND_COMPLETION)
    );
    let ep = b.context(OUTPUT + 0x20 * 3);
    assert_eq!((ep[0] & 7, ep[2]), (3, EP1_RING as u32 | 1));
    let again = b.command([0, 0, 0, STOP_ENDPOINT << 10 | 3 << 16 | 1 << 24 | CYCLE]);
    assert_eq!(again[2] >> 24, CONTEXT_STATE_ERROR);
    // Disconnection clears the port; Disable Slot frees the slot.
    b.x.attach(None);
    assert_eq!(rd(&mut b.x, PORTSC1) & (CCS | PED | CSC), CSC);
    b.service();
    assert_eq!(b.event()[3] >> 10 & 0x3f, PORT_STATUS_CHANGE);
    let e = b.command([0, 0, 0, DISABLE_SLOT << 10 | 1 << 24 | CYCLE]);
    assert_eq!(e[2] >> 24, SUCCESS);
}

/// macOS reads the output context to choose Stop Endpoint: one left saying
/// Stopped sends it straight to Set TR Dequeue Pointer, which fails.
#[test]
fn a_doorbell_shows_a_restarted_endpoint_as_running_in_its_output_context() {
    let mut b = enumerated();
    let stop = b.command([0, 0, 0, STOP_ENDPOINT << 10 | 3 << 16 | 1 << 24 | CYCLE]);
    assert_eq!(stop[2] >> 24, SUCCESS);
    assert_eq!(b.context(OUTPUT + 0x20 * 3)[0] & 7, 3);
    wr(&mut b.x, DOORBELLS + 4, 3);
    b.service();
    assert_eq!(b.context(OUTPUT + 0x20 * 3)[0] & 7, 1);
    let dequeue = b.command([
        EP1_RING as u32 | 1,
        (EP1_RING >> 32) as u32,
        0,
        SET_TR_DEQUEUE << 10 | 3 << 16 | 1 << 24 | CYCLE,
    ]);
    assert_eq!(dequeue[2] >> 24, CONTEXT_STATE_ERROR);
    let stop = b.command([0, 0, 0, STOP_ENDPOINT << 10 | 3 << 16 | 1 << 24 | CYCLE]);
    assert_eq!(stop[2] >> 24, SUCCESS);
}

#[test]
fn route_strings_reach_functions_behind_an_enabled_hub_port() {
    use crate::usb::{
        Speed,
        control::tests::{setup, transfer},
        hid::Peripheral,
        hub::{Hub, Ports},
    };
    struct One(Peripheral);
    impl Ports for One {
        fn count(&self) -> usize {
            1
        }
        fn speed(&self, _: usize) -> Speed {
            Speed::Full
        }
        fn function(&mut self, _: usize) -> &mut dyn Function {
            &mut self.0
        }
        fn function_ref(&self, _: usize) -> &dyn Function {
            &self.0
        }
    }
    let mut hub = Hub::new(One(Peripheral::new()), "HUB").unwrap();
    let mut b = Bench::new();
    b.x.attach(Some(Usb2Speed::High));
    service_with(&mut b.x, &mut b.ram, 0, Some(&mut hub));
    assert_eq!(b.event()[3] >> 10 & 0x3f, PORT_STATUS_CHANGE);
    wr(&mut b.x, PORTSC1, PP | CSC | PR);
    assert_eq!(rd(&mut b.x, PORTSC1) & SPEED, HIGH_SPEED << SPEED_SHIFT);
    service_with(&mut b.x, &mut b.ram, 0, Some(&mut hub));
    assert_eq!(b.event()[3] >> 10 & 0x3f, PORT_STATUS_CHANGE);
    // The hub as its driver leaves it: configured, port 1 powered and
    // reset, so the keyboard behind it answers at address 0.
    for request in [
        setup(0, 5, 1, 0, 0),
        setup(0, 9, 1, 0, 0),
        setup(0x23, 3, 8, 1, 0),
        setup(0x23, 3, 4, 1, 0),
    ] {
        transfer(&mut hub, request, &[]).unwrap();
    }
    let command = |b: &mut Bench, hub: &mut Hub<One>, words: [u32; 4]| {
        trb(&mut b.ram, b.commands, words);
        b.commands += 16;
        wr(&mut b.x, DOORBELLS, 0);
        service_with(&mut b.x, &mut b.ram, 0, Some(hub));
        b.event()
    };
    let address = |b: &mut Bench, hub: &mut Hub<One>, slot: u32, route: u32| {
        b.ram.put_u64(
            DCBAA + 8 * u64::from(slot),
            OUTPUT + 0x400 * u64::from(slot),
        );
        b.put_context(INPUT, [0, 3, 0, 0, 0, 0, 0, 0]);
        b.put_context(
            INPUT + 0x20,
            [1 << 27 | 1 << 20 | route, 1 << 16, 0, 0, 0, 0, 0, 0],
        );
        b.put_context(
            INPUT + 0x40,
            [
                0,
                3 << 1 | 4 << 3 | 64 << 16,
                EP0_RING as u32 | 1,
                (EP0_RING >> 32) as u32,
                8,
                0,
                0,
                0,
            ],
        );
        let e = command(b, hub, [0, 0, 0, ENABLE_SLOT << 10 | CYCLE]);
        assert_eq!((e[2] >> 24, e[3] >> 24), (SUCCESS, slot));
        let e = command(
            b,
            hub,
            [
                INPUT as u32,
                (INPUT >> 32) as u32,
                0,
                ADDRESS_DEVICE << 10 | slot << 24 | CYCLE,
            ],
        );
        e[2] >> 24
    };
    assert_eq!(address(&mut b, &mut hub, 1, 1), SUCCESS);
    let slot = b.context(OUTPUT + 0x400);
    assert_eq!((slot[0] & 0xf_ffff, slot[3] & 0xff), (1, 1));
    // No port 2: the device is missing, as a disconnected one is.
    assert_eq!(address(&mut b, &mut hub, 2, 2), TRANSACTION_ERROR);
    // Disabling the hub's port hides the keyboard too.
    transfer(&mut hub, setup(0x23, 1, 1, 1, 0), &[]).unwrap();
    assert_eq!(address(&mut b, &mut hub, 3, 1), TRANSACTION_ERROR);
}

/// Stela's own host xHCI driver (`stela_drivers::usb::xhci`) against
/// this model, with Stela's native USB functions as the devices: DMA
/// buffers are host memory, which the model reaches through a `Memory`
/// over the same addresses.
/// Isochronous TDs run in the frame their Frame ID names, or at once with
/// SIA; a frame already behind is a Missed Service Error; a failed packet
/// completes its TD without halting the endpoint, and a ring left empty at
/// the next service interval posts one Ring Underrun.
#[test]
fn isochronous_tds_run_in_their_service_interval() {
    const ISOCH_TRB: u32 = 5 << 10;
    const SIA: u32 = 1 << 31;
    const RING_UNDERRUN: u32 = 14;
    const MISSED_SERVICE: u32 = 23;
    const RING: u64 = RAM + 0xa000;
    let mut b = enumerated();
    // Isochronous OUT endpoint 1 (DCI 2), one frame (2^3 microframes).
    b.put_context(INPUT, [0, 1 | 1 << 2, 0, 0, 0, 0, 0, 0]);
    b.put_context(INPUT + 0x20, [3 << 27, 1 << 16, 0, 0, 0, 0, 0, 0]);
    b.put_context(
        INPUT + 0x20 * 3,
        [
            3 << 16,
            1 << 3 | 192 << 16,
            RING as u32 | 1,
            (RING >> 32) as u32,
            192,
            0,
            0,
            0,
        ],
    );
    let e = b.command([
        INPUT as u32,
        (INPUT >> 32) as u32,
        0,
        CONFIGURE_ENDPOINT << 10 | 1 << 24 | CYCLE,
    ]);
    assert_eq!(e[2] >> 24, SUCCESS);
    let frame = 8 * MICROFRAME;
    let isoch = |control: u32| [BUFFER as u32, (BUFFER >> 32) as u32, 192, control];
    // Frame 10: a TD for frame 12 waits for it.
    b.now = 10 * frame;
    trb(
        &mut b.ram,
        RING,
        isoch(ISOCH_TRB | 12 << 20 | TRB_IOC | CYCLE),
    );
    wr(&mut b.x, DOORBELLS + 4, 2);
    b.service();
    assert_eq!(event(&mut b.ram, b.events)[3] & CYCLE, 0);
    assert_eq!(b.x.deadline(None), Some(12 * frame));
    b.now = 12 * frame;
    b.service();
    // The keyboard has no OUT endpoint: the packet fails, the TD
    // completes and the endpoint keeps running.
    let e = b.event();
    assert_eq!(
        (e[0], e[2] >> 24, e[2] & 0xff_ffff, e[3] >> 16 & 0x1f),
        (RING as u32, TRANSACTION_ERROR, 192, 2)
    );
    assert_eq!(b.context(OUTPUT + 0x20 * 2)[0] & 7, 1);
    // Nothing queued at the next interval: one Ring Underrun.
    assert_eq!(b.x.deadline(None), Some(13 * frame));
    b.now = 13 * frame;
    b.service();
    let e = b.event();
    assert_eq!(
        (e[0], e[1], e[2] >> 24, e[3] >> 16 & 0x1f),
        (0, 0, RING_UNDERRUN, 2)
    );
    b.now = 20 * frame;
    b.service();
    assert_eq!(event(&mut b.ram, b.events)[3] & CYCLE, 0);
    // Frame 5 is behind frame 20: Missed Service, at once.
    trb(
        &mut b.ram,
        RING + 16,
        isoch(ISOCH_TRB | 5 << 20 | TRB_IOC | CYCLE),
    );
    wr(&mut b.x, DOORBELLS + 4, 2);
    b.service();
    let e = b.event();
    assert_eq!(
        (e[0], e[2] >> 24, e[2] & 0xff_ffff),
        ((RING + 16) as u32, MISSED_SERVICE, 192)
    );
    // SIA: the next interval after now.
    b.now = 20 * frame + 3 * MICROFRAME;
    trb(
        &mut b.ram,
        RING + 32,
        isoch(ISOCH_TRB | SIA | TRB_IOC | CYCLE),
    );
    wr(&mut b.x, DOORBELLS + 4, 2);
    b.service();
    assert_eq!(event(&mut b.ram, b.events)[3] & CYCLE, 0);
    assert_eq!(b.x.deadline(None), Some(21 * frame));
    b.now = 21 * frame;
    b.service();
    assert_eq!(b.event()[0], (RING + 32) as u32);
}

pub(crate) mod host {
    use alloc::{boxed::Box, rc::Rc, string::String, vec::Vec};
    use core::cell::RefCell;
    use stela_drivers::{
        Clock,
        bus::dma::RegionAllocator,
        usb::xhci::{Found, Registers, start},
    };
    use {
        crate::dart::Dart, crate::dwc3::xhci::Xhci, crate::typec::Usb2Speed, crate::usb::Function,
        stela_contracts::bus::Access, stela_contracts::bus::MmioDevice,
        stela_contracts::bus::Width, stela_contracts::dma::Memory, stela_contracts::dma::Port,
        stela_contracts::interrupt::InterruptLatch,
    };

    /// Host memory the test owns, by address.
    struct Arena {
        start: u64,
        end: u64,
    }

    impl Memory for Arena {
        fn read(&mut self, physical: u64, buffer: &mut [u8]) -> bool {
            if physical < self.start || physical + buffer.len() as u64 > self.end {
                return false;
            }
            // SAFETY: inside the arena, which outlives the bench.
            unsafe {
                core::ptr::copy_nonoverlapping(
                    physical as *const u8,
                    buffer.as_mut_ptr(),
                    buffer.len(),
                )
            };
            true
        }
        fn write(&mut self, physical: u64, data: &[u8]) -> bool {
            if physical < self.start || physical + data.len() as u64 > self.end {
                return false;
            }
            // SAFETY: as in `read`.
            unsafe {
                core::ptr::copy_nonoverlapping(data.as_ptr(), physical as *mut u8, data.len())
            };
            true
        }
    }

    pub(crate) struct Model<F> {
        pub xhci: Xhci,
        pub function: F,
        arena: Arena,
        now: u64,
    }

    impl<F: Function> Model<F> {
        fn service(&mut self) {
            let mut dart = Dart::new(16, None);
            let mut latch = InterruptLatch::new();
            let mut access = Access::new(0, self.now, &mut latch);
            dart.write(0xc00, Width::Word, 1, &mut access).unwrap();
            dart.write(0x1000, Width::Word, 6, &mut access).unwrap();
            let mut port = Port {
                iommu: &mut dart,
                stream: 0,
                memory: &mut self.arena,
                access: &mut access,
            };
            // The cable stays plugged in across a controller reset, as the
            // machine keeps it (`attach_host_function` each service).
            self.xhci.attach(Some(Usb2Speed::High));
            self.function.set_time(self.now);
            self.xhci
                .service(&mut port, Some(&mut self.function))
                .unwrap();
            assert!(self.xhci.live_endpoints_match());
        }
    }

    struct Shared<F>(Rc<RefCell<Model<F>>>);

    impl<F: Function> Registers for Shared<F> {
        fn read32(&mut self, offset: usize) -> u32 {
            let mut model = self.0.borrow_mut();
            // One microframe per access, so periodic endpoints come due.
            model.now += 3000;
            model.service();
            let now = model.now;
            model.xhci.read(offset as u64, 4, now).unwrap() as u32
        }
        fn write32(&mut self, offset: usize, value: u32) {
            let mut model = self.0.borrow_mut();
            let now = model.now;
            model
                .xhci
                .write(offset as u64, 4, u64::from(value), now)
                .unwrap();
            model.service();
        }
        fn write64(&mut self, offset: usize, value: u64) {
            let mut model = self.0.borrow_mut();
            let now = model.now;
            model.xhci.write(offset as u64, 8, value, now).unwrap();
            model.service();
        }
    }

    // SAFETY: this fixture owns the isolated controller's virtual time.
    // Waiting advances that same model at its 24 MHz rate before servicing it.
    unsafe impl<F: Function> stela_drivers::bus::pci::power::Timer for Shared<F> {
        fn check(&self) -> Result<(), stela_drivers::bus::pci::power::Error> {
            self.0
                .try_borrow_mut()
                .map(|_| ())
                .map_err(|_| stela_drivers::bus::pci::power::Error::Clock)
        }

        fn wait_us(&self, microseconds: u64) -> Result<(), stela_drivers::bus::pci::power::Error> {
            use stela_drivers::bus::pci::power::Error;
            let mut model = self.0.try_borrow_mut().map_err(|_| Error::Clock)?;
            let ticks = microseconds
                .checked_mul(super::SECOND / 1_000_000)
                .ok_or(Error::Clock)?;
            model.now = model.now.checked_add(ticks).ok_or(Error::Clock)?;
            model.service();
            Ok(())
        }
    }

    pub(crate) struct Bench<F> {
        pub model: Rc<RefCell<Model<F>>>,
        dma: RegionAllocator,
        _backing: Vec<u64>,
    }

    /// The model with `function` cabled to its USB2 port at high speed.
    pub(crate) fn bench<F>(function: F) -> Bench<F> {
        let mut backing: Vec<u64> = vec![0; 4 << 20 >> 3];
        let start = backing.as_mut_ptr() as u64;
        let end = start + (backing.len() * 8) as u64;
        let xhci = Xhci::new();
        let model = Rc::new(RefCell::new(Model {
            xhci,
            function,
            arena: Arena { start, end },
            now: 0,
        }));
        Bench {
            model,
            // SAFETY: the vector lives as long as the bench.
            dma: unsafe { RegionAllocator::new(start..end) },
            _backing: backing,
        }
    }

    fn clock() -> Clock {
        Clock {
            ticks_per_microsecond: 1000,
        }
    }

    /// The host driver started on the bench's model.
    pub(crate) fn run<F: Function + 'static>(bench: &mut Bench<F>) -> Found {
        let registers = Box::new(Shared(bench.model.clone()));
        start(
            registers,
            clock(),
            &mut bench.dma,
            String::from("xhci test"),
            Rc::new(Shared(bench.model.clone())),
        )
        .unwrap()
    }
}
