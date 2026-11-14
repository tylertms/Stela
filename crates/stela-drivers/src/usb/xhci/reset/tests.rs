//! Physical-address command/context/event model, independent of reset helpers.
use super::*;
use crate::bus::dma::RegionAllocator;
use alloc::collections::BTreeMap;
use core::cell::RefCell;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Fault {
    None,
    Stop(u8),
    SilentStop(u8),
    Dequeue(u8),
    Disable,
    WrongDisableSlot,
    SilentDisable,
    NoResetChange,
    NoPortEvent,
    BadLink,
    Reserve,
    SilentReserve,
    Identity,
    FatalAfterReset,
    ChangeAfterReset,
    ConfigureAfterReset,
    AddressAfterReset,
    EnableAfterReset,
}
#[derive(Clone, Copy)]
struct Cursor {
    next: u64,
    cycle: u32,
}
#[derive(Clone)]
struct Td {
    trb: u64,
    buffer: u64,
    length: usize,
}
pub(crate) mod abort_peer;

struct Task {
    lun: [u8; 8],
    tag: u16,
    requested: usize,
    input: bool,
    data: Vec<u8>,
    ready: bool,
    data_complete: bool,
    hang: bool,
    lba: u64,
    write: bool,
    sense: Option<u32>,
}
pub(crate) struct Hardware {
    commands: Cursor,
    events: u64,
    event_count: usize,
    consumed: usize,
    context: usize,
    dcbaa: u64,
    outputs: BTreeMap<u8, u64>,
    cursors: BTreeMap<(u8, u8, u16), Cursor>,
    pending: BTreeMap<(u8, u8, u16), Td>,
    deferred: Vec<(u64, u32, u32, u8, u8)>,
    enabled: Vec<bool>,
    pub enables: Vec<u8>,
    pub commands_seen: Vec<(u32, u8, u8)>,
    port: u32,
    speed: u8,
    pub resets: usize,
    reset_ready: u64,
    pub reset_writes: usize,
    pub accesses: usize,
    pub trap_mmio: bool,
    /// An unanswered command keeps CRCR.CRR set; aborts are ignored, so a
    /// timeout finds the host dead.
    stuck: Option<u64>,
    /// The controller ignores its command doorbell and never runs the ring.
    pub deaf: bool,
    pub fault: Fault,
    pub hang_next: bool,
    pub device_task: bool,
    pub empty_media: bool,
    pub sense_once: Option<(u8, u32)>,
    task: Option<Task>,
    pub scsi: Vec<(usize, u8)>,
    writes: Vec<(u64, Vec<u8>)>,
    bot: Option<bot_peer::Peer>,
    preferred_slot: Option<usize>,
    iso: bool,
    force_idle_stop: bool,
    forced_idle_events: usize,
    class_peer: Option<class_peers::Kind>,
    uas_abort: abort_peer::State,
    /// Runs the controller's one-tick-per-microsecond deadlines on virtual time.
    _clock: crate::test_clock::Guard,
}
fn read<T: Copy>(address: u64) -> T {
    // SAFETY: the pointer addresses live memory this test's model owns, in bounds, and nothing
    // accesses it concurrently.
    unsafe { (address as *const T).read_volatile() }
}
fn put<T>(address: u64, value: T) {
    // SAFETY: the pointer addresses live memory this test's model owns, in bounds, and nothing
    // accesses it concurrently.
    unsafe {
        (address as *mut T).write_volatile(value);
    }
}
fn bytes(address: u64, length: usize) -> Vec<u8> {
    // SAFETY: the pointer addresses live memory this test's model owns, in bounds, and nothing
    // accesses it concurrently.
    unsafe { core::slice::from_raw_parts(address as *const u8, length) }.to_vec()
}
fn copy(address: u64, data: &[u8]) {
    // SAFETY: the pointer addresses live memory this test's model owns, in bounds, and nothing
    // accesses it concurrently.
    unsafe {
        core::ptr::copy_nonoverlapping(data.as_ptr(), address as *mut u8, data.len());
    }
}
impl Hardware {
    fn take(cursor: &mut Cursor) -> (u64, u64, u32, u32) {
        loop {
            let at = cursor.next;
            let control = read::<u32>(at + 12);
            assert_eq!(control & 1, cursor.cycle, "doorbell before published TRB");
            if control >> 10 & 63 == LINK {
                cursor.next = read::<u64>(at) & !15;
                if control & 2 != 0 {
                    cursor.cycle ^= 1;
                }
                continue;
            }
            cursor.next += 16;
            return (at, read(at), read(at + 8), control);
        }
    }
    fn event(&mut self, parameter: u64, status: u32, kind: u32, slot: u8, dci: u8) {
        if kind == COMMAND_COMPLETION && self.stuck == Some(parameter) {
            self.stuck = None;
        }
        let index = self.event_count % RING_TRBS;
        let cycle = if (self.event_count / RING_TRBS) & 1 == 0 {
            1
        } else {
            0
        };
        let at = self.events + (index * 16) as u64;
        put(at, parameter);
        put(at + 8, status);
        put(
            at + 12,
            u32::from(slot) << 24 | u32::from(dci) << 16 | kind << 10 | cycle,
        );
        self.event_count += 1;
    }
    fn output(&self, slot: u8, dci: u8) -> u64 {
        self.outputs[&slot] + u64::from(dci) * self.context as u64
    }
    fn command(&mut self) {
        let (at, parameter, status, control) = Self::take(&mut self.commands);
        self.stuck = Some(at);
        let kind = control >> 10 & 63;
        let slot = (control >> 24) as u8;
        let dci = (control >> 16 & 31) as u8;
        self.commands_seen.push((kind, slot, dci));
        let mut returned = slot;
        let mut code = SUCCESS;
        match kind {
            ENABLE_SLOT => {
                if self.fault == Fault::EnableAfterReset && self.resets > 0
                    || self.fault == Fault::Reserve && self.resets > 0
                {
                    code = 9;
                } else {
                    let id = self
                        .enabled
                        .iter()
                        .enumerate()
                        .skip(1)
                        .find(|(_, used)| !**used)
                        .map(|(id, _)| id);
                    let id = self.preferred_slot.take().or(id);
                    if let Some(id) = id {
                        assert!(!self.enabled[id]);
                        self.enabled[id] = true;
                        returned = id as u8;
                        self.enables.push(returned);
                        if self.fault == Fault::SilentReserve && self.resets > 0 {
                            return;
                        }
                    } else {
                        code = 9;
                    }
                }
            }
            ADDRESS_DEVICE => {
                if self.fault == Fault::AddressAfterReset && self.resets > 0 {
                    self.outputs
                        .insert(slot, read::<u64>(self.dcbaa + u64::from(slot) * 8));
                    self.event(at, 4 << 24, COMMAND_COMPLETION, slot, 0);
                    return;
                }
                assert!(
                    !self.device_task,
                    "Address Device before actual firmware reset/retirement"
                );
                assert!(self.enabled[usize::from(slot)]);
                assert!(
                    Clock::ticks() >= self.reset_ready,
                    "Address Device before reset settle"
                );
                let output = read::<u64>(self.dcbaa + u64::from(slot) * 8);
                self.outputs.insert(slot, output);
                for index in 1..3 {
                    copy(
                        output + ((index - 1) * self.context) as u64,
                        &bytes(parameter + (index * self.context) as u64, self.context),
                    );
                }
                let ep0 = output + self.context as u64;
                put(ep0, ENDPOINT_RUNNING);
                let ptr = read::<u64>(ep0 + 8);
                self.cursors.insert(
                    (slot, 1, 0),
                    Cursor {
                        next: ptr & !15,
                        cycle: (ptr & 1) as u32,
                    },
                );
            }
            CONFIGURE_ENDPOINT => {
                if self.fault == Fault::ConfigureAfterReset && self.resets > 0 {
                    self.event(at, 5 << 24, COMMAND_COMPLETION, slot, 0);
                    return;
                }
                let add = read::<u32>(parameter + 4);
                let drop = read::<u32>(parameter);
                for dci in 2..32u8 {
                    if drop & (1 << dci) != 0 {
                        let out = self.output(slot, dci);
                        put(out, read::<u32>(out) & !7);
                        self.cursors.retain(|(s, e, _), _| *s != slot || *e != dci);
                    }
                    if add & (1 << dci) != 0 {
                        let out = self.output(slot, dci);
                        copy(
                            out,
                            &bytes(
                                parameter + ((usize::from(dci) + 1) * self.context) as u64,
                                self.context,
                            ),
                        );
                        let info = read::<u32>(out);
                        put(out, info | ENDPOINT_RUNNING);
                        let ptr = read::<u64>(out + 8);
                        if info & (1 << 15) != 0 {
                            let entries = 1usize << ((info >> 10 & 31) + 1);
                            for stream in 1..entries {
                                let p = read::<u64>((ptr & !15) + (stream * 16) as u64);
                                if p != 0 {
                                    self.cursors.insert(
                                        (slot, dci, stream as u16),
                                        Cursor {
                                            next: p & !15,
                                            cycle: (p & 1) as u32,
                                        },
                                    );
                                }
                            }
                        } else {
                            self.cursors.insert(
                                (slot, dci, 0),
                                Cursor {
                                    next: ptr & !15,
                                    cycle: (ptr & 1) as u32,
                                },
                            );
                        }
                    }
                }
            }
            STOP_ENDPOINT | RESET_ENDPOINT => {
                if self.fault == Fault::Stop(dci) {
                    code = 5;
                } else {
                    let out = self.output(slot, dci);
                    put(out, read::<u32>(out) & !7 | ENDPOINT_STOPPED);
                    if kind == STOP_ENDPOINT {
                        let keys: Vec<_> = self
                            .pending
                            .keys()
                            .filter(|(s, e, _)| *s == slot && *e == dci)
                            .copied()
                            .collect();
                        if keys.is_empty() && self.force_idle_stop {
                            let pointer = self
                                .cursors
                                .iter()
                                .find(|((s, e, _), _)| *s == slot && *e == dci)
                                .map(|(_, cursor)| cursor.next)
                                .unwrap();
                            const STOPPED_LENGTH_INVALID: u32 = 27;
                            self.deferred.push((
                                pointer,
                                STOPPED_LENGTH_INVALID << 24,
                                TRANSFER_EVENT,
                                slot,
                                dci,
                            ));
                            self.forced_idle_events += 1;
                        }
                        for key in keys {
                            let td = self.pending.remove(&key).unwrap();
                            self.deferred
                                .push((td.trb, 26 << 24, TRANSFER_EVENT, slot, dci));
                        }
                        // Spec4.6.9 requires stopped TD events before Stop ACK.
                        // Defer both until the host polls the device.
                        if self.fault != Fault::SilentStop(dci) {
                            self.deferred
                                .push((at, SUCCESS << 24, COMMAND_COMPLETION, slot, 0));
                        }
                        return;
                    }
                }
            }
            SET_DEQUEUE => {
                if self.fault == Fault::Dequeue(dci) {
                    self.event(at, 5 << 24, COMMAND_COMPLETION, slot, 0);
                    return;
                }
                let stream = (status >> 16) as u16;
                self.cursors.insert(
                    (slot, dci, stream),
                    Cursor {
                        next: parameter & !15,
                        cycle: (parameter & 1) as u32,
                    },
                );
            }
            DISABLE_SLOT => {
                assert_eq!(
                    read::<u64>(self.dcbaa + u64::from(slot) * 8),
                    self.outputs[&slot],
                    "DCBAA cleared before Disable Slot ACK"
                );
                assert!(
                    self.deferred.is_empty(),
                    "Disable Slot before deferred command/events received"
                );
                assert_eq!(
                    self.consumed, self.event_count,
                    "Disable Slot before outstanding events received"
                );
                for dci in 1..32u8 {
                    if self.cursors.keys().any(|(s, e, _)| *s == slot && *e == dci) {
                        assert_eq!(
                            read::<u32>(self.output(slot, dci)) & 7,
                            ENDPOINT_STOPPED,
                            "Disable Slot before every endpoint stopped"
                        );
                    }
                }
                assert!(
                    self.pending.keys().all(|(s, _, _)| *s != slot),
                    "Disable Slot with outstanding TD"
                );
                if self.fault == Fault::SilentDisable {
                    return;
                }
                if self.fault == Fault::Disable {
                    code = 5;
                } else {
                    self.enabled[usize::from(slot)] = false;
                    // Successful Disable invalidates the peer's endpoint
                    // fetch/dequeue state. A later rejected Address must not
                    // inherit old DCIs because the numeric ID recurs.
                    self.cursors.retain(|(s, _, _), _| *s != slot);
                    if self.fault == Fault::WrongDisableSlot {
                        returned = slot + 1;
                    }
                }
            }
            _ => panic!("unexpected reset model command {kind}"),
        }
        self.event(at, code << 24, COMMAND_COMPLETION, returned, 0);
    }
    fn control(&mut self, slot: u8) {
        let cursor = self.cursors.get_mut(&(slot, 1, 0)).unwrap();
        let (_, setup, _, ctl) = Self::take(cursor);
        assert_eq!(ctl >> 10 & 63, SETUP);
        let request = setup.to_le_bytes();
        let mut last;
        let mut data = None;
        loop {
            let (at, buffer, length, control) = Self::take(cursor);
            last = at;
            if control >> 10 & 63 == DATA {
                data = Some((buffer, length as usize));
            }
            if control >> 10 & 63 == STATUS {
                break;
            }
        }
        if request[1] == GET_DESCRIPTOR {
            let reply = match request[3] {
                DEVICE_DESCRIPTOR => self.descriptor(),
                CONFIGURATION_DESCRIPTOR => {
                    if let Some(kind) = self.class_peer {
                        class_peers::configuration(kind, self.speed)
                    } else if self.iso {
                        published_resource_tests::iso_configuration(self.speed)
                    } else if self.bot.is_some() {
                        bot_peer::configuration(self.speed)
                    } else {
                        configuration(self.speed)
                    }
                }
                0x22 if self.class_peer == Some(class_peers::Kind::Hid) => {
                    class_peers::HID_REPORT.to_vec()
                }
                other => panic!("descriptor {other}"),
            };
            let (address, length) = data.unwrap();
            copy(address, &reply[..length.min(reply.len())]);
        } else if self.bot.is_some() && request[0] == 0xa1 && request[1] == 0xfe {
            let (address, length) = data.unwrap();
            assert_eq!(length, 1);
            copy(address, &[0]);
        } else if self.class_peer == Some(class_peers::Kind::Hid)
            && request[0] == 0x21
            && matches!(request[1], 0x0a | 0x0b)
        {
            assert!(data.is_none());
        } else {
            assert!(matches!(request[1], SET_CONFIGURATION | SET_INTERFACE));
        }
        self.event(last, SUCCESS << 24, TRANSFER_EVENT, slot, 1);
    }
    fn descriptor(&self) -> Vec<u8> {
        let mut descriptor = vec![
            18,
            1,
            0,
            if self.speed >= SUPER_SPEED { 3 } else { 2 },
            0,
            0,
            0,
            if self.speed >= SUPER_SPEED { 9 } else { 64 },
            0xf4,
            0x46,
            3,
            0,
            if self.fault == Fault::Identity && self.resets > 0 {
                1
            } else {
                0
            },
            0,
            0,
            0,
            0,
            1,
        ];
        if let Some(peer) = &self.bot {
            peer.descriptor(&mut descriptor);
        }
        descriptor
    }
    fn bulk(&mut self, slot: u8, dci: u8, stream: u16) {
        if let Some(mut peer) = self.bot.take() {
            peer.bulk(self, slot, dci, stream);
            self.bot = Some(peer);
            return;
        }
        let cursor = self.cursors.get_mut(&(slot, dci, stream)).unwrap();
        let (trb, buffer, length, control) = Self::take(cursor);
        assert_eq!(control >> 10 & 63, NORMAL);
        let td = Td {
            trb,
            buffer,
            length: (length & 0x1ffff) as usize,
        };
        self.pending.insert((slot, dci, stream), td.clone());
        if dci == 2 {
            let iu = bytes(buffer, td.length);
            if iu[0] == 5 {
                self.abort_task_iu(slot, stream, &td, &iu);
                return;
            }
            assert_eq!(iu[0], 1);
            assert!(
                !self.device_task,
                "new CDB before actual firmware reset/retirement"
            );
            let cdb = &iu[16..];
            let mut task = Task {
                lun: iu[8..16].try_into().unwrap(),
                tag: u16::from_be_bytes([iu[2], iu[3]]),
                requested: 0,
                input: true,
                data: Vec::new(),
                ready: false,
                data_complete: false,
                hang: self.hang_next,
                lba: 0,
                write: false,
                sense: None,
            };
            self.hang_next = false;
            self.scsi.push((self.resets, cdb[0]));
            match cdb[0] {
                0x12 => {
                    task.requested = 36;
                    task.data = vec![0; 36];
                    // RMB: the model's medium comes and goes.
                    task.data[1] = 0x80;
                    task.data[2] = 6;
                    task.data[4] = 31;
                    task.data[8..16].copy_from_slice(b"STELAUSB");
                    task.data[16..32].copy_from_slice(b"RESET DMA MODEL ");
                }
                0 => {}
                0xa0 => {
                    task.requested = u32::from_be_bytes(cdb[6..10].try_into().unwrap()) as usize;
                    task.data = vec![0; 16];
                    task.data[3] = 8;
                }
                0x25 => {
                    task.requested = 8;
                    task.data
                        .extend_from_slice(&(1023u32 + self.resets as u32 * 1024).to_be_bytes());
                    task.data.extend_from_slice(&512u32.to_be_bytes());
                }
                0x1a => {
                    task.requested = 4;
                    task.data = vec![3, 0, 0, 0];
                }
                0x28 | 0x2a => {
                    task.requested =
                        usize::from(u16::from_be_bytes(cdb[7..9].try_into().unwrap())) * 512;
                    task.lba = u64::from(u32::from_be_bytes(cdb[2..6].try_into().unwrap()));
                    task.write = cdb[0] == 0x2a;
                    task.input = !task.write;
                    if task.input {
                        task.data = self
                            .writes
                            .iter()
                            .rev()
                            .find(|(lba, data)| *lba == task.lba && data.len() == task.requested)
                            .map_or_else(|| vec![0x6d; task.requested], |(_, bytes)| bytes.clone());
                    }
                }
                0x35 => {}
                other => panic!("unexpected SCSI {other:#x}"),
            }
            if self.empty_media && !matches!(cdb[0], 0x12 | 0xa0) {
                task.sense = Some(0x023a00);
            }
            if self.sense_once.is_some_and(|(op, _)| op == cdb[0]) {
                task.sense = self.sense_once.take().map(|(_, sense)| sense);
            }
            if task.sense.is_some() {
                task.requested = 0;
                task.data.clear();
            }
            self.device_task = task.hang;
            self.task = Some(task);
            self.pending.remove(&(slot, dci, stream));
            if !self.uas_abort.hold_command_event {
                self.event(trb, SUCCESS << 24, TRANSFER_EVENT, slot, dci);
            }
        }
        self.progress(slot);
    }
    fn progress(&mut self, slot: u8) {
        let Some(mut task) = self.task.take() else {
            return;
        };
        let stream = if self.speed >= SUPER_SPEED {
            task.tag
        } else {
            0
        };
        let status_key = (slot, 5, stream);
        if self.speed < HIGH_SPEED {
            panic!("unsupported model speed");
        }
        if self.speed < SUPER_SPEED && task.requested != 0 && !task.ready {
            if let Some(td) = self.pending.remove(&status_key) {
                let frame = [
                    if task.input { 6 } else { 7 },
                    0,
                    (task.tag >> 8) as u8,
                    task.tag as u8,
                ];
                copy(td.buffer, &frame);
                self.event(
                    td.trb,
                    SHORT_PACKET << 24 | (td.length - 4) as u32,
                    TRANSFER_EVENT,
                    slot,
                    5,
                );
                task.ready = true;
            }
            self.task = Some(task);
            return;
        }
        if task.hang {
            self.task = Some(task);
            return;
        }
        if task.requested != 0 && !task.data_complete {
            let dci = if task.input { 7 } else { 8 };
            let Some(td) = self.pending.remove(&(slot, dci, stream)) else {
                self.task = Some(task);
                return;
            };
            let count = if task.input {
                task.data.len().min(td.length)
            } else {
                td.length
            };
            if task.input {
                copy(td.buffer, &task.data[..count]);
            } else {
                self.writes.push((task.lba, bytes(td.buffer, count)));
            }
            self.event(
                td.trb,
                (if count < td.length {
                    SHORT_PACKET
                } else {
                    SUCCESS
                }) << 24
                    | (td.length - count) as u32,
                TRANSFER_EVENT,
                slot,
                dci,
            );
            // Status may be posted after this data TD has completed. Keep
            // the completed phase when waiting for that separate IU.
            task.data_complete = true;
        }
        if let Some(td) = self.pending.remove(&status_key) {
            let mut frame = vec![0u8; if task.sense.is_some() { 34 } else { 16 }];
            frame[0] = 3;
            frame[2..4].copy_from_slice(&task.tag.to_be_bytes());
            if let Some(sense) = task.sense {
                frame[6] = 2;
                frame[14..16].copy_from_slice(&18u16.to_be_bytes());
                frame[16] = 0x70;
                frame[18] = (sense >> 16) as u8;
                frame[23] = 10;
                frame[28] = (sense >> 8) as u8;
                frame[29] = sense as u8;
            }
            copy(td.buffer, &frame);
            self.event(
                td.trb,
                SHORT_PACKET << 24 | (td.length - frame.len()) as u32,
                TRANSFER_EVENT,
                slot,
                5,
            );
        } else {
            self.task = Some(task);
        }
    }
}
struct Model(Rc<RefCell<Hardware>>);
impl Registers for Model {
    fn read32(&mut self, offset: usize) -> u32 {
        let mut s = self.0.borrow_mut();
        assert!(!s.trap_mmio, "retained stale consumer reached native MMIO");
        s.accesses += 1;
        if offset == 0x40 + USBSTS {
            let deferred = core::mem::take(&mut s.deferred);
            for (parameter, status, kind, slot, dci) in deferred {
                s.event(parameter, status, kind, slot, dci);
            }
            if s.fault == Fault::FatalAfterReset && s.resets > 0 && Clock::ticks() >= s.reset_ready
            {
                HCE
            } else {
                0
            }
        } else if offset == 0x40 + CRCR {
            if s.stuck.is_some() { CRCR_RUNNING } else { 0 }
        } else if offset == 0x40 + PORTSC {
            if s.fault == Fault::ChangeAfterReset && s.resets > 0 && Clock::ticks() >= s.reset_ready
            {
                s.port | CSC
            } else {
                s.port
            }
        } else {
            0
        }
    }
    fn write32(&mut self, offset: usize, value: u32) {
        let mut s = self.0.borrow_mut();
        assert!(!s.trap_mmio, "retained stale consumer reached native MMIO");
        s.accesses += 1;
        if offset == 0x1000 {
            if !s.deaf {
                s.command();
            }
        } else if (0x1004..0x1100).contains(&offset) {
            let slot = ((offset - 0x1000) / 4) as u8;
            let dci = (value & 31) as u8;
            let stream = (value >> 16) as u16;
            if dci == 1 {
                s.control(slot);
            } else if s.class_peer.is_some() {
                s.queue_class_transfer(slot, dci, stream);
            } else if s.iso {
                assert_eq!(stream, 0);
                s.queue_iso_service(slot, dci);
            } else {
                s.bulk(slot, dci, stream);
            }
        } else if offset == 0x40 + PORTSC {
            s.port &= !(value & PORT_CHANGES);
            if value & PR != 0 {
                s.reset_writes += 1;
                assert!(
                    s.commands_seen
                        .iter()
                        .any(|(kind, _, _)| *kind == DISABLE_SLOT)
                );
                s.port |= PR;
                if s.fault != Fault::NoResetChange {
                    s.port = s.port & !PR | PRC | CCS | PED;
                    if s.fault == Fault::BadLink {
                        s.port |= 6 << 5;
                    }
                    s.resets += 1;
                    s.device_task = false;
                    s.task = None;
                    if let Some(peer) = &mut s.bot {
                        peer.reset();
                    }
                    s.reset_ready = Clock::ticks() + POST_RESET_US;
                    if s.fault != Fault::NoPortEvent {
                        s.event(1 << 24, SUCCESS << 24, PORT_STATUS_CHANGE, 0, 0);
                    }
                }
            }
        }
    }
    fn write64(&mut self, offset: usize, value: u64) {
        let mut s = self.0.borrow_mut();
        assert!(!s.trap_mmio, "retained stale consumer reached native MMIO");
        s.accesses += 1;
        if offset == 0x800 + ERDP {
            let index = ((value & !15) - s.events) as usize / 16;
            s.consumed = s.event_count / RING_TRBS * RING_TRBS + index;
            if s.consumed > s.event_count {
                s.consumed -= RING_TRBS;
            }
        }
    }
}
fn configuration(speed: u8) -> Vec<u8> {
    let mut raw = vec![9, 2, 0, 0, 1, 1, 0, 0x80, 0, 9, 4, 0, 0, 4, 8, 6, 0x62, 0];
    for (role, address) in [(1, 1), (2, 0x82), (3, 0x83), (4, 4)] {
        let packet = if speed >= SUPER_SPEED { 1024u16 } else { 512 };
        raw.extend_from_slice(&[7, 5, address, 2, packet as u8, (packet >> 8) as u8, 0]);
        if speed >= SUPER_SPEED {
            raw.extend_from_slice(&[6, 48, 0, if role == 1 { 0 } else { 1 }, 0, 0]);
        }
        raw.extend_from_slice(&[4, 0x24, role, 0]);
    }
    let length = raw.len() as u16;
    raw[2..4].copy_from_slice(&length.to_le_bytes());
    raw
}
pub(crate) fn fixture(
    memory: &mut [u64],
    speed: u8,
    context: usize,
    max_slots: usize,
) -> (Shared, Device, Rc<RefCell<Hardware>>, RegionAllocator) {
    fixture_profile(
        memory,
        speed,
        context,
        max_slots,
        (false, false, dma::Profile::SPEC),
    )
}
pub(crate) fn fixture_bot(
    memory: &mut [u64],
    speed: u8,
    context: usize,
    max_slots: usize,
    write_cache: bool,
) -> (Shared, Device, Rc<RefCell<Hardware>>, RegionAllocator) {
    fixture_profile_peer(
        memory,
        speed,
        context,
        max_slots,
        (true, false, dma::Profile::SPEC),
        (None, true),
        Some(bot_peer::Peer::new(write_cache)),
    )
}
fn fixture_profile(
    memory: &mut [u64],
    speed: u8,
    context: usize,
    max_slots: usize,
    profile: (bool, bool, dma::Profile),
) -> (Shared, Device, Rc<RefCell<Hardware>>, RegionAllocator) {
    fixture_profile_class(memory, speed, context, max_slots, profile, None, true)
}
fn fixture_profile_class(
    memory: &mut [u64],
    speed: u8,
    context: usize,
    max_slots: usize,
    profile: (bool, bool, dma::Profile),
    class_peer: Option<class_peers::Kind>,
    configured: bool,
) -> (Shared, Device, Rc<RefCell<Hardware>>, RegionAllocator) {
    fixture_profile_peer(
        memory,
        speed,
        context,
        max_slots,
        profile,
        (class_peer, configured),
        None,
    )
}
fn fixture_profile_peer(
    memory: &mut [u64],
    speed: u8,
    context: usize,
    max_slots: usize,
    profile: (bool, bool, dma::Profile),
    class: (Option<class_peers::Kind>, bool),
    peer: Option<bot_peer::Peer>,
) -> (Shared, Device, Rc<RefCell<Hardware>>, RegionAllocator) {
    let (class_peer, configured) = class;
    let (bot, iso, profile) = profile;
    let range = memory.as_mut_ptr() as u64
        ..memory.as_mut_ptr() as u64 + core::mem::size_of_val(memory) as u64;
    // SAFETY: the model device, its pages and its domain belong to this test alone; no other
    // driver, worker or DMA touches them.
    let mut dma = unsafe { RegionAllocator::new(range) };
    let commands = Ring::with_profile(&mut dma, profile).unwrap();
    let events = dma.allocate(RING_TRBS * 16, 64).unwrap();
    let dcbaa = dma.allocate((max_slots + 1) * 8, 64).unwrap();
    let hardware = Rc::new(RefCell::new(Hardware {
        commands: Cursor {
            next: commands.buffer.physical(),
            cycle: 1,
        },
        events: events.physical(),
        event_count: 0,
        consumed: 0,
        context,
        dcbaa: dcbaa.physical(),
        outputs: BTreeMap::new(),
        cursors: BTreeMap::new(),
        pending: BTreeMap::new(),
        deferred: Vec::new(),
        enabled: vec![false; max_slots + 1],
        enables: Vec::new(),
        commands_seen: Vec::new(),
        port: PP | CCS | PED | PRC | u32::from(speed) << 10,
        speed,
        resets: 0,
        reset_ready: 0,
        reset_writes: 0,
        accesses: 0,
        trap_mmio: false,
        stuck: None,
        deaf: false,
        fault: Fault::None,
        hang_next: false,
        device_task: false,
        empty_media: false,
        sense_once: None,
        task: None,
        scsi: Vec::new(),
        writes: Vec::new(),
        bot: bot.then(|| peer.unwrap_or_default()),
        preferred_slot: None,
        iso,
        force_idle_stop: false,
        forced_idle_events: 0,
        class_peer,
        uas_abort: abort_peer::State::default(),
        _clock: crate::test_clock::virtualize(),
    }));
    let mut controller = Controller {
        minimum: crate::usb::xhci::minimum::fixture(Clock {
            ticks_per_microsecond: 1,
        }),
        native_memory: None,
        registers: Box::new(Model(hardware.clone())),
        clock: Clock {
            ticks_per_microsecond: 1,
        },
        operational: 0x40,
        runtime: 0x800,
        doorbells: 0x1000,
        context_bytes: context,
        dma_profile: profile,
        hardware: hardware::State::Running,
        quirks: quirks::Profile::default(),
        max_primary_streams: 4,
        stream_policy: streams::Policy::Standard,
        version: 0x100,
        iso_caps: iso::Capabilities::new(0x100, 0, 0),
        max_slots,
        max_ports: 1,
        port_major: vec![if speed >= SUPER_SPEED { 3 } else { 2 }],
        lpm: lpm::Host::default(),
        dcbaa: dcbaa.into(),
        _scratchpads: Vec::new(),
        commands,
        events: events.into(),
        _event_table: dma.allocate(16, 64).unwrap().into(),
        event_index: 0,
        event_cycle: true,
        slots: core::iter::repeat_with(|| None)
            .take(max_slots + 1)
            .collect(),
        retry_resources: slot_resources::Cache::new(max_slots + 1),
        queued_resets: Vec::new(),
        after_boot: false,
        slot_ownership: vec![Ownership::Vacant; max_slots + 1],
        test_retired_numbers: vec![false; max_slots + 1],
        power: power::Ledger::new(max_slots + 1),
        pending: Vec::new(),
        error: None,
        settle_from: None,
        command_poisoned: false,
        death: None,
        command_ring_stopped: false,
        unpublished: 0,
        signalled: false,
        quiet_drains: 0,
        outstanding: None,
        finished: Vec::new(),
        location: "reset model".into(),
    };
    let mut device = controller
        .identify(
            Attachment {
                root_port: 1,
                route: 0,
                tier: 0,
                speed,
                translator: None,
                parent: None,
            },
            "native root UAS".into(),
            &mut dma,
        )
        .unwrap();
    device.configuration = device.configurations[0].clone();
    if class_peer == Some(class_peers::Kind::Hid) {
        controller.claim_native(device.id).unwrap();
    }
    if configured {
        controller.set_configuration(device.slot, 1).unwrap();
    }
    (Rc::new(RefCell::new(controller)), device, hardware, dma)
}
fn prepared(
    memory: &mut [u64],
    speed: u8,
    context: usize,
    max_slots: usize,
) -> (
    Shared,
    Device,
    Vec<Endpoint>,
    Rc<RefCell<Hardware>>,
    RegionAllocator,
) {
    let (shared, device, hardware, mut dma) = fixture(memory, speed, context, max_slots);
    let endpoints = interfaces(&device.configuration)[0].endpoints.clone();
    if speed >= SUPER_SPEED {
        shared
            .borrow_mut()
            .configure_streams(device.slot, &endpoints, &[0x82, 0x83, 4], 2, &mut dma)
            .unwrap();
    } else {
        shared
            .borrow_mut()
            .configure(device.slot, &endpoints, None, &mut dma)
            .unwrap();
    }
    (shared, device, endpoints, hardware, dma)
}
/// A controller that stops running its command ring ends the command in two
/// command timeouts, one restart included, and is then dead. A passed device
/// start budget changes neither: a command keeps its own time, so that the
/// commands that cancel a transfer the budget ended are answered and a
/// working controller is not declared dead at their doorbell.
#[test]
fn a_controller_that_never_runs_its_command_ring_times_out_and_dies() {
    let mut memory = vec![0u64; 262_144];
    let (shared, _device, hardware, _dma) = fixture(&mut memory, HIGH_SPEED, 32, 4);
    hardware.borrow_mut().deaf = true;
    let started = Clock::ticks();
    let result = shared.borrow_mut().command(0, 0, ENABLE_SLOT << 10);
    let took = Clock::ticks() - started;
    assert!(matches!(result, Err(BlockError::Timeout)), "{result:?}");
    assert!(
        (2 * COMMAND_US..2 * COMMAND_US + HALT_US + 1_000_000).contains(&took),
        "{took}"
    );
    assert!(matches!(shared.borrow().hardware, hardware::State::Died));

    let mut memory = vec![0u64; 262_144];
    let (shared, _device, hardware, _dma) = fixture(&mut memory, HIGH_SPEED, 32, 4);
    hardware.borrow_mut().deaf = true;
    let clock = Clock {
        ticks_per_microsecond: 1,
    };
    let budget = stela_driver_base::budget::Budget::start(clock, 100_000).unwrap();
    let started = Clock::ticks();
    let result = shared.borrow_mut().command(0, 0, ENABLE_SLOT << 10);
    let took = Clock::ticks() - started;
    assert!(matches!(result, Err(BlockError::Timeout)), "{result:?}");
    assert!(
        (2 * COMMAND_US..2 * COMMAND_US + HALT_US + 1_000_000).contains(&took),
        "{took}"
    );
    assert!(budget.expired().is_some());

    // And one that answers, started after the budget passed, succeeds.
    let mut memory = vec![0u64; 262_144];
    let (shared, _device, _hardware, _dma) = fixture(&mut memory, HIGH_SPEED, 32, 4);
    let budget = stela_driver_base::budget::Budget::start(clock, 1).unwrap();
    assert!(!clock.wait(10, || false));
    assert!(budget.expired().is_some());
    shared
        .borrow_mut()
        .command(0, 0, ENABLE_SLOT << 10)
        .unwrap();
    assert!(!matches!(shared.borrow().hardware, hardware::State::Died));
}

#[test]
fn reset_requires_every_retirement_ack_then_physical_reset_and_fresh_slot() {
    for speed in [HIGH_SPEED, SUPER_SPEED] {
        for context in [32, 64] {
            let mut memory = vec![0u64; 512 * 1024];
            let (shared, old, endpoints, hardware, mut dma) =
                prepared(&mut memory, speed, context, 3);
            let payload = dma.allocate(512, 65536).unwrap();
            let mut command = dma.allocate(512, 512).unwrap();
            command.bytes_mut()[0] = 1;
            command.bytes_mut()[3] = 1;
            command.bytes_mut()[16] = 0x28;
            command.bytes_mut()[24] = 1;
            hardware.borrow_mut().hang_next = true;
            let ordinary = shared
                .borrow_mut()
                .submit(old.slot, 2, command.physical(), 32)
                .unwrap();
            let stream = if speed >= SUPER_SPEED {
                Some(
                    shared
                        .borrow_mut()
                        .submit_stream(old.slot, 7, 1, payload.physical(), 512)
                        .unwrap(),
                )
            } else {
                None
            };
            let old_output = shared
                .borrow_mut()
                .slot(old.slot)
                .unwrap()
                ._output
                .physical();
            // SAFETY: the model controller is stopped, and its old slot's endpoints are drained
            // before the reset.
            let fresh = unsafe {
                shared
                    .borrow_mut()
                    .reset_and_identify(&old, &endpoints, &mut dma)
            }
            .unwrap();
            let mut c = shared.borrow_mut();
            let before = hardware.borrow().accesses;
            assert!(c.connected(old.slot).is_err());
            assert!(c.current_configuration(old.slot).is_err());
            assert!(c.set_configuration(old.slot, 1).is_err());
            assert!(
                c.control(old.slot, setup(0x80, GET_DESCRIPTOR, 1 << 8, 0, 8), &[])
                    .is_err()
            );
            assert!(c.cancel(old.slot, 2).is_err());
            assert!(c.configure(old.slot, &endpoints, None, &mut dma).is_err());
            assert!(c.submit(old.slot, 2, payload.physical(), 512).is_err());
            assert!(c.completed(old.slot, 2, ordinary, 32).unwrap().is_err());
            if let Some(token) = &stream {
                assert!(c.completed_stream(token).unwrap().is_err());
                assert!(
                    c.submit_stream(old.slot, 7, 1, payload.physical(), 512)
                        .is_err()
                );
                assert!(
                    c.configure_streams(old.slot, &endpoints, &[0x82, 0x83, 4], 2, &mut dma)
                        .is_err()
                );
            }
            // SAFETY: the model device, its pages and its domain belong to this test alone; no
            // other driver, worker or DMA touches them.
            assert!(unsafe { c.reset_and_identify(&old, &endpoints, &mut dma) }.is_err());
            assert!(c.clear(old.slot, &endpoints[0]).is_err());
            assert!(c.string(old.slot, 0x0409, 1).is_none());
            assert_eq!(
                hardware.borrow().accesses,
                before,
                "stale handle reached MMIO"
            );
            // The reset port's grant retired with the old device, so the
            // slot goes to the fresh one, as xhci_alloc_dev reuses IDs.
            assert!(c.power.retained(fresh.slot));
            assert_eq!((old.slot, fresh.slot), (1, 1));
            assert_ne!(old.id, fresh.id);
            assert_eq!(hardware.borrow().enables, vec![1, 1]);
            assert_eq!(hardware.borrow().resets, 1);
            assert_ne!(c.slot(fresh.slot).unwrap()._output.physical(), 0);
            let _ = old_output;
            assert_eq!(c.slot_ownership[1], Ownership::Enabled);
        }
    }
}
#[test]
fn failed_contributor_never_reaches_disable_slot_or_port_reset() {
    for speed in [HIGH_SPEED, SUPER_SPEED] {
        for dci in [1, 2, 5, 7, 8] {
            for fault in [
                Fault::Stop(dci),
                Fault::SilentStop(dci),
                Fault::Dequeue(dci),
            ] {
                let mut memory = vec![0u64; 512 * 1024];
                let (shared, old, endpoints, hardware, mut dma) =
                    prepared(&mut memory, speed, 32, 3);
                hardware.borrow_mut().fault = fault;
                let output = shared
                    .borrow_mut()
                    .slot(old.slot)
                    .unwrap()
                    ._output
                    .physical();
                assert!(
                    // SAFETY: the model controller is stopped, and its old slot's endpoints are
                    // drained before the reset.
                    unsafe {
                        shared
                            .borrow_mut()
                            .reset_and_identify(&old, &endpoints, &mut dma)
                    }
                    .is_err()
                );
                assert!(
                    !hardware
                        .borrow()
                        .commands_seen
                        .iter()
                        .any(|(kind, _, _)| *kind == DISABLE_SLOT)
                );
                assert_eq!(hardware.borrow().reset_writes, 0);
                assert_eq!(shared.borrow().dcbaa.read::<u64>(8), output);
                assert!(shared.borrow_mut().endpoint_ready(old.slot, 2).is_err());
            }
        }
    }
}
#[test]
fn reset_failures_keep_tombstones_and_never_publish_fresh_handles() {
    for fault in [
        Fault::Disable,
        Fault::WrongDisableSlot,
        Fault::SilentDisable,
        Fault::NoResetChange,
        Fault::NoPortEvent,
        Fault::BadLink,
        Fault::Reserve,
        Fault::SilentReserve,
    ] {
        let mut memory = vec![0u64; 512 * 1024];
        let (shared, old, endpoints, hardware, mut dma) = prepared(&mut memory, SUPER_SPEED, 32, 3);
        hardware.borrow_mut().fault = fault;
        assert!(
            // SAFETY: the model controller is stopped, and its old slot's endpoints are drained
            // before the reset.
            unsafe {
                shared
                    .borrow_mut()
                    .reset_and_identify(&old, &endpoints, &mut dma)
            }
            .is_err(),
            "{fault:?}"
        );
        assert!(shared.borrow_mut().endpoint_ready(old.slot, 1).is_err());
        // Disable Slot only revokes DMA: the physical device may still draw
        // current. Release follows port reset and its required ACKs.
        assert_eq!(
            shared.borrow().power.retained(old.slot),
            !matches!(fault, Fault::Reserve | Fault::SilentReserve),
            "{fault:?}"
        );
        if matches!(
            fault,
            Fault::Disable | Fault::WrongDisableSlot | Fault::SilentDisable
        ) {
            assert_eq!(hardware.borrow().reset_writes, 0);
        } else {
            assert_eq!(hardware.borrow().reset_writes, 1);
        }
        assert!(
            hardware
                .borrow()
                .commands_seen
                .iter()
                .filter(|(kind, _, _)| *kind == ADDRESS_DEVICE)
                .count()
                == 1
        );
    }
}
#[test]
fn single_slot_reset_reuses_its_slot_and_fresh_allocation_failure_keeps_grants() {
    let mut memory = vec![0u64; 512 * 1024];
    let (shared, old, endpoints, hardware, mut dma) = prepared(&mut memory, HIGH_SPEED, 32, 1);
    // One slot is enough: the reset retires the old device's grant and
    // its slot goes to the fresh one.
    // SAFETY: the model controller is stopped, and its old slot's endpoints are drained before
    // the reset.
    let fresh = unsafe {
        shared
            .borrow_mut()
            .reset_and_identify(&old, &endpoints, &mut dma)
    }
    .unwrap();
    assert_eq!(
        (fresh.slot, hardware.borrow().enables.clone()),
        (1, vec![1, 1])
    );
    assert_ne!(fresh.id, old.id);
    assert_eq!(shared.borrow().slot_ownership[1], Ownership::Enabled);
    struct Failing<'a> {
        inner: &'a mut RegionAllocator,
        at: usize,
        calls: usize,
        allocations: Vec<(u64, usize)>,
    }
    impl DmaAllocator for Failing<'_> {
        fn allocate(&mut self, bytes: usize, align: usize) -> Result<DmaBuffer, BlockError> {
            self.calls += 1;
            if self.calls == self.at {
                return Err(BlockError::NoDmaMemory);
            }
            let buffer = self.inner.allocate(bytes, align)?;
            self.allocations.push((buffer.physical(), bytes));
            Ok(buffer)
        }
    }
    for at in 1..=4 {
        let mut memory = vec![0u64; 512 * 1024];
        let (shared, old, endpoints, hardware, mut dma) = prepared(&mut memory, HIGH_SPEED, 32, 3);
        let mut failing = Failing {
            inner: &mut dma,
            at,
            calls: 0,
            allocations: Vec::new(),
        };
        assert!(
            // SAFETY: the model controller is stopped, and its old slot's endpoints are drained
            // before the reset.
            unsafe {
                shared
                    .borrow_mut()
                    .reset_and_identify(&old, &endpoints, &mut failing)
            }
            .is_err()
        );
        assert_eq!(hardware.borrow().enables, vec![1, 1]);
        assert_eq!(shared.borrow().slot_ownership[1], Ownership::Enabled);
        assert!(shared.borrow().slots[1].is_none());
        assert_eq!(shared.borrow().dcbaa.read::<u64>(8), 0);
        assert!(shared.borrow_mut().endpoint_ready(old.id, 1).is_err());
        let end = failing
            .allocations
            .last()
            .map_or(0, |(at, bytes)| *at + *bytes as u64);
        drop(failing);
        assert!(
            dma.allocate(64, 64).unwrap().physical() >= end,
            "partial DMA allocation recycled"
        );
    }
}

#[test]
fn reset_rejects_downstream_and_inexact_inventory_before_mmio() {
    let mut memory = vec![0u64; 512 * 1024];
    let (shared, old, endpoints, hardware, mut dma) = prepared(&mut memory, HIGH_SPEED, 32, 3);
    let before = hardware.borrow().accesses;
    assert!(
        // SAFETY: the model controller is stopped, and its old slot's endpoints are drained
        // before the reset.
        unsafe {
            shared
                .borrow_mut()
                .reset_and_identify(&old, &endpoints[..3], &mut dma)
        }
        .is_err()
    );
    assert_eq!(hardware.borrow().accesses, before);
    shared.borrow_mut().slot(old.slot).unwrap().attachment.tier = 1;
    assert!(
        // SAFETY: the model controller is stopped, and its old slot's endpoints are drained
        // before the reset.
        unsafe {
            shared
                .borrow_mut()
                .reset_and_identify(&old, &endpoints, &mut dma)
        }
        .is_err()
    );
    assert_eq!(hardware.borrow().accesses, before);
}

#[test]
fn reset_rejects_other_unretired_root_port_owners_before_mmio() {
    for other_state in 0..3 {
        let mut memory = vec![0u64; 512 * 1024];
        let (shared, old, endpoints, hardware, mut dma) = prepared(&mut memory, HIGH_SPEED, 32, 3);
        // Inject a second addressed route into the ownership inventory. This is
        // an adverse topology check, not a claim that this fixture models hubs.
        let other = shared.borrow_mut().fresh_slot().unwrap();
        if other_state < 2 {
            // Corrupt the software ownership inventory directly, as this
            // test intends; do not invent a checked physical hub parent.
            let mut controller = shared.borrow_mut();
            let source = controller.slots[usize::from(old.slot)]
                .as_ref()
                .unwrap()
                .attachment;
            let context = controller.context();
            let input = dma.allocate(33 * context, 64).unwrap();
            let output = dma.allocate(32 * context, 64).unwrap();
            let control = dma.allocate(CONTROL_BYTES, CONTROL_BYTES).unwrap();
            controller.slots[usize::from(other)] = Some(Slot {
                generation: 0,
                parent_id: None,
                inventory: None,
                consumer: identity::Consumer::Unclaimed,
                suspended: false,
                attachment: Attachment {
                    route: 1,
                    tier: 1,
                    ..source
                },
                retiring: other_state == 1,
                input: input.into(),
                _output: output.into(),
                context: [0; 4],
                endpoints: Default::default(),
                retired_rings: Default::default(),
                control: control.into(),
                speed: HIGH_SPEED,
                current_configuration: Some(0),
            });
        } else {
            assert!(shared.borrow().slots[usize::from(other)].is_none());
        }
        let before = hardware.borrow().accesses;
        // SAFETY: the model controller is stopped, and its old slot's endpoints are drained
        // before the reset.
        let error = unsafe {
            shared
                .borrow_mut()
                .reset_and_identify(&old, &endpoints, &mut dma)
        }
        .unwrap_err();
        assert!(matches!(
            error.1,
            BlockError::Unsupported("USB reset physical port has another unretired slot owner")
        ));
        assert_eq!(hardware.borrow().accesses, before);
    }
}

// Additional whole-slot retirement peer operations consume the original
// command/TD DMA. Existing reset-model paths and test bodies are unchanged.
impl Hardware {
    pub(crate) fn pending_command_kind(&self) -> u32 {
        let mut cursor = self.commands;
        Self::take(&mut cursor).3 >> 10 & 63
    }

    pub(crate) fn queue_iso_service(&mut self, slot: u8, dci: u8) {
        let cursor = self.cursors.get_mut(&(slot, dci, 0)).unwrap();
        let (trb, buffer, status, control) = Self::take(cursor);
        assert_eq!(control >> 10 & 63, 5, "expected an actual Isoch TRB");
        assert_eq!(control & IOC, IOC);
        assert_eq!(read::<u32>(self.output(slot, dci)) & 7, ENDPOINT_RUNNING);
        assert!(
            self.pending
                .insert(
                    (slot, dci, 0),
                    Td {
                        trb,
                        buffer,
                        length: (status & 0x1ffff) as usize,
                    }
                )
                .is_none()
        );
    }

    pub(crate) fn reject_pending_address(&mut self) -> bool {
        let mut cursor = self.commands;
        let (at, _, _, control) = Self::take(&mut cursor);
        if control >> 10 & 63 != ADDRESS_DEVICE {
            return false;
        }
        self.commands = cursor;
        let slot = (control >> 24) as u8;
        assert!(self.enabled[usize::from(slot)]);
        let output = read::<u64>(self.dcbaa + u64::from(slot) * 8);
        assert_ne!(output, 0);
        assert_eq!(read::<u32>(output + self.context as u64) & 7, 0);
        self.outputs.insert(slot, output);
        self.commands_seen.push((ADDRESS_DEVICE, slot, 0));
        self.event(at, 4 << 24, COMMAND_COMPLETION, slot, 0);
        true
    }

    pub(crate) fn stall_completed_control(&mut self, slot: u8) {
        let out = self.output(slot, 1);
        put(out, read::<u32>(out) & !7 | ENDPOINT_HALTED);
        let at = self.events + ((self.event_count - 1) % RING_TRBS * TRB) as u64;
        let control = read::<u32>(at + 12);
        assert_eq!(control >> 10 & 63, TRANSFER_EVENT);
        assert_eq!(control >> 24, u32::from(slot));
        assert_eq!(control >> 16 & 31, 1);
        put(at + 8, STALL << 24);
    }
}
mod bot_peer;
mod class_peers;
pub(crate) use class_peers::{fixture_forwarding, fixture_hid};
mod published_resource_tests;
mod raw_boundary_tests;
