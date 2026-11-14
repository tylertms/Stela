use super::super::SHORT_READ;
use super::*;
use crate::bus::dma::RegionAllocator;
use crate::usb::storage::media::scan_bot;
use alloc::{rc::Rc, vec};
use core::cell::RefCell;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Fault {
    None,
    Minimum,
    ShortCbw,
    ShortData,
    RecoveredData,
    RecoveredShortData,
    AbortReadThenWrongTag,
    DataStall,
    CswStall,
    CswZero,
    CswZeroThenStall,
    SkippedData,
    WrongTag,
    BadSignature,
    BadResidue,
    Phase,
    Timeout,
    Reset,
    ClearInput,
    ClearOutput,
}
#[derive(Clone)]
struct Unit {
    media: Vec<u8>,
    sense: u32,
    readonly: bool,
    attention: bool,
    huge: bool,
    descriptor_sense: bool,
    /// Spun down until START STOP UNIT starts it.
    stopped: bool,
}
struct Command {
    lun: u8,
    cdb: Vec<u8>,
    tag: u32,
    length: usize,
    input: bool,
    payload: Vec<u8>,
    status: u8,
    data_done: bool,
    /// Data bytes moved so far, over one transfer or several.
    moved: usize,
}
#[derive(Debug, PartialEq, Eq)]
enum Order {
    Reset,
    Delay(u64),
    Minimum(u64),
    Clear(u8),
    Cbw(u8),
}
struct State {
    range: core::ops::Range<u64>,
    max: Result<Vec<u8>, BlockError>,
    units: Vec<Option<Unit>>,
    command: Option<Command>,
    history: Vec<(u8, u32, Vec<u8>)>,
    controls: Vec<[u8; 8]>,
    wrappers: Vec<usize>,
    clears: Vec<u8>,
    fault: Fault,
    override_payload: Vec<(u8, Vec<u8>)>,
    poisoned: bool,
    transfers: usize,
    /// If enabled, each REQUEST SENSE transfer waits on its first poll.
    sense_polls: Option<usize>,
    delays: usize,
    now: u64,
    reset_ready: Option<u64>,
    reset_minimum: bool,
    order: Vec<Order>,
    /// Whether the model's device sits on a port [`Io::reset_device`]
    /// can reset, and how many resets it ran.
    port_reset: bool,
    port_resets: usize,
    /// Whether the model's timer keeps minimum intervals a caller polls.
    intervals: bool,
}
#[derive(Clone)]
struct Model(Rc<RefCell<State>>);
/// A minimum interval of the model's timer, which proves the reset's
/// settle as a waited minimum does once its time has passed.
struct ModelInterval(Rc<RefCell<State>>, u64);
impl crate::bus::pci::power::Interval for ModelInterval {
    fn elapsed(&self) -> Result<bool, crate::bus::pci::power::Error> {
        let mut state = self.0.borrow_mut();
        let elapsed = state.now >= self.1;
        state.reset_minimum |= elapsed;
        Ok(elapsed)
    }
}
impl Io for Model {
    type Buffer = DmaBuffer;

    /// A port reset: the device drops whatever wedged it, and each unit
    /// reports the reset once, as a device's power-on UNIT ATTENTION.
    fn reset_device(
        bot: &mut Bot<Self>,
        _dma: &mut dyn DmaAllocator,
    ) -> Option<Result<Bot<Self>, (alloc::string::String, BlockError)>> {
        let state = bot.io.0.clone();
        {
            let mut s = state.borrow_mut();
            if !s.port_reset {
                return None;
            }
            s.port_resets += 1;
            s.poisoned = false;
            s.fault = Fault::None;
            s.command = None;
            for unit in s.units.iter_mut().flatten() {
                unit.attention = true;
            }
        }
        let (interface, flags, max_lun) = (bot.interface, bot.flags, bot.max_lun);
        Some(Ok(bot.moved_to(Model(state), interface, flags, max_lun)))
    }
    /// The model has no device to isolate.
    fn quarantine(&mut self) {}
    fn ready(&mut self) -> Result<(), BlockError> {
        if self.0.borrow().poisoned {
            Err(BlockError::Unsupported("model unacknowledged cancellation"))
        } else {
            Ok(())
        }
    }
    fn control(&mut self, request: [u8; 8]) -> Result<Vec<u8>, BlockError> {
        let mut s = self.0.borrow_mut();
        s.controls.push(request);
        match request[1] {
            GET_MAX_LUN => {
                assert_eq!(request, setup(0xa1, GET_MAX_LUN, 0, 3, 1));
                s.max.clone()
            }
            RESET => {
                assert_eq!(request, setup(0x21, RESET, 0, 3, 0));
                if s.fault == Fault::Reset {
                    Err(BlockError::Timeout)
                } else {
                    s.command = None;
                    s.reset_ready = Some(s.now + RESET_RECOVERY_US);
                    s.reset_minimum = false;
                    s.order.push(Order::Reset);
                    Ok(Vec::new())
                }
            }
            _ => panic!("unexpected control"),
        }
    }
    fn clear(&mut self, endpoint: Endpoint) -> Result<(), BlockError> {
        let mut s = self.0.borrow_mut();
        assert!(
            s.reset_ready
                .is_none_or(|at| s.now >= at && s.reset_minimum),
            "halt clear before reset settled"
        );
        s.order.push(Order::Clear(endpoint.address));
        s.clears.push(endpoint.address);
        if (s.fault == Fault::ClearInput && endpoint.address == 0x82)
            || (s.fault == Fault::ClearOutput && endpoint.address == 0x02)
        {
            Err(BlockError::Timeout)
        } else {
            Ok(())
        }
    }
    fn delay(&self, micros: u64) {
        assert!(matches!(micros, 125 | 100_000 | RESET_RECOVERY_US));
        let mut s = self.0.borrow_mut();
        s.now += micros;
        s.order.push(Order::Delay(micros));
        if micros == 100_000 {
            s.delays += 1;
        }
    }
    fn minimum_wait(&mut self, micros: u64) -> Result<(), BlockError> {
        let mut state = self.0.borrow_mut();
        state.order.push(Order::Minimum(micros));
        if state.fault == Fault::Minimum {
            return Err(BlockError::PciPower(
                crate::bus::pci::power::Error::Interrupted,
            ));
        }
        state.now += micros;
        if micros == RESET_RECOVERY_US {
            state.reset_minimum = true;
        }
        if micros == 100_000 {
            state.delays += 1;
        }
        Ok(())
    }
    fn begin_minimum(
        &mut self,
        micros: u64,
    ) -> Result<Option<Box<dyn crate::bus::pci::power::Interval>>, BlockError> {
        let state = self.0.borrow();
        Ok(state.intervals.then(|| {
            Box::new(ModelInterval(self.0.clone(), state.now + micros))
                as Box<dyn crate::bus::pci::power::Interval>
        }))
    }
    fn now_us(&self) -> u64 {
        self.0.borrow().now
    }
    fn reap(&mut self, pending: &mut Pending) -> Option<Result<usize, BlockError>> {
        let mut state = self.0.borrow_mut();
        if state
            .history
            .last()
            .is_some_and(|(_, _, cdb)| cdb[0] == scsi::REQUEST_SENSE)
            && let Some(polls) = &mut state.sense_polls
        {
            *polls += 1;
            if !polls.is_multiple_of(2) {
                return None;
            }
        }
        let Pending::Done(result) = pending else {
            unreachable!("BOT model has no transfer tickets");
        };
        Some(result.clone())
    }
    fn bulk(
        &mut self,
        endpoint: Endpoint,
        buffer: &mut DmaBuffer,
        length: usize,
    ) -> Result<usize, BlockError> {
        let address = buffer.physical();
        let mut s = self.0.borrow_mut();
        s.transfers += 1;
        assert!(
            address >= s.range.start && address.checked_add(length as u64).unwrap() <= s.range.end
        );
        // SAFETY: the test's private DMA allocation is live, within range,
        // and only this synchronous model accesses it during a transfer.
        let bytes = unsafe { core::slice::from_raw_parts_mut(address as *mut u8, length) };
        if s.command.is_none() {
            assert!(!endpoint.is_in());
            assert!(matches!(length, 31 | 32));
            if length == 32 {
                assert_eq!(bytes[31], 0);
            }
            s.wrappers.push(length);
            assert_eq!(
                u32::from_le_bytes(bytes[..4].try_into().unwrap()),
                CBW_SIGNATURE
            );
            assert_eq!(bytes[12] & 0x7f, 0);
            assert_eq!(bytes[13] & 0xf0, 0);
            assert!((1..=16).contains(&bytes[14]));
            let lun = bytes[13];
            assert!(
                s.reset_ready
                    .is_none_or(|at| s.now >= at && s.reset_minimum),
                "CBW before reset settled"
            );
            s.order.push(Order::Cbw(lun));
            let tag = u32::from_le_bytes(bytes[4..8].try_into().unwrap());
            let cdb = bytes[15..15 + usize::from(bytes[14])].to_vec();
            let requested = u32::from_le_bytes(bytes[8..12].try_into().unwrap()) as usize;
            let input = bytes[12] & 0x80 != 0;
            s.history.push((lun, tag, cdb.clone()));
            let mut payload = Vec::new();
            let mut status = 0;
            let fault = s.fault;
            let unit = s.units.get_mut(usize::from(lun)).and_then(Option::as_mut);
            if let Some(u) = unit {
                match cdb[0] {
                    0x12 => {
                        payload = vec![0; 36];
                        payload[1] = 0x80;
                        payload[4] = 31;
                        payload[8..16].copy_from_slice(b"STELA   ");
                        payload[16..32].copy_from_slice(b"MULTI LUN MODEL ");
                    }
                    op if u.attention && !matches!(op, 0x12 | 0x03) => {
                        u.attention = false;
                        u.sense = 0x062900;
                        status = 1;
                    }
                    0x00 if u.stopped => {
                        // NOT READY: initializing command required.
                        u.sense = 0x020402;
                        status = 1;
                    }
                    0x00 => {}
                    0x1b => u.stopped = cdb[4] & 1 == 0,
                    0x03 => {
                        payload = vec![0; 18];
                        if u.descriptor_sense {
                            payload[0] = 0x72;
                            payload[1] = (u.sense >> 16) as u8;
                            payload[2] = (u.sense >> 8) as u8;
                            payload[3] = u.sense as u8;
                        } else {
                            payload[0] = 0x70;
                            payload[2] = (u.sense >> 16) as u8;
                            payload[7] = 10;
                            payload[12] = (u.sense >> 8) as u8;
                            payload[13] = u.sense as u8;
                        }
                    }
                    0x25 => {
                        payload = vec![0; 8];
                        let last = if u.huge {
                            u32::MAX
                        } else {
                            (u.media.len() / 512 - 1) as u32
                        };
                        payload[..4].copy_from_slice(&last.to_be_bytes());
                        payload[4..8].copy_from_slice(&512u32.to_be_bytes());
                    }
                    0x9e => {
                        payload = vec![0; 32];
                        payload[..8].copy_from_slice(&((1u64 << 32) + 7).to_be_bytes());
                        payload[8..12].copy_from_slice(&4096u32.to_be_bytes());
                    }
                    0x1a => {
                        payload = vec![3, 0, if u.readonly { 0x80 } else { 0 }, 0];
                    }
                    0x28 | 0x88 => {
                        let lba = if cdb[0] == 0x28 {
                            u64::from(u32::from_be_bytes(cdb[2..6].try_into().unwrap()))
                        } else {
                            u64::from_be_bytes(cdb[2..10].try_into().unwrap())
                        };
                        let at = lba as usize * 512;
                        payload = u.media[at..at + requested].to_vec();
                        if matches!(fault, Fault::RecoveredData | Fault::RecoveredShortData) {
                            if fault == Fault::RecoveredShortData {
                                payload.pop();
                            }
                            status = CSW_FAILED;
                            u.sense = 0x011701;
                        } else if fault == Fault::AbortReadThenWrongTag {
                            status = CSW_FAILED;
                            u.sense = 0x0b0000;
                        }
                    }
                    0x2a | 0x8a | 0x35 => {}
                    _ => {
                        u.sense = 0x052000;
                        status = 1;
                    }
                }
            } else {
                status = 1;
                if cdb[0] == 0x12 {
                    // Empty reader slots identify normally, but TUR fails.
                    payload = vec![0; 36];
                    payload[1] = 0x80;
                    payload[4] = 31;
                    status = 0;
                } else if cdb[0] == 3 {
                    payload = vec![0; 18];
                    payload[0] = 0x70;
                    payload[2] = 2;
                    payload[7] = 10;
                    payload[12] = 0x3a;
                    status = 0;
                }
            }
            if let Some((_, bytes)) = s.override_payload.iter().find(|(op, _)| *op == cdb[0]) {
                payload = bytes.clone();
            }
            s.command = Some(Command {
                lun,
                cdb,
                tag,
                length: requested,
                input,
                payload,
                status,
                data_done: requested == 0,
                moved: 0,
            });
            if s.fault == Fault::ShortCbw {
                return Ok(30);
            }
            return Ok(length);
        }
        if s.fault == Fault::Timeout {
            s.poisoned = true;
            return Err(BlockError::Timeout);
        }
        let mut command = s.command.take().unwrap();
        if !command.data_done {
            assert_eq!(command.input, endpoint.is_in());
            // A data stage moves in transfers of whole packets but its
            // last, up to the command's length.
            let remaining = command.length - command.moved;
            assert!(length == remaining || length < remaining && length.is_multiple_of(512));
            command.data_done = true;
            if s.fault == Fault::SkippedData {
                assert!(command.input && length > 13);
                bytes[..13].fill(0);
                bytes[..4].copy_from_slice(&CSW_SIGNATURE.to_le_bytes());
                bytes[4..8].copy_from_slice(&command.tag.to_le_bytes());
                bytes[8..12].copy_from_slice(&(command.length as u32).to_le_bytes());
                s.fault = Fault::None;
                return Ok(13);
            }
            if s.fault == Fault::DataStall {
                s.command = Some(command);
                return Err(BlockError::Device {
                    status: super::super::STALL,
                });
            }
            let count = if s.fault == Fault::ShortData {
                length - 1
            } else if command.input {
                length.min(command.payload.len().saturating_sub(command.moved))
            } else {
                length
            };
            let moved = command.moved;
            if command.input {
                bytes[..count].copy_from_slice(&command.payload[moved..moved + count]);
            } else if command.status == 0 && matches!(command.cdb[0], 0x2a | 0x8a) {
                let at = moved
                    + if command.cdb[0] == 0x2a {
                        u32::from_be_bytes(command.cdb[2..6].try_into().unwrap()) as usize * 512
                    } else {
                        u64::from_be_bytes(command.cdb[2..10].try_into().unwrap()) as usize * 512
                    };
                s.units[usize::from(command.lun)].as_mut().unwrap().media[at..at + count]
                    .copy_from_slice(&bytes[..count]);
            }
            command.moved += count;
            // A short transfer ends the stage early.
            command.data_done = command.moved == command.length || count < length;
            s.command = Some(command);
            return Ok(count);
        }
        assert!(endpoint.is_in());
        assert_eq!(length, 13);
        if matches!(
            s.fault,
            Fault::CswStall | Fault::CswZero | Fault::CswZeroThenStall
        ) {
            let fault = s.fault;
            s.fault = if fault == Fault::CswZeroThenStall {
                Fault::CswStall
            } else {
                Fault::None
            };
            s.command = Some(command);
            return if matches!(fault, Fault::CswZero | Fault::CswZeroThenStall) {
                Ok(0)
            } else {
                Err(BlockError::Device {
                    status: super::super::STALL,
                })
            };
        }
        bytes.fill(0);
        bytes[..4].copy_from_slice(
            &if s.fault == Fault::BadSignature {
                0
            } else {
                CSW_SIGNATURE
            }
            .to_le_bytes(),
        );
        bytes[4..8]
            .copy_from_slice(&(command.tag + u32::from(s.fault == Fault::WrongTag)).to_le_bytes());
        bytes[8..12].copy_from_slice(
            &if s.fault == Fault::BadResidue {
                command.length as u32 + 1
            } else {
                0
            }
            .to_le_bytes(),
        );
        bytes[12] = if s.fault == Fault::Phase
            || matches!(
                s.fault,
                Fault::Reset | Fault::ClearInput | Fault::ClearOutput
            ) {
            2
        } else {
            command.status
        };
        if s.fault == Fault::AbortReadThenWrongTag && command.cdb[0] == scsi::REQUEST_SENSE {
            s.fault = Fault::WrongTag;
        }
        Ok(13)
    }
}
fn endpoints() -> [Endpoint; 2] {
    [0x82, 0x02].map(|address| Endpoint {
        address,
        attributes: 2,
        max_packet: 64,
        interval: 0,
        companion: None,
    })
}
fn unit(byte: u8) -> Option<Unit> {
    Some(Unit {
        media: vec![byte; 8192],
        sense: 0,
        readonly: false,
        attention: false,
        huge: false,
        descriptor_sense: false,
        stopped: false,
    })
}
fn model(start: u64, end: u64, units: Vec<Option<Unit>>) -> Model {
    Model(Rc::new(RefCell::new(State {
        range: start..end,
        max: Ok(vec![(units.len() - 1) as u8]),
        units,
        command: None,
        history: Vec::new(),
        controls: Vec::new(),
        wrappers: Vec::new(),
        clears: Vec::new(),
        fault: Fault::None,
        override_payload: Vec::new(),
        poisoned: false,
        transfers: 0,
        sense_polls: None,
        delays: 0,
        now: 0,
        reset_ready: None,
        reset_minimum: false,
        order: Vec::new(),
        port_reset: false,
        port_resets: 0,
        intervals: false,
    })))
}
/// The begun command's result, from calls of [`Bot::end`], each of which
/// moves it by one transfer at most.
fn ended<I: Io>(bot: &mut Bot<I>) -> Result<Vec<u8>, BlockError> {
    for _ in 0..16 {
        if let Some(result) = bot.end() {
            return result;
        }
    }
    panic!("a BOT command did not end in 16 calls");
}
fn dma(backing: &mut [u64]) -> RegionAllocator {
    let start = backing.as_mut_ptr() as u64;
    // SAFETY: each caller keeps the exclusive backing alive until all Bot
    // owners and the synchronous private model are dropped.
    unsafe { RegionAllocator::new(start..start + core::mem::size_of_val(backing) as u64) }
}
struct NativeFixture {
    storage: super::super::UsbStorage,
    disk: super::super::UsbDisk,
    hardware: Rc<RefCell<crate::usb::xhci::model::ResetHardware>>,
    dma: RegionAllocator,
}
fn native_fixture(
    memory: &mut [u64],
    speed: u8,
    context: usize,
    write_cache: bool,
) -> NativeFixture {
    let (controller, device, hardware, mut dma) =
        crate::usb::xhci::model::reset_bot_fixture(memory, speed, context, 3, write_cache);
    let identity = &device.descriptor[8..12];
    assert_eq!(
        identity,
        if write_cache {
            &[0xc2, 0x0b, 0x00, 0x23]
        } else {
            &[0xf4, 0x46, 0x03, 0x00]
        },
    );
    assert_eq!(
        quirks::Flags::from_device(&device).unwrap().write_cache(),
        write_cache
    );
    let interface = crate::usb::descriptor::interfaces(&device.configuration).remove(0);
    let endpoints = [true, false].map(|input| {
        *interface
            .endpoints
            .iter()
            .find(|endpoint| endpoint.is_in() == input)
            .unwrap()
    });
    controller
        .borrow_mut()
        .device(device.id)
        .unwrap()
        .configure(&endpoints, None, &mut dma)
        .unwrap();
    let mut failures = Vec::new();
    let (storage, mut disks) = super::super::discover_retained(
        controller,
        &device,
        &interface,
        endpoints,
        &mut dma,
        &mut failures,
    )
    .unwrap();
    assert!(failures.is_empty());
    assert_eq!(disks.len(), 1);
    let super::super::UsbTransport::Bot(disk) = &disks[0].disk else {
        panic!("native BOT fixture published another transport")
    };
    assert_eq!(disk.identity.features.write_cache, write_cache);
    NativeFixture {
        storage,
        disk: disks.remove(0),
        hardware,
        dma,
    }
}
#[test]
fn native_root_bot_reuses_original_transport_buffers_across_96_resets_and_real_io() {
    use crate::block::BlockDevice;
    use crate::usb::xhci::{HIGH_SPEED, SUPER_SPEED};
    // The unquirked profile has no flush command. The pinned
    // WRITE_CACHE identity requires a real acknowledged 0x35 every epoch.
    for write_cache in [false, true] {
        for speed in [HIGH_SPEED, SUPER_SPEED] {
            for context in [32, 64] {
                let mut memory = vec![0u64; 512 * 1024];
                let NativeFixture {
                    mut storage,
                    mut disk,
                    hardware,
                    mut dma,
                } = native_fixture(&mut memory, speed, context, write_cache);
                let super::super::media::TransportOwner::Bot(owner) = &storage.owner else {
                    panic!("native BOT fixture published another transport")
                };
                let original = {
                    let owner = owner.borrow();
                    (
                        owner.transport.buffer.physical(),
                        owner.transport.buffer.len(),
                        owner.transport.wrapper.physical(),
                        owner.transport.wrapper.len(),
                    )
                };
                assert_eq!(original.1, MAX_TD);
                assert_eq!(original.3, 64);
                let remaining = dma.remaining();
                let mut retired = Vec::new();
                for epoch in 1..=96 {
                    let old_owner = match &storage.owner {
                        super::super::media::TransportOwner::Bot(owner) => owner.clone(),
                        _ => panic!("native BOT recovery changed transport"),
                    };
                    let mut failures = Vec::new();
                    let mut fresh = storage.reset_and_rebind(&mut dma, &mut failures).unwrap();
                    assert!(failures.is_empty());
                    assert_eq!(fresh.len(), 1);
                    assert_eq!(dma.remaining(), remaining, "reset allocated new DMA");
                    assert_eq!(hardware.borrow().resets, epoch);
                    {
                        let old = old_owner.borrow();
                        assert!(old.transport.poisoned());
                        assert!(!old.transport.buffer.present());
                        assert!(!old.transport.wrapper.present());
                    }
                    let super::super::media::TransportOwner::Bot(owner) = &storage.owner else {
                        panic!("native BOT recovery changed transport")
                    };
                    {
                        let owner = owner.borrow();
                        assert_eq!(
                            (
                                owner.transport.buffer.physical(),
                                owner.transport.buffer.len(),
                                owner.transport.wrapper.physical(),
                                owner.transport.wrapper.len(),
                            ),
                            original,
                        );
                    }
                    let current_bytes = {
                        let owner = owner.borrow();
                        (
                            owner.transport.buffer.bytes().to_vec(),
                            owner.transport.wrapper.bytes().to_vec(),
                        )
                    };
                    retired.push((disk, old_owner));
                    let accesses = hardware.borrow().accesses;
                    for (old_disk, old_owner) in &mut retired {
                        let mut poison = [0xb7; 512];
                        assert!(old_disk.read(7, &mut poison).is_err());
                        assert_eq!(poison, [0xb7; 512]);
                        assert!(old_disk.write(7, &[0xee; 512]).is_err());
                        assert!(old_disk.flush_cache().is_err());
                        let mut old = old_owner.borrow_mut();
                        assert!(old.transport.io.ready().is_err());
                        assert!(
                            old.transport
                                .command(0, &scsi::inquiry(), Data::In(36))
                                .is_err()
                        );
                        assert!(old.transport.reset_and_rebind(&mut dma).is_err());
                    }
                    assert_eq!(hardware.borrow().accesses, accesses);
                    assert_eq!(dma.remaining(), remaining);
                    {
                        let owner = owner.borrow();
                        assert_eq!(owner.transport.buffer.bytes(), current_bytes.0);
                        assert_eq!(owner.transport.wrapper.bytes(), current_bytes.1);
                    }
                    disk = fresh.remove(0);
                    let byte = epoch as u8;
                    disk.write(7, &[byte; 512]).unwrap();
                    disk.flush().unwrap();
                    let mut actual = [0; 512];
                    disk.read(7, &mut actual).unwrap();
                    assert_eq!(actual, [byte; 512]);
                    for opcode in [scsi::READ_10, scsi::WRITE_10] {
                        assert!(
                            hardware.borrow().scsi.contains(&(epoch, opcode)),
                            "missing CDB {opcode:#x}: cache={write_cache}, speed={speed}, context={context}, epoch={epoch}",
                        );
                    }
                    assert_eq!(
                        hardware
                            .borrow()
                            .scsi
                            .iter()
                            .filter(|&&(seen_epoch, opcode)| {
                                seen_epoch == epoch && opcode == scsi::SYNCHRONIZE_CACHE_10
                            })
                            .count(),
                        usize::from(write_cache),
                        "flush CDB: cache={write_cache}, speed={speed}, context={context}, epoch={epoch}",
                    );
                    assert_eq!(dma.remaining(), remaining);
                }
                assert_eq!(retired.len(), 96);
            }
        }
    }
}
#[test]
fn native_bot_failed_reset_keeps_both_original_buffer_owners_and_bytes() {
    use crate::block::BlockDevice;
    use crate::usb::xhci::{HIGH_SPEED, SUPER_SPEED, model::ResetFault};
    for speed in [HIGH_SPEED, SUPER_SPEED] {
        for context in [32, 64] {
            for fault in [
                ResetFault::Stop(2),
                ResetFault::Disable,
                ResetFault::NoResetChange,
                ResetFault::Identity,
                ResetFault::ConfigureAfterReset,
                ResetFault::FatalAfterReset,
                ResetFault::ChangeAfterReset,
            ] {
                let mut memory = vec![0u64; 512 * 1024];
                let NativeFixture {
                    mut storage,
                    mut disk,
                    hardware,
                    mut dma,
                } = native_fixture(&mut memory, speed, context, false);
                let owner = match &storage.owner {
                    super::super::media::TransportOwner::Bot(owner) => owner.clone(),
                    _ => panic!("native BOT fixture published another transport"),
                };
                let before = {
                    let mut owner = owner.borrow_mut();
                    owner.transport.buffer.bytes_mut().fill(0xa5);
                    owner.transport.wrapper.bytes_mut().fill(0x5a);
                    (
                        owner.transport.buffer.physical(),
                        owner.transport.buffer.bytes().to_vec(),
                        owner.transport.wrapper.physical(),
                        owner.transport.wrapper.bytes().to_vec(),
                    )
                };
                hardware.borrow_mut().fault = fault;
                let mut failures = Vec::new();
                assert!(storage.reset_and_rebind(&mut dma, &mut failures).is_err());
                assert!(!failures.is_empty());
                {
                    let owner = owner.borrow();
                    assert!(owner.transport.poisoned());
                    assert!(owner.transport.buffer.present());
                    assert!(owner.transport.wrapper.present());
                    assert_eq!(owner.transport.buffer.physical(), before.0);
                    assert_eq!(owner.transport.buffer.bytes(), before.1);
                    assert_eq!(owner.transport.wrapper.physical(), before.2);
                    assert_eq!(owner.transport.wrapper.bytes(), before.3);
                }
                let accesses = hardware.borrow().accesses;
                let mut poison = [0xc3; 512];
                assert!(disk.read(0, &mut poison).is_err());
                assert_eq!(poison, [0xc3; 512]);
                assert!(disk.write(0, &[0x11; 512]).is_err());
                assert!(disk.flush_cache().is_err());
                assert!(
                    owner
                        .borrow_mut()
                        .transport
                        .command(0, &scsi::inquiry(), Data::In(36))
                        .is_err()
                );
                assert_eq!(hardware.borrow().accesses, accesses);
                let owner = owner.borrow();
                assert_eq!(owner.transport.buffer.bytes(), before.1);
                assert_eq!(owner.transport.wrapper.bytes(), before.3);
            }
        }
    }
}
#[test]
fn all_sixteen_luns_have_independent_media_sense_and_shared_tags() {
    let mut memory = vec![0u64; 32 * 1024];
    let mut dma = dma(&mut memory);
    let start = memory.as_mut_ptr() as u64;
    let io = model(
        start,
        start + memory.len() as u64 * 8,
        (0..16).map(unit).collect(),
    );
    let state = io.0.clone();
    let mut bot = Bot::new(io, 3, endpoints(), &mut dma).unwrap();
    for lun in 0..=bot.max_lun() {
        let (identity, _) = super::super::probe(&mut bot, lun, "slot", "serial").unwrap();
        assert_eq!(identity.blocks, 16);
        assert_eq!(
            identity.serial,
            if lun == 0 {
                "serial".into()
            } else {
                alloc::format!("serial:lun{lun}")
            }
        );
        let read = bot
            .command(lun, &[0x28, 0, 0, 0, 0, 0, 0, 0, 1, 0], Data::In(512))
            .unwrap();
        assert_eq!(read, vec![lun; 512]);
        let bytes = vec![lun.wrapping_add(40); 512];
        bot.command(lun, &[0x2a, 0, 0, 0, 0, 1, 0, 0, 1, 0], Data::Out(&bytes))
            .unwrap();
        assert_eq!(
            &state.borrow().units[usize::from(lun)]
                .as_ref()
                .unwrap()
                .media[512..1024],
            &bytes
        );
        bot.command(lun, &[0x35, 0, 0, 0, 0, 0, 0, 0, 0, 0], Data::None)
            .unwrap();
    }
    let state = state.borrow();
    assert!(state.history.windows(2).all(|w| w[1].1 == w[0].1 + 1));
    // Probe reads block zero before MODE SENSE. Check every CDB,
    // rather than only a total that could hide a missing read or a command
    // sent to the wrong LUN after the shared transport's tags advance.
    let expected: [&[u8]; 8] = [
        &[0x12, 0, 0, 0, 36, 0],
        &[0x00, 0, 0, 0, 0, 0],
        &[0x25, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        &[0x28, 0, 0, 0, 0, 0, 0, 0, 1, 0],
        &[0x1a, 0, 0x3f, 0, 192, 0],
        &[0x28, 0, 0, 0, 0, 0, 0, 0, 1, 0],
        &[0x2a, 0, 0, 0, 0, 1, 0, 0, 1, 0],
        &[0x35, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    ];
    assert_eq!(state.history.len(), 16 * expected.len());
    for (lun, commands) in state.history.chunks_exact(expected.len()).enumerate() {
        for ((actual_lun, _, cdb), expected_cdb) in commands.iter().zip(expected) {
            assert_eq!(usize::from(*actual_lun), lun);
            assert_eq!(cdb.as_slice(), expected_cdb);
        }
    }
}
#[test]
fn empty_slot_does_not_hide_later_ready_media_and_auto_sense_keeps_lun() {
    let mut memory = vec![0u64; 32 * 1024];
    let start = memory.as_mut_ptr() as u64;
    let mut dma = dma(&mut memory);
    let io = model(
        start,
        start + memory.len() as u64 * 8,
        vec![None, unit(1), None, unit(3)],
    );
    let state = io.0.clone();
    let mut bot = Bot::new(io, 3, endpoints(), &mut dma).unwrap();
    for lun in 0..4 {
        let result = super::super::probe(&mut bot, lun, "slot", "serial");
        assert_eq!(result.is_ok(), lun == 1 || lun == 3);
        assert!(!bot.poisoned());
    }
    let state = state.borrow();
    for w in state.history.windows(2) {
        if w[1].2[0] == 3 {
            assert_eq!(w[0].0, w[1].0);
        }
    }
}
#[test]
fn attention_descriptor_sense_capacity16_and_write_protection_are_real() {
    for descriptor in [false, true] {
        let mut memory = vec![0u64; 32 * 1024];
        let start = memory.as_mut_ptr() as u64;
        let mut dma = dma(&mut memory);
        let mut u = unit(7).unwrap();
        u.attention = true;
        u.descriptor_sense = descriptor;
        u.huge = true;
        u.media.resize(4096, 7);
        u.readonly = true;
        let io = model(start, start + memory.len() as u64 * 8, vec![Some(u)]);
        let state = io.0.clone();
        let mut bot = Bot::new(io, 3, endpoints(), &mut dma).unwrap();
        let (identity, _) = super::super::probe(&mut bot, 0, "slot", "serial").unwrap();
        assert_eq!(identity.blocks, (1 << 32) + 8);
        assert_eq!(identity.block_size, 4096);
        assert!(identity.read_only);
        assert_eq!(state.borrow().delays, 1);
    }
}
#[test]
fn a_stopped_unit_is_started_once_and_then_identified() {
    let mut memory = vec![0u64; 32 * 1024];
    let start = memory.as_mut_ptr() as u64;
    let mut dma = dma(&mut memory);
    let mut stopped = unit(0x11).unwrap();
    stopped.stopped = true;
    let io = model(start, start + memory.len() as u64 * 8, vec![Some(stopped)]);
    let state = io.0.clone();
    let mut bot = Bot::new(io, 3, endpoints(), &mut dma).unwrap();
    super::super::probe(&mut bot, 0, "slot", "serial").unwrap();
    assert!(!state.borrow().units[0].as_ref().unwrap().stopped);
}

#[test]
fn get_max_lun_takes_one_bounded_byte_and_otherwise_one_unit() {
    for (response, max_lun) in [
        (Ok(vec![0]), 0),
        (Ok(vec![15]), 15),
        (Ok(vec![]), 0),
        (Ok(vec![16]), 0),
        (Ok(vec![0, 1]), 0),
        (
            Err(BlockError::Device {
                status: super::super::STALL,
            }),
            0,
        ),
        (Err(BlockError::Timeout), 0),
    ] {
        let mut memory = vec![0u64; 32 * 1024];
        let start = memory.as_mut_ptr() as u64;
        let mut dma = dma(&mut memory);
        let io = model(start, start + memory.len() as u64 * 8, vec![unit(0)]);
        io.0.borrow_mut().max = response;
        assert_eq!(
            Bot::new(io, 3, endpoints(), &mut dma).unwrap().max_lun(),
            max_lun
        );
    }
}
#[test]
fn short_cbw_phase_csw_and_residue_errors_require_interface_reset() {
    for fault in [
        Fault::ShortCbw,
        Fault::WrongTag,
        Fault::BadSignature,
        Fault::BadResidue,
        Fault::Phase,
    ] {
        let mut memory = vec![0u64; 32 * 1024];
        let start = memory.as_mut_ptr() as u64;
        let mut dma = dma(&mut memory);
        let io = model(start, start + memory.len() as u64 * 8, vec![unit(0)]);
        let state = io.0.clone();
        let mut bot = Bot::new(io, 3, endpoints(), &mut dma).unwrap();
        state.borrow_mut().fault = fault;
        assert!(bot.command(0, &[0; 6], Data::None).is_err());
        state.borrow_mut().now += RESET_RECOVERY_US;
        assert!(!bot.busy());
        let state = state.borrow();
        assert_eq!(state.controls.last().unwrap()[1], RESET);
        assert_eq!(state.clears, vec![0x82, 0x02]);
        assert!(!bot.poisoned());
    }
}
#[test]
fn data_stall_reads_csw_and_csw_stall_or_zero_gets_one_retry() {
    for fault in [Fault::DataStall, Fault::CswStall, Fault::CswZero] {
        let mut memory = vec![0u64; 32 * 1024];
        let start = memory.as_mut_ptr() as u64;
        let mut dma = dma(&mut memory);
        let io = model(start, start + memory.len() as u64 * 8, vec![unit(0)]);
        let state = io.0.clone();
        let mut bot = Bot::new(io, 3, endpoints(), &mut dma).unwrap();
        state.borrow_mut().fault = fault;
        let result = bot
            .command(0, &[0x12, 0, 0, 0, 36, 0], Data::In(36))
            .unwrap();
        assert_eq!(result.len(), if fault == Fault::DataStall { 0 } else { 36 });
        assert!(!bot.poisoned());
    }
}
#[test]
fn short_out_cannot_be_reported_as_successful_write() {
    let mut memory = vec![0u64; 32 * 1024];
    let start = memory.as_mut_ptr() as u64;
    let mut dma = dma(&mut memory);
    let io = model(start, start + memory.len() as u64 * 8, vec![unit(0)]);
    let state = io.0.clone();
    let mut bot = Bot::new(io, 3, endpoints(), &mut dma).unwrap();
    state.borrow_mut().fault = Fault::ShortData;
    assert_eq!(
        bot.command(
            0,
            &[0x2a, 0, 0, 0, 0, 0, 0, 0, 1, 0],
            Data::Out(&vec![2; 512])
        ),
        Err(SHORT_WRITE)
    );
}
#[test]
fn failed_reset_clear_or_cancellation_quarantines_every_lun_without_dma_rewrite() {
    for fault in [
        Fault::Reset,
        Fault::ClearInput,
        Fault::ClearOutput,
        Fault::Timeout,
    ] {
        let mut memory = vec![0u64; 32 * 1024];
        let start = memory.as_mut_ptr() as u64;
        let mut dma = dma(&mut memory);
        let io = model(
            start,
            start + memory.len() as u64 * 8,
            vec![unit(0), unit(1)],
        );
        let state = io.0.clone();
        let mut bot = Bot::new(io, 3, endpoints(), &mut dma).unwrap();
        state.borrow_mut().fault = fault;
        assert!(
            bot.command(0, &[0x12, 0, 0, 0, 36, 0], Data::In(36))
                .is_err()
        );
        // A failed halt clear shows once the reset has settled.
        state.borrow_mut().now += RESET_RECOVERY_US;
        assert!(!bot.busy());
        assert!(bot.poisoned());
        let wrapper = bot.wrapper.bytes().to_vec();
        let buffer = bot.buffer.bytes().to_vec();
        let transfers = state.borrow().transfers;
        assert!(
            bot.command(
                1,
                &[0x2a, 0, 0, 0, 0, 0, 0, 0, 1, 0],
                Data::Out(&vec![0xa5; 512])
            )
            .is_err()
        );
        assert_eq!(state.borrow().transfers, transfers);
        assert_eq!(bot.wrapper.bytes(), wrapper);
        assert_eq!(bot.buffer.bytes(), buffer);
    }
}
#[test]
fn invalid_lun_cdb_and_lengths_are_rejected_without_touching_dma() {
    let mut memory = vec![0u64; 32 * 1024];
    let start = memory.as_mut_ptr() as u64;
    let mut dma = dma(&mut memory);
    let io = model(start, start + memory.len() as u64 * 8, vec![unit(0)]);
    let state = io.0.clone();
    let mut bot = Bot::new(io, 3, endpoints(), &mut dma).unwrap();
    for (lun, cdb, length) in [
        (1, vec![0; 6], 0),
        (0, vec![], 0),
        (0, vec![0; 17], 0),
        (0, vec![0; 6], bot.max_transfer_bytes() + 1),
    ] {
        assert!(bot.command(lun, &cdb, Data::In(length)).is_err());
    }
    assert_eq!(state.borrow().transfers, 0);
    assert_eq!(bot.tag, 0);
}
#[test]
fn native_poison_preflight_preserves_all_buffers() {
    for (dci, halted) in [1, 4, 5]
        .into_iter()
        .flat_map(|dci| [false, true].map(move |halted| (dci, halted)))
    {
        let mut memory = vec![0u64; 64 * 1024];
        let mut dma = dma(&mut memory);
        let mut wrapper = dma.allocate(64, 64).unwrap();
        wrapper.bytes_mut().fill(0xa5);
        let mut buffer = dma.allocate(512, 512).unwrap();
        buffer.bytes_mut().fill(0x5a);
        let controller = super::super::super::xhci::model::controller_with_unresolved_td(
            &mut dma,
            wrapper.physical(),
            wrapper.len(),
            dci,
            halted,
        );
        let id = controller.borrow().test_device_id(1).unwrap();
        let mut bot = Bot {
            io: Native {
                controller,
                id,
                endpoints: endpoints(),
                command_us: scan_command_time(),
            },
            interface: 3,
            endpoints: endpoints(),
            buffer: TransportBuffer::new(buffer),
            wrapper: TransportBuffer::new(wrapper),
            tag: 7,
            max_lun: 1,
            poisoned: false,
            minimum_error: None,
            flags: quirks::Flags::default(),
            initial_read: false,
            attachment: None,
            reset_generation: 0,
            recovery: None,
            begun: None,
            failed_data: None,
            failure_status: None,
        };
        assert!(
            bot.command(1, &[0x2a, 0, 0, 0, 0, 0, 0, 0, 1, 0], Data::Out(&[1, 2, 3]))
                .is_err()
        );
        assert_eq!(bot.tag, 7);
        assert!(bot.wrapper.bytes().iter().all(|b| *b == 0xa5));
        assert!(bot.buffer.bytes().iter().all(|b| *b == 0x5a));
    }
}
#[test]
fn kept_reader_discovers_inserted_media_and_never_revives_removed_aliases() {
    use super::super::media::{Owner, scan_bot};
    use crate::block::BlockDevice;
    let mut memory = vec![0u64; 32 * 1024];
    let start = memory.as_mut_ptr() as u64;
    let io = model(start, start + memory.len() as u64 * 8, vec![None, None]);
    let state = io.0.clone();
    let mut dma = dma(&mut memory);
    let owner = Rc::new(RefCell::new(Owner::new(
        Bot::new(io, 3, endpoints(), &mut dma).unwrap(),
        "reader".into(),
        "serial".into(),
    )));
    let mut failures = Vec::new();
    assert!(scan_bot(owner.clone(), &mut failures).unwrap().is_empty());
    assert!(
        state
            .borrow()
            .history
            .iter()
            .all(|(_, _, cdb)| !matches!(cdb[0], 0x25 | 0x1a))
    );
    state.borrow_mut().units[0] = unit(0x31);
    let mut disks = scan_bot(owner.clone(), &mut failures).unwrap();
    assert_eq!(disks.len(), 1);
    let mut same = scan_bot(owner.clone(), &mut failures).unwrap();
    assert!(Rc::ptr_eq(&disks[0].media, &same[0].media));
    disks[0].write(0, &[0x72; 512]).unwrap();
    disks[0].flush().unwrap();
    let mut data = [0; 512];
    same[0].read(0, &mut data).unwrap();
    assert_eq!(data, [0x72; 512]);
    // A real failed data command returns UA. The target does not commit
    // its payload; the caller must never silently retry it on new media.
    state.borrow_mut().units[0].as_mut().unwrap().attention = true;
    let before = state.borrow().history.len();
    assert_eq!(
        disks[0].write(0, &[0xa4; 512]),
        Err(BlockError::Sense(Sense {
            key: 0x06,
            asc: 0x29,
            ascq: 0x00
        }))
    );
    assert_eq!(
        state.borrow().history[before..]
            .iter()
            .filter(|(_, _, c)| c[0] == 0x2a)
            .count(),
        1
    );
    let before = state.borrow().transfers;
    assert!(same[0].read(0, &mut data).is_err());
    assert!(same[0].read(0, &mut []).is_err());
    assert!(same[0].write(0, &[0; 512]).is_err());
    assert!(same[0].flush_cache().is_err());
    assert!(same[0].command(&[0; 6], super::super::Data::None).is_err());
    assert_eq!(state.borrow().transfers, before);
    let mut current = scan_bot(owner.clone(), &mut failures).unwrap();
    assert!(!Rc::ptr_eq(&current[0].media, &disks[0].media));
    current[0].read(0, &mut data).unwrap();
    assert_eq!(data, [0x72; 512], "failed write was replayed");
    state.borrow_mut().units[0] = None;
    data.fill(0x5a);
    assert_eq!(
        current[0].read(0, &mut data),
        Err(BlockError::Sense(Sense {
            key: 0x02,
            asc: 0x3a,
            ascq: 0x00
        }))
    );
    assert_eq!(data, [0x5a; 512]);
    assert!(scan_bot(owner.clone(), &mut failures).unwrap().is_empty());
    state.borrow_mut().units[0] = unit(0x99);
    let mut inserted = scan_bot(owner, &mut failures).unwrap();
    inserted[0].read(0, &mut data).unwrap();
    assert_eq!(data, [0x99; 512]);
    let before = state.borrow().transfers;
    assert!(current[0].read(0, &mut data).is_err());
    assert!(disks[0].flush_cache().is_err());
    assert_eq!(state.borrow().transfers, before);
    assert_eq!(
        state
            .borrow()
            .controls
            .iter()
            .filter(|c| c[1] == GET_MAX_LUN)
            .count(),
        1
    );
}

#[test]
fn rescan_revalidates_geometry_protection_and_quarantines_all_on_transport_poison() {
    use super::super::media::{Owner, scan_bot};
    use crate::block::BlockDevice;
    let mut memory = vec![0u64; 32 * 1024];
    let start = memory.as_mut_ptr() as u64;
    let io = model(
        start,
        start + memory.len() as u64 * 8,
        vec![unit(0x11), unit(0x22)],
    );
    let state = io.0.clone();
    let mut dma = dma(&mut memory);
    let owner = Rc::new(RefCell::new(Owner::new(
        Bot::new(io, 3, endpoints(), &mut dma).unwrap(),
        "reader".into(),
        "serial".into(),
    )));
    let mut failures = Vec::new();
    let mut old = scan_bot(owner.clone(), &mut failures).unwrap();
    state.borrow_mut().units[0]
        .as_mut()
        .unwrap()
        .media
        .resize(4096, 0x33);
    let mut fresh = scan_bot(owner.clone(), &mut failures).unwrap();
    assert_eq!(fresh[0].identity().blocks, 8);
    assert!(old[0].flush_cache().is_err());
    old[1].read(0, &mut [0; 512]).unwrap();
    state.borrow_mut().units[0].as_mut().unwrap().readonly = true;
    let protected = scan_bot(owner.clone(), &mut failures).unwrap();
    assert!(protected[0].identity().read_only);
    assert!(fresh[0].flush_cache().is_err());
    // Failed revalidation cannot preserve an older apparently usable view.
    state.borrow_mut().override_payload.push((0x25, vec![0; 7]));
    assert!(scan_bot(owner.clone(), &mut failures).unwrap().is_empty());
    assert!(old[1].flush_cache().is_err());
    state.borrow_mut().override_payload.clear();
    let mut ready = scan_bot(owner.clone(), &mut failures).unwrap();
    state.borrow_mut().fault = Fault::Timeout;
    assert!(ready[0].read(0, &mut [0; 512]).is_err());
    let before = state.borrow().transfers;
    assert!(ready[1].flush_cache().is_err());
    assert_eq!(state.borrow().transfers, before);
    assert!(scan_bot(owner, &mut failures).is_err());
}

#[test]
fn real_block_api_chunks_units_checks_bounds_and_never_copies_short_reads() {
    use crate::block::BlockDevice;
    let mut memory = vec![0u64; 32 * 1024];
    let start = memory.as_mut_ptr() as u64;
    let mut dma = dma(&mut memory);
    let mut first = unit(0x22).unwrap();
    first.media = vec![0x22; MAX_TD * 3];
    let mut second = unit(0x44).unwrap();
    second.readonly = true;
    let io = model(
        start,
        start + memory.len() as u64 * 8,
        vec![Some(first), Some(second)],
    );
    let state = io.0.clone();
    let bot = Rc::new(RefCell::new(
        Bot::new(io, 3, endpoints(), &mut dma).unwrap(),
    ));
    let mut disks = Vec::new();
    for lun in 0..2 {
        let (identity, removable) =
            super::super::probe(&mut *bot.borrow_mut(), lun, "slot", "serial").unwrap();
        disks.push(super::super::Disk {
            bot: bot.clone(),
            lun,
            identity: crate::block::Identity {
                features: crate::block::Features {
                    write_cache: true,
                    ..identity.features
                },
                ..identity
            },
            media: Rc::new(core::cell::Cell::new(true)),
            removable,
            running: None,
            next_tag: 0,
        });
    }
    let mut bytes = vec![0; MAX_TD * 2 + 512];
    disks[0].read(0, &mut bytes).unwrap();
    assert!(bytes.iter().all(|b| *b == 0x22));
    disks[0].write(1, &vec![0x66; MAX_TD + 512]).unwrap();
    disks[0].flush().unwrap();
    let before = state.borrow().transfers;
    assert_eq!(disks[1].write(0, &[0; 512]), Err(BlockError::ReadOnly));
    assert!(disks[0].read(u64::MAX, &mut [0; 512]).is_err());
    assert!(disks[0].read(0, &mut [0; 511]).is_err());
    assert_eq!(state.borrow().transfers, before);
    state.borrow_mut().fault = Fault::ShortData;
    let mut untouched = [0xa5; 512];
    assert!(disks[0].read(0, &mut untouched).is_err());
    assert_eq!(untouched, [0xa5; 512]);
    assert_eq!(
        scsi::read_write(false, false, u32::MAX as u64, 1).as_ref()[0],
        0x28
    );
    assert_eq!(
        scsi::read_write(false, false, u32::MAX as u64, 2).as_ref()[0],
        0x88
    );
    assert_eq!(
        scsi::read_write(true, false, 1u64 << 32, 1).as_ref()[0],
        0x8a
    );
}

#[test]
fn synchronous_read_copies_only_after_valid_status() {
    let mut memory = vec![0u64; 32 * 1024];
    let start = memory.as_mut_ptr() as u64;
    let end = start + core::mem::size_of_val(memory.as_slice()) as u64;
    let mut dma = dma(&mut memory);
    let mut disk = unit(0x52).unwrap();
    disk.media = vec![0x52; MAX_TD + 512];
    let io = model(start, end, vec![Some(disk)]);
    let state = io.0.clone();
    let mut bot = Bot::new(io, 3, endpoints(), &mut dma).unwrap();
    let cdb = scsi::read_write(false, false, 0, (MAX_TD / 512) as u32);
    let mut output = vec![0xa5; MAX_TD];
    bot.read_into(0, cdb.as_ref(), &mut output).unwrap();
    assert_eq!(state.borrow().transfers, 3);
    assert_eq!(output, vec![0x52; MAX_TD]);

    let length = MAX_TD + 512;
    let multi = scsi::read_write(false, false, 0, (length / 512) as u32);
    output.resize(length, 0xa5);
    bot.read_into(0, multi.as_ref(), &mut output).unwrap();
    assert!(output.iter().all(|&byte| byte == 0x52));
    // The model completes each transfer at its submission, yet a call
    // moves the command by one round trip: data, data, status, result.
    let before = state.borrow().transfers;
    bot.begin(0, multi.as_ref(), Data::In(length)).unwrap();
    for submitted in 2..=4 {
        assert_eq!(bot.end(), None);
        assert_eq!(state.borrow().transfers, before + submitted);
    }
    assert_eq!(bot.end(), Some(Ok(output)));
    assert_eq!(state.borrow().transfers, before + 4);
    let mut output = vec![0xa5; MAX_TD];

    for (fault, expected) in [
        (Fault::ShortData, SHORT_READ),
        (Fault::SkippedData, SHORT_READ),
        (Fault::WrongTag, PROTOCOL_ERROR),
    ] {
        output.fill(0xa5);
        state.borrow_mut().fault = fault;
        assert_eq!(bot.read_into(0, cdb.as_ref(), &mut output), Err(expected));
        assert!(output.iter().all(|&byte| byte == 0xa5));
    }
    assert_eq!(bot.generation(), 1);
}

#[test]
fn synchronous_read_keeps_full_recovered_data_across_autosense() {
    use crate::block::BlockDevice;
    let mut memory = vec![0u64; 32 * 1024];
    let start = memory.as_mut_ptr() as u64;
    let end = start + core::mem::size_of_val(memory.as_slice()) as u64;
    let mut dma = dma(&mut memory);
    let io = model(start, end, vec![None, unit(0x64)]);
    let state = io.0.clone();
    let bot = Bot::new(io, 3, endpoints(), &mut dma).unwrap();
    let owner = Rc::new(RefCell::new(super::super::media::Owner::new(
        bot,
        "reader".into(),
        "serial".into(),
    )));
    let mut disks = scan_bot(owner, &mut Vec::new()).unwrap();
    let [disk] = disks.as_mut_slice() else {
        panic!("expected one published disk");
    };
    state.borrow_mut().sense_polls = Some(0);
    state.borrow_mut().fault = Fault::RecoveredData;
    let mut output = [0xa5; 512];
    assert_eq!(disk.read(0, &mut output), Ok(()));
    assert_eq!(output, [0x64; 512]);
    let state_ref = state.borrow();
    assert_eq!(state_ref.sense_polls, Some(6));
    let [read, sense] = &state_ref.history[state_ref.history.len() - 2..] else {
        unreachable!();
    };
    assert_eq!((read.0, read.2[0]), (1, scsi::READ_10));
    assert_eq!((sense.0, sense.2[0]), (1, scsi::REQUEST_SENSE));
    drop(state_ref);

    state.borrow_mut().fault = Fault::RecoveredShortData;
    output.fill(0xa5);
    assert_eq!(
        disk.read(0, &mut output),
        Err(BlockError::Sense(Sense {
            key: Sense::RECOVERED_ERROR,
            asc: 0x17,
            ascq: 0x01,
        }))
    );
    assert_eq!(output, [0xa5; 512]);
    assert_eq!(state.borrow().sense_polls, Some(12));

    state.borrow_mut().fault = Fault::AbortReadThenWrongTag;
    let first = state.borrow().history.len();
    assert_eq!(disk.read(0, &mut output), Err(PROTOCOL_ERROR));
    assert_eq!(output, [0xa5; 512]);
    assert!(
        disk.media.get(),
        "the class reset ends the read, not the disk"
    );
    assert_eq!(state.borrow().sense_polls, Some(18));
    assert!(
        state.borrow().history[first..]
            .iter()
            .map(|(lun, _, cdb)| (*lun, cdb[0]))
            .eq([
                (1, scsi::READ_10),
                (1, scsi::REQUEST_SENSE),
                (1, scsi::READ_10)
            ])
    );
}

#[test]
#[ignore = "oracle: STELA_USB_BOT_CBW_ORACLE"]
fn unchanged_linux_cbw_builder_matches_all_luns_and_transfer_profiles() {
    extern crate std;
    let path = std::env::var("STELA_USB_BOT_CBW_ORACLE").expect("oracle trace path");
    let bytes = std::fs::read(path).unwrap();
    assert_eq!(&bytes[..8], b"BOTCBW01");
    let records = &bytes[8..];
    assert_eq!(records.len() % 58, 0);
    assert_eq!(records.len() / 58, 6144);
    for record in records.as_chunks::<58>().0 {
        let tag = u32::from_le_bytes(record[..4].try_into().unwrap());
        let lun = record[4];
        let n = usize::from(record[5]);
        let input = record[6] != 0;
        let length = u32::from_le_bytes(record[7..11].try_into().unwrap()) as usize;
        assert_eq!(
            cbw(tag, lun, &record[11..11 + n], length, input).as_slice(),
            &record[27..58]
        );
    }
}
#[test]
fn discovery_reports_empty_slots_and_publishes_only_ready_units() {
    let mut memory = vec![0u64; 32 * 1024];
    let start = memory.as_mut_ptr() as u64;
    let mut dma = dma(&mut memory);
    let io = model(
        start,
        start + memory.len() as u64 * 8,
        vec![None, unit(1), None, unit(3)],
    );
    let bot = Rc::new(RefCell::new(
        Bot::new(io, 3, endpoints(), &mut dma).unwrap(),
    ));
    let mut failures = Vec::new();
    let disks =
        super::super::discover_units(bot, "reader interface 3", "reader-serial", &mut failures)
            .unwrap();
    assert_eq!(disks.iter().map(|d| d.lun).collect::<Vec<_>>(), vec![1, 3]);
    assert_eq!(
        failures,
        vec![
            (
                "reader interface 3 LUN 0".into(),
                BlockError::Sense(Sense {
                    key: 0x02,
                    asc: 0x3a,
                    ascq: 0x00
                })
            ),
            (
                "reader interface 3 LUN 2".into(),
                BlockError::Sense(Sense {
                    key: 0x02,
                    asc: 0x3a,
                    ascq: 0x00
                })
            )
        ]
    );
}
#[test]
fn invalid_identification_capacity_and_mode_never_publish_a_block_device() {
    let mut short_capacity = vec![0; 7];
    short_capacity[0] = 1;
    let mut overflow = vec![0; 32];
    overflow[..8].fill(255);
    overflow[8..12].copy_from_slice(&512u32.to_be_bytes());
    let mut bad_block = vec![0; 8];
    bad_block[4..8].copy_from_slice(&513u32.to_be_bytes());
    for (opcode, payload) in [
        (0x12, vec![0; 35]),
        (0x12, vec![0x20; 36]),
        (0x25, short_capacity),
        (0x25, bad_block),
        (0x9e, overflow),
        (0x1a, vec![0; 4]),
    ] {
        let mut memory = vec![0u64; 32 * 1024];
        let start = memory.as_mut_ptr() as u64;
        let mut dma = dma(&mut memory);
        let mut u = unit(0).unwrap();
        u.huge = opcode == 0x9e;
        let io = model(start, start + memory.len() as u64 * 8, vec![Some(u)]);
        io.0.borrow_mut().override_payload.push((opcode, payload));
        let mut bot = Bot::new(io, 3, endpoints(), &mut dma).unwrap();
        assert!(super::super::probe(&mut bot, 0, "slot", "serial").is_err());
    }
}
#[test]
fn malformed_sense_never_invents_a_ready_unit() {
    for bytes in [
        vec![],
        vec![0x70; 13],
        vec![0x72; 7],
        vec![0x71; 8],
        vec![0x75; 18],
    ] {
        assert_eq!(Sense::parse(&bytes), None);
    }
    let mut fixed = vec![0; 18];
    fixed[0] = 0x71;
    fixed[2] = 6;
    fixed[7] = 5;
    assert_eq!(Sense::parse(&fixed), None);
    fixed[7] = 6;
    fixed[12] = 0x29;
    assert_eq!(
        Sense::parse(&fixed),
        Some(Sense {
            key: Sense::UNIT_ATTENTION,
            asc: Sense::RESET_OCCURRED,
            ascq: 0
        })
    );
}
#[test]
fn late_discovery_transport_poison_discards_already_probed_units() {
    struct FailSecond(Model);
    impl Io for FailSecond {
        type Buffer = DmaBuffer;

        fn ready(&mut self) -> Result<(), BlockError> {
            self.0.ready()
        }
        fn control(&mut self, r: [u8; 8]) -> Result<Vec<u8>, BlockError> {
            self.0.control(r)
        }
        fn clear(&mut self, e: Endpoint) -> Result<(), BlockError> {
            self.0.clear(e)
        }
        fn delay(&self, m: u64) {
            self.0.delay(m);
        }
        fn minimum_wait(&mut self, micros: u64) -> Result<(), BlockError> {
            self.0.minimum_wait(micros)
        }
        fn now_us(&self) -> u64 {
            self.0.now_us()
        }
        fn quarantine(&mut self) {
            self.0.quarantine();
        }
        fn bulk(&mut self, e: Endpoint, b: &mut DmaBuffer, n: usize) -> Result<usize, BlockError> {
            let p = b.physical();
            if !e.is_in() && n == 31 {
                let range = self.0.0.borrow().range.clone();
                assert!(p >= range.start && p + 31 <= range.end);
                // SAFETY: same live private DMA range as Model::bulk.
                if unsafe { (p as *const u8).add(13).read() } == 1 {
                    self.0.0.borrow_mut().fault = Fault::Timeout;
                }
            }
            self.0.bulk(e, b, n)
        }
    }
    let mut memory = vec![0u64; 32 * 1024];
    let start = memory.as_mut_ptr() as u64;
    let mut dma = dma(&mut memory);
    let io = model(
        start,
        start + memory.len() as u64 * 8,
        vec![unit(0), unit(1), unit(2)],
    );
    let state = io.0.clone();
    let bot = Rc::new(RefCell::new(
        Bot::new(FailSecond(io), 3, endpoints(), &mut dma).unwrap(),
    ));
    let mut failures = Vec::new();
    assert!(super::super::discover_units(bot.clone(), "reader", "serial", &mut failures).is_err());
    assert!(bot.borrow().poisoned());
    assert_eq!(failures.len(), 1);
    assert_eq!(failures[0].0, "reader LUN 1");
    assert!(!state.borrow().history.iter().any(|(lun, _, _)| *lun == 2));
}
/// No caller waits for a reset to settle: the transport refuses
/// commands until it has, then clears both halts.
#[test]
fn a_settling_reset_holds_commands_without_waiting() {
    let mut memory = vec![0u64; 32 * 1024];
    let start = memory.as_mut_ptr() as u64;
    let mut dma = dma(&mut memory);
    let io = model(
        start,
        start + memory.len() as u64 * 8,
        vec![unit(0), unit(1)],
    );
    let state = io.0.clone();
    let mut bot = Bot::new(io, 3, endpoints(), &mut dma).unwrap();
    state.borrow_mut().fault = Fault::Phase;
    bot.begin(0, &[0; 6], Data::None).unwrap();
    assert!(ended(&mut bot).is_err());
    state.borrow_mut().fault = Fault::None;
    assert!(bot.busy());
    assert!(bot.begin(1, &[0; 6], Data::None).is_err());
    assert!(!state.borrow().order.contains(&Order::Clear(0x82)));
    state.borrow_mut().now += RESET_RECOVERY_US;
    assert!(!bot.busy());
    assert_eq!(bot.generation(), 1);
    bot.command(1, &[0; 6], Data::None).unwrap();
    assert!(
        !state
            .borrow()
            .order
            .iter()
            .any(|order| matches!(order, Order::Delay(_)))
    );
}

#[test]
fn recovery_waits_for_real_reset_settle_before_clears_and_next_lun() {
    let mut memory = vec![0u64; 32 * 1024];
    let start = memory.as_mut_ptr() as u64;
    let mut dma = dma(&mut memory);
    let io = model(
        start,
        start + memory.len() as u64 * 8,
        vec![unit(0), unit(1)],
    );
    let state = io.0.clone();
    let mut bot = Bot::new(io, 3, endpoints(), &mut dma).unwrap();
    state.borrow_mut().fault = Fault::Phase;
    assert!(bot.command(0, &[0; 6], Data::None).is_err());
    assert!(!bot.poisoned());
    assert!(bot.busy(), "the reset settles");
    state.borrow_mut().fault = Fault::None;
    bot.command(1, &[0; 6], Data::None).unwrap();
    assert_eq!(
        state.borrow().order,
        vec![
            Order::Cbw(0),
            Order::Reset,
            Order::Delay(RESET_RECOVERY_US),
            Order::Minimum(RESET_RECOVERY_US),
            Order::Clear(0x82),
            Order::Clear(0x02),
            Order::Cbw(1)
        ]
    );
}
#[test]
fn source_bridge_profiles_change_wire_transfers_and_probes() {
    // Two kept 64-KiB-aligned owners plus wrappers can require
    // 262207 bytes from an arbitrary base, exceeding a 256-KiB fixture.
    let mut memory = vec![0u64; 33 * 1024];
    let start = memory.as_mut_ptr() as u64;
    let mut dma = dma(&mut memory);
    let io = model(
        start,
        start + memory.len() as u64 * 8,
        vec![unit(3), unit(7)],
    );
    let state = io.0.clone();
    // Kobian Mercury Binocam uses a 32-byte wrapper.
    let flags = quirks::Flags::from_id(0x04fc, 0x80c2, 0x0100, true).unwrap();
    let mut bot = Bot::with_flags(io, 3, endpoints(), &mut dma, flags).unwrap();
    bot.command(0, &scsi::inquiry(), Data::In(36)).unwrap();
    assert_eq!(state.borrow().wrappers, vec![32]);
    assert_eq!(bot.wrapper.bytes()[31], 0);
    // Genesis high-speed IDE bridge requires the command/data delay.
    bot.flags = quirks::Flags::from_id(0x05e3, 0x0702, 0x0010, true).unwrap();
    bot.command(0, &scsi::inquiry(), Data::In(36)).unwrap();
    assert!(state.borrow().order.contains(&Order::Minimum(125)));
    let before = state.borrow().controls.len();
    let flags = quirks::Flags::from_id(0x054c, 0x0010, 0x0106, true).unwrap();
    let mut single = Bot::with_flags(bot.io.clone(), 3, endpoints(), &mut dma, flags).unwrap();
    assert_eq!(single.max_lun(), 0);
    assert_eq!(state.borrow().controls.len(), before);
    assert!(single.command(1, &scsi::inquiry(), Data::In(36)).is_err());
    single.flags = quirks::Flags::from_id(0x04e8, 0x5122, 0, true).unwrap();
    state.borrow_mut().fault = Fault::WrongTag;
    assert_eq!(
        single
            .command(0, &scsi::inquiry(), Data::In(36))
            .unwrap()
            .len(),
        36
    );
    assert!(!state.borrow().controls.iter().any(|r| r[1] == RESET));
}

#[test]
fn source_residue_learning_skipped_data_and_chained_csw_retry_use_real_status() {
    for fault in [
        Fault::BadResidue,
        Fault::SkippedData,
        Fault::CswZeroThenStall,
    ] {
        let mut memory = vec![0u64; 32 * 1024];
        let start = memory.as_mut_ptr() as u64;
        let mut dma = dma(&mut memory);
        let io = model(start, start + memory.len() as u64 * 8, vec![unit(4)]);
        let state = io.0.clone();
        let mut bot = Bot::new(io, 3, endpoints(), &mut dma).unwrap();
        state.borrow_mut().fault = fault;
        let bytes = bot.command(0, &scsi::inquiry(), Data::In(36)).unwrap();
        assert_eq!(
            bytes.len(),
            if fault == Fault::SkippedData { 0 } else { 36 }
        );
        assert!(!state.borrow().controls.iter().any(|r| r[1] == RESET));
        if fault == Fault::BadResidue {
            assert!(bot.flags.ignore_residue());
        }
        if fault == Fault::CswZeroThenStall {
            assert_eq!(state.borrow().clears, vec![0x82]);
        }
    }
}

#[test]
fn metadata_warmup_limits_and_last_sector_split_reach_block_commands() {
    use crate::block::BlockDevice;
    let mut memory = vec![0u64; 32 * 1024];
    let start = memory.as_mut_ptr() as u64;
    let mut dma = dma(&mut memory);
    let mut u = unit(0x55).unwrap();
    u.media.resize(128 * 1024, 0x55);
    let io = model(start, start + memory.len() as u64 * 8, vec![Some(u)]);
    let state = io.0.clone();
    let mut bot = Bot::new(io, 3, endpoints(), &mut dma).unwrap();
    // Samsung YP-CP3: 64-sector ceiling and incorrect CSW tags.
    bot.flags = quirks::Flags::from_id(0x04e8, 0x5122, 0x0000, true).unwrap();
    let (identity, _) = super::super::probe(&mut bot, 0, "slot", "serial").unwrap();
    {
        let state = state.borrow();
        let read = state
            .history
            .iter()
            .position(|(_, _, c)| c[0] == scsi::READ_10)
            .unwrap();
        let mode = state
            .history
            .iter()
            .position(|(_, _, c)| c[0] == scsi::MODE_SENSE_6)
            .unwrap();
        assert!(read < mode);
    }
    let maximum = bot.max_transfer_bytes();
    assert_eq!(maximum, 32768);
    let shared = Rc::new(RefCell::new(bot));
    let mut disk = super::super::Disk {
        bot: shared,
        lun: 0,
        identity,
        media: Rc::new(core::cell::Cell::new(true)),
        removable: false,
        running: None,
        next_tag: 0,
    };
    let before = state.borrow().history.len();
    let mut data = vec![0; 128 * 1024];
    disk.read(0, &mut data).unwrap();
    assert!(data.iter().all(|&b| b == 0x55));
    let state = state.borrow();
    let reads: Vec<_> = state.history[before..]
        .iter()
        .filter(|(_, _, c)| c[0] == scsi::READ_10)
        .collect();
    assert!(
        reads
            .iter()
            .all(|(_, _, c)| usize::from(u16::from_be_bytes([c[7], c[8]])) * 512 <= maximum)
    );
    assert_eq!(&reads.last().unwrap().2[7..9], &[0, 1]);
}

#[test]
fn an_acknowledged_class_reset_ends_the_command_and_keeps_every_lun() {
    use super::super::media::{Owner, scan_bot};
    use crate::block::BlockDevice;
    let mut memory = vec![0u64; 32 * 1024];
    let start = memory.as_mut_ptr() as u64;
    let mut dma = dma(&mut memory);
    let io = model(
        start,
        start + memory.len() as u64 * 8,
        vec![unit(1), unit(2)],
    );
    let state = io.0.clone();
    let owner = Rc::new(RefCell::new(Owner::new(
        Bot::new(io, 3, endpoints(), &mut dma).unwrap(),
        "reader".into(),
        "serial".into(),
    )));
    let mut failures = Vec::new();
    let mut old = scan_bot(owner.clone(), &mut failures).unwrap();
    state.borrow_mut().fault = Fault::Phase;
    assert!(old[0].command(&[0; 6], Data::None).is_err());
    state.borrow_mut().fault = Fault::None;
    state.borrow_mut().now += RESET_RECOVERY_US;
    // As in Linux, the reset ends one command, not the disks.
    let mut bytes = [0; 512];
    old[1].read(0, &mut bytes).unwrap();
    assert_eq!(bytes, [2; 512]);
    old[0].flush_cache().unwrap();
    assert_eq!(scan_bot(owner, &mut failures).unwrap().len(), 2);
    old[1].flush_cache().unwrap();
    assert_eq!(state.borrow().port_resets, 0);
}

#[test]
fn a_failed_class_reset_resets_the_port_and_completes_on_the_same_disk() {
    use super::super::media::{Owner, scan_bot};
    use crate::block::BlockDevice;
    let mut memory = vec![0u64; 32 * 1024];
    let start = memory.as_mut_ptr() as u64;
    let mut dma = dma(&mut memory);
    let io = model(
        start,
        start + memory.len() as u64 * 8,
        vec![unit(1), unit(2)],
    );
    let state = io.0.clone();
    let owner = Rc::new(RefCell::new(Owner::new(
        Bot::new(io, 3, endpoints(), &mut dma).unwrap(),
        "reader".into(),
        "serial".into(),
    )));
    let mut failures = Vec::new();
    let mut disks = scan_bot(owner.clone(), &mut failures).unwrap();
    state.borrow_mut().port_reset = true;
    for attempt in 1..=3u8 {
        // The write's phase error starts a class reset, which fails: the
        // owner resets the port, rebinds the same device and retries the
        // write once, on the same disk.
        state.borrow_mut().fault = Fault::Reset;
        disks[1].write(0, &[0x40 + attempt; 512]).unwrap();
        assert_eq!(state.borrow().port_resets, usize::from(attempt));
        let mut bytes = [0; 512];
        disks[1].read(0, &mut bytes).unwrap();
        assert_eq!(bytes, [0x40 + attempt; 512]);
        disks[0].read(0, &mut bytes).unwrap();
        assert_eq!(bytes, [1; 512], "the other LUN kept its disk");
    }
    // A fourth reset within the minute is not tried; every disk fails until
    // an explicit rescan.
    state.borrow_mut().fault = Fault::Reset;
    assert!(disks[1].write(0, &[0; 512]).is_err());
    state.borrow_mut().fault = Fault::None;
    assert!(disks[1].read(0, &mut [0; 512]).is_err());
    assert!(disks[0].read(0, &mut [0; 512]).is_err());
    assert_eq!(state.borrow().port_resets, 3);
}

mod minimum;

/// With a timer that keeps intervals, a reset settles by polling it: no
/// look waits, and the halts clear only once the interval proves the time.
#[test]
fn a_reset_settles_by_the_timers_interval_without_a_wait() {
    let mut memory = vec![0u64; 32 * 1024];
    let start = memory.as_mut_ptr() as u64;
    let mut dma = dma(&mut memory);
    let io = model(start, start + memory.len() as u64 * 8, vec![unit(0)]);
    let state = io.0.clone();
    state.borrow_mut().intervals = true;
    let mut bot = Bot::new(io, 3, endpoints(), &mut dma).unwrap();
    state.borrow_mut().fault = Fault::Phase;
    bot.begin(0, &[0; 6], Data::None).unwrap();
    assert!(ended(&mut bot).is_err());
    state.borrow_mut().fault = Fault::None;
    state.borrow_mut().now += RESET_RECOVERY_US - 1;
    assert!(bot.busy());
    state.borrow_mut().now += 1;
    assert!(!bot.busy());
    assert_eq!(
        state.borrow().order,
        vec![
            Order::Cbw(0),
            Order::Reset,
            Order::Clear(0x82),
            Order::Clear(0x02)
        ]
    );
}

/// A stalled status stage is cleared and read again at once, as Linux's
/// transport does: nothing waits, as no reset ran.
#[test]
fn a_stalled_status_is_cleared_without_a_wait() {
    let mut memory = vec![0u64; 32 * 1024];
    let start = memory.as_mut_ptr() as u64;
    let mut dma = dma(&mut memory);
    let io = model(start, start + memory.len() as u64 * 8, vec![unit(0)]);
    let state = io.0.clone();
    let mut bot = Bot::new(io, 3, endpoints(), &mut dma).unwrap();
    state.borrow_mut().fault = Fault::CswStall;
    bot.begin(0, &scsi::inquiry(), Data::In(36)).unwrap();
    assert_eq!(ended(&mut bot).unwrap().len(), 36);
    assert_eq!(
        state.borrow().order,
        vec![Order::Cbw(0), Order::Clear(0x82)]
    );
}

/// The disks of a device behind its owner begin a command and return once
/// its wrapper is submitted, their caller polling for the end, as a
/// shared disk's queue needs: a start never runs the command through.
#[test]
fn an_owned_disk_begins_commands_and_ends_them_at_later_looks() {
    use super::super::media::{Owner, scan_bot};
    use crate::block::{BlockDevice, Command};
    let mut memory = vec![0u64; 32 * 1024];
    let start = memory.as_mut_ptr() as u64;
    let mut dma = dma(&mut memory);
    let io = model(
        start,
        start + memory.len() as u64 * 8,
        vec![unit(1), unit(2)],
    );
    let state = io.0.clone();
    let owner = Rc::new(RefCell::new(Owner::new(
        Bot::new(io, 3, endpoints(), &mut dma).unwrap(),
        "reader".into(),
        "serial".into(),
    )));
    let mut failures = Vec::new();
    let mut disks = scan_bot(owner, &mut failures).unwrap();
    assert_eq!(disks[1].depth(), 1);
    let before = state.borrow().transfers;
    let tag = disks[1]
        .start(Command::Write {
            lba: 0,
            data: vec![0x77; 512],
            fua: false,
        })
        .unwrap();
    assert_eq!(state.borrow().transfers, before + 1, "the wrapper alone");
    assert!(!disks[0].room() && !disks[1].room());
    // A waiting command of the other unit ends the begun one first and
    // keeps its result for its disk.
    let mut bytes = [0; 512];
    disks[0].read(0, &mut bytes).unwrap();
    assert_eq!(bytes, [1; 512]);
    assert!(matches!(disks[1].finish(tag), Some(Ok(None))));
    assert!(disks[1].room());
    let tag = disks[1]
        .start(Command::Read { lba: 0, bytes: 512 })
        .unwrap();
    let read = (0..16).find_map(|_| disks[1].finish(tag)).unwrap();
    assert_eq!(read.unwrap().unwrap(), vec![0x77; 512]);
}

/// A begun command keeps a waiting one's retries: once for the UNIT
/// ATTENTION a class reset raised, and once more after a port reset.
#[test]
fn an_owned_begun_command_is_retried_as_a_waiting_one_is() {
    use super::super::media::{Owner, scan_bot};
    use crate::block::{BlockDevice, Command};
    let mut memory = vec![0u64; 32 * 1024];
    let start = memory.as_mut_ptr() as u64;
    let mut dma = dma(&mut memory);
    let io = model(
        start,
        start + memory.len() as u64 * 8,
        vec![unit(1), unit(2)],
    );
    let state = io.0.clone();
    let owner = Rc::new(RefCell::new(Owner::new(
        Bot::new(io, 3, endpoints(), &mut dma).unwrap(),
        "reader".into(),
        "serial".into(),
    )));
    let mut failures = Vec::new();
    let mut disks = scan_bot(owner, &mut failures).unwrap();
    let finish = |disk: &mut super::super::Disk<Owner<Bot<Model>>>, tag| {
        (0..64)
            .find_map(|_| disk.finish(tag))
            .expect("a begun command ended")
    };
    // A class reset ends one command; the device's UNIT ATTENTION after it
    // is retried once.
    state.borrow_mut().fault = Fault::Phase;
    let tag = disks[1].start(Command::Flush).unwrap();
    assert!(finish(&mut disks[1], tag).is_err());
    state.borrow_mut().fault = Fault::None;
    state.borrow_mut().now += RESET_RECOVERY_US;
    state.borrow_mut().units[1].as_mut().unwrap().attention = true;
    assert!(disks[1].room());
    let tag = disks[1]
        .start(Command::Read { lba: 0, bytes: 512 })
        .unwrap();
    assert_eq!(finish(&mut disks[1], tag).unwrap().unwrap(), vec![2; 512]);
    // A failed class reset resets the port and runs the write once more.
    state.borrow_mut().port_reset = true;
    state.borrow_mut().fault = Fault::Reset;
    let tag = disks[1]
        .start(Command::Write {
            lba: 0,
            data: vec![0x41; 512],
            fua: false,
        })
        .unwrap();
    assert!(matches!(finish(&mut disks[1], tag), Ok(None)));
    assert_eq!(state.borrow().port_resets, 1);
    assert_eq!(
        state.borrow().units[1].as_ref().unwrap().media[..512],
        [0x41; 512]
    );
}
