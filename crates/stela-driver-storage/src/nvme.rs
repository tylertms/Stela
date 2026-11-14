// SPDX-License-Identifier: GPL-2.0-only
// Copyright (c) 2011-2014, Intel Corporation.
//! NVMe controllers with one admin and one I/O queue pair. Each active NVM
//! namespace with plain logical blocks becomes one device. The I/O queue
//! runs up to [`DEPTH`] commands at once, each copying its data through the
//! bounce pages it takes from the controller's [`Pool`].
//! Errors are handled as Linux's `nvme_timeout` and `nvme_decide_disposition`
//! handle them: a command past its deadline is aborted and given another,
//! then the controller is reset with its DMA contained, and interrupted
//! commands, and those failing without Do Not Retry, run again up to
//! `nvme_max_retries`. Namespace discovery and rescans, asynchronous events,
//! the host memory buffer, shutdown and recovery follow Linux
//! `drivers/nvme/host/core.c` and `pci.c` at
//! a804c5742bff224d4cb4d25dd383554d598be32b.
use crate::bytes::le32;
use crate::{
    Clock, Handoff,
    block::{
        BlockDevice, BlockError, Command, DiscardLimits, Features, Finished, Identity, Transport,
        WriteZeroes, ascii,
    },
    bus::{dma::DmaAllocator, pci::Function},
    registers,
};
use alloc::{boxed::Box, format, rc::Rc, vec::Vec};

mod admin;
mod features;
mod io;
pub mod owned;
pub mod parts;
mod quirks;
mod recovery;
pub mod vmd;
use core::{
    cell::RefCell,
    sync::atomic::{Ordering, fence},
};
use owned::{Buffer, RegisterIo};
pub use quirks::Quirks;
use stela_driver_core::firmware::smbios::SystemIdentity;

pub const CLASS: [u8; 3] = [0x01, 0x08, 0x02];

const CAPABILITIES: usize = 0x00;
const VERSION: usize = 0x08;
const INTERRUPT_MASK_SET: usize = 0x0c;
const CONFIGURATION: usize = 0x14;
const STATUS: usize = 0x1c;
const ADMIN_ATTRIBUTES: usize = 0x24;
const ADMIN_SUBMISSION: usize = 0x28;
const ADMIN_COMPLETION: usize = 0x30;
/// CRTO: CRWMT in its low half, in 500 ms units.
const READY_TIMEOUTS: usize = 0x68;
const DOORBELLS: usize = 0x1000;

const ENABLE: u32 = 1;
const READY: u32 = 1;
const FATAL: u32 = 1 << 1;
/// CSTS.PP: processing paused for a firmware activation.
const PROCESSING_PAUSED: u32 = 1 << 5;
/// A controller that is gone reads all ones (Linux `nvme_should_reset`
/// treats `~0` CSTS as removal).
const REMOVED: u32 = u32::MAX;
/// CC.SHN normal shutdown and CSTS.SHST (NVMe Base 2.1 §3.1.4.5–6).
const SHUTDOWN_NOTIFICATION: u32 = 3 << 14;
const SHUTDOWN_NORMAL: u32 = 1 << 14;
const SHUTDOWN_STATUS: u32 = 3 << 2;
const SHUTDOWN_COMPLETE: u32 = 2 << 2;
/// Linux bounds a shutdown by the controller's RTD3 entry latency, at least
/// five seconds (`shutdown_timeout`) and at most a minute.
const SHUTDOWN_MINIMUM_US: u64 = 5_000_000;
const SHUTDOWN_MAXIMUM_US: u64 = 60_000_000;
const MICROSECONDS_PER_SECOND: u64 = 1_000_000;
/// Completion polls between controller-fatal checks while a command runs.
const STATUS_POLL_INTERVAL: u32 = 1024;
/// The first version with the active namespace list (Linux `NVME_VS(1, 1, 0)`);
/// Identify Namespace reports an NGUID from 1.2 and the identification
/// descriptor list exists from 1.3.
const VERSION_1_1: u32 = 0x0001_0100;
const VERSION_1_2: u32 = 0x0001_0200;
const VERSION_1_3: u32 = 0x0001_0300;
const VERSION_2_0: u32 = 0x0002_0000;
/// 64-byte submission and 16-byte completion entries, 4 KiB pages, NVM set.
const CONFIGURATION_VALUE: u32 = ENABLE | 6 << 16 | 4 << 20;
/// CC.CSS: every I/O command set the controller supports.
const COMMAND_SETS_ALL: u32 = 6 << 4;
/// CAP.CSS's I/O command sets, and CAP.CRMS's CRWMS and CRIMS.
const CAPABILITY_COMMAND_SETS: u64 = 1 << 43;
const CAPABILITY_READY_WITH_MEDIA: u64 = 1 << 59;
const CAPABILITY_READY_INDEPENDENT: u64 = 1 << 60;

const ADMIN_DELETE_SUBMISSION: u8 = 0x00;
const ADMIN_CREATE_SUBMISSION: u8 = 0x01;
const ADMIN_GET_LOG_PAGE: u8 = 0x02;
const ADMIN_DELETE_COMPLETION: u8 = 0x04;
const ADMIN_CREATE_COMPLETION: u8 = 0x05;
const ADMIN_IDENTIFY: u8 = 0x06;
const ADMIN_ABORT: u8 = 0x08;
const ADMIN_SET_FEATURES: u8 = 0x09;
const ADMIN_ASYNC_EVENT: u8 = 0x0c;
/// One MSI-X table entry: message address, data and vector control.
const MSIX_ENTRY_BYTES: u64 = 16;
/// Autonomous Power State Transition (NVMe Base 2.1 5.27.1.6).
/// Number of Queues: CDW11 asks the I/O submission and completion queue
/// counts, zero-based.
const FEATURE_QUEUE_COUNT: u32 = 0x07;
const FEATURE_APST: u32 = 0x0c;
/// Asynchronous Event Configuration, and the notices Linux enables from
/// OAES (`NVME_AEN_SUPPORTED`): namespace attributes, firmware activation,
/// ANA and discovery log changes.
const FEATURE_ASYNC_EVENTS: u32 = 0x0b;
const EVENTS_SUPPORTED: u32 = 1 << 8 | 1 << 9 | 1 << 11 | 1 << 31;
/// Asynchronous event types and the subtypes Linux acts on.
const EVENT_ERROR: u32 = 0;
const EVENT_NOTICE: u32 = 2;
const ERROR_PERSISTENT_INTERNAL: u32 = 0x03;
const NOTICE_NAMESPACES_CHANGED: u32 = 0x00;
const NOTICE_FIRMWARE_ACTIVATION: u32 = 0x01;
/// Firmware Slot Information and Changed Namespace List log pages.
const LOG_FIRMWARE_SLOT: u32 = 0x03;
const LOG_CHANGED_NAMESPACES: u32 = 0x04;
/// Host Behavior Support, with its ACRE and LBAFEE bytes.
const FEATURE_HOST_BEHAVIOR: u32 = 0x16;
const HOST_BEHAVIOR_ACRE: usize = 0;
const HOST_BEHAVIOR_LBAFEE: usize = 2;
const HOST_BEHAVIOR_BYTES: usize = 512;
/// Host Memory Buffer: CDW11 EHM and MR, CDW12 HSIZE in pages, CDW13-14
/// the descriptor list and CDW15 its entry count.
const FEATURE_HOST_MEMORY: u32 = 0x0d;
const HOST_MEMORY_ENABLE: u32 = 1 << 0;
const HOST_MEMORY_RETURN: u32 = 1 << 1;
/// `max_host_mem_size_mb`, and the largest chunk Linux tries first
/// (`PAGE_SIZE * MAX_ORDER_NR_PAGES` on x86-64).
const HOST_MEMORY_MAXIMUM: u64 = 128 << 20;
const HOST_MEMORY_CHUNK: u64 = 4 << 20;
/// One descriptor: address, size in pages, reserved.
const HOST_MEMORY_DESCRIPTOR: usize = 16;
const IO_FLUSH: u8 = 0x00;
const IO_WRITE: u8 = 0x01;
const IO_READ: u8 = 0x02;
const IO_WRITE_ZEROES: u8 = 0x08;
const IO_DATASET_MANAGEMENT: u8 = 0x09;

const IDENTIFY_NAMESPACE: u32 = 0;
const IDENTIFY_CONTROLLER: u32 = 1;
const IDENTIFY_ACTIVE_NAMESPACES: u32 = 2;
const IDENTIFY_DESCRIPTORS: u32 = 3;
/// The I/O command set's Identify Controller, here for CSI 0 (NVM).
const IDENTIFY_COMMAND_SET_CONTROLLER: u32 = 6;
/// Its WZSL, DMRL and DMRSL (NVM Command Set 1.1 Figure 112).
const NVM_WRITE_ZEROES_LIMIT: usize = 1;
const NVM_DEALLOCATE_RANGES: usize = 3;
const NVM_DEALLOCATE_RANGE_LIMIT: usize = 4;
/// The ranges one Dataset Management command takes, and each range's bytes.
const MAX_DEALLOCATE_RANGES: usize = 256;
const RANGE_BYTES: usize = 16;
/// The command-set-independent Identify Namespace and its NSTAT.NRDY.
const IDENTIFY_INDEPENDENT_NAMESPACE: u32 = 8;
const INDEPENDENT_STATUS: usize = 14;
const NAMESPACE_READY: u8 = 1;

/// Identify Controller fields (NVMe Base 2.1 Figure 312).
const CONTROLLER_SERIAL: core::ops::Range<usize> = 4..24;
const CONTROLLER_MODEL: core::ops::Range<usize> = 24..64;
const CONTROLLER_MAXIMUM_TRANSFER: usize = 77;
const CONTROLLER_RTD3_ENTRY: core::ops::Range<usize> = 88..92;
/// OAES and CTRATT, whose ELBAS reports extended LBA formats.
const CONTROLLER_EVENTS: usize = 92;
const CONTROLLER_ATTRIBUTES: usize = 96;
const ATTRIBUTE_EXTENDED_LBA: u32 = 1 << 15;
/// CRDT1–3, in units of 100 ms.
const CONTROLLER_RETRY_DELAYS: usize = 128;
/// MTFA, in 100 ms units.
const CONTROLLER_ACTIVATION: usize = 258;
/// HMPRE and HMMIN in 4 KiB units, HMMINDS in 4 KiB units, HMMAXD.
const CONTROLLER_HOST_MEMORY_PREFERRED: usize = 272;
const CONTROLLER_HOST_MEMORY_MINIMUM: usize = 276;
const CONTROLLER_HOST_MEMORY_DESCRIPTOR_MINIMUM: usize = 332;
const CONTROLLER_HOST_MEMORY_DESCRIPTORS: usize = 336;
const CONTROLLER_NAMESPACES: core::ops::Range<usize> = 516..520;
/// ONCS: Dataset Management (bit 2) and Write Zeroes (bit 3).
const CONTROLLER_COMMANDS: usize = 520;
const COMMANDS_DATASET_MANAGEMENT: u16 = 1 << 2;
const COMMANDS_WRITE_ZEROES: u16 = 1 << 3;
/// VWC bit 0: a volatile write cache is present.
const CONTROLLER_VOLATILE_CACHE: usize = 525;
/// NPSS: the deepest power state's number.
const CONTROLLER_POWER_STATES: usize = 263;
/// APSTA bit 0: autonomous power state transitions are supported.
const CONTROLLER_APST: usize = 265;
/// The power state descriptors, 32 bytes each: flags at byte 3 (bit 1 a
/// non-operational state), entry and exit latencies in microseconds at
/// bytes 4 and 8.
const CONTROLLER_POWER_DESCRIPTORS: usize = 2048;
const POWER_NON_OPERATIONAL: u8 = 1 << 1;
/// Linux `nvme_core`'s defaults: states whose exit latency exceeds
/// `default_ps_max_latency_us` are never used; a state whose total latency is
/// within `apst_primary_latency_tol_us` is entered after
/// `apst_primary_timeout_ms` idle, one within `apst_secondary_latency_tol_us`
/// after `apst_secondary_timeout_ms`.
const APST_MAX_LATENCY_US: u64 = 100_000;
const APST_PRIMARY_TIMEOUT_MS: u64 = 100;
const APST_PRIMARY_TOLERANCE_US: u64 = 15_000;
const APST_SECONDARY_TIMEOUT_MS: u64 = 2_000;
const APST_SECONDARY_TOLERANCE_US: u64 = 100_000;
/// Identify Namespace fields (NVM Command Set 1.1 Figure 114).
const NAMESPACE_SIZE: core::ops::Range<usize> = 0..8;
const NAMESPACE_CAPACITY: core::ops::Range<usize> = 8..16;
const NAMESPACE_FORMAT: usize = 26;
/// DLFEAT: bits 2:0 what a deallocated block reads (001b zeros), bit 3
/// Write Zeroes may deallocate.
const NAMESPACE_DEALLOCATE: usize = 33;
const NAMESPACE_ATTRIBUTES: usize = 99;
const NAMESPACE_WRITE_PROTECTED: u8 = 1 << 0;
const NAMESPACE_NGUID: core::ops::Range<usize> = 104..120;
const NAMESPACE_EUI64: core::ops::Range<usize> = 120..128;
const NAMESPACE_FORMATS: usize = 128;
/// FLBAS: format index bits 3:0, and bits 6:5 for formats past sixteen.
const FORMAT_LOW: u8 = 0xf;
const FORMAT_HIGH: u8 = 3 << 5;
/// The most namespaces a sequential scan visits, one Identify each.
const SEQUENTIAL_SCAN_LIMIT: u32 = 1024;
/// Namespace Identification Descriptor types (NVMe Base 2.1 Figure 315).
const DESCRIPTOR_EUI64: u8 = 1;
const DESCRIPTOR_NGUID: u8 = 2;
const DESCRIPTOR_UUID: u8 = 3;
const DESCRIPTOR_CSI: u8 = 4;
/// Command set identifiers: NVM and zoned namespaces.
const CSI_NVM: u8 = 0;
const CSI_ZONED: u8 = 2;

const PAGE: usize = 4096;
const ADMIN_ENTRIES: u16 = 16;
/// The I/O queue's entries: one page of submissions.
const IO_ENTRIES: u16 = 64;
/// The commands the I/O queue runs at once, as Linux's tag set bounds them:
/// every submission entry but the one that tells a full queue from an empty
/// one, fewer when CAP.MQES is smaller.
const DEPTH: usize = IO_ENTRIES as usize - 1;
/// The most data one command moves.
const BOUNCE_BYTES: usize = 128 * 1024;
/// The I/O bounce pages of one controller: as much memory as eight commands
/// of [`BOUNCE_BYTES`] take, which many more small commands share.
const POOL_PAGES: usize = 256;
const _: () = assert!(POOL_PAGES >= pages_for(BOUNCE_BYTES) && POOL_PAGES <= u16::MAX as usize);
// Linux PCI NVME_MAX_BYTES bounds the logical command range independently of
// payload storage. Write Zeroes transfers no host data through the bounce pool.
const MAX_COMMAND_BYTES: usize = 8 * 1024 * 1024;
const COMMAND_TIMEOUT_US: u64 = 30_000_000;
/// Linux `NVME_ADMIN_TIMEOUT`.
const ADMIN_TIMEOUT_US: u64 = 60_000_000;
/// Linux `nvme_max_retries`: a command the controller's reset interrupted,
/// or that failed without Do Not Retry, runs again at most this often.
const RETRIES: u8 = 5;
/// The completion status field's Do Not Retry bit and Command Retry Delay
/// index.
const STATUS_DO_NOT_RETRY: u32 = 1 << 14;
const STATUS_RETRY_DELAY: u32 = 3 << 11;
const RETRY_DELAY_UNIT_US: u64 = 100_000;
/// Read/Write CDW12 bit 30: complete only once the data is durable.
const FORCE_UNIT_ACCESS: u32 = 1 << 30;
/// Write Zeroes CDW12 bit 25: the blocks may be deallocated.
const DEALLOCATE: u32 = 1 << 25;
/// Dataset Management CDW11 bit 2: deallocate the ranges.
const ATTRIBUTE_DEALLOCATE: u32 = 1 << 2;
/// I/O to a namespace Linux's rescan removed.
const NAMESPACE_REMOVED: BlockError =
    BlockError::Protocol("the NVMe namespace changed or was removed");

/// The bounce pages a transfer of `bytes` takes: its data, and a PRP list
/// past two pages.
const fn pages_for(bytes: usize) -> usize {
    let data = bytes.div_ceil(PAGE);
    if data > 2 { data + 1 } else { data }
}

/// An I/O read or write of `blocks` blocks at `lba` through `pointers`.
fn transfer_command(
    opcode: u8,
    namespace: u32,
    lba: u64,
    blocks: u32,
    (prp1, prp2): (u64, u64),
    access: u32,
) -> [u32; 16] {
    let mut command = [0u32; 16];
    command[0] = u32::from(opcode);
    command[1] = namespace;
    command[6] = prp1 as u32;
    command[7] = (prp1 >> 32) as u32;
    command[8] = prp2 as u32;
    command[9] = (prp2 >> 32) as u32;
    command[10] = lba as u32;
    command[11] = (lba >> 32) as u32;
    command[12] = access | (blocks - 1);
    command
}

/// Linux `nvme_ns_ids`: what identifies a namespace across rescans.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
struct Ids {
    eui64: [u8; 8],
    nguid: [u8; 16],
    uuid: [u8; 16],
    csi: u8,
}

impl Ids {
    /// Linux `nvme_identify_ns_descs` parsing: descriptors until a zero
    /// length, stopping at one whose length is wrong for its type.
    fn from_descriptors(data: &[u8]) -> Self {
        let mut ids = Self::default();
        let mut at = 0;
        while at + 4 <= data.len() {
            let (kind, length) = (data[at], usize::from(data[at + 1]));
            if length == 0 {
                break;
            }
            let Some(value) = data.get(at + 4..at + 4 + length) else {
                break;
            };
            let expected = match kind {
                DESCRIPTOR_EUI64 => 8,
                DESCRIPTOR_NGUID | DESCRIPTOR_UUID => 16,
                DESCRIPTOR_CSI => 1,
                // Unknown types are skipped.
                _ => length,
            };
            if length != expected {
                break;
            }
            match kind {
                DESCRIPTOR_EUI64 => ids.eui64.copy_from_slice(value),
                DESCRIPTOR_NGUID => ids.nguid.copy_from_slice(value),
                DESCRIPTOR_UUID => ids.uuid.copy_from_slice(value),
                DESCRIPTOR_CSI => ids.csi = value[0],
                _ => {}
            }
            at += 4 + length;
        }
        ids
    }

    /// Linux `nvme_ns_info_from_identify`: EUI64 and NGUID from Identify
    /// Namespace when the descriptors gave none.
    fn fill(&mut self, version: u32, data: &[u8]) {
        if version >= VERSION_1_1 && self.eui64 == [0; 8] {
            self.eui64.copy_from_slice(&data[NAMESPACE_EUI64]);
        }
        if version >= VERSION_1_2 && self.nguid == [0; 16] {
            self.nguid.copy_from_slice(&data[NAMESPACE_NGUID]);
        }
    }
}

/// How an admin command was lost, when it produced no completion to report.
enum Lost {
    /// Linux resets the controller: a timeout, or a fatal controller status.
    Reset(BlockError),
    /// The controller broke the protocol, is gone, or its registers failed.
    Fatal(BlockError),
}

struct Queue {
    submissions: Buffer,
    completions: Buffer,
    id: u16,
    entries: u16,
    tail: u16,
    head: u16,
    phase: bool,
    /// The submission head the controller last reported.
    consumed: u16,
}

impl Queue {
    fn new(id: u16, entries: u16, dma: &mut dyn DmaAllocator) -> Result<Self, BlockError> {
        Ok(Self {
            submissions: Buffer::Legacy(dma.allocate(PAGE, PAGE)?),
            completions: Buffer::Legacy(dma.allocate(PAGE, PAGE)?),
            id,
            entries,
            tail: 0,
            head: 0,
            phase: true,
            consumed: 0,
        })
    }

    /// Empty, as a controller reset leaves it.
    fn restart(&mut self) {
        self.tail = 0;
        self.head = 0;
        self.phase = true;
        self.consumed = 0;
    }

    /// The next position after an already validated index.
    fn next(&self, slot: u16) -> u16 {
        let next = slot + 1;
        if next == self.entries { 0 } else { next }
    }

    /// Whether a reported submission head lies no earlier than the last
    /// one and no further than the tail, in ring order: commands fetched
    /// together may complete reporting the same head.
    fn consumes(&self, head: u16) -> bool {
        head < self.entries
            && self.ahead(self.consumed, head) <= self.ahead(self.consumed, self.tail)
    }

    /// Whether `head` shows the command at submission `slot` fetched: past
    /// it, and no further than the tail.
    fn fetched(&self, slot: u16, head: u16) -> bool {
        let past = self.ahead(slot, head);
        past != 0 && past <= self.ahead(slot, self.tail)
    }

    /// How many entries `to` lies after `from`, in ring order.
    fn ahead(&self, from: u16, to: u16) -> u16 {
        if to >= from {
            to - from
        } else {
            self.entries - (from - to)
        }
    }
}

/// The I/O queue's bounce memory: the pages a command takes at its start
/// for its data, PRP list or Dataset Management ranges, and gives back once
/// it completed for good. A command's pages stay with its tag through
/// retries and controller resets. A command a fault leaves unresolved keeps
/// its pages, which the never-freed DMA pool does not hand out again.
struct Pool {
    pages: Vec<Buffer>,
    /// Indices into `pages`.
    free: Vec<u16>,
}

impl Pool {
    /// The lowest free page is taken first.
    fn new(pages: Vec<Buffer>) -> Self {
        let free = (0..pages.len() as u16).rev().collect();
        Self { pages, free }
    }

    fn legacy(dma: &mut dyn DmaAllocator) -> Result<Self, BlockError> {
        let pages = (0..POOL_PAGES)
            .map(|_| dma.allocate(PAGE, PAGE).map(Buffer::Legacy))
            .collect::<Result<_, _>>()?;
        Ok(Self::new(pages))
    }

    fn take(&mut self, count: usize) -> Option<Vec<u16>> {
        let at = self.free.len().checked_sub(count)?;
        let mut pages = self.free.split_off(at);
        pages.reverse();
        Some(pages)
    }

    fn give_back(&mut self, pages: &mut Vec<u16>) {
        self.free.extend(pages.drain(..).rev());
    }
}

/// The bounce pages `command` takes.
fn command_pages(command: &Command) -> usize {
    match command {
        Command::Read { bytes, .. } => pages_for(*bytes),
        Command::Write { data, .. } => pages_for(data.len()),
        Command::Discard { .. } => 1,
        Command::Flush | Command::WriteZeroes { .. } => 0,
    }
}

/// What memory a published command hands the controller.
#[derive(Clone, Copy)]
enum Data {
    None,
    /// The admin page.
    Admin,
    /// The pages an I/O tag holds.
    Tag(usize),
}

/// A command of the I/O queue: under its tag until the controller completes
/// it, then its result until the namespace takes it.
#[derive(Default)]
enum Tag {
    #[default]
    Free,
    Running {
        id: u16,
        /// Its submission queue entry.
        slot: u16,
        /// An accepted submission head has passed this command.
        fetched: bool,
        namespace: u32,
        block: usize,
        command: Command,
        deadline: crate::Deadline,
        /// The runs before this one: retries and resets.
        retries: u8,
        /// An Abort for it was sent at its first deadline.
        aborted: bool,
    },
    /// Interrupted by a controller reset, to run again once it completes.
    Waiting {
        namespace: u32,
        block: usize,
        command: Command,
        retries: u8,
    },
    /// Failed without Do Not Retry, to run again after the delay its
    /// Command Retry Delay selects (`nvme_retry_req`).
    Delayed {
        namespace: u32,
        block: usize,
        command: Command,
        retries: u8,
        until: crate::Deadline,
    },
    Done(Finished),
}

/// The admin commands bringing a controller up, as Linux's
/// `nvme_reset_work` runs them after every reset and probe runs them once:
/// `nvme_init_ctrl_finish`'s Identify, host options and APST,
/// `nvme_setup_host_mem`, `nvme_setup_io_queues`' queue count and I/O
/// queues, and `nvme_start_ctrl`'s events.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Step {
    Identify,
    /// `nvme_init_non_mdts_limits`'s NVM Identify Controller.
    Limits,
    HostOptions,
    Apst,
    HostMemory,
    /// `nvme_set_queue_count`'s Set Features, before any I/O queue.
    QueueCount,
    CreateCompletion,
    CreateSubmission,
    Events,
}

const STEPS: [Step; 9] = [
    Step::Identify,
    Step::Limits,
    Step::HostOptions,
    Step::Apst,
    Step::HostMemory,
    Step::QueueCount,
    Step::CreateCompletion,
    Step::CreateSubmission,
    Step::Events,
];

/// A controller reset in progress ([`Controller::recover`]), each step
/// looked for without waiting.
enum Recovery {
    /// Deleting the I/O submission, then completion, queue of a live
    /// controller (`nvme_delete_io_queues`), and the command's identifier.
    Deleting {
        submission: bool,
        id: u16,
        deadline: crate::Deadline,
    },
    /// CC.EN cleared; waiting for CSTS.RDY to clear.
    Disabling(crate::Deadline),
    /// CC.EN set; waiting for CSTS.RDY.
    Enabling(crate::Deadline),
    /// A bring-up command, its identifier, and the Identify Controller data
    /// once read.
    Admin {
        step: Step,
        id: u16,
        deadline: crate::Deadline,
        identify: Vec<u8>,
    },
}

/// Linux's asynchronous events: what Identify Controller enables, the
/// outstanding Asynchronous Event Request and a completion not yet acted on.
#[derive(Default)]
struct Events {
    supported: u32,
    /// MTFA, in microseconds.
    activation_us: u64,
    request: Option<u16>,
    completed: Option<(u32, u32)>,
}

impl Events {
    fn from_identify(identify: &[u8]) -> Self {
        Self {
            supported: le32(identify, CONTROLLER_EVENTS) & EVENTS_SUPPORTED,
            activation_us: u64::from(u16::from_le_bytes([
                identify[CONTROLLER_ACTIVATION],
                identify[CONTROLLER_ACTIVATION + 1],
            ])) * 100_000,
            ..Self::default()
        }
    }
}

/// Linux's host memory buffer: its descriptor list and chunks.
struct HostMemory {
    descriptors: Buffer,
    chunks: Vec<Buffer>,
    bytes: u64,
    /// The controller accepted it, so a re-enable may ask for its contents
    /// back (`host_mem_descs` kept).
    returnable: bool,
}

/// What Identify Controller asks of a host memory buffer, in bytes.
#[derive(Clone, Copy)]
struct HostMemoryRequest {
    preferred: u64,
    minimum: u64,
    descriptor_minimum: u64,
    descriptors: u32,
}

impl HostMemoryRequest {
    fn from_identify(identify: &[u8]) -> Self {
        let pages = |at| u64::from(le32(identify, at)) * PAGE as u64;
        Self {
            preferred: pages(CONTROLLER_HOST_MEMORY_PREFERRED),
            minimum: pages(CONTROLLER_HOST_MEMORY_MINIMUM),
            descriptor_minimum: pages(CONTROLLER_HOST_MEMORY_DESCRIPTOR_MINIMUM),
            descriptors: u32::from(u16::from_le_bytes([
                identify[CONTROLLER_HOST_MEMORY_DESCRIPTORS],
                identify[CONTROLLER_HOST_MEMORY_DESCRIPTORS + 1],
            ])),
        }
    }
}

/// Allocations for a host memory buffer: its descriptor list, then its
/// chunks.
trait HostMemorySource {
    fn allocate(&mut self, bytes: usize, chunk: bool) -> Result<Buffer, BlockError>;
}

impl HostMemorySource for &mut dyn DmaAllocator {
    fn allocate(&mut self, bytes: usize, _chunk: bool) -> Result<Buffer, BlockError> {
        DmaAllocator::allocate(*self, bytes, PAGE).map(Buffer::Legacy)
    }
}

/// One controller, shared by its namespaces.
struct Controller {
    registers: RegisterIo,
    doorbell_stride: usize,
    admin: Queue,
    io: Queue,
    /// The page admin commands move their data through, one at a time.
    admin_data: Buffer,
    pool: Pool,
    /// The pool pages each tag holds, in PRP order: data, then any list.
    held: Vec<Vec<u16>>,
    tags: Vec<Tag>,
    recovery: Option<Recovery>,
    /// The I/O completion queue raises MSI-X vector 0, which the host
    /// gave the message of its completion signal.
    signalled: bool,
    /// I/O submissions published since the I/O submission doorbell last
    /// rang ([`Controller::ring_submissions`]): a batch rings once, after
    /// its last command, as Linux's `nvme_commit_rqs` does.
    owed: bool,
    /// Maximum data payload, at most [`BOUNCE_BYTES`].
    max_transfer: usize,
    /// MDTS/host command bound used when no non-MDTS limit is supplied.
    max_command_bytes: usize,
    /// How long a normal shutdown may take.
    shutdown_timeout_us: u64,
    /// The controller acknowledged a shutdown; it takes no more commands.
    shut_down: bool,
    command_id: u16,
    clock: Clock,
    command_timeout_us: u64,
    admin_timeout_us: u64,
    /// An unresolved command may still own the queues, PRPs and bounce RAM,
    /// or the controller is gone. No buffer or queue may be reused until a
    /// new, acknowledged reset.
    fault: Option<BlockError>,
    owned: Option<owned::Owner>,
    teardown: owned::Teardown,
    ready_timeout_us: u64,
    /// CC without EN.
    configuration: u32,
    /// CAP.CRMS.CRIMS: namespaces report readiness of their own.
    independent_identify: bool,
    /// CSTS.CFS already stood when CSTS.RDY rose after the last enable.
    /// Linux looks at CSTS only once a command timed out, so a controller
    /// that answers is used, and only a timeout resets it; a CFS that rises
    /// later still fails a command at once.
    fatal_standing: bool,
    /// The VS register, read at discovery.
    version: u32,
    /// ONCS from Identify Controller.
    commands: u16,
    /// Queued namespace scans: one per reset and namespace-change notice.
    scans: u64,
    /// CRDT1–3 from Identify Controller, in 100 ms units.
    retry_delays: [u16; 3],
    /// The host memory buffer, kept across resets, and whether the
    /// controller uses it (`dev->hmb`).
    host_memory: Option<HostMemory>,
    host_memory_enabled: bool,
    events: Events,
    /// The Abort commands sent and not yet completed.
    aborts: Vec<u16>,
    limits: Limits,
    /// What Linux's quirk tables say of this controller ([`Quirks`]).
    quirks: Quirks,
}

/// `nvme_init_non_mdts_limits`: the NVM command set's Write Zeroes and
/// Deallocate limits, read again after each reset unless the controller
/// once refused the Identify (`NVME_CTRL_SKIP_ID_CNS_CS`).
#[derive(Clone, Copy, Debug, Default)]
struct Limits {
    /// WZSL: Write Zeroes up to 2^WZSL minimum pages; 0 for MDTS.
    write_zeroes: u8,
    /// DMRSL: the most blocks one Deallocate range takes; 0 for none.
    deallocate: u32,
    /// DMRL: the most ranges one Deallocate takes; 0 for none.
    deallocate_ranges: u8,
    refused: bool,
}

impl Controller {
    fn ready(&mut self) -> Result<(), BlockError> {
        if let Some(error) = &self.fault {
            return Err(error.clone());
        }
        if self.shut_down {
            return Err(BlockError::Unsupported("NVMe controller is shut down"));
        }
        Ok(())
    }

    /// CSTS: a removed controller or CSTS.CFS, as an error.
    fn check_fatal(&self) -> Result<(), BlockError> {
        match self.registers.read32(STATUS)? {
            REMOVED => Err(removed()),
            status if status & FATAL != 0 => Err(BlockError::Device { status }),
            _ => Ok(()),
        }
    }

    /// Whether CSTS `status` fails a command at once: CSTS.CFS, unless it
    /// already stood when the controller became ready
    /// ([`Self::fatal_standing`]). A command's timeout still looks at the
    /// raw CSTS, as `nvme_timeout` does.
    fn fails(&self, status: u32) -> bool {
        status & FATAL != 0 && !self.fatal_standing
    }

    fn poison(&mut self, error: BlockError) -> BlockError {
        self.fault = Some(error.clone());
        error
    }

    fn wait_status(&self, timeout: u64, condition: impl Fn(u32) -> bool) -> Result<(), BlockError> {
        let mut failure = None;
        let done = self
            .clock
            .wait(timeout, || match self.registers.read32(STATUS) {
                Ok(REMOVED) => {
                    failure = Some(removed());
                    true
                }
                Ok(status) => condition(status),
                Err(error) => {
                    failure = Some(error);
                    true
                }
            });
        if let Some(error) = failure {
            return Err(error);
        }
        if done {
            Ok(())
        } else {
            Err(BlockError::Timeout)
        }
    }
}

fn removed() -> BlockError {
    BlockError::Unsupported("the NVMe controller is gone")
}

/// What a failed start says of a CSTS with CSTS.CFS set: a register's
/// value, never a command's completion status.
const FATAL_STATUS: &str = "NVMe controller fatal status, CSTS";
const DISABLE_WAIT: &str = "NVMe controller disable, waiting for CSTS.RDY to clear";
const ENABLE_WAIT: &str = "NVMe controller enable, waiting for CSTS.RDY";

/// `error` under `step`, whatever it names already.
fn within(step: &'static str, error: BlockError) -> BlockError {
    BlockError::Step {
        step,
        error: Box::new(error),
    }
}

/// A CSTS read as fatal, said to be one.
fn fatal_status(error: BlockError) -> BlockError {
    match error {
        error @ BlockError::Device { .. } => within(FATAL_STATUS, error),
        other => other,
    }
}

/// The name an admin command's failure is reported under: its opcode and
/// its Identify CNS or its feature.
fn admin_name(opcode: u8, selector: u32) -> &'static str {
    match (opcode, selector & 0xff) {
        (ADMIN_IDENTIFY, IDENTIFY_NAMESPACE) => "NVMe Identify Namespace (opcode 0x06, CNS 0x00)",
        (ADMIN_IDENTIFY, IDENTIFY_CONTROLLER) => "NVMe Identify Controller (opcode 0x06, CNS 0x01)",
        (ADMIN_IDENTIFY, IDENTIFY_ACTIVE_NAMESPACES) => {
            "NVMe Identify Active Namespace List (opcode 0x06, CNS 0x02)"
        }
        (ADMIN_IDENTIFY, IDENTIFY_DESCRIPTORS) => {
            "NVMe Identify Namespace Identification Descriptors (opcode 0x06, CNS 0x03)"
        }
        (ADMIN_IDENTIFY, IDENTIFY_COMMAND_SET_CONTROLLER) => {
            "NVMe Identify Controller, NVM command set (opcode 0x06, CNS 0x06)"
        }
        (ADMIN_IDENTIFY, IDENTIFY_INDEPENDENT_NAMESPACE) => {
            "NVMe Identify Namespace, command set independent (opcode 0x06, CNS 0x08)"
        }
        (ADMIN_IDENTIFY, _) => "NVMe Identify (opcode 0x06)",
        (ADMIN_SET_FEATURES, FEATURE_QUEUE_COUNT) => {
            "NVMe Set Features, Number of Queues (opcode 0x09, feature 0x07)"
        }
        (ADMIN_SET_FEATURES, FEATURE_ASYNC_EVENTS) => {
            "NVMe Set Features, Asynchronous Event Configuration (opcode 0x09, feature 0x0b)"
        }
        (ADMIN_SET_FEATURES, FEATURE_APST) => {
            "NVMe Set Features, Autonomous Power State Transition (opcode 0x09, feature 0x0c)"
        }
        (ADMIN_SET_FEATURES, FEATURE_HOST_MEMORY) => {
            "NVMe Set Features, Host Memory Buffer (opcode 0x09, feature 0x0d)"
        }
        (ADMIN_SET_FEATURES, FEATURE_HOST_BEHAVIOR) => {
            "NVMe Set Features, Host Behavior Support (opcode 0x09, feature 0x16)"
        }
        (ADMIN_SET_FEATURES, _) => "NVMe Set Features (opcode 0x09)",
        (ADMIN_CREATE_COMPLETION, _) => "NVMe Create I/O Completion Queue (opcode 0x05)",
        (ADMIN_CREATE_SUBMISSION, _) => "NVMe Create I/O Submission Queue (opcode 0x01)",
        (ADMIN_DELETE_COMPLETION, _) => "NVMe Delete I/O Completion Queue (opcode 0x04)",
        (ADMIN_DELETE_SUBMISSION, _) => "NVMe Delete I/O Submission Queue (opcode 0x00)",
        (ADMIN_GET_LOG_PAGE, _) => "NVMe Get Log Page (opcode 0x02)",
        (ADMIN_ABORT, _) => "NVMe Abort (opcode 0x08)",
        (ADMIN_ASYNC_EVENT, _) => "NVMe Asynchronous Event Request (opcode 0x0c)",
        _ => "NVMe admin command",
    }
}

fn identify_name(cns: u32) -> &'static str {
    admin_name(ADMIN_IDENTIFY, cns)
}

impl Step {
    /// The name of this step's admin command.
    fn name(self) -> &'static str {
        match self {
            Self::Identify => identify_name(IDENTIFY_CONTROLLER),
            Self::Limits => identify_name(IDENTIFY_COMMAND_SET_CONTROLLER),
            Self::HostOptions => admin_name(ADMIN_SET_FEATURES, FEATURE_HOST_BEHAVIOR),
            Self::Apst => admin_name(ADMIN_SET_FEATURES, FEATURE_APST),
            Self::HostMemory => admin_name(ADMIN_SET_FEATURES, FEATURE_HOST_MEMORY),
            Self::QueueCount => admin_name(ADMIN_SET_FEATURES, FEATURE_QUEUE_COUNT),
            Self::CreateCompletion => admin_name(ADMIN_CREATE_COMPLETION, 0),
            Self::CreateSubmission => admin_name(ADMIN_CREATE_SUBMISSION, 0),
            Self::Events => admin_name(ADMIN_SET_FEATURES, FEATURE_ASYNC_EVENTS),
        }
    }
}

/// One namespace of a controller.
pub struct Namespace {
    controller: Rc<RefCell<Controller>>,
    namespace: u32,
    identity: Identity,
    ids: Ids,
    /// The controller scan count this namespace was last scanned at.
    scanned: u64,
    /// Set once a rescan removed the namespace.
    removed: Option<BlockError>,
}

/// Reset `function`'s controller, bring up its queues and return its usable
/// namespaces. `platform` names the board for Linux's board quirks.
pub fn attach(
    function: &mut Function,
    handoff: &Handoff,
    dma: &mut dyn DmaAllocator,
    minimum: &(impl crate::bus::pci::power::Timer + ?Sized),
    platform: &SystemIdentity<'_>,
) -> Result<Attached, BlockError> {
    minimum.check().map_err(BlockError::PciPower)?;
    validate_clock(handoff.clock)?;
    function.power_up(minimum)?;
    let bars = function.bars()?;
    let registers = RegisterIo::Legacy(registers(handoff, bars[0], 0, 0x2000)?);
    function.enable_polled()?;
    let signalled = match (handoff.completion, function.interrupts()?.msix) {
        (Some(message), Some(msix)) => {
            let table = crate::registers(
                handoff,
                bars[usize::from(msix.table_bar)],
                u64::from(msix.table_offset),
                MSIX_ENTRY_BYTES,
            )?;
            function.enable_msix(msix, || {
                table.write32(0, message.address as u32);
                table.write32(4, (message.address >> 32) as u32);
                table.write32(8, message.data);
                table.write32(12, 0);
                Ok(())
            })?;
            true
        }
        _ => false,
    };
    let mut controller = Controller {
        registers,
        doorbell_stride: 0,
        admin: Queue::new(0, ADMIN_ENTRIES, dma)?,
        io: Queue::new(1, IO_ENTRIES, dma)?,
        admin_data: Buffer::Legacy(dma.allocate(PAGE, PAGE)?),
        pool: Pool::legacy(dma)?,
        held: (0..DEPTH).map(|_| Vec::new()).collect(),
        tags: (0..DEPTH).map(|_| Tag::Free).collect(),
        recovery: None,
        signalled,
        owed: false,
        max_transfer: BOUNCE_BYTES,
        max_command_bytes: MAX_COMMAND_BYTES,
        shutdown_timeout_us: SHUTDOWN_MINIMUM_US,
        shut_down: false,
        command_id: 0,
        clock: handoff.clock,
        command_timeout_us: COMMAND_TIMEOUT_US,
        admin_timeout_us: ADMIN_TIMEOUT_US,
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
        quirks: Quirks::for_device(function.vendor, function.device)
            | Quirks::for_platform(function.vendor, function.device, platform),
    };
    controller.initialize()?;
    discover(controller, format!("{}", function.address), &mut { dma })
}

/// Linux `nvme_wait_ready`'s bound for a ready timeout in 500 ms units:
/// whole seconds, rounded up.
fn ready_wait_us(units: u64) -> u64 {
    units.div_ceil(2) * MICROSECONDS_PER_SECOND
}

pub(crate) fn validate_clock(clock: Clock) -> Result<(), BlockError> {
    // CRTO.CRWMT permits 65535 half-second intervals, beyond CAP.TO's 255.
    // The fixed command and maximum shutdown policies are shorter; admit
    // every later native wait upfront.
    clock.check_duration(ready_wait_us(0xffff)).map_err(|_| {
        BlockError::Unsupported("NVMe requires a calibrated, representable operation clock")
    })
}

fn discover(
    mut controller: Controller,
    location: alloc::string::String,
    host_memory: &mut dyn HostMemorySource,
) -> Result<Attached, BlockError> {
    // A refusal that ends the start names the command it refused.
    let identify = controller
        .identify(IDENTIFY_CONTROLLER, 0)
        .map_err(|error| error.at(identify_name(IDENTIFY_CONTROLLER)))?
        .to_vec();
    let request = HostMemoryRequest::from_identify(&identify);
    let serial = ascii(&identify[CONTROLLER_SERIAL]);
    let model = ascii(&identify[CONTROLLER_MODEL]);
    let rtd3_entry_us = u64::from(le32(&identify[CONTROLLER_RTD3_ENTRY], 0));
    let namespaces = le32(&identify[CONTROLLER_NAMESPACES], 0);
    controller.commands = u16::from_le_bytes([
        identify[CONTROLLER_COMMANDS],
        identify[CONTROLLER_COMMANDS + 1],
    ]);
    // Linux `nvme_init_identify` takes a present cache as one to flush and
    // leaves the controller's Volatile Write Cache feature as it is.
    let write_cache = identify[CONTROLLER_VOLATILE_CACHE] & 1 != 0;
    // Linux `nvme_init_identify` adds `core_quirks` once, from the first
    // Identify Controller.
    controller.quirks = controller.quirks | Quirks::for_controller(&identify);
    controller.shutdown_timeout_us = shutdown_timeout_us(rtd3_entry_us);
    controller.identified(&identify);
    controller.allocate_host_memory(request, host_memory);
    let version = controller.registers.read32(VERSION)?;
    controller.version = version;
    // The rest of the bring-up, as a reset repeats it: the NVM limits, host
    // options, APST, the host memory buffer, the I/O queues and events.
    controller.bring_up(Step::Limits, identify)?;
    controller.settle()?;

    // Linux `nvme_scan_work`: the active list from NVMe 1.1, otherwise (or
    // when the controller refuses it) every namespace ID in turn.
    let listed = if version >= VERSION_1_1 {
        controller
            .identify(IDENTIFY_ACTIVE_NAMESPACES, 0)
            .map(|list| {
                list.as_chunks::<4>()
                    .0
                    .iter()
                    .map(|word| u32::from_le_bytes(*word))
                    .take_while(|namespace| *namespace != 0)
                    .collect::<Vec<u32>>()
            })
            .ok()
    } else {
        None
    };
    controller.ready()?;
    let (candidates, sequential) = match listed {
        Some(active) => (active, false),
        None => ((1..=namespaces.min(SEQUENTIAL_SCAN_LIMIT)).collect(), true),
    };
    let mut attached = Attached {
        namespaces: Vec::new(),
        failures: Vec::new(),
        quirks: controller.quirks,
    };
    let mut found = Vec::new();
    let (max_transfer, max_command_bytes, commands, scans, limits) = (
        controller.max_transfer,
        controller.max_command_bytes,
        controller.commands,
        controller.scans,
        controller.limits,
    );
    for namespace in candidates {
        let Some((mut ids, independent)) = controller.namespace_ids(namespace)? else {
            continue;
        };
        // Zoned and other command sets need drivers Stela does not have.
        if ids.csi != CSI_NVM {
            attached
                .failures
                .push((namespace, BlockError::Unsupported("NVMe I/O command set")));
            continue;
        }
        let data = match controller.identify(IDENTIFY_NAMESPACE, namespace) {
            Ok(data) => data,
            // Linux `nvme_scan_ns` passes over a namespace whose Identify
            // the controller refuses, and keeps the controller: a
            // sequential scan meets the IDs of no namespace this way.
            Err(error @ BlockError::Device { .. }) => {
                if !sequential {
                    let error = error.at(identify_name(IDENTIFY_NAMESPACE));
                    attached.failures.push((namespace, error));
                }
                continue;
            }
            Err(error) => return Err(error),
        };
        if !independent {
            ids.fill(version, data);
        }
        match plain_format(data, max_transfer) {
            // An inactive namespace reads as all zeros in a sequential scan.
            Ok(None) if sequential => {}
            Ok(None) => attached
                .failures
                .push((namespace, BlockError::Unsupported("empty NVMe namespace"))),
            Ok(Some(format)) => found.push((namespace, format, ids)),
            Err(error) => attached.failures.push((namespace, error)),
        }
    }
    let controller = Rc::new(RefCell::new(controller));
    let first = found.iter().map(|&(namespace, ..)| namespace).min();
    attached.namespaces = found
        .into_iter()
        .map(|(namespace, format, ids)| Namespace {
            controller: controller.clone(),
            namespace,
            ids,
            scanned: scans,
            removed: None,
            identity: Identity {
                transport: Transport::Nvme,
                location: format!("{location} namespace {namespace}"),
                model: model.clone(),
                serial: namespace_serial(&serial, namespace, first),
                blocks: format.blocks,
                block_size: format.block_size,
                read_only: format.read_only,
                features: features(
                    write_cache,
                    commands,
                    limits,
                    &format,
                    TransferLimits {
                        data: max_transfer,
                        command: max_command_bytes,
                    },
                ),
            },
        })
        .collect();
    Ok(attached)
}

/// The APST table Linux `nvme_configure_apst` gives a controller that
/// supports it, and whether APST is enabled with it, or `None`: from the
/// deepest power state up, each state goes to the deepest usable
/// non-operational state below it once idle for its timeout, at most one
/// state per latency tolerance. A state whose exit latency exceeds
/// `default_ps_max_latency_us` is never a target, nor the deepest state
/// under `NVME_QUIRK_NO_DEEPEST_PS`; `NVME_QUIRK_NO_APST` turns APST off
/// with an empty table.
fn apst_table(identify: &[u8], quirks: Quirks) -> Option<([u64; 32], bool)> {
    let deepest = usize::from(identify[CONTROLLER_POWER_STATES]);
    if identify[CONTROLLER_APST] & 1 == 0 || deepest > 31 {
        return None;
    }
    let mut table = [0; 32];
    if quirks.contains(Quirks::NO_APST) {
        return Some((table, false));
    }
    let mut target = 0;
    let mut tolerance = u32::MAX;
    for state in (0..=deepest).rev() {
        table[state] = target;
        if state == deepest && quirks.contains(Quirks::NO_DEEPEST_POWER_STATE) {
            continue;
        }
        let descriptor = &identify[CONTROLLER_POWER_DESCRIPTORS + 32 * state..][..32];
        if descriptor[3] & POWER_NON_OPERATIONAL == 0 {
            continue;
        }
        let exit = u64::from(le32(descriptor, 8));
        if exit > APST_MAX_LATENCY_US {
            continue;
        }
        let total = exit + u64::from(le32(descriptor, 4));
        let idle = if total <= APST_PRIMARY_TOLERANCE_US && tolerance != 1 {
            tolerance = 1;
            APST_PRIMARY_TIMEOUT_MS
        } else if total > APST_PRIMARY_TOLERANCE_US
            && total <= APST_SECONDARY_TOLERANCE_US
            && tolerance > 2
        {
            tolerance = 2;
            APST_SECONDARY_TIMEOUT_MS
        } else {
            continue;
        };
        target = (state as u64) << 3 | idle << 8;
    }
    Some((table, true))
}

/// Payload storage and logical command-range limits are separate: a no-data
/// command does not consume the smaller DMA bounce-buffer allowance.
#[derive(Clone, Copy)]
struct TransferLimits {
    data: usize,
    command: usize,
}

/// A namespace's features, as Linux `nvme_update_disk_info` sets them:
/// FUA with a volatile write cache; Deallocate as discard with Dataset
/// Management, up to DMRL ranges or else 256, each of up to DMRSL blocks or
/// else `UINT_MAX` sectors;
/// and Write Zeroes up to WZSL, or else the controller's command range
/// (`nvme_init_non_mdts_limits`), which may deallocate where the namespace
/// reads deallocated blocks as zeros and says Write Zeroes deallocates
/// (`NVME_NS_DEAC`). A Write Zeroes command takes at most 2^16 blocks.
fn features(
    write_cache: bool,
    commands: u16,
    limits: Limits,
    format: &Format,
    transfer: TransferLimits,
) -> Features {
    let block_shift = format.block_size.trailing_zeros();
    let sectors = u64::from(u32::MAX) >> (block_shift - 9);
    let deallocate = u64::from(limits.deallocate);
    let zeroes = match limits.write_zeroes {
        0 => transfer.command as u64,
        // 2^WZSL minimum pages of 4 KiB, saturating as nvme_mps_to_sectors.
        units => 1u64
            .checked_shl(u32::from(units) + 12)
            .filter(|bytes| *bytes >> 9 <= u64::from(u32::MAX))
            .unwrap_or(u64::from(u32::MAX) << 9),
    };
    Features {
        write_cache,
        fua: write_cache,
        max_transfer: transfer.data,
        max_read_transfer: None,
        discard: (commands & COMMANDS_DATASET_MANAGEMENT != 0).then_some(DiscardLimits {
            blocks: if deallocate != 0 && deallocate <= sectors {
                deallocate
            } else {
                sectors
            },
            ranges: match limits.deallocate_ranges {
                0 => MAX_DEALLOCATE_RANGES,
                ranges => usize::from(ranges).min(MAX_DEALLOCATE_RANGES),
            },
        }),
        write_zeroes: (commands & COMMANDS_WRITE_ZEROES != 0).then_some(WriteZeroes {
            blocks: (zeroes >> block_shift).min(1 << 16),
            unmap: format.deallocate & 0x7 == 1 && format.deallocate & 1 << 3 != 0,
        }),
    }
}

/// The namespaces of one controller, those it cannot serve, and the
/// quirks it runs with.
pub struct Attached {
    pub namespaces: Vec<Namespace>,
    pub failures: Vec<(u32, BlockError)>,
    pub quirks: Quirks,
}

/// A namespace's size and its in-use LBA format.
struct Format {
    blocks: u64,
    block_size: u32,
    read_only: bool,
    /// DLFEAT.
    deallocate: u8,
}

/// The format of an Identify Namespace page: none for an empty namespace,
/// and a refusal for formats this driver cannot transfer.
fn plain_format(data: &[u8], max_transfer: usize) -> Result<Option<Format>, BlockError> {
    let blocks = u64::from_le_bytes(data[NAMESPACE_SIZE].try_into().unwrap());
    if blocks == 0 {
        return Ok(None);
    }
    let flbas = data[NAMESPACE_FORMAT];
    let index = usize::from(flbas & FORMAT_LOW | (flbas & FORMAT_HIGH) >> 1);
    let descriptor = le32(data, NAMESPACE_FORMATS + 4 * index);
    let metadata = descriptor & 0xffff;
    let shift = descriptor >> 16 & 0xff;
    if metadata != 0 {
        return Err(BlockError::Unsupported("NVMe format with metadata"));
    }
    if !(9..=16).contains(&shift) || 1usize << shift > max_transfer {
        return Err(BlockError::Unsupported("NVMe logical block size"));
    }
    Ok(Some(Format {
        blocks,
        block_size: 1 << shift,
        read_only: data[NAMESPACE_ATTRIBUTES] & NAMESPACE_WRITE_PROTECTED != 0,
        deallocate: data[NAMESPACE_DEALLOCATE],
    }))
}

/// Linux's shutdown bound: the RTD3 entry latency in whole seconds, between
/// five seconds and a minute.
fn shutdown_timeout_us(rtd3_entry_us: u64) -> u64 {
    rtd3_entry_us
        .div_ceil(MICROSECONDS_PER_SECOND)
        .saturating_mul(MICROSECONDS_PER_SECOND)
        .clamp(SHUTDOWN_MINIMUM_US, SHUTDOWN_MAXIMUM_US)
}

impl Namespace {
    fn chunk_bytes(&self, controller: &Controller) -> usize {
        let block = self.identity.block_size as usize;
        controller.max_transfer / block * block
    }

    /// Linux `nvme_scan_ns` and `nvme_validate_ns` after a controller reset
    /// or namespace-change notice, once asynchronous events are acted on:
    /// a namespace no longer allocated, or whose identifiers changed, is
    /// removed; otherwise its size and format are updated. A refused
    /// Identify leaves the namespace as it was. Admin commands of its own:
    /// only while no command runs.
    fn rescan(&mut self, controller: &mut Controller) -> Result<(), BlockError> {
        controller.ready()?;
        if let Some(error) = &self.removed {
            return Err(error.clone());
        }
        controller.service_events()?;
        if self.scanned == controller.scans {
            return Ok(());
        }
        let scans = controller.scans;
        let Some((mut ids, independent)) = controller.namespace_ids(self.namespace)? else {
            self.scanned = scans;
            return Ok(());
        };
        let (version, max_transfer, max_command_bytes) = (
            controller.version,
            controller.max_transfer,
            controller.max_command_bytes,
        );
        let (commands, limits) = (controller.commands, controller.limits);
        let data = match controller.identify(IDENTIFY_NAMESPACE, self.namespace) {
            Ok(data) => data,
            Err(BlockError::Device { .. }) => {
                self.scanned = scans;
                return Ok(());
            }
            Err(error) => return Err(error),
        };
        if !independent {
            ids.fill(version, data);
        }
        let allocated = data[NAMESPACE_CAPACITY] != [0; 8];
        let format = plain_format(data, max_transfer);
        self.scanned = scans;
        match format {
            Ok(Some(format)) if allocated && ids == self.ids => {
                self.identity.blocks = format.blocks;
                self.identity.block_size = format.block_size;
                self.identity.read_only = format.read_only;
                let write_cache = self.identity.features.write_cache;
                self.identity.features = features(
                    write_cache,
                    commands,
                    limits,
                    &format,
                    TransferLimits {
                        data: max_transfer,
                        command: max_command_bytes,
                    },
                );
                Ok(())
            }
            Err(error) if allocated && ids == self.ids => {
                // nvme_update_ns_info hides a namespace it cannot use.
                self.removed = Some(error.clone());
                Err(error)
            }
            _ => {
                self.removed = Some(NAMESPACE_REMOVED);
                Err(NAMESPACE_REMOVED)
            }
        }
    }

    /// Run `command` alone once rescanned, waiting for it.
    fn command(&mut self, command: Command) -> Finished {
        self.command_to(command, None)
    }

    fn command_to(&mut self, command: Command, destination: Option<&mut [u8]>) -> Finished {
        let controller = self.controller.clone();
        let mut controller = controller.borrow_mut();
        self.rescan(&mut controller)?;
        let command = command.within(&self.identity, 0)?;
        let block = self.identity.block_size as usize;
        controller.run(self.namespace, block, command, destination)
    }

    fn write_with(&mut self, lba: u64, data: &[u8], fua: bool) -> Result<(), BlockError> {
        let block = self.identity.block_size as usize;
        let chunk = self.chunk_bytes(&self.controller.borrow());
        for (index, part) in data.chunks(chunk).enumerate() {
            let lba = lba + (index * chunk / block) as u64;
            let data = part.to_vec();
            self.command(Command::Write { lba, data, fua })?;
        }
        Ok(())
    }
}

impl BlockDevice for Namespace {
    fn identity(&self) -> &Identity {
        &self.identity
    }

    fn read(&mut self, lba: u64, buffer: &mut [u8]) -> Result<(), BlockError> {
        let block = self.identity.block_size as usize;
        let chunk = self.chunk_bytes(&self.controller.borrow());
        for (index, part) in buffer.chunks_mut(chunk).enumerate() {
            let lba = lba + (index * chunk / block) as u64;
            let bytes = part.len();
            self.command_to(Command::Read { lba, bytes }, Some(part))?;
        }
        Ok(())
    }

    fn write(&mut self, lba: u64, data: &[u8]) -> Result<(), BlockError> {
        self.write_with(lba, data, false)
    }

    fn write_fua(&mut self, lba: u64, data: &[u8]) -> Result<(), BlockError> {
        self.write_with(lba, data, true)
    }

    fn flush_cache(&mut self) -> Result<(), BlockError> {
        self.command(Command::Flush).map(drop)
    }

    fn discard(&mut self, lba: u64, blocks: u64) -> Result<(), BlockError> {
        self.command(Command::discard(lba, blocks)).map(drop)
    }

    fn write_zeroes(&mut self, lba: u64, blocks: u64, unmap: bool) -> Result<(), BlockError> {
        self.command(Command::WriteZeroes { lba, blocks, unmap })
            .map(drop)
    }

    /// One transfer runs as it is, a read handing back its bounce copy.
    fn execute(&mut self, command: Command) -> Finished {
        let chunk = self.chunk_bytes(&self.controller.borrow());
        match command {
            Command::Read { lba, bytes } if bytes > chunk => {
                let mut data = alloc::vec![0; bytes];
                self.read(lba, &mut data).map(|()| Some(data))
            }
            Command::Write { lba, data, fua } if data.len() > chunk => {
                self.write_with(lba, &data, fua).map(|()| None)
            }
            command => self.command(command),
        }
    }

    /// The controller's tags, shared by its namespaces: [`Self::finish`]
    /// looks for completions in memory, never at a register, until a
    /// command's timeout.
    fn depth(&self) -> usize {
        self.controller.borrow().tags.len()
    }

    fn room(&self) -> bool {
        self.removed.is_some() || self.controller.borrow().room()
    }

    /// Asynchronous events and rescans run admin commands, which wait: only
    /// while no command runs, as Linux's work items run between requests.
    fn start(&mut self, command: Command) -> Result<u32, BlockError> {
        self.start_with(command, true)
    }

    /// As [`Self::start`], its doorbell left to [`Self::commit`] or the
    /// next command that rings.
    fn start_more(&mut self, command: Command) -> Result<u32, BlockError> {
        self.start_with(command, false)
    }

    fn commit(&mut self) {
        self.controller.borrow_mut().commit();
    }

    fn finish(&mut self, tag: u32) -> Option<Finished> {
        self.controller.borrow_mut().finish(tag as usize)
    }

    /// A finish reaps the whole completion queue.
    fn reaps_together(&self) -> bool {
        true
    }

    fn finished(&mut self, tag: u32) -> Option<Finished> {
        self.controller.borrow_mut().finished(tag as usize)
    }

    fn posted(&self) -> bool {
        self.controller.borrow().posted_completion()
    }

    fn signals(&self) -> bool {
        self.controller.borrow().signalled
    }

    /// Flush, then shut the controller down once for all its namespaces.
    fn shutdown(&mut self, _power_off: bool) -> Result<(), BlockError> {
        if !self.controller.borrow().shut_down {
            self.flush()?;
        }
        self.controller.borrow_mut().shutdown()
    }
}

impl Namespace {
    /// Asynchronous events and rescans run admin commands, which wait: only
    /// while no command runs, as Linux's work items run between requests.
    /// The doorbell rings at once when `ring`.
    fn start_with(&mut self, command: Command, ring: bool) -> Result<u32, BlockError> {
        let controller = self.controller.clone();
        let mut controller = controller.borrow_mut();
        if !controller.busy() {
            parts::timed(parts::Part::Rescan, || self.rescan(&mut controller))?;
        } else if let Some(error) = &self.removed {
            return Err(error.clone());
        }
        let command = command.within(&self.identity, 0)?;
        let block = self.identity.block_size as usize;
        parts::timed(parts::Part::Start, || {
            controller.start_with(self.namespace, block, command, ring)
        })
    }
}

/// The serial a namespace's disk is named by: the controller's for its
/// first namespace, and with `/n` and the namespace ID for the others, so
/// that a serial names one disk.
fn namespace_serial(serial: &str, namespace: u32, first: Option<u32>) -> alloc::string::String {
    if Some(namespace) == first {
        alloc::string::String::from(serial)
    } else {
        format!("{serial}/n{namespace}")
    }
}

#[cfg(test)]
mod tests;
