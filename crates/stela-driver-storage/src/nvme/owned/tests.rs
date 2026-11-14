// SPDX-License-Identifier: GPL-2.0-only
// Copyright (c) 2011-2014, Intel Corporation.
mod limits;
pub(crate) mod protocol;
use super::*;
use crate::bus::dma::{
    mapping::DomainMapping,
    memory::{MemoryError, PageBacking, PinnedPages},
    pages::DmaPages,
};
use alloc::{
    alloc::{alloc_zeroed, dealloc},
    collections::BTreeMap,
};
use core::{
    alloc::Layout,
    ops::Range,
    ptr::NonNull,
    sync::atomic::{AtomicU64, AtomicUsize},
};
extern crate std;
use std::sync::Mutex;

struct Backing {
    pointer: NonNull<u8>,
    pages: Vec<u64>,
    bytes: usize,
    released: Arc<AtomicUsize>,
}
// SAFETY: exclusive aligned model allocation, kept by pins; raw
// device/CPU effects are serialized by the model and ownership protocol.
unsafe impl Send for Backing {}
// SAFETY: same serialized model accesses, no safe overlapping slices.
unsafe impl Sync for Backing {}
// SAFETY: these physical numbers are explicitly model metadata; cpu_base
// names independently allocated, aligned, kept zeroed model RAM.
unsafe impl PageBacking for Backing {
    fn cpu_base(&self) -> NonNull<u8> {
        self.pointer
    }
    fn physical_pages(&self) -> &[u64] {
        &self.pages
    }
    fn page_shift(&self) -> u8 {
        12
    }
    fn cache_attribute(&self) -> CacheAttribute {
        CacheAttribute::WriteBack
    }
}
impl Drop for Backing {
    fn drop(&mut self) {
        self.released.fetch_add(1, Ordering::SeqCst);
        // SAFETY: exact unique allocation kept until its final pin.
        unsafe {
            dealloc(
                self.pointer.as_ptr(),
                Layout::from_size_align(self.bytes, PAGE).unwrap(),
            )
        };
    }
}
struct Source {
    next: AtomicU64,
    released: Arc<AtomicUsize>,
}
impl PageSource for Source {
    fn allocate(
        &self,
        bytes: usize,
        alignment: usize,
        attribute: CacheAttribute,
    ) -> Result<PinnedPages, MemoryError> {
        assert_eq!((alignment, attribute), (PAGE, CacheAttribute::WriteBack));
        let bytes = bytes.next_multiple_of(PAGE);
        let layout = Layout::from_size_align(bytes, PAGE).unwrap();
        // SAFETY: aligned model RAM, checked for allocation failure.
        let pointer =
            NonNull::new(unsafe { alloc_zeroed(layout) }).ok_or(MemoryError::OutOfMemory)?;
        let first = self.next.fetch_add((bytes * 2) as u64, Ordering::SeqCst);
        let pages = (0..bytes / PAGE)
            .map(|n| first + (n * PAGE * 2) as u64)
            .collect();
        PinnedPages::new(
            Box::new(Backing {
                pointer,
                pages,
                bytes,
                released: self.released.clone(),
            }),
            bytes,
            alignment,
        )
    }
}
struct Lease {
    cpu: CpuMapping,
    token: DomainMapping,
    direction: Direction,
    device: Vec<u8>,
    visible: bool,
}
/// The buffers an owned controller maps: four queues, the admin page and
/// the I/O pool.
const BUFFERS: usize = 5 + POOL_PAGES;
#[derive(Clone, Copy, Default)]
enum Mutation {
    #[default]
    None,
    Cid,
    Sqid,
    Head,
    Phase,
    Delayed,
}
struct Model {
    access: Option<Arc<dyn protocol::Access>>,
    registers: BTreeMap<usize, u32>,
    maps: BTreeMap<u64, Lease>,
    next: u64,
    queues: [(u64, u64); 2],
    heads: [u16; 2],
    phases: [bool; 2],
    /// Submission queue heads; the admin one runs ahead of its completion
    /// head while an Asynchronous Event Request is held.
    sq_heads: [u16; 2],
    disk: Vec<u8>,
    /// Namespace 1's size and capacity, and its EUI64.
    blocks: u64,
    eui64: [u8; 8],
    /// CRDT1–3, and the Host Behavior Support pages set.
    retry_delays: [u16; 3],
    host_behavior: Vec<[u8; 3]>,
    /// HMPRE, HMMIN, HMMINDS and HMMAXD; Host Memory Buffer feature
    /// commands as CDW11-15 with the first descriptor; refuse them.
    host_memory: (u32, u32, u32, u16),
    host_memory_sets: Vec<([u32; 5], (u64, u32))>,
    refuse_host_memory: bool,
    /// APSTA with a non-operational PS1; APSTE and the first entry of each
    /// APST table set.
    apst: bool,
    apst_sets: Vec<(bool, u64)>,
    /// Identify Controller's VID and model, the model space-padded.
    vendor: u16,
    model: &'static str,
    /// OAES; the held Asynchronous Event Request; Asynchronous Event
    /// Configurations set and log pages read.
    async_events: u32,
    event_request: Option<u32>,
    event_configs: Vec<u32>,
    logs: Vec<u32>,
    /// The namespace's command set and NSTAT.NRDY.
    csi: u8,
    not_ready: bool,
    prepared: bool,
    bme: bool,
    enabled: bool,
    drained: bool,
    outstanding: bool,
    stopped: bool,
    retired: bool,
    released: bool,
    mutation: Mutation,
    fail_read: bool,
    fail_after_doorbell: bool,
    fail_device_sync: bool,
    fail_cpu_sync: bool,
    fail_drain: bool,
    fail_unmap: bool,
    fail_unmap_at: Option<usize>,
    unmap_attempts: usize,
    fail_prepare: bool,
    fail_retire: bool,
    fail_release: bool,
    mmio_accesses: usize,
    register_owner_drops: usize,
    lifecycle_owner_drops: usize,
    fail_disable: bool,
    stuck_ready: bool,
    /// CSTS.RDY never follows CC.EN: the controller never answers.
    never_ready: bool,
    /// After each enable, this many CSTS reads report CSTS.CFS without
    /// CSTS.RDY before the controller is ready; `usize::MAX` for ever.
    fatal_while_enabling: usize,
    enabling_reads: usize,
    coherent: bool,
    events: Vec<&'static str>,
    commands: usize,
    sync_cpu: usize,
    pending: Option<(usize, [u32; 16])>,
    /// Which queues hold an outstanding command.
    busy: [bool; 2],
    /// I/O commands still to hold unanswered, one each.
    delay_io: usize,
    /// Hold every I/O command, however many, until [`Model::complete_held`].
    hold_io: bool,
    held_io: Vec<[u32; 16]>,
    /// Whether an Abort completes the held command.
    abortable: bool,
    /// Statuses for the next I/O completions.
    statuses: alloc::collections::VecDeque<u32>,
    /// CSTS.CFS, until the controller is reset.
    fatal: bool,
    aborts: usize,
    /// ONCS, and the NVM Identify Controller's WZSL and DMRSL; none
    /// refuses the Identify.
    oncs: u16,
    mdts: u8,
    nvm_limits: Option<(u8, u32)>,
    nvm_identifies: usize,
    /// Number of Queues features set; whether one was since the last
    /// controller reset; refuse them; and refuse an I/O queue created
    /// before one, with Invalid Field in Command.
    /// Refuse the Identify with this CNS, with Invalid Field in Command.
    refuse_identify: Option<u32>,
    /// Leave every admin command unanswered; and report CSTS.CFS once one
    /// is.
    ignore_admin: bool,
    fatal_on_admin: bool,
    queue_counts: usize,
    queue_count_set: bool,
    refuse_queue_count: bool,
    queues_need_count: bool,
}
impl Model {
    fn new() -> Self {
        let mut registers = BTreeMap::new();
        // Model literal CAP: NVM supported,16-entry depth,500ms timeout.
        registers.insert(0, 15 | 1 << 24);
        registers.insert(4, 1 << 5);
        registers.insert(8, 0x0001_0100);
        Self {
            access: None,
            registers,
            maps: BTreeMap::new(),
            next: 1,
            queues: [(0, 0); 2],
            heads: [0; 2],
            phases: [true; 2],
            sq_heads: [0; 2],
            disk: vec![0; 1024 * 512],
            blocks: 1024,
            eui64: [0x11; 8],
            retry_delays: [0; 3],
            host_behavior: Vec::new(),
            host_memory: (0, 0, 0, 0),
            host_memory_sets: Vec::new(),
            refuse_host_memory: false,
            apst: false,
            apst_sets: Vec::new(),
            vendor: 0,
            model: "",
            async_events: 0,
            event_request: None,
            event_configs: Vec::new(),
            logs: Vec::new(),
            csi: 0,
            not_ready: false,
            prepared: false,
            bme: false,
            enabled: false,
            drained: false,
            outstanding: false,
            stopped: true,
            retired: false,
            released: false,
            mutation: Mutation::None,
            fail_read: false,
            fail_after_doorbell: false,
            fail_device_sync: false,
            fail_cpu_sync: false,
            fail_drain: false,
            fail_unmap: false,
            fail_unmap_at: None,
            unmap_attempts: 0,
            fail_prepare: false,
            fail_retire: false,
            fail_release: false,
            mmio_accesses: 0,
            register_owner_drops: 0,
            lifecycle_owner_drops: 0,
            fail_disable: false,
            stuck_ready: false,
            never_ready: false,
            fatal_while_enabling: 0,
            enabling_reads: 0,
            coherent: true,
            events: Vec::new(),
            commands: 0,
            sync_cpu: 0,
            pending: None,
            busy: [false; 2],
            delay_io: 0,
            hold_io: false,
            held_io: Vec::new(),
            abortable: false,
            statuses: Default::default(),
            fatal: false,
            aborts: 0,
            oncs: 0,
            mdts: 0,
            nvm_limits: None,
            nvm_identifies: 0,
            refuse_identify: None,
            ignore_admin: false,
            fatal_on_admin: false,
            queue_counts: 0,
            queue_count_set: false,
            refuse_queue_count: false,
            queues_need_count: true,
        }
    }
    fn registers_available(&self) -> bool {
        self.access
            .as_ref()
            .map_or(self.prepared, |a| a.registers_available())
    }
    fn dma_enabled(&self) -> bool {
        self.access.as_ref().map_or(self.bme, |a| a.dma_enabled())
    }
    fn any_bus_master(&self) -> bool {
        self.access
            .as_ref()
            .map_or(self.bme, |a| a.any_bus_master())
    }
    fn record(&mut self, event: &'static str) {
        self.events.push(event);
        if let Some(access) = &self.access {
            access.observe(event);
        }
    }
    fn register64(&self, offset: usize) -> u64 {
        u64::from(*self.registers.get(&offset).unwrap_or(&0))
            | u64::from(*self.registers.get(&(offset + 4)).unwrap_or(&0)) << 32
    }
    fn read_iova(&self, address: u64, bytes: usize) -> Vec<u8> {
        let lease = self
            .maps
            .values()
            .find(|m| {
                address >= m.token.address
                    && address
                        .checked_add(bytes as u64)
                        .is_some_and(|end| end <= m.token.address + m.token.bytes as u64)
            })
            .expect("actual mapped IOVA");
        assert!(lease.visible, "device read before cache visibility");
        assert_ne!(lease.direction, Direction::FromDevice);
        let at = (address - lease.token.address) as usize;
        lease.device[at..at + bytes].to_vec()
    }
    fn write_iova(&mut self, address: u64, data: &[u8]) {
        let lease = self
            .maps
            .values_mut()
            .find(|m| {
                address >= m.token.address
                    && address
                        .checked_add(data.len() as u64)
                        .is_some_and(|end| end <= m.token.address + m.token.bytes as u64)
            })
            .expect("actual mapped writable IOVA");
        assert!(lease.visible, "device write before ownership transfer");
        assert_ne!(lease.direction, Direction::ToDevice);
        let at = (address - lease.token.address) as usize;
        lease.device[at..at + data.len()].copy_from_slice(data);
    }
    fn pages(&self, command: &[u32; 16], bytes: usize) -> Vec<u64> {
        let first = u64::from(command[6]) | u64::from(command[7]) << 32;
        let second = u64::from(command[8]) | u64::from(command[9]) << 32;
        let mut out = vec![first];
        let count = bytes.div_ceil(4096);
        if count == 2 {
            out.push(second);
        } else if count > 2 {
            let list = self.read_iova(second, (count - 1) * 8);
            out.extend(
                list.as_chunks::<8>()
                    .0
                    .iter()
                    .map(|b| u64::from_le_bytes(*b)),
            );
        }
        out
    }
    fn process(&mut self, queue: usize, tail: u16) {
        let holding = queue == 1 && self.hold_io;
        assert!(
            self.registers_available()
                && self.dma_enabled()
                && self.enabled
                && (holding || !self.busy[queue])
        );
        assert_eq!(tail, (self.sq_heads[queue] + 1) % 16);
        let bytes = self.read_iova(
            self.queues[queue].0 + u64::from(self.sq_heads[queue]) * 64,
            64,
        );
        self.sq_heads[queue] = tail;
        let command: [u32; 16] = core::array::from_fn(|i| {
            u32::from_le_bytes(bytes[i * 4..i * 4 + 4].try_into().unwrap())
        });
        self.commands += 1;
        if queue == 0 && command[0] as u8 == 0x0c {
            assert_eq!(self.event_request.replace(command[0] >> 16), None);
            return;
        }
        self.busy[queue] = true;
        self.outstanding = true;
        self.drained = false;
        self.record("command");
        if queue == 0 && self.ignore_admin {
            self.fatal |= self.fatal_on_admin;
            return;
        }
        if holding {
            self.held_io.push(command);
            return;
        }
        if queue == 1 && (matches!(self.mutation, Mutation::Delayed) || self.delay_io > 0) {
            self.delay_io = self.delay_io.saturating_sub(1);
            self.pending = Some((queue, command));
            return;
        }
        self.complete(queue, command);
    }
    fn complete(&mut self, queue: usize, command: [u32; 16]) {
        let mut result = 0u32;
        let mut status = 0u32;
        if queue == 1 {
            status = self.statuses.pop_front().unwrap_or(0);
        }
        match (queue, command[0] as u8) {
            // Abort: complete the held command with Command Abort
            // Requested, or report it not aborted (Dword 0 bit 0).
            (0, 8) => {
                self.aborts += 1;
                let held = self
                    .pending
                    .filter(|(q, c)| command[10] == (c[0] >> 16) << 16 | *q as u32);
                if self.abortable
                    && let Some((q, c)) = held
                {
                    self.pending = None;
                    self.statuses.push_front(7);
                    self.complete(q, c);
                } else {
                    result = 1;
                }
            }
            (0, 6) if self.refuse_identify == Some(command[10]) => status = 2,
            (0, 6) => {
                let mut data = vec![0; 4096];
                match command[10] {
                    1 => {
                        data[..2].copy_from_slice(&self.vendor.to_le_bytes());
                        data[4..24].fill(b'S');
                        data[24..64].fill(b'M');
                        if !self.model.is_empty() {
                            data[24..64].fill(b' ');
                            data[24..24 + self.model.len()].copy_from_slice(self.model.as_bytes());
                        }
                        data[77] = self.mdts;
                        data[516..520].copy_from_slice(&1u32.to_le_bytes());
                        data[520..522].copy_from_slice(&self.oncs.to_le_bytes());
                        data[525] = 1; // A volatile write cache.
                        for (index, delay) in self.retry_delays.iter().enumerate() {
                            data[128 + 2 * index..][..2].copy_from_slice(&delay.to_le_bytes());
                        }
                        let (preferred, minimum, smallest, entries) = self.host_memory;
                        data[272..276].copy_from_slice(&preferred.to_le_bytes());
                        data[276..280].copy_from_slice(&minimum.to_le_bytes());
                        data[332..336].copy_from_slice(&smallest.to_le_bytes());
                        data[336..338].copy_from_slice(&entries.to_le_bytes());
                        data[92..96].copy_from_slice(&self.async_events.to_le_bytes());
                        if self.apst {
                            (data[263], data[265], data[2048 + 32 + 3]) = (1, 1, 2);
                            // 1 ms entry and exit latency.
                            data[2048 + 32 + 4..][..2].copy_from_slice(&1000u16.to_le_bytes());
                            data[2048 + 32 + 8..][..2].copy_from_slice(&1000u16.to_le_bytes());
                        }
                    }
                    2 => data[..4].copy_from_slice(&1u32.to_le_bytes()),
                    3 => {
                        data[..4].copy_from_slice(&[1, 8, 0, 0]);
                        data[4..12].copy_from_slice(&self.eui64);
                        data[12..17].copy_from_slice(&[4, 1, 0, 0, self.csi]);
                    }
                    8 => data[14] = u8::from(!self.not_ready),
                    6 => {
                        assert_eq!(command[11], 0);
                        self.nvm_identifies += 1;
                        match self.nvm_limits {
                            Some((wzsl, dmrsl)) => {
                                data[1] = wzsl;
                                data[4..8].copy_from_slice(&dmrsl.to_le_bytes());
                            }
                            None => status = 2,
                        }
                    }
                    0 => {
                        assert_eq!(command[1], 1);
                        data[..8].copy_from_slice(&self.blocks.to_le_bytes());
                        data[8..16].copy_from_slice(&self.blocks.to_le_bytes());
                        data[120..128].copy_from_slice(&self.eui64);
                        data[128..132].copy_from_slice(&(9u32 << 16).to_le_bytes());
                    }
                    _ => panic!("unsupported model CNS"),
                }
                self.write_iova(u64::from(command[6]) | u64::from(command[7]) << 32, &data);
            }
            (0, 9) if command[10] == 0x0d => {
                let list = u64::from(command[13]) | u64::from(command[14]) << 32;
                let first = self.read_iova(list, 16);
                let first = (
                    u64::from_le_bytes(first[..8].try_into().unwrap()),
                    u32::from_le_bytes(first[8..12].try_into().unwrap()),
                );
                self.host_memory_sets
                    .push((core::array::from_fn(|i| command[11 + i]), first));
                if self.refuse_host_memory {
                    // Invalid Field in Command.
                    status = 2;
                }
            }
            // Number of Queues: only before any I/O queue; one pair granted.
            (0, 9) if command[10] == 0x07 => {
                assert_eq!(command[11], 0);
                assert_eq!(self.queues[1], (0, 0), "queue count after a queue");
                self.queue_counts += 1;
                if self.refuse_queue_count {
                    status = 2;
                } else {
                    self.queue_count_set = true;
                }
            }
            (0, 5 | 1) if self.queues_need_count && !self.queue_count_set => status = 2,
            (0, 9) if command[10] == 0x0b => self.event_configs.push(command[11]),
            (0, 2) => {
                assert_eq!(command[1], u32::MAX);
                self.logs.push(command[10] & 0xff);
            }
            (0, 9) if command[10] == 0x0c => {
                assert!(command[11] <= 1);
                let table = self.read_iova(u64::from(command[6]) | u64::from(command[7]) << 32, 8);
                self.apst_sets.push((
                    command[11] == 1,
                    u64::from_le_bytes(table[..8].try_into().unwrap()),
                ));
            }
            (0, 9) => {
                assert_eq!(command[10], 0x16);
                let page = self.read_iova(u64::from(command[6]) | u64::from(command[7]) << 32, 512);
                self.host_behavior.push([page[0], page[1], page[2]]);
            }
            (0, 5) => {
                assert_eq!(
                    (command[10] & 0xffff, command[10] >> 16, command[11]),
                    (1, 15, 1)
                );
                self.queues[1].1 = u64::from(command[6]) | u64::from(command[7]) << 32;
            }
            (0, 1) => {
                assert_eq!(
                    (command[10] & 0xffff, command[10] >> 16, command[11]),
                    (1, 15, 0x10001)
                );
                self.queues[1].0 = u64::from(command[6]) | u64::from(command[7]) << 32;
            }
            // Delete I/O SQ, then its CQ: a CQ with a live SQ is Invalid
            // Queue Deletion (NVMe Base 2.1 §5.2.2).
            (0, 0) => {
                assert_eq!(command[10] & 0xffff, 1);
                assert_ne!(self.queues[1].0, 0, "deleting an absent SQ");
                self.queues[1].0 = 0;
                // Its held command is aborted with the queue.
                if self.pending.is_some_and(|(q, _)| q == 1) {
                    self.pending = None;
                    self.busy[1] = false;
                }
            }
            (0, 4) => {
                assert_eq!(command[10] & 0xffff, 1);
                assert_eq!(self.queues[1].0, 0, "CQ deleted before its SQ");
                self.queues[1].1 = 0;
            }
            (1, 0) => assert_eq!(command[1], 1),
            (1, opcode @ (1 | 2)) => {
                assert_eq!(command[1], 1);
                let lba = (u64::from(command[10]) | u64::from(command[11]) << 32) as usize;
                let bytes = ((command[12] & 0xffff) + 1) as usize * 512;
                for (index, page) in self.pages(&command, bytes).into_iter().enumerate() {
                    let count = (bytes - index * 4096).min(4096);
                    let at = lba * 512 + index * 4096;
                    if opcode == 1 {
                        let data = self.read_iova(page, count);
                        self.disk[at..at + count].copy_from_slice(&data);
                    } else {
                        let data = self.disk[at..at + count].to_vec();
                        self.write_iova(page, &data);
                    }
                }
            }
            _ => panic!("unimplemented independent NVMe model command"),
        }
        let mut cid = command[0] >> 16;
        let mut sqid = queue as u32;
        let mut head = u32::from(self.sq_heads[queue]);
        let mut phase = self.phases[queue];
        match self.mutation {
            Mutation::Cid => cid ^= 0x10,
            Mutation::Sqid => sqid ^= 1,
            Mutation::Head => head = (head + 1) % 16,
            Mutation::Phase => phase = !phase,
            _ => {}
        }
        let mut cqe = [0u8; 16];
        cqe[..4].copy_from_slice(&result.to_le_bytes());
        cqe[8..12].copy_from_slice(&(sqid << 16 | head).to_le_bytes());
        cqe[12..16].copy_from_slice(&(status << 17 | u32::from(phase) << 16 | cid).to_le_bytes());
        self.write_iova(
            self.queues[queue].1 + u64::from(self.heads[queue]) * 16,
            &cqe,
        );
        self.busy[queue] = false;
        self.outstanding = self.busy.iter().any(|busy| *busy);
        self.record("completion");
    }
    /// The pages the I/O commands not yet completed reach: their data and
    /// PRP list, or their Dataset Management ranges.
    fn busy_pages(&self) -> Vec<u64> {
        let held = self.held_io.iter().copied();
        let pending = self
            .pending
            .iter()
            .filter(|(queue, _)| *queue == 1)
            .map(|(_, command)| *command);
        let mut pages = Vec::new();
        for command in held.chain(pending) {
            let first = u64::from(command[6]) | u64::from(command[7]) << 32;
            match command[0] as u8 {
                1 | 2 => {
                    let bytes = ((command[12] & 0xffff) + 1) as usize * 512;
                    if bytes.div_ceil(4096) > 2 {
                        pages.push(u64::from(command[8]) | u64::from(command[9]) << 32);
                    }
                    pages.extend(self.pages(&command, bytes));
                }
                9 => pages.push(first),
                _ => {}
            }
        }
        pages
    }
    /// Complete held I/O command `index`, in the order the test chooses.
    fn complete_held(&mut self, index: usize) {
        let command = self.held_io.remove(index);
        self.complete(1, command);
        self.busy[1] = !self.held_io.is_empty();
        self.outstanding = self.busy.iter().any(|busy| *busy);
    }
    /// Complete the held Asynchronous Event Request with `result`.
    fn post_event(&mut self, result: u32) {
        let cid = self.event_request.take().expect("a held event request");
        let mut cqe = [0u8; 16];
        cqe[..4].copy_from_slice(&result.to_le_bytes());
        cqe[8..12].copy_from_slice(&u32::from(self.sq_heads[0]).to_le_bytes());
        cqe[12..16].copy_from_slice(&(u32::from(self.phases[0]) << 16 | cid).to_le_bytes());
        self.write_iova(self.queues[0].1 + u64::from(self.heads[0]) * 16, &cqe);
    }
}
struct Domain(Arc<Mutex<Model>>);
// SAFETY: independent device/CPU visibility model keeps exact page pins
// via CpuMapping and isolated IOVAs. It models no real hardware capability.
unsafe impl DmaDomain for Domain {
    unsafe fn map(
        &self,
        pages: &DmaPages,
        direction: Direction,
        limit: u64,
    ) -> Result<DomainMapping, DmaError> {
        let mut m = self.0.lock().unwrap();
        let id = m.next;
        m.next += 1;
        let token = DomainMapping {
            id,
            address: 0x4_0000_0000 + id * 0x10_0000,
            bytes: pages.bytes(),
        };
        assert!(token.address + token.bytes as u64 - 1 <= limit);
        let cpu = pages
            .as_single()
            .unwrap()
            .cpu_mapping(CacheAttribute::WriteBack)
            .unwrap();
        assert_ne!(cpu.address(0, 1).unwrap().as_ptr() as u64, token.address);
        assert!(pages.physical_pages().iter().all(|pa| *pa < 0x4_0000_0000));
        m.maps.insert(
            id,
            Lease {
                cpu,
                token,
                direction,
                device: vec![0; token.bytes],
                visible: false,
            },
        );
        m.record("map");
        Ok(token)
    }
    unsafe fn unmap(&self, token: DomainMapping) -> Result<(), DmaError> {
        let mut m = self.0.lock().unwrap();
        let unpublished = !m.maps[&token.id].visible;
        assert!(unpublished || (!m.enabled && !m.any_bus_master() && m.drained && !m.outstanding));
        m.unmap_attempts += 1;
        if m.fail_unmap || m.fail_unmap_at == Some(m.unmap_attempts) {
            return Err(DmaError::InvalidationFailed);
        }
        m.maps.remove(&token.id).expect("live exact map");
        m.record("unmap");
        Ok(())
    }
    unsafe fn sync_for_device(
        &self,
        token: DomainMapping,
        range: Range<usize>,
        direction: Direction,
    ) -> Result<(), DmaError> {
        let mut m = self.0.lock().unwrap();
        if m.fail_device_sync {
            return Err(DmaError::VisibilityFailed);
        }
        let b = m.maps.get_mut(&token.id).unwrap();
        assert_eq!(direction, b.direction);
        let p = b.cpu.address(range.start, range.len()).unwrap();
        for (i, at) in range.enumerate() {
            // explicit CPU-to-device visibility
            // SAFETY: CPU owns this exact range before publication.
            b.device[at] = unsafe { p.as_ptr().add(i).read_volatile() };
        }
        b.visible = true;
        m.record("visible");
        Ok(())
    }
    unsafe fn sync_for_cpu(
        &self,
        token: DomainMapping,
        range: Range<usize>,
        direction: Direction,
    ) -> Result<(), DmaError> {
        let mut m = self.0.lock().unwrap();
        let start = m.maps[&token.id].token.address;
        let end = start + m.maps[&token.id].token.bytes as u64;
        assert!(
            !m.busy[0]
                && !m
                    .busy_pages()
                    .iter()
                    .any(|page| (start..end).contains(page)),
            "CPU acquire before command completion"
        );
        if m.fail_cpu_sync {
            return Err(DmaError::VisibilityFailed);
        }
        let b = m.maps.get_mut(&token.id).unwrap();
        assert_eq!(direction, b.direction);
        let p = b.cpu.address(range.start, range.len()).unwrap();
        for (i, at) in range.enumerate() {
            // SAFETY: model command completion established CPU ownership.
            unsafe { p.as_ptr().add(i).write_volatile(b.device[at]) };
        }
        m.sync_cpu += 1;
        m.record("cpu");
        Ok(())
    }
    unsafe fn read_coherent_u32(
        &self,
        token: DomainMapping,
        offset: usize,
        direction: Direction,
    ) -> Result<u32, DmaError> {
        let m = self.0.lock().unwrap();
        if !m.coherent {
            return Err(DmaError::CoherenceUnavailable);
        }
        let b = &m.maps[&token.id];
        assert_eq!(direction, b.direction);
        assert_ne!(direction, Direction::ToDevice);
        Ok(u32::from_le_bytes(
            b.device[offset..offset + 4].try_into().unwrap(),
        ))
    }
    unsafe fn drain_posted_writes(&self) -> Result<(), DmaError> {
        let mut m = self.0.lock().unwrap();
        assert!(!m.any_bus_master() && !m.enabled && !m.outstanding);
        m.record("drain");
        if m.fail_drain {
            return Err(DmaError::DrainUnavailable);
        }
        m.drained = true;
        Ok(())
    }
}
struct Regs(Arc<Mutex<Model>>);
// SAFETY: this independent controller model serializes exact bounded
// register effects and keeps model RAM/configuration until retirement.
unsafe impl Registers for Regs {
    fn read32(&self, offset: usize) -> Result<u32, BlockError> {
        assert!(offset < 0x2000 && offset.is_multiple_of(4));
        let mut m = self.0.lock().unwrap();
        m.mmio_accesses += 1;
        assert!(m.registers_available() && !m.retired);
        if m.fail_read {
            return Err(register_error());
        }
        if offset == 0x1c && m.enabled && m.enabling_reads > 0 {
            if m.enabling_reads != usize::MAX {
                m.enabling_reads -= 1;
            }
            return Ok(1 << 1);
        }
        if offset == 0x1c {
            return Ok(u32::from((m.enabled && !m.never_ready) || m.stuck_ready)
                | u32::from(m.fatal) << 1
                | if m.stopped { 8 } else { 0 });
        }
        Ok(*m.registers.get(&offset).unwrap_or(&0))
    }
    fn write32(&self, offset: usize, value: u32) -> Result<(), BlockError> {
        assert!(offset < 0x2000 && offset.is_multiple_of(4));
        let mut m = self.0.lock().unwrap();
        m.mmio_accesses += 1;
        assert!(m.registers_available() && !m.retired);
        m.registers.insert(offset, value);
        match offset {
            0x14 => {
                if value & 1 != 0 {
                    assert!(m.dma_enabled());
                    m.enabled = true;
                    m.enabling_reads = m.fatal_while_enabling;
                    m.stopped = false;
                    m.queues[0] = (m.register64(0x28), m.register64(0x30));
                    m.record("enable");
                } else {
                    m.record("stop");
                    if !m.stuck_ready {
                        // A controller reset ends every queue and command.
                        m.enabled = false;
                        m.outstanding = false;
                        m.busy = [false; 2];
                        m.pending = None;
                        m.held_io.clear();
                        m.stopped = true;
                        m.fatal = false;
                        m.queue_count_set = false;
                        m.queues[1] = (0, 0);
                        m.heads = [0; 2];
                        m.phases = [true; 2];
                        m.sq_heads = [0; 2];
                        m.event_request = None;
                    }
                }
                if value & (3 << 14) == 1 << 14 {
                    m.stopped = true;
                }
            }
            0x1000 => m.process(0, value as u16),
            0x1008 => {
                m.process(1, value as u16);
                if m.fail_after_doorbell {
                    m.fail_read = true;
                }
            }
            0x1004 | 0x100c => {
                let q = usize::from(offset == 0x100c);
                assert_eq!(value as u16, (m.heads[q] + 1) % 16);
                m.heads[q] = value as u16;
                if value == 0 {
                    m.phases[q] = !m.phases[q];
                }
                m.record("cq_ack");
            }
            _ => {}
        }
        Ok(())
    }
    unsafe fn retire(&mut self) -> Result<(), BlockError> {
        let mut m = self.0.lock().unwrap();
        assert!(m.maps.is_empty() && m.drained && !m.any_bus_master() && !m.enabled);
        m.record("retire_regs_attempt");
        if m.fail_retire {
            return Err(register_error());
        }
        m.retired = true;
        m.record("retire_regs");
        Ok(())
    }
}
impl Drop for Regs {
    fn drop(&mut self) {
        self.0.lock().unwrap().register_owner_drops += 1;
    }
}
struct Life(Arc<Mutex<Model>>);
impl Drop for Life {
    fn drop(&mut self) {
        self.0.lock().unwrap().lifecycle_owner_drops += 1;
    }
}
// SAFETY: models exclusive hierarchy ownership, separate preparation
// and BME effects, and refuses release before full native-owner teardown.
unsafe impl Lifecycle for Life {
    fn prepare(&mut self) -> Result<(), BlockError> {
        let mut m = self.0.lock().unwrap();
        assert!(!m.bme);
        m.record("prepare_attempt");
        if m.fail_prepare {
            // Power remains D3; no BAR access is possible.
            return Err(register_error());
        }
        m.prepared = true;
        m.record("prepare");
        Ok(())
    }
    fn enable_dma(&mut self) -> Result<(), BlockError> {
        let mut m = self.0.lock().unwrap();
        // The six queue and bounce buffers, and any host memory buffer
        // kept across a reset.
        assert!(m.prepared && !m.enabled && m.maps.len() >= 6);
        m.bme = true;
        m.record("bme_on");
        Ok(())
    }
    fn disable_dma(&mut self) -> Result<(), BlockError> {
        let mut m = self.0.lock().unwrap();
        assert!(!m.enabled);
        m.record("bme_off");
        if m.fail_disable {
            return Err(register_error());
        }
        m.bme = false;
        Ok(())
    }
    fn release(&mut self) -> Result<(), BlockError> {
        let mut m = self.0.lock().unwrap();
        assert!(m.retired && m.maps.is_empty() && !m.bme && m.drained);
        m.record("release_attempt");
        if m.fail_release {
            return Err(register_error());
        }
        m.released = true;
        m.record("release");
        Ok(())
    }
}
struct Fixture {
    model: Arc<Mutex<Model>>,
    source: Source,
    /// The quirks the controller's PCI ID and board give it.
    quirks: Quirks,
}
impl Fixture {
    fn new() -> Self {
        Self {
            model: Arc::new(Mutex::new(Model::new())),
            source: Source {
                next: AtomicU64::new(0x9000_0000),
                released: Arc::new(AtomicUsize::new(0)),
            },
            quirks: Quirks::NONE,
        }
    }
    fn attach(&self) -> Result<Attached, BlockError> {
        // SAFETY: every owner keeps the same exclusive independent
        // model controller/root and domain, initially BME-contained.
        unsafe {
            attach_with_timeout(
                Box::new(Regs(self.model.clone())),
                Box::new(Life(self.model.clone())),
                &self.source,
                Arc::new(Domain(self.model.clone())),
                Clock {
                    ticks_per_microsecond: 1000,
                },
                "VMD model requester".into(),
                1000,
                self.quirks,
            )
        }
    }
}

#[test]
fn owned_protocol_reuses_prps_across_reset_then_wraps_and_retires() {
    let f = Fixture::new();
    let mut attached = f.attach().unwrap();
    assert!(attached.failures.is_empty());
    assert_eq!(attached.namespaces.len(), 1);
    let disk = &mut attached.namespaces[0];
    assert_eq!(
        (disk.identity().block_size, disk.identity().blocks),
        (512, 1024)
    );
    let pattern: Vec<u8> = (0..128 * 1024).map(|n| (n * 17) as u8).collect();
    disk.write(8, &pattern).unwrap();
    let mut read = vec![0; pattern.len()];
    disk.read(8, &mut read).unwrap();
    assert_eq!(read, pattern);
    let (before, mappings) = {
        let mut m = f.model.lock().unwrap();
        m.fatal = true;
        (
            m.events.len(),
            m.maps.values().map(|lease| lease.token).collect::<Vec<_>>(),
        )
    };
    let replacement: Vec<u8> = pattern.iter().map(|byte| byte ^ 0xa5).collect();
    disk.write(8, &replacement).unwrap();
    disk.read(8, &mut read).unwrap();
    assert_eq!(read, replacement);
    {
        let m = f.model.lock().unwrap();
        assert!(m.events[before..].contains(&"drain"));
        assert!(m.maps.values().map(|lease| lease.token).eq(mappings));
    }
    for _ in 0..35 {
        disk.flush().unwrap();
    }
    disk.shutdown(false).unwrap();
    assert!(disk.flush().is_err());
    disk.retire_owned().unwrap();
    let m = f.model.lock().unwrap();
    assert!(m.released && m.maps.is_empty());
    assert_eq!(f.source.released.load(Ordering::SeqCst), BUFFERS);
    let at = |event| m.events.iter().rposition(|e| *e == event).unwrap();
    assert!(
        at("stop") < at("bme_off")
            && at("bme_off") < at("drain")
            && at("drain") < at("unmap")
            && at("unmap") < at("retire_regs")
            && at("retire_regs") < at("release")
    );
}
#[test]
fn owned_queue_runs_eight_commands_at_once_and_completes_them_out_of_order() {
    // No deadline passes while the eight commands are held.
    let _clock = crate::test_clock::virtualize();
    let f = Fixture::new();
    let mut attached = f.attach().unwrap();
    let disk = &mut attached.namespaces[0];
    // The model's queue has 16 entries: 15 tags.
    assert_eq!(disk.depth(), 15);
    {
        let mut m = f.model.lock().unwrap();
        for (at, byte) in m.disk.iter_mut().enumerate() {
            *byte = (at / 512) as u8 ^ (at % 251) as u8;
        }
        m.hold_io = true;
    }
    // Reads of one, two and four pages, PRP lists past two, and a write.
    let reads: Vec<(u64, usize)> = (0..7)
        .map(|index| (index * 40, [512, PAGE + 512, 4 * PAGE][index as usize % 3]))
        .collect();
    let mut tags: Vec<u32> = reads
        .iter()
        .map(|&(lba, bytes)| disk.start(Command::Read { lba, bytes }).unwrap())
        .collect();
    let written: Vec<u8> = (0..3 * PAGE).map(|at| (at * 7) as u8).collect();
    tags.push(
        disk.start(Command::Write {
            lba: 600,
            data: written.clone(),
            fua: true,
        })
        .unwrap(),
    );
    assert!(disk.room());
    let expected = f.model.lock().unwrap().disk.clone();
    {
        let m = f.model.lock().unwrap();
        assert_eq!(m.held_io.len(), 8, "all eight reached the controller");
    }
    for &tag in &tags {
        assert_eq!(disk.finish(tag), None);
    }
    // The controller completes them out of order, each while the others
    // still run: the last, one from the middle, the first, and so on.
    let mut held: Vec<usize> = (0..8).collect();
    for index in [7, 3, 0, 4, 2, 1, 1, 0] {
        f.model.lock().unwrap().complete_held(index);
        let command = held.remove(index);
        let result = disk.finish(tags[command]).expect("completed");
        for &other in &held {
            assert_eq!(disk.finish(tags[other]), None);
        }
        if command == 7 {
            assert_eq!(result, Ok(None));
        } else {
            let (lba, bytes) = reads[command];
            let at = lba as usize * 512;
            assert_eq!(result, Ok(Some(expected[at..at + bytes].to_vec())));
        }
    }
    let m = f.model.lock().unwrap();
    assert_eq!(&m.disk[600 * 512..][..written.len()], &written[..]);
    assert!(m.held_io.is_empty());
}

#[test]
fn invalid_clock_keeps_unadmitted_original_owners_without_any_effect() {
    for ticks_per_microsecond in [0, u64::MAX, u64::MAX / (2 * 255 * 500_000) + 1] {
        let f = Fixture::new();
        // SAFETY: the complete same-controller permission is transferred;
        // clock refusal must keep it before PCI/MMIO or DMA allocation.
        let result = unsafe {
            attach(
                Box::new(Regs(f.model.clone())),
                Box::new(Life(f.model.clone())),
                &f.source,
                Arc::new(Domain(f.model.clone())),
                Clock {
                    ticks_per_microsecond,
                },
                "unadmitted original NVMe".into(),
                Quirks::NONE,
            )
        };
        assert!(result.is_err());
        let model = f.model.lock().unwrap();
        assert!(model.events.is_empty());
        assert!(model.maps.is_empty());
        assert_eq!(model.register_owner_drops, 0);
        assert_eq!(model.lifecycle_owner_drops, 0);
        assert_eq!(f.source.next.load(Ordering::SeqCst), 0x9000_0000);
        assert_eq!(f.source.released.load(Ordering::SeqCst), 0);
    }
}
#[test]
fn owned_bad_cqe_identity_never_reuses_dma() {
    for mutation in [Mutation::Cid, Mutation::Sqid, Mutation::Head] {
        let f = Fixture::new();
        let mut a = f.attach().unwrap();
        let disk = &mut a.namespaces[0];
        let mut sibling = Namespace {
            controller: disk.controller.clone(),
            namespace: disk.namespace,
            identity: disk.identity.clone(),
            ids: disk.ids.clone(),
            scanned: disk.scanned,
            removed: None,
        };
        let before = f.model.lock().unwrap().sync_cpu;
        f.model.lock().unwrap().mutation = mutation;
        assert!(disk.read(0, &mut [0xa5; 512]).is_err());
        let commands = f.model.lock().unwrap().commands;
        assert!(disk.write(0, &[1; 512]).is_err());
        assert!(disk.flush().is_err());
        let mut output = [0xa5; 512];
        assert!(sibling.read(0, &mut output).is_err());
        assert_eq!(output, [0xa5; 512]);
        let m = f.model.lock().unwrap();
        assert_eq!(m.commands, commands);
        assert_eq!(m.sync_cpu, before);
        assert_eq!(m.maps.len(), BUFFERS);
        drop(m);
        disk.retire_owned().unwrap();
        assert!(f.model.lock().unwrap().released);
    }
}
/// A controller that never sets CSTS.RDY fails its attach at CAP.TO; with
/// CAP.TO at its largest (127.5 s) a passed device start budget ends the
/// attach at the budget instead.
#[test]
fn a_controller_that_never_becomes_ready_fails_within_its_bound() {
    let _clock = crate::test_clock::virtualize();
    let clock = Clock {
        ticks_per_microsecond: 1000,
    };
    let f = Fixture::new();
    f.model.lock().unwrap().never_ready = true;
    let started = Clock::ticks();
    assert_eq!(f.attach().err(), Some(BlockError::Timeout.at(ENABLE_WAIT)));
    let took_us = (Clock::ticks() - started) / 1000;
    assert!((500_000..2_000_000).contains(&took_us), "{took_us}");

    let f = Fixture::new();
    {
        let mut m = f.model.lock().unwrap();
        m.never_ready = true;
        m.registers.insert(0, 15 | 255 << 24);
    }
    let budget = stela_driver_base::budget::Budget::start(clock, 20_000).unwrap();
    let started = Clock::ticks();
    assert_eq!(f.attach().err(), Some(BlockError::Timeout.at(ENABLE_WAIT)));
    let took_us = (Clock::ticks() - started) / 1000;
    assert!(took_us < 40_000, "{took_us}");
    assert!(budget.expired().is_some());
}

/// Linux `nvme_setup_io_queues` sets the Number of Queues before it creates
/// an I/O queue, at probe and after every reset: a controller that refuses
/// queues it was not asked for attaches and resets. A refused feature
/// still tries the queues, whose refusal then fails the attach.
#[test]
fn the_queue_count_is_set_before_the_io_queues_like_nvme_set_queue_count() {
    let f = Fixture::new();
    let mut a = f.attach().unwrap();
    assert_eq!(f.model.lock().unwrap().queue_counts, 1);
    let disk = &mut a.namespaces[0];
    disk.write(0, &[0x77; 512]).unwrap();
    disk.controller.borrow_mut().reset_and_settle().unwrap();
    {
        let m = f.model.lock().unwrap();
        assert_eq!(m.queue_counts, 2);
        assert!(m.queue_count_set);
    }
    let mut data = [0; 512];
    disk.read(0, &mut data).unwrap();
    assert_eq!(data, [0x77; 512]);
    disk.retire_owned().unwrap();

    // Refused, by a controller that creates its queues anyway.
    let f = Fixture::new();
    {
        let mut m = f.model.lock().unwrap();
        m.refuse_queue_count = true;
        m.queues_need_count = false;
    }
    let mut a = f.attach().unwrap();
    assert_eq!(f.model.lock().unwrap().queue_counts, 1);
    a.namespaces[0].write(0, &[1; 512]).unwrap();
    a.namespaces[0].retire_owned().unwrap();

    // Refused, and so are the queues.
    let f = Fixture::new();
    f.model.lock().unwrap().refuse_queue_count = true;
    assert_eq!(
        f.attach().err(),
        Some(BlockError::Device { status: 2 }.at("NVMe Create I/O Completion Queue (opcode 0x05)"))
    );
}

/// Linux `nvme_wait_ready` waits for CSTS.RDY alone: a controller showing
/// CSTS.CFS while it starts is waited for, at attach and at a reset, and
/// one that stays fatal fails at its ready timeout with its CSTS.
#[test]
fn a_fatal_status_while_enabling_is_waited_out_like_nvme_wait_ready() {
    let _clock = crate::test_clock::virtualize();
    let f = Fixture::new();
    f.model.lock().unwrap().fatal_while_enabling = 3;
    let mut a = f.attach().unwrap();
    assert_eq!(f.model.lock().unwrap().enabling_reads, 0);
    let disk = &mut a.namespaces[0];
    disk.write(0, &[0x3c; 512]).unwrap();
    // A reset enables the controller the same way.
    disk.controller.borrow_mut().reset_and_settle().unwrap();
    assert_eq!(f.model.lock().unwrap().enabling_reads, 0);
    let mut data = [0; 512];
    disk.read(0, &mut data).unwrap();
    assert_eq!(data, [0x3c; 512]);
    disk.retire_owned().unwrap();

    let f = Fixture::new();
    f.model.lock().unwrap().fatal_while_enabling = usize::MAX;
    let started = Clock::ticks();
    let error = f.attach().err().unwrap();
    assert_eq!(
        format!("{error}"),
        "NVMe controller enable, waiting for CSTS.RDY: \
         NVMe controller fatal status, CSTS: device status 0x2"
    );
    let took_us = (Clock::ticks() - started) / 1000;
    assert!((500_000..2_000_000).contains(&took_us), "{took_us}");
    assert_eq!(f.model.lock().unwrap().commands, 0);
}

/// A start that fails names the admin command that failed, by opcode and
/// Identify CNS or feature, and tells a completion's status from the
/// controller's CSTS and from a command left unanswered.
#[test]
fn a_failed_start_names_its_admin_command() {
    let _clock = crate::test_clock::virtualize();
    let message = |prepare: fn(&mut Model)| {
        let f = Fixture::new();
        prepare(&mut f.model.lock().unwrap());
        format!("{}", f.attach().err().unwrap())
    };
    // A refused command: its completion status.
    assert_eq!(
        message(|m| m.refuse_identify = Some(1)),
        "NVMe Identify Controller (opcode 0x06, CNS 0x01): device status 0x2"
    );
    assert_eq!(
        message(|m| m.refuse_queue_count = true),
        "NVMe Create I/O Completion Queue (opcode 0x05): device status 0x2"
    );
    // An unanswered command: a timeout, here of the first Identify.
    assert_eq!(
        message(|m| m.ignore_admin = true),
        "NVMe Identify Controller (opcode 0x06, CNS 0x01): device timeout"
    );
    // A controller that fails while its command waits: CSTS, said so.
    assert_eq!(
        message(|m| {
            m.ignore_admin = true;
            m.fatal_on_admin = true;
        }),
        "NVMe Identify Controller (opcode 0x06, CNS 0x01): \
         NVMe controller fatal status, CSTS: device status 0x3"
    );
    // A controller that never becomes ready, and one that never stops.
    assert_eq!(
        message(|m| m.never_ready = true),
        "NVMe controller enable, waiting for CSTS.RDY: device timeout"
    );
    assert_eq!(
        message(|m| m.stuck_ready = true),
        "NVMe controller disable, waiting for CSTS.RDY to clear: device timeout"
    );
}

/// Linux `nvme_scan_ns` passes over a namespace whose Identify Namespace
/// the controller refuses: the controller stays, the namespace is
/// reported with the command it refused.
#[test]
fn a_refused_identify_namespace_leaves_the_controller_attached() {
    let f = Fixture::new();
    f.model.lock().unwrap().refuse_identify = Some(0);
    let a = f.attach().unwrap();
    assert!(a.namespaces.is_empty());
    assert_eq!(a.failures.len(), 1);
    assert_eq!(a.failures[0].0, 1);
    assert_eq!(
        format!("{}", a.failures[0].1),
        "NVMe Identify Namespace (opcode 0x06, CNS 0x00): device status 0x2"
    );
}

#[test]
fn owned_timeouts_abort_then_reset_contain_dma_and_retry_like_linux() {
    // An abort that completes the held write: it is retried, no reset.
    let f = Fixture::new();
    let mut a = f.attach().unwrap();
    let disk = &mut a.namespaces[0];
    let start = {
        let mut m = f.model.lock().unwrap();
        m.delay_io = 1;
        m.abortable = true;
        m.events.len()
    };
    disk.write(3, &[0x5a; 512]).unwrap();
    {
        let m = f.model.lock().unwrap();
        assert_eq!(m.aborts, 1);
        assert!(!m.events[start..].contains(&"stop"));
        assert_eq!(m.disk[3 * 512..4 * 512], [0x5a; 512]);
    }

    // An abort that does not: the controller is reset with its DMA
    // contained, then brought back and the write retried.
    f.model.lock().unwrap().delay_io = 1;
    f.model.lock().unwrap().abortable = false;
    let before = f.model.lock().unwrap().events.len();
    disk.write(4, &[0x6b; 512]).unwrap();
    {
        let m = f.model.lock().unwrap();
        let events = &m.events[before..];
        let at = |event| events.iter().position(|e| *e == event).unwrap();
        assert!(at("stop") < at("bme_off") && at("bme_off") < at("drain"));
        assert!(at("drain") < at("enable"));
        assert_eq!(m.disk[4 * 512..5 * 512], [0x6b; 512]);
    }
    let mut read = [0; 512];
    disk.read(4, &mut read).unwrap();
    assert_eq!(read, [0x6b; 512]);

    // A fatal controller is reset before the command is sent.
    f.model.lock().unwrap().fatal = true;
    disk.flush().unwrap();
    assert!(!f.model.lock().unwrap().fatal);

    // An error status without DNR is retried; with DNR it is not.
    f.model.lock().unwrap().statuses.extend([2, 2]);
    disk.flush().unwrap();
    f.model
        .lock()
        .unwrap()
        .statuses
        .push_back(STATUS_DO_NOT_RETRY | 2);
    assert_eq!(
        disk.flush(),
        Err(BlockError::Device {
            status: STATUS_DO_NOT_RETRY | 2
        })
    );
    // Five retries, as nvme_max_retries allows.
    f.model.lock().unwrap().statuses.extend([2; 7]);
    let commands = f.model.lock().unwrap().commands;
    assert_eq!(disk.flush(), Err(BlockError::Device { status: 2 }));
    assert_eq!(f.model.lock().unwrap().commands, commands + 6);
    f.model.lock().unwrap().statuses.clear();
    disk.flush().unwrap();
    disk.retire_owned().unwrap();
    assert!(f.model.lock().unwrap().released);
}
#[test]
fn retry_delays_enable_acre_and_delay_retries_like_linux() {
    let _clock = crate::test_clock::virtualize();
    let f = Fixture::new();
    // No delay reported: Host Behavior Support is left alone.
    drop(f.attach().unwrap());
    assert!(f.model.lock().unwrap().host_behavior.is_empty());
    let f = Fixture::new();
    f.model.lock().unwrap().retry_delays = [2, 0, 7];
    let mut a = f.attach().unwrap();
    let disk = &mut a.namespaces[0];
    assert_eq!(f.model.lock().unwrap().host_behavior, [[1, 0, 0]]);
    // CRD 1 waits CRDT1 (200 ms) before the retry; CRD 3 waits 700 ms.
    for (crd, delay) in [(1u32, 200_000u64), (3, 700_000)] {
        f.model.lock().unwrap().statuses.push_back(2 | crd << 11);
        let start = Clock::ticks();
        disk.flush().unwrap();
        assert!(Clock::ticks() - start >= delay * 1000);
    }
    // Each reset sets it again.
    let mut m = f.model.lock().unwrap();
    m.delay_io = 1;
    m.abortable = false;
    drop(m);
    disk.flush().unwrap();
    assert_eq!(f.model.lock().unwrap().host_behavior.len(), 2);
    disk.retire_owned().unwrap();
}

#[test]
fn async_events_are_handled_and_requested_again_like_nvme_complete_async_event() {
    let f = Fixture::new();
    // Namespace attribute and firmware activation notices, and one Linux
    // leaves disabled.
    f.model.lock().unwrap().async_events = 1 << 8 | 1 << 9 | 1 << 10;
    let mut a = f.attach().unwrap();
    let disk = &mut a.namespaces[0];
    let event = |result| {
        let mut m = f.model.lock().unwrap();
        m.post_event(result);
        m.blocks += 1024;
    };
    // A namespace change clears the changed list and rescans: the
    // namespace grew.
    event(2);
    disk.flush().unwrap();
    assert_eq!(disk.identity().blocks, 2048);
    // A firmware activation reads the slot log once processing resumes.
    event(2 | 1 << 8);
    disk.flush().unwrap();
    assert_eq!(disk.identity().blocks, 2048);
    // A persistent internal error resets the controller, whose rescan
    // sees the namespace grow again.
    event(3 << 8);
    disk.flush().unwrap();
    assert_eq!(disk.identity().blocks, 4096);
    let m = f.model.lock().unwrap();
    assert_eq!(m.event_configs, [1 << 8 | 1 << 9; 2]);
    assert_eq!(m.logs, [4, 3]);
    assert!(m.event_request.is_some());
    drop(m);
    disk.retire_owned().unwrap();
}

#[test]
fn namespaces_are_scanned_like_nvme_scan_ns_with_io_command_sets() {
    // NVMe 2.0 with I/O command sets: CC.CSS enables them all and the
    // command-set-independent Identify Namespace reports readiness.
    let fixture = |csi, not_ready| {
        let f = Fixture::new();
        let mut m = f.model.lock().unwrap();
        m.registers.insert(4, 1 << 5 | 1 << 11);
        m.registers.insert(8, 0x0002_0000);
        (m.csi, m.not_ready) = (csi, not_ready);
        drop(m);
        f
    };
    // A namespace not ready is passed over, a zoned one is unsupported.
    let attached = |f: Fixture| {
        let a = f.attach().unwrap();
        (a.namespaces.len(), a.failures.len())
    };
    assert_eq!(attached(fixture(0, true)), (0, 0));
    assert_eq!(attached(fixture(2, false)), (0, 1));
    let f = fixture(0, false);
    let a = f.attach().unwrap();
    assert_eq!(a.namespaces.len(), 1);
    assert_eq!(f.model.lock().unwrap().registers[&0x14] >> 4 & 7, 6);
}

#[test]
fn nvm_limits_cap_deallocate_and_write_zeroes_like_nvme_init_non_mdts_limits() {
    let f = Fixture::new();
    f.model.lock().unwrap().oncs = 1 << 2 | 1 << 3;
    // Before NVMe 1.2: UINT_MAX sectors and the largest transfer.
    let attached = f.attach().unwrap();
    let features = attached.namespaces[0].identity().features;
    assert_eq!(
        features.discard,
        Some(DiscardLimits {
            blocks: u64::from(u32::MAX),
            ranges: 256,
        })
    );
    let transfer = (MAX_COMMAND_BYTES / 512) as u64;
    assert_eq!(features.max_transfer, BOUNCE_BYTES);
    assert_eq!(features.write_zeroes.unwrap().blocks, transfer);
    assert_eq!(f.model.lock().unwrap().nvm_identifies, 0);
    drop(attached);
    // DMRSL blocks, and 2^WZSL 4 KiB pages of 512-byte blocks.
    let f = Fixture::new();
    {
        let mut m = f.model.lock().unwrap();
        m.registers.insert(8, 0x0001_0200);
        m.oncs = 1 << 2 | 1 << 3;
        m.nvm_limits = Some((1, 100));
    }
    let attached = f.attach().unwrap();
    let features = attached.namespaces[0].identity().features;
    assert_eq!(
        features.discard,
        Some(DiscardLimits {
            blocks: 100,
            ranges: 256,
        })
    );
    assert_eq!(features.write_zeroes.unwrap().blocks, 16);
    drop(attached);
    // A refused Identify keeps the defaults.
    let f = Fixture::new();
    {
        let mut m = f.model.lock().unwrap();
        m.registers.insert(8, 0x0001_0200);
        m.oncs = 1 << 3;
    }
    let attached = f.attach().unwrap();
    let features = attached.namespaces[0].identity().features;
    assert_eq!(features.discard, None);
    assert_eq!(features.write_zeroes.unwrap().blocks, transfer);
    assert!(attached.failures.is_empty());
}

#[test]
fn apst_is_configured_at_attach_and_after_resets() {
    let f = Fixture::new();
    f.model.lock().unwrap().apst = true;
    let mut a = f.attach().unwrap();
    let mut m = f.model.lock().unwrap();
    m.delay_io = 1;
    m.abortable = false;
    drop(m);
    a.namespaces[0].flush().unwrap();
    // PS0 idles into PS1 after 100 ms, at attach and after the reset.
    assert_eq!(
        f.model.lock().unwrap().apst_sets,
        [(true, 1 << 3 | 100 << 8); 2]
    );
    a.namespaces[0].retire_owned().unwrap();
}

/// A PCI ID with `NVME_QUIRK_NO_DEEPEST_PS` keeps APST out of PS1, here the
/// deepest state; a model `core_quirks` names turns APST off, at attach and
/// after a reset, and the attachment reports both.
#[test]
fn apst_quirks_from_the_pci_id_and_the_model_apply_at_attach_and_after_resets() {
    let mut f = Fixture::new();
    f.model.lock().unwrap().apst = true;
    f.quirks = Quirks::for_device(0x2646, 0x2263);
    let a = f.attach().unwrap();
    assert_eq!(a.quirks, Quirks::NO_DEEPEST_POWER_STATE);
    assert_eq!(f.model.lock().unwrap().apst_sets, [(true, 0)]);
    drop(a);

    let f = Fixture::new();
    {
        let mut m = f.model.lock().unwrap();
        m.apst = true;
        (m.vendor, m.model) = (0x1179, "THNSF5256GPUK TOSHIBA");
    }
    let mut a = f.attach().unwrap();
    assert_eq!(a.quirks, Quirks::NO_APST);
    let mut m = f.model.lock().unwrap();
    m.delay_io = 1;
    m.abortable = false;
    drop(m);
    a.namespaces[0].flush().unwrap();
    assert_eq!(f.model.lock().unwrap().apst_sets, [(false, 0); 2]);
    a.namespaces[0].retire_owned().unwrap();
}

#[test]
fn host_memory_is_set_up_and_returned_after_resets_like_nvme_setup_host_mem() {
    // None preferred: no Host Memory Buffer feature.
    let f = Fixture::new();
    drop(f.attach().unwrap());
    assert!(f.model.lock().unwrap().host_memory_sets.is_empty());
    // 1 MiB preferred, 512 KiB minimum: one 1 MiB chunk, enabled.
    let f = Fixture::new();
    f.model.lock().unwrap().host_memory = (256, 128, 0, 0);
    let mut a = f.attach().unwrap();
    let disk = &mut a.namespaces[0];
    {
        let m = f.model.lock().unwrap();
        let [bits, size, _, _, entries] = m.host_memory_sets[0].0;
        assert_eq!((bits, size, entries), (1, 256, 1));
        assert_eq!(m.host_memory_sets[0].1.1, 256);
        assert_ne!(m.host_memory_sets[0].1.0, 0);
    }
    // A reset offers the same buffer back with MR.
    let mut m = f.model.lock().unwrap();
    m.delay_io = 1;
    m.abortable = false;
    drop(m);
    disk.flush().unwrap();
    {
        let m = f.model.lock().unwrap();
        assert_eq!(m.host_memory_sets.len(), 2);
        assert_eq!(m.host_memory_sets[1].0[0], 3);
        assert_eq!(m.host_memory_sets[1].1, m.host_memory_sets[0].1);
    }
    // A refusal leaves the controller without one; the next reset offers
    // it afresh, without MR.
    let mut m = f.model.lock().unwrap();
    m.refuse_host_memory = true;
    m.delay_io = 1;
    drop(m);
    disk.flush().unwrap();
    let mut m = f.model.lock().unwrap();
    m.refuse_host_memory = false;
    m.delay_io = 1;
    drop(m);
    disk.flush().unwrap();
    assert_eq!(f.model.lock().unwrap().host_memory_sets[3].0[0], 1);
    disk.retire_owned().unwrap();

    // A descriptor limit caps the buffer: 8 MiB preferred in 4 MiB
    // chunks, at most one descriptor.
    let f = Fixture::new();
    f.model.lock().unwrap().host_memory = (2048, 0, 0, 1);
    drop(f.attach().unwrap());
    let m = f.model.lock().unwrap();
    let [bits, size, _, _, entries] = m.host_memory_sets[0].0;
    assert_eq!((bits, size, entries), (1, 1024, 1));
}

#[test]
fn a_reset_rescans_namespaces_like_nvme_scan_ns() {
    let f = Fixture::new();
    let mut a = f.attach().unwrap();
    let disk = &mut a.namespaces[0];
    let reset = |f: &Fixture, disk: &mut Namespace| {
        let mut m = f.model.lock().unwrap();
        m.delay_io = 1;
        m.abortable = false;
        drop(m);
        disk.write(4, &[0x6b; 512]).unwrap();
    };
    // The namespace grew: the reset's rescan updates its size.
    f.model.lock().unwrap().blocks = 2048;
    reset(&f, disk);
    assert_eq!(disk.identity().blocks, 1024);
    disk.flush().unwrap();
    assert_eq!(disk.identity().blocks, 2048);
    // Its EUI64 changed: nvme_validate_ns removes it.
    f.model.lock().unwrap().eui64 = [0x22; 8];
    reset(&f, disk);
    assert_eq!(disk.read(0, &mut [0; 512]), Err(NAMESPACE_REMOVED));
    assert_eq!(disk.flush(), Err(NAMESPACE_REMOVED));
    f.model.lock().unwrap().eui64 = [0x11; 8];
    assert_eq!(disk.flush(), Err(NAMESPACE_REMOVED));
    disk.retire_owned().unwrap();
}

#[test]
fn owned_controller_that_keeps_timing_out_fails_after_five_resets() {
    let f = Fixture::new();
    let mut a = f.attach().unwrap();
    let disk = &mut a.namespaces[0];
    f.model.lock().unwrap().mutation = Mutation::Delayed;
    assert_eq!(disk.flush(), Err(BlockError::Timeout));
    {
        let m = f.model.lock().unwrap();
        assert_eq!(m.aborts, 6);
        assert_eq!(m.events.iter().filter(|e| **e == "drain").count(), 6);
    }
    // Each reset succeeded, so the controller still serves commands.
    f.model.lock().unwrap().mutation = Mutation::None;
    disk.flush().unwrap();
    disk.retire_owned().unwrap();
}
#[test]
fn owned_stop_bme_drain_and_unmap_failures_keep_original_owners() {
    for stage in 0..4 {
        let f = Fixture::new();
        let mut a = f.attach().unwrap();
        {
            let mut m = f.model.lock().unwrap();
            match stage {
                0 => m.stuck_ready = true,
                1 => m.fail_disable = true,
                2 => m.fail_drain = true,
                _ => m.fail_unmap = true,
            }
        }
        assert!(a.namespaces[0].retire_owned().is_err());
        {
            let m = f.model.lock().unwrap();
            assert_eq!(m.maps.len(), BUFFERS);
            assert!(!m.retired && !m.released);
            assert_eq!(f.source.released.load(Ordering::SeqCst), 0);
        }
        {
            let mut m = f.model.lock().unwrap();
            m.stuck_ready = false;
            m.fail_disable = false;
            m.fail_drain = false;
            m.fail_unmap = false;
        }
        a.namespaces[0].retire_owned().unwrap();
        assert!(f.model.lock().unwrap().released);
    }
}
#[test]
fn owned_visibility_failures_poison_without_exposing_unacquired_data() {
    for device in [true, false] {
        let f = Fixture::new();
        let mut a = f.attach().unwrap();
        {
            let mut m = f.model.lock().unwrap();
            m.fail_device_sync = device;
            m.fail_cpu_sync = !device;
        }
        let mut output = [0xa5; 512];
        assert!(a.namespaces[0].read(0, &mut output).is_err());
        assert_eq!(output, [0xa5; 512]);
        let commands = f.model.lock().unwrap().commands;
        assert!(a.namespaces[0].write(0, &[1; 512]).is_err());
        assert_eq!(f.model.lock().unwrap().commands, commands);
        {
            let mut m = f.model.lock().unwrap();
            m.fail_device_sync = false;
            m.fail_cpu_sync = false;
        }
        a.namespaces[0].retire_owned().unwrap();
        assert!(f.model.lock().unwrap().released);
    }
}
#[test]
fn owned_last_namespace_drop_retires_successful_controller() {
    let f = Fixture::new();
    let a = f.attach().unwrap();
    drop(a);
    let m = f.model.lock().unwrap();
    assert!(m.released && m.retired && m.maps.is_empty());
    assert_eq!(f.source.released.load(Ordering::SeqCst), BUFFERS);
}
#[test]
fn owned_drop_quarantines_hierarchy_and_cpu_pins_after_failed_drain() {
    let f = Fixture::new();
    let a = f.attach().unwrap();
    f.model.lock().unwrap().fail_drain = true;
    drop(a);
    let m = f.model.lock().unwrap();
    assert_eq!(m.maps.len(), BUFFERS);
    assert!(!m.retired && !m.released);
    assert_eq!(f.source.released.load(Ordering::SeqCst), 0);
}
#[test]
fn owned_absent_coherent_observation_refuses_before_enable_and_releases_unpublished_maps() {
    let f = Fixture::new();
    f.model.lock().unwrap().coherent = false;
    assert!(f.attach().is_err());
    let m = f.model.lock().unwrap();
    assert!(!m.bme && !m.enabled && !m.prepared);
    assert!(m.released && m.maps.is_empty());
    assert!(!m.events.contains(&"command") && !m.events.contains(&"bme_on"));
}
/// Registers that fail after a doorbell are found at the next status read:
/// a completion that looks invalid reads the status, and the read's failure
/// poisons the controller with no CPU reacquire. (A valid completion is
/// taken with no register read, as Linux takes it.)
#[test]
fn owned_register_failure_after_doorbell_poison_keeps_no_cpu_reacquire() {
    let f = Fixture::new();
    let mut a = f.attach().unwrap();
    let before = f.model.lock().unwrap().sync_cpu;
    let commands = f.model.lock().unwrap().commands;
    f.model.lock().unwrap().fail_after_doorbell = true;
    f.model.lock().unwrap().mutation = Mutation::Cid;
    assert!(a.namespaces[0].flush().is_err());
    {
        let m = f.model.lock().unwrap();
        assert!(!m.released);
        assert_eq!(m.commands, commands + 1);
        assert_eq!(m.sync_cpu, before);
    }
    assert!(a.namespaces[0].write(0, &[1; 512]).is_err());
    f.model.lock().unwrap().fail_read = false;
    a.namespaces[0].retire_owned().unwrap();
}
#[test]
fn owned_failed_prepare_in_d3_quarantines_without_any_nvme_mmio() {
    let f = Fixture::new();
    f.model.lock().unwrap().fail_prepare = true;
    assert!(f.attach().is_err());
    let m = f.model.lock().unwrap();
    assert!(!m.prepared && !m.enabled && !m.bme);
    assert_eq!(m.mmio_accesses, 0);
    assert_eq!(
        m.events.iter().filter(|e| **e == "prepare_attempt").count(),
        1
    );
    assert_eq!(m.maps.len(), BUFFERS);
    assert_eq!(m.unmap_attempts, 0);
    assert!(!m.retired && !m.released);
    assert_eq!((m.register_owner_drops, m.lifecycle_owner_drops), (0, 0));
    assert_eq!(f.source.released.load(Ordering::SeqCst), 0);
}

#[test]
fn owned_register_retirement_failure_retries_only_that_step_and_release() {
    let f = Fixture::new();
    let mut a = f.attach().unwrap();
    f.model.lock().unwrap().fail_retire = true;
    assert!(a.namespaces[0].retire_owned().is_err());
    let accesses = {
        let m = f.model.lock().unwrap();
        assert!(m.maps.is_empty() && m.drained && !m.bme && !m.enabled);
        assert!(!m.retired && !m.released);
        assert_eq!(m.unmap_attempts, BUFFERS);
        assert_eq!((m.register_owner_drops, m.lifecycle_owner_drops), (0, 0));
        assert_eq!(f.source.released.load(Ordering::SeqCst), BUFFERS);
        m.mmio_accesses
    };
    f.model.lock().unwrap().fail_retire = false;
    a.namespaces[0].retire_owned().unwrap();
    let m = f.model.lock().unwrap();
    assert_eq!(m.mmio_accesses, accesses);
    assert_eq!(m.unmap_attempts, BUFFERS);
    assert_eq!(
        m.events
            .iter()
            .filter(|e| **e == "retire_regs_attempt")
            .count(),
        2
    );
    assert_eq!(m.events.iter().filter(|e| **e == "bme_off").count(), 1);
    assert_eq!(m.events.iter().filter(|e| **e == "drain").count(), 1);
    assert!(m.retired && m.released);
}

#[test]
fn owned_hierarchy_release_failure_never_accesses_retired_registers_on_retry() {
    let f = Fixture::new();
    let mut a = f.attach().unwrap();
    f.model.lock().unwrap().fail_release = true;
    assert!(a.namespaces[0].retire_owned().is_err());
    let accesses = {
        let m = f.model.lock().unwrap();
        assert!(m.retired && !m.released && m.maps.is_empty());
        assert_eq!((m.register_owner_drops, m.lifecycle_owner_drops), (0, 0));
        m.mmio_accesses
    };
    f.model.lock().unwrap().fail_release = false;
    a.namespaces[0].retire_owned().unwrap();
    a.namespaces[0].retire_owned().unwrap();
    let m = f.model.lock().unwrap();
    assert_eq!(m.mmio_accesses, accesses);
    assert_eq!(m.unmap_attempts, BUFFERS);
    assert_eq!(
        m.events
            .iter()
            .filter(|e| **e == "retire_regs_attempt")
            .count(),
        1
    );
    assert_eq!(
        m.events.iter().filter(|e| **e == "release_attempt").count(),
        2
    );
    assert!(m.released);
}

#[test]
fn owned_partial_ram_unmap_retries_only_remaining_original_leases() {
    for nth in 2..=6 {
        let f = Fixture::new();
        let mut a = f.attach().unwrap();
        let ids: Vec<_> = f.model.lock().unwrap().maps.keys().copied().collect();
        f.model.lock().unwrap().fail_unmap_at = Some(nth);
        assert!(a.namespaces[0].retire_owned().is_err());
        let accesses = {
            let m = f.model.lock().unwrap();
            assert_eq!(m.maps.keys().copied().collect::<Vec<_>>(), ids[nth - 1..]);
            assert_eq!(m.unmap_attempts, nth);
            assert!(m.drained && !m.bme && !m.enabled && !m.retired && !m.released);
            assert_eq!(f.source.released.load(Ordering::SeqCst), nth - 1);
            m.mmio_accesses
        };
        f.model.lock().unwrap().fail_unmap_at = None;
        a.namespaces[0].retire_owned().unwrap();
        let m = f.model.lock().unwrap();
        assert_eq!(m.mmio_accesses, accesses);
        assert_eq!(m.unmap_attempts, BUFFERS + 1);
        assert_eq!(m.events.iter().filter(|e| **e == "drain").count(), 1);
        assert_eq!(m.events.iter().filter(|e| **e == "bme_off").count(), 1);
        assert!(m.released && m.retired && m.maps.is_empty());
        assert_eq!(f.source.released.load(Ordering::SeqCst), BUFFERS);
    }
}

#[repr(align(4096))]
struct RegisterPages([u8; 0x4000]);
struct RegisterRam(core::cell::UnsafeCell<RegisterPages>);
// SAFETY: actual isolated register RAM, accessed raw only by serialized fixture
// users. No native/control/Authority/Source product is included in this pin.
unsafe impl Send for RegisterRam {}
// SAFETY: same exclusive hosted raw-memory contract as Send.
unsafe impl Sync for RegisterRam {}
impl RegisterRam {
    fn pointer(&self) -> NonNull<u8> {
        // SAFETY: retained exact UnsafeCell allocation, without a shared slice.
        unsafe { NonNull::new_unchecked(core::ptr::addr_of_mut!((*self.0.get()).0).cast()) }
    }
}
struct RegisterPlatform {
    pages: Arc<RegisterRam>,
    unmaps: AtomicUsize,
}
struct RegisterResource(
    Arc<AtomicUsize>,
    crate::bus::mmio::owned::model::MappingRetention<Arc<RegisterRam>>,
);
impl Drop for RegisterResource {
    fn drop(&mut self) {
        self.0.fetch_add(1, Ordering::SeqCst);
    }
}
// SAFETY: isolated model reservation, kept through the real Permission;
// this physical interval is metadata, not a CPU or DMA pointer.
unsafe impl crate::bus::mmio::owned::ResourceBacking for RegisterResource {
    fn begin_mapping(
        &self,
    ) -> Result<crate::bus::pci::assignment::OpaqueUse, crate::bus::mmio::owned::Error> {
        self.1.begin_mapping()
    }

    fn physical_span(&self) -> crate::bus::mmio::owned::PhysicalSpan {
        crate::bus::mmio::owned::PhysicalSpan::new(0x2000_0000, 0x4000).unwrap()
    }
    fn kind(&self) -> crate::bus::mmio::owned::ResourceKind {
        crate::bus::mmio::owned::ResourceKind::Registers
    }
}
// SAFETY: isolated aligned RAM models UC register storage, kept by this
// platform owner. Real registry reservations/retirement are used;
// only a legitimate sub-page Mapping offset creates an unaligned accessor.
unsafe impl crate::bus::mmio::owned::MappingPlatform for RegisterPlatform {
    fn check_context(
        &self,
        _: crate::bus::mmio::owned::AddressSpace,
    ) -> Result<(), crate::bus::mmio::owned::Error> {
        Ok(())
    }
    unsafe fn map(
        &self,
        resource: &crate::bus::mmio::owned::Resource,
        request: crate::bus::mmio::owned::MapRequest,
    ) -> Result<crate::bus::mmio::owned::InstalledMapping, crate::bus::mmio::owned::MapFailure>
    {
        assert!(resource.backing().is::<RegisterResource>());
        assert_eq!(request.physical.start(), 0x2000_0000);
        assert!(request.physical.bytes() <= 0x4000);
        let pointer = self.pages.pointer();
        Ok(crate::bus::mmio::owned::InstalledMapping {
            token: request.id,
            cpu_base: pointer,
            physical: request.physical,
            attribute: request.attribute,
            protection: request.protection,
        })
    }
    unsafe fn unmap(
        &self,
        request: crate::bus::mmio::owned::MapRequest,
        installed: crate::bus::mmio::owned::InstalledMapping,
    ) -> Result<(), crate::bus::mmio::owned::Error> {
        assert_eq!(request.id, installed.token);
        assert_eq!(request.physical, installed.physical);
        self.unmaps.fetch_add(1, Ordering::SeqCst);
        Ok(())
    }
}

#[test]
fn owned_mapping_registers_reject_unaligned_mapping_and_return_its_owner() {
    use crate::bus::mmio::owned::{AddressSpace, Authority, Resource};
    for offset in [1, 2, 3, 0] {
        let platform = Arc::new(RegisterPlatform {
            pages: Arc::new(RegisterRam(core::cell::UnsafeCell::new(RegisterPages(
                [0; 0x4000],
            )))),
            unmaps: AtomicUsize::new(0),
        });
        let released = Arc::new(AtomicUsize::new(0));
        // SAFETY: exclusive isolated model resource/root, no other aliases.
        let authority = unsafe {
            Authority::isolated_model(
                platform.clone(),
                AddressSpace {
                    identity: 1,
                    physical_bits: 48,
                    virtual_bits: 48,
                },
                vec![],
            )
        }
        .unwrap();
        // SAFETY: retained original isolated allocation and exclusive model
        // reservation, with no Source/Authority/view backedge or native device.
        let mut mapping_owner = unsafe {
            crate::bus::mmio::owned::model::MappingOwner::from_owned(
                platform.pages.clone(),
                |owner| {
                    if (owner.pointer().as_ptr() as usize).is_multiple_of(4096) {
                        Ok(())
                    } else {
                        Err(crate::bus::pci::assignment::Error::Context)
                    }
                },
            )
        };
        let mapping = authority
            .map(
                Resource::new(Box::new(RegisterResource(
                    released.clone(),
                    mapping_owner.retention().unwrap(),
                ))),
                offset,
                0x2000,
                CacheAttribute::Uncached,
                Protection::ReadWrite,
            )
            .unwrap();
        let original = mapping.address(0, 0x2000).unwrap();
        assert_eq!((original.as_ptr() as usize) % 4, offset);
        if offset == 0 {
            let mut registers = MappingRegisters::new(mapping).ok().unwrap();
            assert!(registers.read32(1).is_err());
            assert!(registers.write32(0x1ffe, 1).is_err());
            assert!(registers.read32(0x2000).is_err());
            registers.write32(0x1ffc, 0x1234_abcd).unwrap();
            assert_eq!(registers.read32(0x1ffc).unwrap(), 0x1234_abcd);
            // SAFETY: only these synchronous CPU accesses existed.
            unsafe { registers.retire() }.unwrap();
            assert!(registers.read32(0).is_err());
        } else {
            let (_, mut returned) = MappingRegisters::new(mapping).err().unwrap();
            assert_eq!(returned.address(0, 0x2000).unwrap(), original);
            assert_eq!(
                returned.physical_span().unwrap().start(),
                0x2000_0000 + offset as u64
            );
            assert_eq!(authority.status().unwrap().live, 1);
            assert_eq!(released.load(Ordering::SeqCst), 0);
            assert_eq!(platform.unmaps.load(Ordering::SeqCst), 0);
            // SAFETY: constructor refused before any register access.
            unsafe { returned.retire() }.unwrap();
        }
        assert_eq!(platform.unmaps.load(Ordering::SeqCst), 1);
        assert_eq!(released.load(Ordering::SeqCst), 1);
        assert_eq!(authority.status().unwrap().live, 0);
        // SAFETY: all returned mapping users ended and acknowledged unmap;
        // only this external fixture controller can recover its real RAM.
        unsafe { mapping_owner.retire() }.unwrap();
    }
}
