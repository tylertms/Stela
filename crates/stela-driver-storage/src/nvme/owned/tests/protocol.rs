// SPDX-License-Identifier: GPL-2.0-only
// Copyright (c) 2011-2014, Intel Corporation.
//! Reuse the independent SQ/PRP/CQ peer with a model hierarchy observer.
//! This is cfg(test) support, never a native provider or domain-identity proof.
use super::*;

/// Use the same attachment sequence with a captured finite test budget.
///
/// # Safety
/// The caller provides the same exclusive original-controller and requester
/// permission required by `owned::attach`; this only shortens model timeouts.
pub(crate) unsafe fn attach(
    registers: Box<dyn Registers>,
    lifecycle: Box<dyn Lifecycle>,
    source: &dyn PageSource,
    domain: Arc<dyn DmaDomain>,
    clock: Clock,
    location: String,
    quirks: Quirks,
) -> Result<Attached, BlockError> {
    // SAFETY: forwarded unchanged kept-owner attachment contract.
    unsafe {
        super::super::attach_with_timeout(
            registers, lifecycle, source, domain, clock, location, 1_000, quirks,
        )
    }
}

/// Read the independent configuration storage at each protocol boundary.
/// Implementors must not substitute a lifecycle's cached phase or intended write.
pub(crate) trait Access: Send + Sync {
    fn registers_available(&self) -> bool;
    fn dma_enabled(&self) -> bool;
    fn any_bus_master(&self) -> bool;
    fn observe(&self, event: &'static str);
}

#[derive(Clone, Copy)]
pub(crate) enum Fault {
    None,
    Cid,
    Sqid,
    Head,
    Phase,
    Delayed,
    DeviceVisibility,
    CpuVisibility,
    Drain,
    Unmap,
    RegistersRetire,
}

pub(crate) struct Snapshot {
    pub(crate) mappings: Vec<u64>,
    pub(crate) commands: usize,
    pub(crate) sync_cpu: usize,
    pub(crate) mmio_accesses: usize,
    pub(crate) unmap_attempts: usize,
    pub(crate) register_owner_drops: usize,
    pub(crate) pages_released: usize,
    pub(crate) drained: bool,
    pub(crate) enabled: bool,
    pub(crate) retired: bool,
    pub(crate) events: Vec<&'static str>,
}

pub(crate) struct Protocol {
    model: Arc<Mutex<Model>>,
    source: Source,
}
impl Protocol {
    pub(crate) fn new(access: Arc<dyn Access>) -> Self {
        let mut model = Model::new();
        model.access = Some(access);
        Self {
            model: Arc::new(Mutex::new(model)),
            source: Source {
                next: AtomicU64::new(0x9000_0000),
                released: Arc::new(AtomicUsize::new(0)),
            },
        }
    }
    pub(crate) fn registers(&self) -> Box<dyn Registers> {
        Box::new(Regs(self.model.clone()))
    }
    pub(crate) fn domain(&self) -> Arc<dyn DmaDomain> {
        Arc::new(Domain(self.model.clone()))
    }
    pub(crate) fn pages(&self) -> &dyn PageSource {
        &self.source
    }
    pub(crate) fn fault(&self, fault: Fault) {
        let mut model = self.model.lock().unwrap();
        model.mutation = match fault {
            Fault::Cid => Mutation::Cid,
            Fault::Sqid => Mutation::Sqid,
            Fault::Head => Mutation::Head,
            Fault::Phase => Mutation::Phase,
            Fault::Delayed => Mutation::Delayed,
            _ => Mutation::None,
        };
        model.fail_device_sync = matches!(fault, Fault::DeviceVisibility);
        model.fail_cpu_sync = matches!(fault, Fault::CpuVisibility);
        model.fail_drain = matches!(fault, Fault::Drain);
        model.fail_unmap = matches!(fault, Fault::Unmap);
        model.fail_retire = matches!(fault, Fault::RegistersRetire);
    }
    pub(crate) fn sibling(namespace: &Namespace) -> Namespace {
        Namespace {
            controller: namespace.controller.clone(),
            namespace: namespace.namespace,
            identity: namespace.identity.clone(),
            ids: namespace.ids.clone(),
            scanned: namespace.scanned,
            removed: namespace.removed.clone(),
        }
    }
    pub(crate) fn snapshot(&self) -> Snapshot {
        let model = self.model.lock().unwrap();
        Snapshot {
            mappings: model.maps.keys().copied().collect(),
            commands: model.commands,
            sync_cpu: model.sync_cpu,
            mmio_accesses: model.mmio_accesses,
            unmap_attempts: model.unmap_attempts,
            register_owner_drops: model.register_owner_drops,
            pages_released: self.source.released.load(Ordering::SeqCst),
            drained: model.drained,
            enabled: model.enabled,
            retired: model.retired,
            events: model.events.clone(),
        }
    }
}
