//! Layout data only; the constructor separately validates the original
//! handoff, RootIdentity, sealed table/cache state and complete raw allocation.
use super::*;
use stela_boot_core::guarded_stacks::Slot;
pub(super) struct Plan {
    base: u64,
    virtual_bytes: u64,
    raw_bytes: u64,
    pub(super) raw: Region,
}
impl Plan {
    pub(super) fn new(handoff: &Handoff) -> Result<Self, Error> {
        let base = layout::task_stacks();
        let geometry = native_smp::resources::task_stacks(base).map_err(|_| Error::Geometry)?;
        Ok(Self {
            base,
            virtual_bytes: geometry.virtual_bytes(),
            raw_bytes: geometry.raw_allocation_bytes(),
            raw: handoff.native_resources.tasks,
        })
    }
    pub(super) fn validate(&self, handoff: &Handoff) -> Result<(), Error> {
        let actual = handoff.native_resources.tasks;
        if self.raw.physical != actual.physical
            || self.raw.bytes != actual.bytes
            || self.raw.bytes != self.raw_bytes
        {
            return Err(Error::Inventory);
        }
        self.raw
            .physical
            .checked_add(self.raw.bytes)
            .ok_or(Error::Geometry)?;
        Ok(())
    }
    pub(super) fn region(&self) -> Region {
        self.raw
    }
    pub(super) fn count(&self) -> usize {
        native_smp::resources::TASK_STACKS
    }
    pub(super) fn raw_bytes(&self) -> u64 {
        self.raw_bytes
    }
    pub(super) fn virtual_range(&self) -> Range<u64> {
        self.base..self.base + self.virtual_bytes
    }
    pub(super) fn slot(&self, index: usize) -> Result<(Slot, u64), Error> {
        // Recompute this fixed checked descriptor on the preparation stack;
        // do not keep its 1.6 KiB payload in every Plan.
        let layout = native_smp::resources::task_stacks(self.base).map_err(|_| Error::Geometry)?;
        let end = self
            .raw
            .physical
            .checked_add(self.raw.bytes)
            .ok_or(Error::Geometry)?;
        let backing = layout
            .backing(self.raw.physical..end, index)
            .map_err(|_| Error::Geometry)?;
        Ok((
            layout.slot(index).map_err(|_| Error::Geometry)?,
            backing.start,
        ))
    }
}
pub(super) fn profile() -> Result<work::StackProfile, Error> {
    let kernel = stela_linux::task::stack_requirement().map_err(Error::Work)?;
    let bytes = native_smp::resources::LINUX_TASK_STACK_BYTES as usize;
    if kernel != work::StackRequirement::exact(bytes, bytes, 4096).map_err(Error::Work)? {
        return Err(Error::Geometry);
    }
    work::StackProfile::new(
        kernel,
        work::StackRequirement::at_least(
            native_smp::resources::ORDINARY_TASK_STACK_BYTES as usize,
            4096,
            4096,
        )
        .map_err(Error::Work)?,
        native_smp::resources::KERNEL_GENERAL_TASKS,
        native_smp::resources::ORDINARY_GENERAL_TASKS,
    )
    .map_err(Error::Work)
}
