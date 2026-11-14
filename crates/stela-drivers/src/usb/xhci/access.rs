// SPDX-License-Identifier: GPL-2.0-only
//! The controller as enumeration and hub bring-up reach it. At boot the
//! work runs to its end in one call ([`Access::blocking`]): each command,
//! control transfer and wait below is the same blocking call it always was,
//! so the async code never suspends. After boot it runs in steps from the
//! guest's service turns ([`Access::stepped`]), as Linux's hub_event runs
//! in its own thread: each wait ends the step and is looked at again at the
//! next one, under the same deadline, and the controller is borrowed only
//! within a step.
use super::*;
use core::{cell::Cell, ptr::NonNull};
use stela_driver_base::stepped::{until, yield_now};

enum Source<'a> {
    Blocking(RefCell<&'a mut Controller>),
    Stepped(Shared),
}

pub(super) struct Access<'a> {
    controller: Source<'a>,
    /// The DMA allocator of the call or step under way.
    dma: Cell<Option<NonNull<dyn DmaAllocator + 'a>>>,
}

impl<'a> Access<'a> {
    pub(super) fn blocking(
        controller: &'a mut Controller,
        dma: Option<&'a mut dyn DmaAllocator>,
    ) -> Self {
        Self {
            controller: Source::Blocking(RefCell::new(controller)),
            dma: Cell::new(dma.map(NonNull::from)),
        }
    }

    pub(super) fn stepped(controller: Shared) -> Access<'static> {
        Access {
            controller: Source::Stepped(controller),
            dma: Cell::new(None),
        }
    }

    /// Run one step `f` with `dma` as the allocator.
    pub(super) fn lend<R>(&self, dma: &mut dyn DmaAllocator, f: impl FnOnce() -> R) -> R {
        struct Return<'b, 'a>(&'b Cell<Option<NonNull<dyn DmaAllocator + 'a>>>);
        impl Drop for Return<'_, '_> {
            fn drop(&mut self) {
                self.0.set(None);
            }
        }
        let pointer = NonNull::from(dma);
        // SAFETY: only the lifetime bound changes. The pointer is taken back
        // before this call returns, even by unwinding, and is dereferenced
        // only by `with_dma` within `f`, while `dma` is lent to it.
        let pointer = unsafe {
            core::mem::transmute::<NonNull<dyn DmaAllocator + '_>, NonNull<dyn DmaAllocator + 'a>>(
                pointer,
            )
        };
        self.dma.set(Some(pointer));
        let _return = Return(&self.dma);
        f()
    }

    fn stepping(&self) -> bool {
        matches!(self.controller, Source::Stepped(_))
    }

    /// The shared controller of work in steps.
    pub(super) fn shared(&self) -> Option<&Shared> {
        match &self.controller {
            Source::Stepped(shared) => Some(shared),
            Source::Blocking(_) => None,
        }
    }

    pub(super) fn with<R>(&self, f: impl FnOnce(&mut Controller) -> R) -> R {
        match &self.controller {
            Source::Blocking(controller) => f(&mut controller.borrow_mut()),
            Source::Stepped(controller) => f(&mut controller.borrow_mut()),
        }
    }

    pub(super) fn with_dma<R>(
        &self,
        f: impl FnOnce(&mut Controller, &mut dyn DmaAllocator) -> R,
    ) -> Result<R, BlockError> {
        let Some(mut dma) = self.dma.get() else {
            return Err(BlockError::Unsupported("USB step has no DMA allocator"));
        };
        // SAFETY: the allocator is lent for this call or step (`blocking`,
        // `lend`); the controller borrow `with` holds refuses a nested
        // `with_dma`, so no second reference to it exists.
        Ok(self.with(|controller| f(controller, unsafe { dma.as_mut() })))
    }

    /// [`Controller::command`].
    pub(super) async fn command(
        &self,
        parameter: u64,
        status: u32,
        control: u32,
    ) -> Result<Event, BlockError> {
        if !self.stepping() {
            return self.with(|c| c.command(parameter, status, control));
        }
        // Each command and transfer starts a step of its own, as each waits
        // for the hardware.
        yield_now().await;
        let mut started = None;
        loop {
            let step = self.with(|c| c.step_command(&mut started, parameter, status, control));
            if let Some(result) = step {
                return result;
            }
            yield_now().await;
        }
    }

    /// [`Controller::control_owned`].
    pub(super) async fn control(
        &self,
        slot: u8,
        request: [u8; 8],
        out: &[u8],
    ) -> Result<Vec<u8>, BlockError> {
        if !self.stepping() {
            return self.with(|c| c.control_owned(slot, request, out));
        }
        yield_now().await;
        let mut control = Controller::start_control_owned_with(self, slot, request, out).await?;
        loop {
            if let Some(result) = self.with(|c| c.poll_control(&mut control)) {
                return result;
            }
            yield_now().await;
        }
    }

    /// [`DeviceAccess::control`] for this exact device.
    pub(super) async fn device_control(
        &self,
        id: SlotId,
        request: [u8; 8],
        out: &[u8],
    ) -> Result<Vec<u8>, BlockError> {
        if !self.stepping() {
            return self.with(|c| c.device(id)?.control(request, out));
        }
        yield_now().await;
        let mut operation = self.with(|c| c.device(id)?.start_control(request, out))?;
        loop {
            if let Some(result) = self.with(|c| match c.device(id) {
                Ok(mut device) => device.poll(&mut operation),
                Err(error) => Some(Err(error)),
            }) {
                return result;
            }
            yield_now().await;
        }
    }

    /// [`Controller::minimum_wait`]: in steps where the minimum timer keeps
    /// an interval, otherwise the blocking wait.
    pub(super) async fn minimum_wait(&self, microseconds: u64) -> Result<(), BlockError> {
        if !self.stepping() {
            return self.with(|c| c.minimum_wait(microseconds));
        }
        let Some(interval) = self.with(|c| c.begin_minimum(microseconds))? else {
            return self.with(|c| c.minimum_wait(microseconds));
        };
        while !self.with(|c| c.minimum_elapsed(interval.as_ref()))? {
            yield_now().await;
        }
        Ok(())
    }

    /// The controller clock's busy delay.
    pub(super) async fn delay(&self, microseconds: u64) {
        let clock = self.with(|c| c.clock);
        if !self.stepping() {
            return clock.delay(microseconds);
        }
        until(clock.deadline(microseconds)).await;
    }

    /// Poll `ready` against the registers until it holds or `microseconds`
    /// pass ([`super::wait`]).
    pub(super) async fn wait(
        &self,
        microseconds: u64,
        mut ready: impl FnMut(&mut dyn Registers) -> bool,
    ) -> bool {
        if !self.stepping() {
            return self.with(|c| {
                let clock = c.clock;
                wait(&mut *c.registers, clock, microseconds, ready)
            });
        }
        let deadline = self.with(|c| c.clock.deadline(microseconds));
        loop {
            if self.with(|c| ready(&mut *c.registers)) {
                return true;
            }
            if deadline.passed() {
                return self.with(|c| ready(&mut *c.registers));
            }
            yield_now().await;
        }
    }
}
