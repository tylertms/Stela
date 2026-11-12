// SPDX-License-Identifier: GPL-2.0-or-later
//! Class control custody and sticky first minimum failure.
use super::{BlockError, Control, Vec};
use core::mem::ManuallyDrop;
/// Retains the original control owner after a minimum source fails. There is
/// no class retirement acknowledgement that could make that failure releasable.
pub(super) struct MinimumControl<C> {
    source: ManuallyDrop<C>,
    failure: Option<BlockError>,
}
impl<C> MinimumControl<C> {
    #[cfg(test)]
    pub(super) fn source_mut(&mut self) -> &mut C {
        &mut self.source
    }
    pub fn error(&self) -> Option<&BlockError> {
        self.failure.as_ref()
    }
    pub fn new(source: C) -> Self {
        Self {
            source: ManuallyDrop::new(source),
            failure: None,
        }
    }
}
impl<C: Control> Control for MinimumControl<C> {
    async fn control(&mut self, request: [u8; 8], output: &[u8]) -> Result<Vec<u8>, BlockError> {
        if let Some(error) = &self.failure {
            return Err(error.clone());
        }
        let result = self.source.control(request, output).await;
        if let Err(error @ BlockError::PciPower(_)) = &result {
            self.failure = Some(error.clone());
        }
        result
    }
    async fn delay_us(&mut self, us: u64) {
        if self.failure.is_none() {
            self.source.delay_us(us).await;
        }
    }
    async fn minimum_wait(&mut self, us: u64) -> Result<(), BlockError> {
        if let Some(error) = &self.failure {
            return Err(error.clone());
        }
        let result = self.source.minimum_wait(us).await;
        if let Err(error) = &result {
            self.failure = Some(error.clone());
        }
        result
    }
}
impl<C> Drop for MinimumControl<C> {
    fn drop(&mut self) {
        if self.failure.is_none() {
            // SAFETY: the ordinary class control lifetime is unchanged when
            // no minimum source failed; this is the sole extraction path.
            unsafe { ManuallyDrop::drop(&mut self.source) };
        }
    }
}
#[cfg(test)]
mod tests;
