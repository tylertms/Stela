// SPDX-License-Identifier: BSD-3-Clause-Clear
// Copyright (C) 2020-2023 MediaTek Inc.
//! Private same-input binding from a completed native download. Input
//! digests/provenance are not firmware readback, authentication or radio policy.
use super::{Mailbox, QueueIo};
use crate::{
    firmware::{Image, Provenance, Target},
    wifi::mediatek::{
        download::{self, Downloader, PatchState, ReleaseState, Transport, TransportFailure},
        firmware::{Prepared, Requests, auxiliary::Auxiliary},
    },
};
use alloc::string::String;

#[derive(Debug)]
pub struct Input {
    name: String,
    digest: [u8; 32],
    driver: String,
    hardware: String,
    revision: String,
    board: Option<String>,
    source: String,
    source_revision: String,
    member: String,
    license: String,
}
impl Input {
    fn copy(image: Image<'_>) -> Self {
        let target = image.target();
        let provenance = image.provenance();
        Self {
            name: image.name().into(),
            digest: image.sha256(),
            driver: target.driver.into(),
            hardware: target.hardware.into(),
            revision: target.revision.into(),
            board: target.board.map(String::from),
            source: provenance.source.into(),
            source_revision: provenance.revision.into(),
            member: provenance.member.into(),
            license: provenance.license.into(),
        }
    }
    pub fn name(&self) -> &str {
        &self.name
    }
    pub fn sha256(&self) -> [u8; 32] {
        self.digest
    }
    pub fn target(&self) -> Target<'_> {
        Target {
            driver: &self.driver,
            hardware: &self.hardware,
            revision: &self.revision,
            board: self.board.as_deref(),
        }
    }
    pub fn provenance(&self) -> Provenance<'_> {
        Provenance {
            source: &self.source,
            revision: &self.source_revision,
            member: &self.member,
            license: &self.license,
        }
    }
}
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Outcome {
    pub patch: PatchState,
    pub release: ReleaseState,
    pub commands_completed: usize,
    pub scatter_transfers_completed: usize,
    pub downloaded_regions: usize,
    pub skipped_ram_regions: usize,
    pub start_address: Option<u32>,
    pub firmware_start_acknowledged: bool,
}
impl Outcome {
    fn copy(report: &download::Report) -> Self {
        Self {
            patch: report.patch,
            release: report.release,
            commands_completed: report.commands_completed,
            scatter_transfers_completed: report.scatter_transfers_completed,
            downloaded_regions: report.regions.len(),
            skipped_ram_regions: report.skipped_ram_regions.len(),
            start_address: report.start_address,
            firmware_start_acknowledged: report.firmware_start_acknowledged,
        }
    }
}
/// Immutable diagnostics; native consumers check the current private binding
/// on their mailbox, never a caller-constructed public Report. The
/// supplied patch input is not physically identified when AlreadyApplied.
#[derive(Debug)]
pub struct Binding {
    requests: Requests,
    patch: Input,
    ram: Input,
    outcome: Outcome,
}
impl Binding {
    pub fn requests(&self) -> &Requests {
        &self.requests
    }
    pub fn patch_input(&self) -> &Input {
        &self.patch
    }
    /// This includes host-only non-download CLC/release bytes from the same
    /// RAM input. It does not claim those bytes were uploaded/read back.
    pub fn ram_input(&self) -> &Input {
        &self.ram
    }
    pub fn outcome(&self) -> Outcome {
        self.outcome
    }
    /// Same selected input only. This diagnostic comparison never checks
    /// current hardware/reset/policy state or a caller-supplied public outcome.
    pub fn matches_auxiliary(&self, auxiliary: &Auxiliary) -> bool {
        let provenance = self.ram.provenance();
        let auxiliary_provenance = auxiliary.provenance();
        &self.requests == auxiliary.requests()
            && self.ram.name() == auxiliary.name()
            && self.ram.sha256() == auxiliary.sha256()
            && provenance.source == auxiliary_provenance.source
            && provenance.revision == auxiliary_provenance.revision
            && provenance.member == auxiliary_provenance.member
            && provenance.license == auxiliary_provenance.license
    }
}
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum Error {
    NotFresh,
    Protocol(download::Error),
    MissingStartAcknowledgement,
}
impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "MediaTek bound download: {self:?}")
    }
}
impl core::error::Error for Error {}

/// The mailbox remains owned by the caller on every outcome. This
/// local adapter delegates all ownership/DMA/error behavior unchanged.
struct Borrowed<'a, W: QueueIo, F: QueueIo, R: QueueIo>(&'a mut Mailbox<W, F, R>);
impl<W: QueueIo, F: QueueIo, R: QueueIo> Transport for Borrowed<'_, W, F, R> {
    fn validate_firmware(&self, prepared: &Prepared<'_>) -> Result<(), TransportFailure> {
        Transport::validate_firmware(&*self.0, prepared)
    }
    fn command(&mut self, command: u8, payload: &[u8]) -> Result<u32, TransportFailure> {
        Transport::command(self.0, command, payload)
    }
    fn scatter(&mut self, payload: &[u8]) -> Result<(), TransportFailure> {
        Transport::scatter(self.0, payload)
    }
}
impl<W: QueueIo, F: QueueIo, R: QueueIo> Mailbox<W, F, R> {
    /// Historical input/protocol diagnostics only, not a policy or ready radio.
    pub fn download_binding(&self) -> Option<&Binding> {
        self.download_binding.as_ref()
    }
    /// Execute one complete native download on a fresh unused mailbox.
    /// Source AlreadyApplied patch remains distinct from uploaded patch bytes.
    /// On any failure the owned mailbox remains here, terminally poisoned with
    /// no old input permission; independent quiescence is still required
    /// before its owned queues/context can be released. No reset is invented.
    pub async fn download_with_binding(
        &mut self,
        prepared: &Prepared<'_>,
    ) -> Result<&Binding, Error> {
        self.download_binding = None;
        if self.poisoned || self.cookie != 0 {
            self.poisoned = true;
            return Err(Error::NotFresh);
        }
        self.download_bound_input(prepared).await
    }
    /// Only the private NIC_POWER_CTRL one-way completion on this
    /// otherwise unused owner permits source sequence continuation. This is
    /// no reset/fresh-epoch proof and is invalidated by any intervening command.
    pub(in crate::wifi::mediatek) async fn download_after_source_restart(
        &mut self,
        prepared: &Prepared<'_>,
    ) -> Result<&Binding, Error> {
        self.download_binding = None;
        if self.poisoned || self.cookie != 1 || self.source_restart_cookie != Some(1) {
            self.poisoned = true;
            return Err(Error::NotFresh);
        }
        self.source_restart_cookie = None;
        self.download_bound_input(prepared).await
    }
    async fn download_bound_input(&mut self, prepared: &Prepared<'_>) -> Result<&Binding, Error> {
        // Own the exact canonical Prepared identity/provenance before any
        // device mutation. Canonical Requests::load checked returned Images.
        let requests = prepared.requests().clone();
        let patch = Input::copy(prepared.patch_image());
        let ram = Input::copy(prepared.ram_image());
        let outcome = {
            let mut downloader = Downloader::new(Borrowed(self));
            downloader.download(prepared).await.map(Outcome::copy)
        };
        let outcome = match outcome {
            Ok(outcome) => outcome,
            Err(error) => {
                self.poisoned = true;
                self.download_binding = None;
                return Err(Error::Protocol(error));
            }
        };
        if !outcome.firmware_start_acknowledged {
            self.poisoned = true;
            return Err(Error::MissingStartAcknowledgement);
        }
        self.download_binding = Some(Binding {
            requests,
            patch,
            ram,
            outcome,
        });
        Ok(self
            .download_binding
            .as_ref()
            .expect("actual completed native input binding"))
    }
}
