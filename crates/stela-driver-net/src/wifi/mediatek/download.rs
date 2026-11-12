// SPDX-License-Identifier: BSD-3-Clause-Clear
// Copyright (C) 2020-2023 MediaTek Inc.
// Copyright (C) 2019 Lorenzo Bianconi <lorenzo.bianconi83@gmail.com>
//! Patch semaphore, 4096-byte scatter and WM-start protocol from pinned mt76.
//! A successful download is not NIC capability discovery, CLC installation,
//! radio calibration, regulatory permission, association or a ready data link.
use super::{
    firmware::{Patch, Prepared, Ram},
    identity::Chip,
};
use alloc::vec::Vec;
use core::fmt;
use stela_driver_base::stepped::yield_now;

const COMMAND_DOWNLOAD: u8 = 0x01;
const COMMAND_START_FIRMWARE: u8 = 0x02;
const COMMAND_PATCH_START: u8 = 0x05;
const COMMAND_PATCH_FINISH: u8 = 0x07;
const COMMAND_PATCH_SEMAPHORE: u8 = 0x10;
const SCATTER_BYTES: usize = 4096;
const PATCH_ALREADY_APPLIED: u32 = 1;
const PATCH_SEMAPHORE_ACQUIRED: u32 = 2;
const PATCH_SEMAPHORE_RELEASED: u32 = 3;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PortState {
    Usable,
    Uncertain,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum FailureKind {
    FirmwareTarget,
    Timeout,
    Disconnected,
    TransportUnavailable,
    InvalidResponse,
    AlreadyAttempted,
    Device(u32),
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct TransportFailure {
    pub kind: FailureKind,
    /// Usable means the command/transfer has completed and no stale
    /// DMA, response or worker can touch a subsequently reused buffer.
    pub port: PortState,
}

/// A live owned MCU transport. The implementation validates response
/// sequence/event framing and returns the donor parser's semantic result
/// (patch semaphore/status byte, zero for a validated ordinary acknowledgement).
///
/// `scatter` uses MCU FW_SCATTER, has no invented MCU response, and must
/// keep bytes until real TX-DMA completion. On uncertain failure it must
/// quarantine its DMA/command resources and return `Uncertain`; the downloader
/// will not attempt cleanup through an unresolved command port. A successful
/// enqueue alone never permits that transport to release DMA ownership.
pub trait Transport {
    /// Pure exact-target validation against this owned transport's
    /// observed device identity. No MMIO, DMA publication or commands occur.
    fn validate_firmware(&self, prepared: &Prepared<'_>) -> Result<(), TransportFailure>;
    fn command(&mut self, command: u8, payload: &[u8]) -> Result<u32, TransportFailure>;
    fn scatter(&mut self, payload: &[u8]) -> Result<(), TransportFailure>;
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Stage {
    FirmwareTarget,
    AcquirePatch,
    PatchStart(usize),
    PatchData(usize),
    PatchFinish,
    ReleasePatch,
    RamStart(usize),
    RamData(usize),
    FirmwareStart,
    AlreadyAttempted,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PatchState {
    NotAttempted,
    AlreadyApplied,
    Uploaded,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ReleaseState {
    NotHeld,
    Held,
    Released,
    Failed(TransportFailure),
    UnsafeToAttempt,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct RegionProgress {
    pub stage: Stage,
    pub address: u32,
    pub mode: u32,
    pub expected_bytes: usize,
    pub completed_bytes: usize,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Report {
    pub patch: PatchState,
    pub release: ReleaseState,
    pub commands_completed: usize,
    pub scatter_transfers_completed: usize,
    pub regions: Vec<RegionProgress>,
    pub skipped_ram_regions: Vec<usize>,
    pub start_address: Option<u32>,
    pub firmware_start_acknowledged: bool,
}
impl Default for Report {
    fn default() -> Self {
        Self {
            patch: PatchState::NotAttempted,
            release: ReleaseState::NotHeld,
            commands_completed: 0,
            scatter_transfers_completed: 0,
            regions: Vec::new(),
            skipped_ram_regions: Vec::new(),
            start_address: None,
            firmware_start_acknowledged: false,
        }
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Error {
    pub stage: Stage,
    pub failure: TransportFailure,
    pub report: Report,
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "MediaTek download {:?}: {:?} ({:?})",
            self.stage, self.failure.kind, self.failure.port
        )
    }
}
impl core::error::Error for Error {}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum State {
    Fresh,
    Complete,
    Poisoned,
}

/// One firmware-download attempt. Failed or completed instances cannot be
/// silently reused; a reset/reinitialization owner must establish a fresh
/// hardware/transport epoch before constructing another downloader.
pub struct Downloader<T: Transport> {
    transport: T,
    state: State,
    report: Report,
}
impl<T: Transport> Downloader<T> {
    pub fn new(transport: T) -> Self {
        Self {
            transport,
            state: State::Fresh,
            report: Report::default(),
        }
    }
    pub fn report(&self) -> &Report {
        &self.report
    }
    /// Inspect kept transport diagnostics without issuing another command.
    pub fn transport(&self) -> &T {
        &self.transport
    }
    /// Return the still-owned transport to its controller for continuation or
    /// acknowledged teardown. This never clears its poisoned epoch, releases
    /// DMA, resets hardware or changes the historical download result.
    pub fn into_transport(self) -> T {
        self.transport
    }

    /// The whole download, a step per command or scatter transfer.
    pub async fn download(&mut self, prepared: &Prepared<'_>) -> Result<&Report, Error> {
        if self.state == State::Fresh
            && let Err(failure) = self.transport.validate_firmware(prepared)
        {
            return Err(self.fail(Stage::FirmwareTarget, failure).await);
        }
        self.download_parts(prepared.requests.chip, &prepared.patch, &prepared.ram)
            .await
    }

    fn command(&mut self, command: u8, payload: &[u8]) -> Result<u32, TransportFailure> {
        let result = self.transport.command(command, payload)?;
        self.report.commands_completed += 1;
        Ok(result)
    }

    /// One command whose status must be 0, ending its step.
    async fn acknowledged(&mut self, command: u8, payload: &[u8]) -> Result<(), TransportFailure> {
        let status = self.command(command, payload);
        yield_now().await;
        match status? {
            0 => Ok(()),
            status => Err(TransportFailure {
                kind: FailureKind::Device(status),
                port: PortState::Usable,
            }),
        }
    }

    async fn fail(&mut self, stage: Stage, failure: TransportFailure) -> Error {
        self.state = State::Poisoned;
        if self.report.release == ReleaseState::Held {
            self.report.release = if failure.port == PortState::Usable {
                let released = self.command(COMMAND_PATCH_SEMAPHORE, &0u32.to_le_bytes());
                yield_now().await;
                match released {
                    Ok(PATCH_SEMAPHORE_RELEASED) => ReleaseState::Released,
                    Ok(status) => ReleaseState::Failed(TransportFailure {
                        kind: FailureKind::Device(status),
                        port: PortState::Usable,
                    }),
                    Err(error) => ReleaseState::Failed(error),
                }
            } else {
                ReleaseState::UnsafeToAttempt
            };
        }
        Error {
            stage,
            failure,
            report: self.report.clone(),
        }
    }

    async fn region(
        &mut self,
        chip: Chip,
        stage: Stage,
        address: u32,
        mode: u32,
        payload: &[u8],
    ) -> Result<(), Error> {
        let command = if chip.uses_patch_start(address) {
            COMMAND_PATCH_START
        } else {
            COMMAND_DOWNLOAD
        };
        let mut request = [0u8; 12];
        request[..4].copy_from_slice(&address.to_le_bytes());
        request[4..8].copy_from_slice(&(payload.len() as u32).to_le_bytes());
        request[8..].copy_from_slice(&mode.to_le_bytes());
        if let Err(failure) = self.acknowledged(command, &request).await {
            return Err(self.fail(stage, failure).await);
        }
        let data_stage = match stage {
            Stage::PatchStart(i) => Stage::PatchData(i),
            Stage::RamStart(i) => Stage::RamData(i),
            _ => unreachable!(),
        };
        self.report.regions.push(RegionProgress {
            stage: data_stage,
            address,
            mode,
            expected_bytes: payload.len(),
            completed_bytes: 0,
        });
        for chunk in payload.chunks(SCATTER_BYTES) {
            let scattered = self.transport.scatter(chunk);
            yield_now().await;
            if let Err(failure) = scattered {
                return Err(self.fail(data_stage, failure).await);
            }
            self.report.scatter_transfers_completed += 1;
            self.report.regions.last_mut().unwrap().completed_bytes += chunk.len();
        }
        Ok(())
    }

    async fn download_parts(
        &mut self,
        chip: Chip,
        patch: &Patch<'_>,
        ram: &Ram<'_>,
    ) -> Result<&Report, Error> {
        if self.state != State::Fresh {
            return Err(Error {
                stage: Stage::AlreadyAttempted,
                failure: TransportFailure {
                    kind: FailureKind::AlreadyAttempted,
                    port: if self.state == State::Complete {
                        PortState::Usable
                    } else {
                        PortState::Uncertain
                    },
                },
                report: self.report.clone(),
            });
        }
        // The one-attempt gate closes before the first hardware side effect.
        self.state = State::Poisoned;
        let semaphore = self.command(COMMAND_PATCH_SEMAPHORE, &1u32.to_le_bytes());
        yield_now().await;
        let semaphore = match semaphore {
            Ok(semaphore) => semaphore,
            Err(failure) => return Err(self.fail(Stage::AcquirePatch, failure).await),
        };
        match semaphore {
            PATCH_ALREADY_APPLIED => self.report.patch = PatchState::AlreadyApplied,
            PATCH_SEMAPHORE_ACQUIRED => {
                self.report.release = ReleaseState::Held;
                for (i, region) in patch.regions().iter().enumerate() {
                    self.region(
                        chip,
                        Stage::PatchStart(i),
                        region.address,
                        region.mode,
                        region.payload,
                    )
                    .await?;
                }
                if let Err(failure) = self.acknowledged(COMMAND_PATCH_FINISH, &[0; 4]).await {
                    return Err(self.fail(Stage::PatchFinish, failure).await);
                }
                self.report.patch = PatchState::Uploaded;
                let released = self.command(COMMAND_PATCH_SEMAPHORE, &0u32.to_le_bytes());
                yield_now().await;
                match released {
                    Ok(PATCH_SEMAPHORE_RELEASED) => self.report.release = ReleaseState::Released,
                    Ok(status) => {
                        let failure = TransportFailure {
                            kind: FailureKind::Device(status),
                            port: PortState::Usable,
                        };
                        self.report.release = ReleaseState::Failed(failure);
                        return Err(self.fail(Stage::ReleasePatch, failure).await);
                    }
                    Err(failure) => {
                        self.report.release = ReleaseState::Failed(failure);
                        return Err(self.fail(Stage::ReleasePatch, failure).await);
                    }
                }
            }
            status => {
                return Err(self
                    .fail(
                        Stage::AcquirePatch,
                        TransportFailure {
                            kind: FailureKind::Device(status),
                            port: PortState::Usable,
                        },
                    )
                    .await);
            }
        }
        let mut start = 0u32;
        for (i, region) in ram.regions().iter().enumerate() {
            if region.non_download() {
                self.report.skipped_ram_regions.push(i);
                continue;
            }
            if region.overrides_start() {
                start = region.address;
            }
            self.region(
                chip,
                Stage::RamStart(i),
                region.address,
                region.mode(),
                region.payload,
            )
            .await?;
        }
        let mut request = [0u8; 8];
        request[..4].copy_from_slice(&u32::from(start != 0).to_le_bytes());
        request[4..].copy_from_slice(&start.to_le_bytes());
        if let Err(failure) = self.acknowledged(COMMAND_START_FIRMWARE, &request).await {
            return Err(self.fail(Stage::FirmwareStart, failure).await);
        }
        self.report.start_address = Some(start);
        self.report.firmware_start_acknowledged = true;
        self.state = State::Complete;
        Ok(&self.report)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::rc::Rc;
    use core::cell::RefCell;
    use stela_driver_base::stepped::block_on;
    #[derive(Default)]
    struct Model {
        operations: Vec<(u8, Vec<u8>)>,
        fail_at: Option<usize>,
        failure: Option<TransportFailure>,
        already: bool,
        release_status: Option<u32>,
    }
    impl Transport for Rc<RefCell<Model>> {
        fn validate_firmware(&self, _prepared: &Prepared<'_>) -> Result<(), TransportFailure> {
            // This synthetic command oracle has no claimed physical device;
            // its exact-target Provider binding is tested independently.
            Ok(())
        }
        fn command(&mut self, command: u8, payload: &[u8]) -> Result<u32, TransportFailure> {
            let mut m = self.borrow_mut();
            m.operations.push((command, payload.to_vec()));
            if m.fail_at == Some(m.operations.len()) {
                return Err(m.failure.unwrap());
            }
            Ok(if command == COMMAND_PATCH_SEMAPHORE {
                if payload[0] == 1 {
                    if m.already { 1 } else { 2 }
                } else {
                    m.release_status.unwrap_or(3)
                }
            } else {
                0
            })
        }
        fn scatter(&mut self, payload: &[u8]) -> Result<(), TransportFailure> {
            self.command(0xee, payload).map(|_| ())
        }
    }
    fn images() -> (Vec<u8>, Vec<u8>) {
        let mut patch = vec![0u8; 160 + 8193];
        patch[44..48].copy_from_slice(&1u32.to_be_bytes());
        for (at, value) in [
            (96, 2u32),
            (100, 160),
            (104, 8193),
            (108, 0x900000),
            (112, 8193),
        ] {
            patch[at..at + 4].copy_from_slice(&value.to_be_bytes());
        }
        patch[160..].fill(0x57);
        let mut ram = vec![0u8; 4097 + 7 + 80 + 36];
        ram[..4097].fill(0x61);
        ram[4097..4104].fill(0x72);
        for (at, address, length, features, kind) in
            [(4104, 0x910000u32, 4097u32, 0x21, 0), (4144, 0, 7, 0x40, 2)]
        {
            ram[at + 16..at + 20].copy_from_slice(&address.to_le_bytes());
            ram[at + 20..at + 24].copy_from_slice(&length.to_le_bytes());
            ram[at + 24] = features;
            ram[at + 25] = kind;
        }
        let trailer = ram.len() - 36;
        ram[trailer + 2] = 2;
        ram[trailer + 3] = 2;
        (patch, ram)
    }
    #[test]
    fn exact_semaphore_download_scatter_skip_and_start_order_is_kept() {
        let (p, r) = images();
        let (p, r) = (Patch::parse(&p).unwrap(), Ram::parse(&r).unwrap());
        let model = Rc::new(RefCell::new(Model::default()));
        let mut engine = Downloader::new(model.clone());
        let report = block_on(engine.download_parts(Chip::Mt7922, &p, &r)).unwrap();
        assert_eq!(report.patch, PatchState::Uploaded);
        assert_eq!(report.release, ReleaseState::Released);
        assert_eq!(report.scatter_transfers_completed, 5);
        assert_eq!(report.skipped_ram_regions, [1]);
        assert!(report.firmware_start_acknowledged);
        let m = model.borrow();
        assert_eq!(
            m.operations.iter().map(|v| v.0).collect::<Vec<_>>(),
            [0x10, 5, 0xee, 0xee, 0xee, 7, 0x10, 1, 0xee, 0xee, 2]
        );
        assert_eq!(m.operations[2].1.len(), 4096);
        assert_eq!(m.operations[4].1.len(), 1);
        assert_eq!(m.operations[10].1, [1, 0, 0, 0, 0, 0, 0x91, 0]);
        drop(m);
        let count = model.borrow().operations.len();
        assert_eq!(
            block_on(engine.download_parts(Chip::Mt7922, &p, &r))
                .unwrap_err()
                .stage,
            Stage::AlreadyAttempted
        );
        assert_eq!(model.borrow().operations.len(), count);
    }
    #[test]
    fn every_uncertain_failure_closes_the_port_without_unsafe_cleanup_or_retry() {
        let (p, r) = images();
        let (p, r) = (Patch::parse(&p).unwrap(), Ram::parse(&r).unwrap());
        for fail_at in 1..=11 {
            let model = Rc::new(RefCell::new(Model {
                fail_at: Some(fail_at),
                failure: Some(TransportFailure {
                    kind: FailureKind::Timeout,
                    port: PortState::Uncertain,
                }),
                ..Model::default()
            }));
            let mut engine = Downloader::new(model.clone());
            let error = block_on(engine.download_parts(Chip::Mt7922, &p, &r)).unwrap_err();
            assert_eq!(error.failure.kind, FailureKind::Timeout);
            assert_eq!(model.borrow().operations.len(), fail_at);
            if (2..=6).contains(&fail_at) {
                assert_eq!(error.report.release, ReleaseState::UnsafeToAttempt);
            }
            assert_eq!(
                block_on(engine.download_parts(Chip::Mt7922, &p, &r))
                    .unwrap_err()
                    .stage,
                Stage::AlreadyAttempted
            );
            assert_eq!(model.borrow().operations.len(), fail_at);
        }
    }
    #[test]
    fn usable_failure_releases_owned_semaphore_but_never_retries_data() {
        let (p, r) = images();
        let (p, r) = (Patch::parse(&p).unwrap(), Ram::parse(&r).unwrap());
        for release_status in [None, Some(0)] {
            let model = Rc::new(RefCell::new(Model {
                fail_at: Some(3),
                failure: Some(TransportFailure {
                    kind: FailureKind::Device(7),
                    port: PortState::Usable,
                }),
                release_status,
                ..Model::default()
            }));
            let mut engine = Downloader::new(model.clone());
            let error = block_on(engine.download_parts(Chip::Mt7921, &p, &r)).unwrap_err();
            assert_eq!(error.stage, Stage::PatchData(0));
            assert_eq!(error.failure.kind, FailureKind::Device(7));
            assert_eq!(model.borrow().operations.len(), 4);
            assert_eq!(
                model.borrow().operations.last().unwrap(),
                &(0x10, vec![0, 0, 0, 0])
            );
            assert_eq!(
                error.report.release,
                release_status.map_or(ReleaseState::Released, |v| ReleaseState::Failed(
                    TransportFailure {
                        kind: FailureKind::Device(v),
                        port: PortState::Usable
                    }
                ))
            );
        }
    }
    #[test]
    fn already_applied_patch_is_not_reuploaded_or_released() {
        let (p, r) = images();
        let (p, r) = (Patch::parse(&p).unwrap(), Ram::parse(&r).unwrap());
        let model = Rc::new(RefCell::new(Model {
            already: true,
            ..Model::default()
        }));
        let mut engine = Downloader::new(model.clone());
        let report = block_on(engine.download_parts(Chip::Mt7925, &p, &r)).unwrap();
        assert_eq!(report.patch, PatchState::AlreadyApplied);
        assert_eq!(report.release, ReleaseState::NotHeld);
        assert_eq!(
            model
                .borrow()
                .operations
                .iter()
                .filter(|v| v.0 == 0x10)
                .count(),
            1
        );
        assert_eq!(model.borrow().operations.len(), 5);
    }
    #[test]
    #[ignore = "oracle: STELA_MT792X_FIRMWARE, STELA_MT792X_ORACLE"]
    fn pinned_download_matches_every_unchanged_donor_command_byte() {
        extern crate std;
        use super::super::firmware::tests::{catalog, requests};
        use sha2::{Digest, Sha256};
        let images = std::path::PathBuf::from(std::env::var("STELA_MT792X_FIRMWARE").unwrap());
        let oracle = std::path::PathBuf::from(std::env::var("STELA_MT792X_ORACLE").unwrap());
        for (chip, number, count, hash) in [
            (
                Chip::Mt7921,
                7921,
                205,
                "3d6d7aaa078980259c08e04467ac500ca57735887ffd2a66d60d3da1f28bb458",
            ),
            (
                Chip::Mt7922,
                7922,
                230,
                "a7796d0533de77419ee5925ad4c0060f3dbbd58ae02086ac106af9f56c8015aa",
            ),
            (
                Chip::Mt7925,
                7925,
                324,
                "1903493fa3e5b5fb058924b0529f35b5cd646e036486275cde074fade4d78146",
            ),
        ] {
            let expected = std::fs::read(oracle.join(alloc::format!("{number}.trace"))).unwrap();
            assert_eq!(alloc::format!("{:x}", Sha256::digest(&expected)), hash);
            let requests = requests(chip);
            let provider = catalog(
                &requests,
                std::fs::read(images.join(chip.patch_name())).unwrap(),
                std::fs::read(images.join(chip.ram_name())).unwrap(),
            );
            let prepared = requests.load(&provider).unwrap();
            let model = Rc::new(RefCell::new(Model::default()));
            let mut engine = Downloader::new(model.clone());
            let report = block_on(engine.download(&prepared)).unwrap();
            assert!(report.firmware_start_acknowledged);
            assert_eq!(report.release, ReleaseState::Released);
            let operations = &model.borrow().operations;
            assert_eq!(operations.len(), count);
            let mut trace = Vec::new();
            for (command, payload) in operations {
                trace.extend_from_slice(&[*command, u8::from(*command != 0xee)]);
                trace.extend_from_slice(&(payload.len() as u32).to_le_bytes());
                trace.extend_from_slice(payload);
            }
            assert_eq!(trace, expected, "chip {number}");
            // Regulatory regions remain borrowed and accessible after upload.
            assert!(
                prepared
                    .ram()
                    .regions()
                    .iter()
                    .any(|r| r.non_download() && r.kind == 2)
            );
            assert!(!prepared.ram().release_information.is_empty());
        }
    }
}
