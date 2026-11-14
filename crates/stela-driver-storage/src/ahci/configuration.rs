// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright 2004-2005 Red Hat, Inc.
//! Port start and device (re)configuration: link reset, IDENTIFY revalidation,
//! log pages, link power and transfer modes.

use super::*;

impl AhciDisk {
    /// Reset the link on one port and, when an ATA disk answers, start the
    /// port and identify the disk.
    #[cfg(test)]
    pub(super) fn start(
        port: Mmio,
        capabilities: u32,
        capabilities_2: u32,
        clock: Clock,
        dma: &mut dyn DmaAllocator,
        minimum: Rc<Control>,
    ) -> Result<Option<Self>, PortRefusal> {
        let link = Self::link_ports(&[&port], capabilities, clock, &minimum).remove(0);
        match link {
            Ok(Some(link)) => Self::configure_port(
                port,
                link,
                capabilities,
                capabilities_2,
                clock,
                dma,
                minimum,
            ),
            Ok(None) => Ok(None),
            Err(error) => Err(PortRefusal::unstarted(error, minimum, port)),
        }
    }

    /// Reset every port's link at once, as Linux probes ports in parallel:
    /// stop each port and hold COMRESET, release them all, settle once, then
    /// debounce them together with `sata_deb_timing_normal`. A port whose
    /// SStatus.DET does not end at 3 has no device (`ata_phys_link_offline`):
    /// an empty port costs one debounce, 100 ms of stable DET 0, beside the
    /// others. A timing failure reaches every port.
    pub(super) fn link_ports<P: CommandPort>(
        ports: &[&P],
        capabilities: u32,
        clock: Clock,
        minimum: &Control,
    ) -> Vec<Result<Option<Link>, BlockError>> {
        let mut links: Vec<Result<Link, BlockError>> = ports
            .iter()
            .map(|port| Self::reset_link(*port, capabilities, clock, minimum))
            .collect();
        let released = minimum.wait_after(RESUME_SETTLE_US, || {
            for (port, link) in ports.iter().zip(&links) {
                if let Ok(link) = link {
                    port.write32(PORT_SATA_CONTROL, link.control);
                    port.read32(PORT_SATA_CONTROL);
                }
            }
        });
        let reset: Vec<&P> = ports
            .iter()
            .zip(&links)
            .filter(|(_, link)| link.is_ok())
            .map(|(port, _)| *port)
            .collect();
        let debounced = released.and_then(|()| {
            debounce_ports(&reset, clock, minimum, DEBOUNCE_NORMAL, DEBOUNCE_NORMAL[2])
        });
        let mut debounced = match debounced {
            Ok(debounced) => debounced.into_iter(),
            Err(error) => {
                for link in &mut links {
                    *link = Err(error.clone());
                }
                Vec::new().into_iter()
            }
        };
        links
            .into_iter()
            .map(|link| {
                let link = link?;
                let present = debounced
                    .next()
                    .is_some_and(|found| found.detection == DEVICE_PRESENT);
                Ok(present.then_some(link))
            })
            .collect()
    }

    /// Stop `port` and assert COMRESET for its minimum hold.
    fn reset_link(
        port: &impl CommandPort,
        capabilities: u32,
        clock: Clock,
        minimum: &Control,
    ) -> Result<Link, BlockError> {
        minimum.check()?;
        let command = port.read32(PORT_COMMAND);
        stop_port(port, clock)?;
        if capabilities & CAP_STAGGERED_SPIN_UP != 0 {
            port.write32(PORT_COMMAND, port.read32(PORT_COMMAND) | COMMAND_SPIN_UP);
        }
        // sata_link_init_spd: the firmware's SPD caps the limit.
        let initial = (port.read32(PORT_SATA_CONTROL) & SATA_CONTROL_SPEED) >> SPEED_SHIFT;
        let speed_limit = match initial {
            0 => u32::MAX,
            speed => (1 << speed) - 1,
        };
        let control = port.read32(PORT_SATA_CONTROL) & !SATA_CONTROL_DETECTION;
        minimum.wait_after(COMRESET_HOLD_US, || {
            port.write32(PORT_SATA_CONTROL, control | SATA_CONTROL_RESET);
            port.read32(PORT_SATA_CONTROL);
        })?;
        Ok(Link {
            command,
            speed_limit,
            control,
        })
    }

    /// Start a port whose link came up and identify its disk.
    pub(super) fn configure_port(
        port: Mmio,
        link: Link,
        capabilities: u32,
        capabilities_2: u32,
        clock: Clock,
        dma: &mut dyn DmaAllocator,
        minimum: Rc<Control>,
    ) -> Result<Option<Self>, PortRefusal> {
        let mut kept = crate::Retained::new(alloc::boxed::Box::new(minimum::PortBuild {
            owner: minimum,
            port,
            commands: None,
            table: None,
            bounce: None,
            disk: None,
        }));
        let result = (|| {
            let port = kept.port;
            let minimum = kept.owner.clone();
            minimum.ready()?;
            let lpm = Lpm::new(capabilities, capabilities_2, link.command);
            let speed_limit = link.speed_limit;
            port.write32(PORT_SATA_ERROR, u32::MAX);
            kept.get_mut().commands =
                Some(dma.allocate(COMMAND_LIST_BYTES + RECEIVED_FIS_BYTES, COMMAND_LIST_BYTES)?);
            kept.get_mut().table = Some(dma.allocate(COMMAND_TABLE_BYTES, COMMAND_TABLE_ALIGN)?);
            kept.get_mut().bounce = Some(dma.allocate(BOUNCE_BYTES, 4096)?);
            let commands = kept.commands.as_ref().expect("same command allocation");
            let table = kept.table.as_ref().expect("same table allocation");
            let bounce = kept.bounce.as_ref().expect("same bounce allocation");
            if capabilities & CAP_64BIT == 0
                && [&commands, &table, &bounce]
                    .iter()
                    .any(|buffer| buffer.bus_address() + buffer.len() as u64 > 1 << 32)
            {
                return Err(BlockError::Unsupported("32-bit AHCI with DMA above 4 GiB"));
            }
            port.write64(PORT_LIST, commands.bus_address());
            port.write64(PORT_FIS, commands.bus_address() + COMMAND_LIST_BYTES as u64);
            port.write32(
                PORT_COMMAND,
                port.read32(PORT_COMMAND) | COMMAND_FIS_RECEIVE,
            );
            if !clock.wait(SPIN_UP_TIMEOUT_US, || {
                port.read32(PORT_TASK_FILE) & TASK_FILE_BUSY == 0
            }) {
                return Err(BlockError::Timeout);
            }
            let unsupported = match port.read32(PORT_SIGNATURE) {
                SIGNATURE_ATA => None,
                SIGNATURE_ATAPI => Some("ATAPI device"),
                SIGNATURE_PORT_MULTIPLIER => Some("SATA port multiplier"),
                SIGNATURE_ENCLOSURE => Some("SATA enclosure management bridge"),
                _ => Some("unknown SATA device signature"),
            };
            if let Some(device) = unsupported {
                stop_port(&port, clock)?;
                return Err(BlockError::Unsupported(device));
            }
            port.write32(PORT_INTERRUPTS, u32::MAX);
            port.write32(PORT_COMMAND, port.read32(PORT_COMMAND) | COMMAND_START);
            let speed = (port.read32(PORT_SATA_STATUS) >> SPEED_SHIFT) & 0xf;
            let retain = Rc::new(Cell::new(false));
            kept.get_mut().disk = Some(Self {
                port,
                commands: DmaResource::new(
                    kept.get_mut().commands.take().expect("same commands"),
                    minimum.clone(),
                    retain.clone(),
                ),
                table: DmaResource::new(
                    kept.get_mut().table.take().expect("same table"),
                    minimum.clone(),
                    retain.clone(),
                ),
                bounce: DmaResource::new(
                    kept.get_mut().bounce.take().expect("same bounce"),
                    minimum.clone(),
                    retain.clone(),
                ),
                minimum: minimum.clone(),
                retain: retain.clone(),
                identity: Identity {
                    transport: Transport::Ahci,
                    location: alloc::string::String::new(),
                    model: alloc::string::String::new(),
                    serial: alloc::string::String::new(),
                    blocks: 0,
                    block_size: ATA_SECTOR_BYTES,
                    read_only: false,
                    features: Features::default(),
                },
                clock,
                command_timeout_us: COMMAND_TIMEOUT_US,
                fault: None,
                flush: None,
                errors: ErrorRing::default(),
                // ata_eh_set_mode configured the disk: the transfer is unverified.
                dubious: true,
                speed_limit,
                speed,
                sense_reporting: false,
                transfer: Transfer::default(),
                pio_mask: u32::MAX,
                udma_mask: u32::MAX,
                mode: 0,
                step_downs: 0,
                locked: false,
                lpm,
                ncq: Ncq {
                    host: capabilities & CAP_NCQ != 0,
                    host_slots: (capabilities >> CAP_SLOTS_SHIFT & CAP_SLOTS) as usize + 1,
                    ..Ncq::default()
                },
                log_dma: false,
                no_dma_log: false,
                quirks: 0,
                devslp: false,
                slots: DmaResource::new(Vec::new(), minimum.clone(), retain.clone()),
                pool: DmaResource::new(Pool::default(), minimum.clone(), retain.clone()),
                running: Vec::new(),
                next_tag: 0,
                looks: 0,
                signal: None,
            });
            let disk = kept.get_mut().disk.as_mut().expect("same initialized disk");
            let port = disk.port;
            let buffer = disk.bounce.bus_address();
            disk.internal(&port, ATA_IDENTIFY, 0, 0, ATA_SECTOR_BYTES as usize, buffer)
                .map_err(|failure| failure.error)?;
            let found = identify(disk.bounce.bytes())?;
            if found.quirks & quirks::DISABLE != 0 {
                stop_port(&port, clock)?;
                return Err(BlockError::Unsupported("SATA device libata disables"));
            }
            disk.sense_reporting = found.sense_reporting.1;
            disk.flush = found.flush;
            disk.identity.serial = found.serial.clone();
            disk.identity.model = found.model.clone();
            disk.identity.blocks = found.blocks;
            disk.identity.block_size = found.block_size;
            // libata's `ata_dev_config_fua` uses FUA only with NCQ and its
            // quirk lists; this driver leaves it off: a durable write is
            // followed by a flush, which the write cache's state decides.
            disk.identity.features = Features {
                write_cache: found.flush.is_some(),
                fua: false,
                max_transfer: BOUNCE_BYTES,
                max_read_transfer: None,
                discard: found
                    .trim
                    .then_some(DiscardLimits::single(TRIM_RANGES * TRIM_RANGE_SECTORS)),
                write_zeroes: None,
                rotation: found.rotation,
            };
            disk.transfer = found.transfer;
            disk.locked = found.locked;
            (disk.lpm.hipm, disk.lpm.dipm) = (found.hipm, found.dipm);
            // ata_dev_configure, then ATA_EHI_SETMODE and config_lpm; a failure
            // takes EH's next try.
            let mut configured = disk.configure(&port, &found);
            if configured.is_ok() {
                disk.set_mode(&port)?;
                configured = disk.configure_lpm(&port, DEVICE_TRIES);
            }
            if let Err(error) = configured {
                let tries = if error == AGAIN {
                    DEVICE_TRIES
                } else {
                    DEVICE_TRIES - 1
                };
                disk.minimum.ready()?;
                disk.reconfigure(&port, true, false, error, tries)?;
            }
            // Linux `__ata_change_queue_depth`: as many queued commands as the
            // host's slots and the disk take, at most ATA_MAX_QUEUE. Memory
            // the DMA pool cannot give, or a 32-bit host cannot reach, leaves
            // the queue shallower: slots 1 and up need the pool pages of at
            // least one command.
            if disk.ncq.device {
                let depth = DEPTH.min(disk.ncq.host_slots).min(disk.ncq.depth);
                let reachable = |buffer: &DmaBuffer| {
                    capabilities & CAP_64BIT != 0
                        || buffer.bus_address() + buffer.len() as u64 <= 1 << 32
                };
                let pages = POOL_PAGES.min(CHUNK_PAGES * depth.saturating_sub(1));
                while disk.pool.pages.len() < pages {
                    match dma.allocate(PAGE, PAGE) {
                        Ok(page) if reachable(&page) => disk.pool.pages.push(page),
                        _ => break,
                    }
                }
                if disk.pool.pages.len() >= CHUNK_PAGES {
                    disk.pool.free = (0..disk.pool.pages.len() as u16).rev().collect();
                    for _ in 1..depth {
                        match dma.allocate(COMMAND_TABLE_BYTES, COMMAND_TABLE_ALIGN) {
                            Ok(table) if reachable(&table) => disk.slots.push(Slot {
                                table,
                                pages: Vec::new(),
                            }),
                            _ => break,
                        }
                    }
                }
            }
            Ok(())
        })();
        match result {
            Ok(()) => {
                let minimum::PortBuild { disk, .. } = *kept.into_inner();
                Ok(Some(disk.expect("same configured disk")))
            }
            Err(error) => Err(PortRefusal::new(error, kept)),
        }
    }
    /// Linux `ata_dev_revalidate`: IDENTIFY again and require the same model,
    /// serial and capacity (`ata_dev_same_device`).
    pub(super) fn revalidate(&mut self, port: &impl CommandPort) -> Result<(), BlockError> {
        let buffer = self.table.bus_address() + IDENTIFY_DATA as u64;
        self.internal(port, ATA_IDENTIFY, 0, 0, ATA_SECTOR_BYTES as usize, buffer)
            .map_err(|failure| failure.error)?;
        let data = &self.table.bytes()[IDENTIFY_DATA..IDENTIFY_DATA + ATA_SECTOR_BYTES as usize];
        let found = identify(data).map_err(|_| DETACHED)?;
        if found.model != self.identity.model
            || found.serial != self.identity.serial
            || found.blocks != self.identity.blocks
        {
            return Err(DETACHED);
        }
        // ata_dev_reread_id keeps the new data.
        self.sense_reporting = found.sense_reporting.1;
        self.transfer = found.transfer;
        self.locked = found.locked;
        (self.lpm.hipm, self.lpm.dipm) = (found.hipm, found.dipm);
        self.configure(port, &found)
    }

    /// What Linux `ata_dev_configure`, which `ata_dev_revalidate` reruns,
    /// asks of the disk. The quirk table's entry accumulates; a 1.5 Gbps
    /// limit on a faster link asks for another reset. `ata_dev_config_ncq`
    /// enables first-party DMA auto-activation; a failure other than the
    /// device's refusal marks it broken and fails the configuration.
    /// `ata_dev_config_lpm` forces max_power for broken LPM and turns DIPM
    /// off on a host without it, as Software Settings Preservation may have
    /// kept it on. `ata_dev_config_devslp` reads the SATA Settings page for
    /// DevSleep. Then Sense Data Reporting, which
    /// a hard reset turns off again (`ata_dev_config_sense_reporting` only
    /// logs a refusal). The kept IDENTIFY data says it is enabled only once
    /// a revalidation rereads it.
    pub(super) fn configure(
        &mut self,
        port: &impl CommandPort,
        found: &Identified,
    ) -> Result<(), BlockError> {
        let buffer = self.bounce.bus_address();
        self.quirks |= found.quirks;
        // ata_do_link_spd_quirk.
        if self.quirks & quirks::LINK_1_5_GBPS != 0 && self.speed_limit > 0b1 {
            self.speed_limit = 0b1;
            if self.speed > 1 {
                return Err(AGAIN);
            }
        }
        let ncq = found.ncq && self.quirks & quirks::NONCQ == 0;
        self.ncq.device = self.ncq.host && ncq;
        self.ncq.depth = found.queue_depth;
        self.ncq.autosense = found.ncq_autosense;
        self.ncq.aa_broken |= self.quirks & quirks::BROKEN_FPDMA_AA != 0;
        self.log_dma = found.read_log_dma;
        self.no_dma_log |= self.quirks & quirks::NO_DMA_LOG != 0;
        if ncq && self.ncq.host && found.fpdma_aa && !self.ncq.aa_broken {
            let enabled = self.internal(
                port,
                ATA_SET_FEATURES,
                FEATURE_SATA_ENABLE,
                SATA_FPDMA_AA,
                0,
                buffer,
            );
            if let Err(failure) = enabled
                && !failure.refused()
            {
                self.ncq.aa_broken = true;
                return Err(failure.error);
            }
        }
        // WD_BROKEN_LPM covers 1.5 Gbps-only disks; NO_LPM_ON_ATI needs an
        // ATI host, which an AMD one is not.
        if self.quirks & quirks::NOLPM != 0
            || self.quirks & quirks::WD_BROKEN_LPM != 0 && found.sata_speeds == 0x2
        {
            self.lpm.broken = true;
        }
        if !self.lpm.host_dipm() && found.dipm {
            let _ = self.internal(
                port,
                ATA_SET_FEATURES,
                FEATURE_SATA_DISABLE,
                SATA_DIPM,
                0,
                buffer,
            );
        }
        self.devslp = found.devslp
            && self.identify_page_supported(port, LOG_SATA_SETTINGS)?
            && self.read_log(port, LOG_IDENTIFY_DEVICE, LOG_SATA_SETTINGS)?;
        if found.sense_reporting == (true, false) {
            let _ = self.internal(port, ATA_SET_FEATURES, FEATURE_SENSE_DATA, 1, 0, buffer);
        }
        Ok(())
    }

    /// Linux `ata_read_log_page` for one page into the table's log area:
    /// READ LOG DMA EXT where IDENTIFY offers it, and once that fails READ
    /// LOG EXT for good (`ATA_QUIRK_NO_DMA_LOG`), at once unless the port
    /// froze. Whether the page was read.
    pub(super) fn read_log(
        &mut self,
        port: &impl CommandPort,
        log: u8,
        page: u8,
    ) -> Result<bool, BlockError> {
        let buffer = self.table.bus_address() + LOG_DATA as u64;
        let lba = u64::from(log) | u64::from(page) << 8;
        loop {
            let dma = self.log_dma && !self.no_dma_log;
            let command = if dma {
                ATA_READ_LOG_DMA_EXT
            } else {
                ATA_READ_LOG_EXT
            };
            let bytes = ATA_SECTOR_BYTES as usize;
            let Err(failure) = self.attempt(
                port,
                AtaRequest {
                    command,
                    feature: 0,
                    lba,
                    count: 1,
                    bytes,
                    write: false,
                    buffer: Some(buffer),
                },
            ) else {
                return Ok(true);
            };
            self.minimum.ready()?;
            stop_engine(port, self.clock)?;
            start_engine(port);
            if !dma {
                return Ok(false);
            }
            self.no_dma_log = true;
            if failure.frozen {
                return Ok(false);
            }
        }
    }

    /// Linux `ata_identify_page_supported` with `ata_log_supported`: the
    /// general purpose log directory, reread at each configuration, lists
    /// the IDENTIFY DEVICE data log, whose page 0 lists `page`. Without that
    /// log the disk gets `ATA_QUIRK_NO_ID_DEV_LOG`.
    pub(super) fn identify_page_supported(
        &mut self,
        port: &impl CommandPort,
        page: u8,
    ) -> Result<bool, BlockError> {
        if self.quirks & quirks::NO_ID_DEV_LOG != 0 {
            return Ok(false);
        }
        let entry = LOG_DATA + 2 * usize::from(LOG_IDENTIFY_DEVICE);
        let listed = self.quirks & quirks::NO_LOG_DIR == 0
            && self.read_log(port, LOG_DIRECTORY, 0)?
            && self.table.bytes()[entry..entry + 2] != [0, 0];
        if !listed {
            self.quirks |= quirks::NO_ID_DEV_LOG;
            return Ok(false);
        }
        if !self.read_log(port, LOG_IDENTIFY_DEVICE, 0)? {
            return Ok(false);
        }
        let pages = &self.table.bytes()[LOG_DATA..LOG_DATA + ATA_SECTOR_BYTES as usize];
        Ok(pages[9..9 + usize::from(pages[8])].contains(&page))
    }

    /// `ata_eh_recover`'s config_lpm: move to the target policy. A failure
    /// with at most two tries left disables LPM on the link, as
    /// `ata_eh_link_set_lpm` does.
    pub(super) fn configure_lpm(
        &mut self,
        port: &impl CommandPort,
        tries: usize,
    ) -> Result<(), BlockError> {
        let target = self.lpm.target();
        if self.lpm.policy == target {
            return Ok(());
        }
        self.set_lpm(port, target, tries)
    }

    /// Linux `ata_eh_link_set_lpm` with `ahci_set_lpm`: DIPM off, the
    /// SControl IPM restrictions and PxCMD aggressive LPM the policy and HIPM
    /// allow, then DIPM on from MED_POWER_WITH_DIPM. A device's refusal is
    /// ignored; any other failure keeps the old policy.
    pub(super) fn set_lpm(
        &mut self,
        port: &impl CommandPort,
        policy: u8,
        tries: usize,
    ) -> Result<(), BlockError> {
        if !self.lpm.allowed {
            return Ok(());
        }
        let result = self.apply_lpm(port, policy);
        if result.is_err() && tries <= 2 {
            self.lpm.allowed = false;
        }
        result
    }

    pub(super) fn apply_lpm(
        &mut self,
        port: &impl CommandPort,
        policy: u8,
    ) -> Result<(), BlockError> {
        self.minimum.check()?;
        let lpm = self.lpm;
        let buffer = self.bounce.bus_address();
        let dipm = |disk: &mut Self, feature| match disk.internal(
            port,
            ATA_SET_FEATURES,
            feature,
            SATA_DIPM,
            0,
            buffer,
        ) {
            Err(failure) if !failure.refused() => Err(failure.error),
            _ => Ok(()),
        };
        if lpm.dipm {
            dipm(self, FEATURE_SATA_DISABLE)?;
        }
        // sata_link_scr_lpm. Linux masks the port's PhyRdy interrupt
        // first, as PxIE does here for a port that sends messages; a look
        // drops PRCS by the policy too, since PxIS latches it whatever PxIE
        // masks.
        let scr_lpm = || {
            let mut control = port.read32(PORT_SATA_CONTROL);
            if policy == LPM_MAX_POWER {
                control |= SATA_CONTROL_POWER;
            } else {
                control &= !SATA_CONTROL_POWER;
                if lpm.capabilities & CAP_PARTIAL == 0 {
                    control |= SATA_CONTROL_NO_PARTIAL;
                }
                if lpm.capabilities & CAP_SLUMBER == 0 {
                    control |= SATA_CONTROL_NO_SLUMBER;
                }
                if lpm.capabilities_2 & CAP2_DEVICE_SLEEP == 0 {
                    control |= SATA_CONTROL_NO_DEVICE_SLEEP;
                }
            }
            port.write32(PORT_SATA_CONTROL, control);
            port.write32(PORT_SATA_ERROR, SERROR_PHY_READY_CHANGE);
        };
        // ahci_set_lpm restricts the link first when saving power, and
        // lifts the restrictions last at max_power.
        if policy != LPM_MAX_POWER {
            self.mask_interrupts(port, policy);
            scr_lpm();
        }
        if lpm.capabilities & CAP_AGGRESSIVE_LPM != 0 {
            let mut command = port.read32(PORT_COMMAND);
            if policy == LPM_MAX_POWER || !lpm.hipm {
                command &= !(COMMAND_AGGRESSIVE_SLUMBER | COMMAND_AGGRESSIVE_LPM);
                self.minimum.wait_after(LPM_WAKE_US, || {
                    port.write32(PORT_COMMAND, command | COMMAND_INTERFACE_ACTIVE);
                    port.read32(PORT_COMMAND);
                })?;
            } else {
                port.write32(PORT_COMMAND, command | COMMAND_AGGRESSIVE_LPM);
            }
        }
        // ahci_set_aggressive_devslp: only the minimum-power policies,
        // which are never the target here, enable it; firmware's is undone.
        let sleep = CAP2_DEVICE_SLEEP | CAP2_AGGRESSIVE_DEVICE_SLEEP;
        let devslp = port.read32(PORT_DEVICE_SLEEP);
        if lpm.capabilities_2 & sleep == sleep
            && self.devslp
            && devslp & DEVICE_SLEEP_PRESENT != 0
            && devslp & DEVICE_SLEEP_AGGRESSIVE != 0
        {
            port.write32(PORT_DEVICE_SLEEP, devslp & !DEVICE_SLEEP_AGGRESSIVE);
            let _ = self.internal(
                port,
                ATA_SET_FEATURES,
                FEATURE_SATA_DISABLE,
                SATA_DEVICE_SLEEP,
                0,
                buffer,
            );
        }
        if policy == LPM_MAX_POWER {
            scr_lpm();
            // ahci_set_lpm turns the PhyRdy interrupt back on last.
            self.mask_interrupts(port, policy);
        }
        let old = self.lpm.policy;
        self.lpm.policy = policy;
        if policy >= LPM_MED_POWER_WITH_DIPM
            && lpm.host_dipm()
            && lpm.dipm
            && let Err(error) = dipm(self, FEATURE_SATA_ENABLE)
        {
            self.lpm.policy = old;
            self.mask_interrupts(port, old);
            return Err(error);
        }
        self.lpm.changed = self.clock.try_start().ok();
        Ok(())
    }

    /// Linux `ata_eh_set_mode` with `ata_set_mode` and `ata_dev_set_mode`:
    /// choose the fastest PIO and UDMA modes the host, the IDENTIFY data and
    /// earlier speed-downs allow, program the UDMA mode with SET FEATURES and
    /// revalidate. A changed mode makes the transfer unverified again.
    pub(super) fn set_mode(&mut self, port: &impl CommandPort) -> Result<(), BlockError> {
        // A verified transfer clears DUBIOUS on the newest error.
        if !self.dubious
            && let Some(top) = self.errors.top()
        {
            top.eflags &= !EFLAG_DUBIOUS;
        }
        // ata_dev_xfermask: no MWDMA on AHCI, no cable limit on SATA.
        self.pio_mask &= HOST_PIO_MASK & self.transfer.pio;
        self.udma_mask &= HOST_UDMA_MASK & self.transfer.udma;
        if self.quirks & quirks::NODMA != 0 {
            self.udma_mask = 0;
        }
        let pio = highest_mode(self.pio_mask, MODE_PIO_0)
            .ok_or(BlockError::Unsupported("SATA disk without PIO support"))?;
        // libata_dma_mask leaves CompactFlash in PIO, as without UDMA; these
        // drivers issue only DMA commands.
        let mode = highest_mode(self.udma_mask, MODE_UDMA_0)
            .filter(|_| !self.transfer.compact_flash)
            .ok_or(BlockError::Unsupported(
                "SATA disk that libata would run in PIO mode",
            ))?;
        if mode != self.mode {
            self.dubious = true;
        }
        self.mode = mode;
        // ata_dev_set_xfermode: with IORDY in use the mode is sent;
        // otherwise there is nothing to set.
        // NOSETXFER skips it between a SATA host and a SATA device.
        let mut refused = false;
        let skip = self.quirks & quirks::NOSETXFER != 0 && self.transfer.sata;
        if !skip && (pio > MODE_PIO_2 || self.transfer.iordy) {
            let buffer = self.bounce.bus_address();
            match self.internal(
                port,
                ATA_SET_FEATURES,
                FEATURE_TRANSFER_MODE,
                u16::from(mode),
                0,
                buffer,
            ) {
                Ok(()) => {}
                Err(failure) if failure.refused() => refused = true,
                Err(failure) => return Err(failure.error),
            }
        }
        self.revalidate(port)?;
        // A device error is ignored when the disk already runs its fastest
        // mode.
        if refused && highest_mode(self.transfer.udma, MODE_UDMA_0) != Some(mode) {
            return Err(BlockError::Protocol(
                "the SATA disk refused its transfer mode",
            ));
        }
        Ok(())
    }
}
