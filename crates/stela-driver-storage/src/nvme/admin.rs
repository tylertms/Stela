// SPDX-License-Identifier: GPL-2.0-only
// Copyright (c) 2011-2014, Intel Corporation.
//! The admin queue: enabling the controller and running Identify, queue-
//! creation and log commands.

use super::*;

impl Controller {
    /// Point the controller at the admin queue, empty, with every pin or
    /// MSI interrupt masked. Under MSI-X the mask registers are left alone:
    /// NVMe leaves a host access to INTMS undefined then, and Linux never
    /// writes it; the MSI-X table masks instead.
    pub(super) fn configure_admin(&mut self) -> Result<(), BlockError> {
        self.admin.restart();
        self.admin.completions.give()?;
        self.io.completions.give()?;
        // Empty SQs are CPU-owned until their first command publication. The
        // controller cannot consume any SQE before the corresponding doorbell.
        let entries = u32::from(self.admin.entries - 1);
        self.registers
            .write32(ADMIN_ATTRIBUTES, entries << 16 | entries)?;
        self.registers
            .write64(ADMIN_SUBMISSION, self.admin.submissions.address()?)?;
        self.admin.submissions.publish();
        self.registers
            .write64(ADMIN_COMPLETION, self.admin.completions.address()?)?;
        if self.signalled {
            return Ok(());
        }
        self.registers.write32(INTERRUPT_MASK_SET, u32::MAX)
    }

    /// Linux `nvme_enable_ctrl`: set CC.EN, returning how long CSTS.RDY may
    /// take, CAP.TO or a longer CRTO.CRWMT. CRIME stays clear, as in Linux.
    pub(super) fn enable(&mut self) -> Result<u64, BlockError> {
        let capabilities = self.registers.read64(CAPABILITIES)?;
        let mut timeout = capabilities >> 24 & 0xff;
        if capabilities & CAPABILITY_READY_WITH_MEDIA != 0 {
            let media = self.registers.read32(READY_TIMEOUTS)? & 0xffff;
            timeout = timeout.max(u64::from(media));
        }
        self.independent_identify = capabilities & CAPABILITY_READY_INDEPENDENT != 0;
        self.registers
            .write32(CONFIGURATION, self.configuration | ENABLE)?;
        Ok(ready_wait_us(timeout))
    }

    /// Stop the controller, then start it ([`Self::stop`],
    /// [`Self::start_admin`]).
    pub(super) fn initialize(&mut self) -> Result<(), BlockError> {
        self.stop()?;
        self.start_admin()
    }

    /// Take the controller's capabilities and stop it, as Linux's
    /// `nvme_pci_configure_admin_queue` does before it sets up the admin
    /// queue: CC cleared, CSTS.RDY clear.
    pub(super) fn stop(&mut self) -> Result<(), BlockError> {
        if let Some(owner) = self.owned.as_mut() {
            // An attempted prepare may have effects without establishing D0.
            // Only success authorizes NVMe MMIO; uncertain failure quarantines.
            self.teardown = owned::Teardown::Preparing;
            owner.lifecycle.prepare()?;
            self.teardown = owned::Teardown::Active;
        }
        let capabilities = self.registers.read64(CAPABILITIES)?;
        self.doorbell_stride = 4usize << (capabilities >> 32 & 0xf);
        self.ready_timeout_us = ready_wait_us(capabilities >> 24 & 0xff);
        if capabilities >> 37 & 1 == 0 {
            return Err(BlockError::Unsupported(
                "NVMe controller without the NVM command set",
            ));
        }
        if capabilities >> 48 & 0xf != 0 {
            return Err(BlockError::Unsupported(
                "NVMe controller without 4 KiB pages",
            ));
        }
        // CAP.MQES, zero-based, bounds the I/O queues alone; AQA sizes the
        // admin queue. Linux shortens its I/O queues to it, refusing none
        // (the value 0, one entry, is reserved). `nvme_pci_enable` gives a
        // quirked controller two entries, one command, and the PM1725's
        // MQES 0 64 entries.
        let mqes = capabilities & 0xffff;
        let longest = if self.quirks.contains(Quirks::QUEUE_DEPTH_ONE) {
            2
        } else if mqes == 0 && self.quirks.contains(Quirks::QUEUE_DEPTH_64) {
            64
        } else {
            mqes + 1
        };
        if longest < 2 {
            return Err(BlockError::Unsupported(
                "NVMe controller reports one-entry queues (CAP.MQES 0)",
            ));
        }
        self.io.entries = self
            .io
            .entries
            .min(u16::try_from(longest).unwrap_or(u16::MAX));
        let depth = self.tags.len().min(usize::from(self.io.entries) - 1);
        self.tags.truncate(depth);
        if 4 * self.doorbell_stride + DOORBELLS > 0x2000 {
            return Err(BlockError::Unsupported("NVMe doorbell stride"));
        }
        self.configuration = CONFIGURATION_VALUE & !ENABLE;
        if capabilities & CAPABILITY_COMMAND_SETS != 0 {
            self.configuration |= COMMAND_SETS_ALL;
        }
        self.stop_and_wait().map_err(|error| error.at(DISABLE_WAIT))
    }

    /// Point the stopped controller at the admin queue, give it DMA where
    /// its owner holds it, and enable it.
    pub(super) fn start_admin(&mut self) -> Result<(), BlockError> {
        self.configure_admin()?;
        if let Some(owner) = self.owned.as_mut() {
            owner.lifecycle.enable_dma()?;
        }
        let timeout = self.enable()?;
        self.wait_enabled(timeout)
    }

    /// Linux `nvme_disable_ctrl`: clear CC, then wait for CSTS.RDY to clear,
    /// first sleeping Linux's delay for a quirked controller. Waits: only at
    /// attach and teardown, outside any service turn; a passed device start
    /// budget ends the delay early.
    pub(super) fn stop_and_wait(&mut self) -> Result<(), BlockError> {
        self.registers.write32(CONFIGURATION, 0)?;
        if self.quirks.contains(Quirks::READY_DELAY) {
            self.clock.wait(quirks::READY_CHECK_DELAY_US, || false);
        }
        self.wait_status(self.ready_timeout_us, |status| status & READY == 0)
    }

    /// Linux `nvme_wait_ready` after CC.EN is set: CSTS.RDY alone ends the
    /// wait, and Linux reads nothing more of CSTS before its first command.
    /// CSTS.CFS while the controller starts is waited out; one still set
    /// once it is ready is kept as standing ([`Self::fails`]), so a
    /// controller that answers its commands is used, as under Linux. At the
    /// timeout, a set CFS is the error reported.
    pub(super) fn wait_enabled(&mut self, timeout: u64) -> Result<(), BlockError> {
        match self.wait_status(timeout, |status| status & READY != 0) {
            Err(BlockError::Timeout) => {
                self.check_fatal()
                    .map_err(|error| within(ENABLE_WAIT, fatal_status(error)))?;
                return Err(BlockError::Timeout.at(ENABLE_WAIT));
            }
            other => other?,
        }
        let status = self.registers.read32(STATUS)?;
        if status == REMOVED {
            return Err(removed());
        }
        self.fatal_standing = status & FATAL != 0;
        Ok(())
    }

    /// The admin command creating the I/O completion queue, or with
    /// `submission` the submission queue, polled.
    pub(super) fn create_queue(&self, submission: bool) -> Result<[u32; 16], BlockError> {
        let entries = u32::from(self.io.entries - 1);
        let mut command = [0u32; 16];
        let address = if submission {
            command[0] = u32::from(ADMIN_CREATE_SUBMISSION);
            // Completion queue 1, contiguous; medium priority for a
            // controller that otherwise enables weighted round robin.
            command[11] = 1 << 16 | 1;
            if self.quirks.contains(Quirks::MEDIUM_PRIORITY_QUEUE) {
                command[11] |= SUBMISSION_PRIORITY_MEDIUM;
            }
            self.io.submissions.address()?
        } else {
            command[0] = u32::from(ADMIN_CREATE_COMPLETION);
            // Physically contiguous; with interrupts enabled, vector 0.
            command[11] = 1 | u32::from(self.signalled) << 1;
            self.io.completions.address()?
        };
        command[6] = address as u32;
        command[7] = (address >> 32) as u32;
        command[10] = entries << 16 | 1;
        Ok(command)
    }

    pub(super) fn identify_command(
        &self,
        cns: u32,
        namespace: u32,
    ) -> Result<[u32; 16], BlockError> {
        let mut command = [0u32; 16];
        command[0] = u32::from(ADMIN_IDENTIFY);
        command[1] = namespace;
        let page = self.admin_data.address()?;
        command[6] = page as u32;
        command[7] = (page >> 32) as u32;
        command[10] = cns;
        Ok(command)
    }

    /// Run admin `command` through the admin page once every queued command
    /// ended, as Linux runs its own admin commands: never retried. One past
    /// its deadline, or a fatal controller, resets the controller, as
    /// `nvme_timeout` does for the admin queue; a failed reset poisons every
    /// sibling. Only a positively identified CQE or an acknowledged reset
    /// authorizes SQ/PRP/data CPU reuse.
    ///
    /// A refusal is the completion's bare status, which callers tell from
    /// every other failure. A command that got no answer names itself, by
    /// opcode and Identify CNS or feature, and says when the controller's
    /// CSTS, not a completion, reported the failure.
    pub(super) fn admin(&mut self, command: [u32; 16]) -> Result<u32, BlockError> {
        self.ready()?;
        self.settle()?;
        match self.admin_wait(command, true) {
            Ok((result, 0)) => Ok(result),
            Ok((_, status)) => Err(BlockError::Device { status }),
            Err(Lost::Reset(error)) => {
                // The lost command leaves no admin queue to delete with.
                self.reset_with(false);
                // A reset that fails poisons the controller; the command
                // that was lost is still the error to report.
                let _ = self.settle();
                Err(within(
                    admin_name(command[0] as u8, command[10]),
                    fatal_status(error),
                ))
            }
            Err(Lost::Fatal(error)) => Err(self.poison(error)),
        }
    }

    /// Publish admin `command`, with the admin page when it moves `data`,
    /// and wait for its completion, taking the others posted before it.
    pub(super) fn admin_wait(
        &mut self,
        command: [u32; 16],
        data: bool,
    ) -> Result<(u32, u32), Lost> {
        match self.registers.read32(STATUS).map_err(Lost::Fatal)? {
            REMOVED => return Err(Lost::Fatal(removed())),
            status if self.fails(status) => {
                return Err(Lost::Reset(BlockError::Device { status }));
            }
            _ => {}
        }
        let memory = if data { Data::Admin } else { Data::None };
        let id = self.publish(false, command, memory).map_err(Lost::Fatal)?;
        let (clock, timeout) = (self.clock, self.admin_timeout_us);
        let mut observed = None;
        let mut polls = 0u32;
        clock.wait(timeout, || {
            // The CQE lives in RAM; CSTS is an MMIO read, which a hypervisor
            // traps. Check for a fatal controller only every few polls.
            polls = polls.wrapping_add(1);
            if polls.is_multiple_of(STATUS_POLL_INTERVAL) {
                let lost = match self.registers.read32(STATUS) {
                    Ok(REMOVED) => Some(Lost::Fatal(removed())),
                    Ok(status) if self.fails(status) => {
                        Some(Lost::Reset(BlockError::Device { status }))
                    }
                    Ok(_) => None,
                    Err(error) => Some(Lost::Fatal(error)),
                };
                if let Some(lost) = lost {
                    observed = Some(Err(lost));
                    return true;
                }
            }
            match self.admin_completion(Some((id, data))) {
                Ok(None) => false,
                Ok(Some(completion)) => {
                    observed = Some(Ok(completion));
                    true
                }
                Err(error) => {
                    observed = Some(Err(Lost::Fatal(error)));
                    true
                }
            }
        });
        observed.unwrap_or(Err(Lost::Reset(BlockError::Timeout)))
    }

    /// Take the admin completions the controller has posted, one doorbell
    /// each: an Asynchronous Event Request's is kept for
    /// [`Self::service_events`] and an Abort's ends it. Command `id`'s result
    /// and status once posted, giving the admin page back to the CPU when
    /// it holds `data`. A completion that names no outstanding command is
    /// passed over ([`Self::stray_completion`]).
    pub(super) fn admin_completion(
        &mut self,
        expected: Option<(u16, bool)>,
    ) -> Result<Option<(u32, u32)>, BlockError> {
        while let Some((id, result, status)) = self.posted(false)? {
            self.ring_completion(false)?;
            if self.events.request == Some(id) {
                self.events.request = None;
                self.events.completed = Some((result, status));
            } else if let Some(at) = self.aborts.iter().position(|abort| *abort == id) {
                self.aborts.swap_remove(at);
            } else if let Some((_, data)) = expected.filter(|(expected, _)| *expected == id) {
                if data {
                    self.trust_completion()?;
                    // SAFETY: the controller completed this exact command;
                    // no other command reaches the admin page meanwhile.
                    unsafe { self.admin_data.take()? };
                }
                return Ok(Some((result, status)));
            } else {
                self.stray_completion()?;
            }
        }
        Ok(None)
    }

    /// An admin Identify into the admin page.
    pub(super) fn identify(&mut self, cns: u32, namespace: u32) -> Result<&[u8], BlockError> {
        let command = self.identify_command(cns, namespace)?;
        self.admin(command)?;
        Ok(&self.admin_data.bytes()[..PAGE])
    }

    /// Linux `nvme_get_log` for every namespace into the admin page. A
    /// failure is only logged.
    pub(super) fn read_log(&mut self, log: u32, bytes: usize) -> Result<(), BlockError> {
        let page = self.admin_data.address()?;
        let mut command = [0u32; 16];
        command[0] = u32::from(ADMIN_GET_LOG_PAGE);
        command[1] = u32::MAX;
        command[6] = page as u32;
        command[7] = (page >> 32) as u32;
        command[10] = ((bytes / 4 - 1) as u32) << 16 | log;
        let _ = self.admin(command);
        self.ready()
    }

    /// Linux `nvme_scan_ns` before it adds or validates a namespace: its
    /// identifiers and whether the command-set-independent Identify
    /// Namespace described it, or why Linux passes it over: refused
    /// descriptors, a command set the controller does not run, or a
    /// namespace that is not ready.
    pub(super) fn namespace_ids(&mut self, namespace: u32) -> Result<Scan, BlockError> {
        let ids = match self.descriptors(namespace)? {
            Ok(ids) => ids,
            Err(error) => return Ok(Scan::Passed(error)),
        };
        if ids.csi != CSI_NVM && self.configuration & COMMAND_SETS_ALL == 0 {
            return Ok(Scan::Passed(BlockError::Unsupported(
                "NVMe namespace of a command set the controller does not run",
            )));
        }
        if self.independent_identify
            || (ids.csi != CSI_NVM && ids.csi != CSI_ZONED)
            || self.version >= VERSION_2_0
        {
            match self.identify(IDENTIFY_INDEPENDENT_NAMESPACE, namespace) {
                Ok(data) => {
                    return Ok(if data[INDEPENDENT_STATUS] & NAMESPACE_READY != 0 {
                        Scan::Found(ids, true)
                    } else {
                        Scan::Passed(BlockError::Unsupported(
                            "NVMe namespace not ready (NSTAT.NRDY clear)",
                        ))
                    });
                }
                Err(BlockError::Device { .. }) => {}
                Err(error) => return Err(error),
            }
        }
        Ok(Scan::Found(ids, false))
    }

    /// Linux `nvme_identify_ns_descs`: none before NVMe 1.3 or under
    /// `NVME_QUIRK_NO_NS_DESC_LIST`, and only the command set under
    /// `NVME_QUIRK_BOGUS_NID`. The controller's refusal, named, when it
    /// refused the list; the scan then skips the namespace.
    pub(super) fn descriptors(
        &mut self,
        namespace: u32,
    ) -> Result<Result<Ids, BlockError>, BlockError> {
        if self.version < VERSION_1_3 || self.quirks.contains(Quirks::NO_DESCRIPTOR_LIST) {
            return Ok(Ok(Ids::default()));
        }
        let bogus = self.quirks.contains(Quirks::BOGUS_IDS);
        match self.identify(IDENTIFY_DESCRIPTORS, namespace) {
            Ok(data) if bogus => Ok(Ok(Ids::from_descriptors(data).without_identifiers())),
            Ok(data) => Ok(Ok(Ids::from_descriptors(data))),
            Err(error @ BlockError::Device { .. }) => {
                Ok(Err(error.at(identify_name(IDENTIFY_DESCRIPTORS))))
            }
            Err(error) => Err(error),
        }
    }
}
