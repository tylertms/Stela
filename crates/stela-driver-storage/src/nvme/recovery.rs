// SPDX-License-Identifier: GPL-2.0-only
// Copyright (c) 2011-2014, Intel Corporation.
//! Controller reset and bring-up, as Linux's timeout and reset work runs them,
//! and shutdown.

use super::*;

impl Controller {
    /// Reset the controller, as Linux's `nvme_timeout` does once a
    /// controller is not gone: every running command waits to run again
    /// once the reset completes, while it has retries left.
    pub(super) fn reset(&mut self) {
        self.reset_with(true);
    }

    /// [`Self::reset`]: `nvme_dev_disable` deletes the I/O queues of a live
    /// controller, unless `delete` is clear or an Abort went unanswered,
    /// either leaving no admin queue to delete with. A removed controller
    /// stays failed.
    pub(super) fn reset_with(&mut self, delete: bool) {
        // The reset empties the queues: an owed doorbell names nothing.
        self.owed = false;
        let status = match self.registers.read32(STATUS) {
            Ok(REMOVED) => Err(removed()),
            other => other,
        };
        let status = match status {
            Ok(status) => status,
            Err(error) => {
                self.poison(error);
                self.fail_all();
                return;
            }
        };
        for tag in &mut self.tags {
            *tag = match core::mem::take(tag) {
                Tag::Running {
                    namespace,
                    block,
                    command,
                    retries,
                    ..
                } => Tag::Waiting {
                    namespace,
                    block,
                    command,
                    retries: retries + 1,
                },
                other => other,
            };
        }
        let dead = status & FATAL != 0 || status & READY == 0;
        let started = if !dead && delete && self.aborts.is_empty() {
            self.delete_next(true)
        } else {
            self.disable()
        };
        if let Err(error) = started {
            self.poison(error);
            self.fail_all();
        }
    }

    /// Delete the I/O submission queue, or then the completion queue.
    pub(super) fn delete_next(&mut self, submission: bool) -> Result<(), BlockError> {
        let mut command = [0u32; 16];
        command[0] = u32::from(if submission {
            ADMIN_DELETE_SUBMISSION
        } else {
            ADMIN_DELETE_COMPLETION
        });
        command[10] = u32::from(self.io.id);
        let id = self.publish(false, command, Data::None)?;
        let deadline = self.deadline(self.admin_timeout_us)?;
        self.recovery = Some(Recovery::Deleting {
            submission,
            id,
            deadline,
        });
        Ok(())
    }

    /// Clear CC.EN (and any shutdown notification). A quirked controller's
    /// CSTS is read only after Linux's delay, which [`Self::recover`] steps
    /// through without waiting.
    pub(super) fn disable(&mut self) -> Result<(), BlockError> {
        let configuration = self.registers.read32(CONFIGURATION)?;
        self.registers.write32(
            CONFIGURATION,
            configuration & !(ENABLE | SHUTDOWN_NOTIFICATION),
        )?;
        let delay = if self.quirks.contains(Quirks::READY_DELAY) {
            quirks::READY_CHECK_DELAY_US
        } else {
            0
        };
        let check = match delay {
            0 => None,
            delay => Some(self.deadline(delay)?),
        };
        let deadline = self.deadline(self.ready_timeout_us + delay)?;
        self.recovery = Some(Recovery::Disabling(deadline, check));
        Ok(())
    }

    /// The disabled controller no longer reaches any queue or buffer:
    /// contain its DMA (`pci_disable_device`), take every one back, end the
    /// commands out of retries (`nvme_cancel_tagset`), empty the queues and
    /// point the controller at the admin queue again.
    pub(super) fn restart_queues(&mut self) -> Result<(), BlockError> {
        if let Some(owner) = self.owned.as_mut() {
            owner.lifecycle.disable_dma()?;
            // SAFETY: the engine acknowledged its reset and BME is off.
            unsafe { owner.domain.drain_posted_writes() }.map_err(owned::map_dma_error)?;
        }
        // SAFETY: CSTS.RDY clear acknowledges the stopped engine.
        unsafe {
            for page in &mut self.pool.pages {
                page.take()?;
            }
            self.admin_data.take()?;
            self.admin.submissions.take()?;
            self.io.submissions.take()?;
            for queue in [&mut self.admin, &mut self.io] {
                queue.completions.take()?;
                queue.completions.bytes_mut().fill(0);
            }
        }
        for tag in 0..self.tags.len() {
            if matches!(self.tags[tag], Tag::Waiting { retries, .. } if retries > RETRIES) {
                self.tags[tag] = Tag::Done(Err(BlockError::Timeout));
                self.release(tag);
            }
        }
        self.io.restart();
        self.aborts.clear();
        // The disable ended the Asynchronous Event Request; the host memory
        // buffer is offered back with the bring-up.
        self.events.request = None;
        self.host_memory_enabled = false;
        self.configure_admin()?;
        if let Some(owner) = self.owned.as_mut() {
            owner.lifecycle.enable_dma()?;
        }
        Ok(())
    }

    /// Move the controller reset on, never waiting: the I/O queues deleted,
    /// disabled, enabled with the admin queue, brought up again, then the
    /// interrupted commands run again under their tags.
    pub(super) fn recover(&mut self) -> Result<(), BlockError> {
        if let Some(Recovery::Disabling(_, Some(check))) = &self.recovery
            && !check.passed()
        {
            return Ok(());
        }
        let status = self.registers.read32(STATUS)?;
        if status == REMOVED {
            return Err(removed());
        }
        match self.recovery.take() {
            None => Ok(()),
            Some(Recovery::Deleting {
                submission,
                id,
                deadline,
            }) => match self.admin_completion(Some((id, false)))? {
                None if !deadline.passed() => {
                    self.recovery = Some(Recovery::Deleting {
                        submission,
                        id,
                        deadline,
                    });
                    Ok(())
                }
                // A refused deletion is ignored and a timed-out one ends
                // the deletions, as in Linux.
                Some(_) if submission => self.delete_next(false),
                _ => self.disable(),
            },
            Some(Recovery::Disabling(deadline, check)) => {
                if status & READY != 0 {
                    if deadline.passed() {
                        return Err(BlockError::Timeout);
                    }
                    self.recovery = Some(Recovery::Disabling(deadline, check));
                    return Ok(());
                }
                self.restart_queues()?;
                let timeout = self.enable()?;
                let deadline = self.deadline(timeout)?;
                self.recovery = Some(Recovery::Enabling(deadline));
                Ok(())
            }
            Some(Recovery::Enabling(deadline)) => {
                // As Linux `nvme_wait_ready`: CSTS.RDY alone ends the wait.
                // A CSTS.CFS still set then stands ([`Self::fails`]); one
                // set at the timeout is the error reported.
                let fatal = status & FATAL != 0;
                if status & READY == 0 {
                    if !deadline.passed() {
                        self.recovery = Some(Recovery::Enabling(deadline));
                        return Ok(());
                    }
                    if fatal {
                        return Err(BlockError::Device { status });
                    }
                    return Err(BlockError::Timeout);
                }
                self.fatal_standing = fatal;
                self.bring_up(Step::Identify, Vec::new())
            }
            Some(Recovery::Admin {
                step,
                id,
                deadline,
                mut identify,
            }) => {
                let Some((_, code)) = self.admin_completion(Some((id, true)))? else {
                    if deadline.passed() {
                        return Err(BlockError::Timeout.at(step.name()));
                    }
                    self.recovery = Some(Recovery::Admin {
                        step,
                        id,
                        deadline,
                        identify,
                    });
                    return Ok(());
                };
                match (step, code) {
                    (Step::Identify, 0) => {
                        identify = self.admin_data.bytes()[..PAGE].to_vec();
                        self.identified(&identify);
                    }
                    (Step::Limits, code) => self.limits_answered(code),
                    (Step::HostMemory, code) => self.host_memory_answered(code == 0),
                    // A refused feature is only logged.
                    (Step::HostOptions | Step::Apst | Step::Events, _) => {}
                    // Every controller grants at least one queue pair. After
                    // a refusal Linux creates no I/O queue; here the queue's
                    // creation is still tried, and reports its own refusal.
                    (Step::QueueCount, _) => {}
                    (Step::CreateCompletion | Step::CreateSubmission, 0) => {}
                    (step, status) => {
                        return Err(BlockError::Device { status }.at(step.name()));
                    }
                }
                let next = STEPS.iter().position(|each| *each == step).unwrap() + 1;
                match STEPS.get(next) {
                    Some(&next) => self.bring_up(next, identify),
                    None => self.restarted(),
                }
            }
        }
    }

    /// Submit the bring-up command of the first step from `from` on that
    /// has one; with none left, the controller is up.
    pub(super) fn bring_up(&mut self, from: Step, identify: Vec<u8>) -> Result<(), BlockError> {
        let first = STEPS.iter().position(|each| *each == from).unwrap();
        for &step in &STEPS[first..] {
            let command = match step {
                Step::Identify => Some(self.identify_command(IDENTIFY_CONTROLLER, 0)?),
                Step::Limits => self.limits_command()?,
                Step::HostOptions => self.host_options(&identify)?,
                Step::Apst => match apst_table(&identify, self.quirks) {
                    Some((table, enable)) => Some(self.apst_command(&table, enable)?),
                    None => None,
                },
                Step::HostMemory => self.host_memory_command()?,
                Step::QueueCount => Some(self.queue_count_command()),
                Step::CreateCompletion => Some(self.create_queue(false)?),
                Step::CreateSubmission => {
                    self.io.submissions.publish();
                    Some(self.create_queue(true)?)
                }
                Step::Events => self.events_command(),
            };
            let Some(command) = command else {
                continue;
            };
            let id = self.publish(false, command, Data::Admin)?;
            let deadline = self.deadline(self.admin_timeout_us)?;
            self.recovery = Some(Recovery::Admin {
                step,
                id,
                deadline,
                identify,
            });
            return Ok(());
        }
        self.restarted()
    }

    /// `nvme_start_ctrl`: leave an Asynchronous Event Request outstanding,
    /// queue a namespace scan and run the interrupted commands again.
    pub(super) fn restarted(&mut self) -> Result<(), BlockError> {
        if self.events.supported != 0 {
            self.request_event()?;
        }
        self.scans += 1;
        for tag in 0..self.tags.len() {
            match core::mem::take(&mut self.tags[tag]) {
                Tag::Waiting {
                    namespace,
                    block,
                    command,
                    retries,
                } => self.submit(tag, namespace, block, command, retries),
                other => self.tags[tag] = other,
            }
        }
        Ok(())
    }

    /// What `nvme_init_ctrl_finish` keeps from Identify Controller: the
    /// retry delays, transfer limits and the events to enable. A reset's fresh
    /// Identify updates both limits before namespace features are rescanned.
    pub(super) fn identified(&mut self, identify: &[u8]) {
        let mdts = identify[CONTROLLER_MAXIMUM_TRANSFER];
        let bytes = if mdts == 0 {
            usize::MAX
        } else {
            1usize
                .checked_shl(u32::from(mdts))
                .unwrap_or(usize::MAX)
                .saturating_mul(PAGE)
        };
        self.max_transfer = BOUNCE_BYTES.min(bytes);
        self.max_command_bytes = MAX_COMMAND_BYTES.min(bytes);
        for (index, delay) in self.retry_delays.iter_mut().enumerate() {
            let at = CONTROLLER_RETRY_DELAYS + 2 * index;
            *delay = u16::from_le_bytes([identify[at], identify[at + 1]]);
        }
        self.events = Events::from_identify(identify);
    }

    /// The NVM Identify Controller of `nvme_init_non_mdts_limits`, which
    /// `nvme_id_cns_ok` sends only from NVMe 1.2. WZSL starts over each
    /// time; a refused command is not sent again.
    pub(super) fn limits_command(&mut self) -> Result<Option<[u32; 16]>, BlockError> {
        self.limits.write_zeroes = 0;
        if self.version < VERSION_1_2 || self.limits.refused {
            return Ok(None);
        }
        // CDW11's CSI stays 0, the NVM command set.
        self.identify_command(IDENTIFY_COMMAND_SET_CONTROLLER, 0)
            .map(Some)
    }

    pub(super) fn limits_answered(&mut self, status: u32) {
        if status != 0 {
            self.limits.refused = true;
            return;
        }
        let page = self.admin_data.bytes();
        self.limits.write_zeroes = page[NVM_WRITE_ZEROES_LIMIT];
        self.limits.deallocate_ranges = page[NVM_DEALLOCATE_RANGES];
        self.limits.deallocate = le32(page, NVM_DEALLOCATE_RANGE_LIMIT);
    }

    pub(super) fn reset_and_settle(&mut self) -> Result<(), BlockError> {
        self.reset();
        self.settle()
    }

    /// Flush callers before this method; acknowledged SHST prevents further
    /// commands, while owned resource retirement separately requires RDY0.
    pub(super) fn shutdown(&mut self) -> Result<(), BlockError> {
        if self.shut_down {
            return Ok(());
        }
        self.ready()?;
        if let Err(error) = self.check_fatal() {
            return Err(self.poison(error));
        }
        self.settle()?;
        self.delete_io_queues()?;
        let result = (|| {
            let configuration = self.registers.read32(CONFIGURATION)?;
            self.registers.write32(
                CONFIGURATION,
                configuration & !SHUTDOWN_NOTIFICATION | SHUTDOWN_NORMAL,
            )?;
            self.wait_status(self.shutdown_timeout_us, |status| {
                status & SHUTDOWN_STATUS == SHUTDOWN_COMPLETE
            })
        })();
        if let Err(error) = result {
            return Err(self.poison(error));
        }
        self.shut_down = true;
        Ok(())
    }

    /// Linux `nvme_dev_disable` deletes the I/O queues, submission queue
    /// first, before the shutdown notification. A refused deletion is
    /// ignored and a timed-out one ends the deletions, as in Linux; a
    /// broken protocol poisons.
    pub(super) fn delete_io_queues(&mut self) -> Result<(), BlockError> {
        for opcode in [ADMIN_DELETE_SUBMISSION, ADMIN_DELETE_COMPLETION] {
            let mut command = [0u32; 16];
            command[0] = u32::from(opcode);
            command[10] = u32::from(self.io.id);
            match self.admin_wait(command, false) {
                Ok(_) => {}
                Err(Lost::Reset(_)) => break,
                Err(Lost::Fatal(error)) => return Err(self.poison(error)),
            }
        }
        Ok(())
    }
}
