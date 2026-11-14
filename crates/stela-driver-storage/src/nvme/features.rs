// SPDX-License-Identifier: GPL-2.0-only
// Copyright (c) 2011-2014, Intel Corporation.
//! Optional controller features: host options, APST, the host memory buffer and
//! async events.

use super::*;

impl Controller {
    /// Linux `nvme_configure_host_options` from `identify`: a Set Features
    /// for Host Behavior Support enabling ACRE when a retry delay is
    /// reported and LBAFEE for extended LBA formats, its data in the
    /// admin page.
    pub(super) fn host_options(
        &mut self,
        identify: &[u8],
    ) -> Result<Option<[u32; 16]>, BlockError> {
        let acre = self.retry_delays[0] != 0;
        let lbafee = le32(identify, CONTROLLER_ATTRIBUTES) & ATTRIBUTE_EXTENDED_LBA != 0;
        if !acre && !lbafee {
            return Ok(None);
        }
        let page = &mut self.admin_data.bytes_mut()[..HOST_BEHAVIOR_BYTES];
        page.fill(0);
        page[HOST_BEHAVIOR_ACRE] = u8::from(acre);
        page[HOST_BEHAVIOR_LBAFEE] = u8::from(lbafee);
        let data = self.admin_data.address()?;
        let mut command = [0u32; 16];
        command[0] = u32::from(ADMIN_SET_FEATURES);
        command[6] = data as u32;
        command[7] = (data >> 32) as u32;
        command[10] = FEATURE_HOST_BEHAVIOR;
        Ok(Some(command))
    }

    /// The Set Features setting the autonomous power state transition
    /// `table`, written to the admin page, and APSTE.
    pub(super) fn apst_command(
        &mut self,
        table: &[u64; 32],
        enable: bool,
    ) -> Result<[u32; 16], BlockError> {
        for (index, entry) in table.iter().enumerate() {
            self.admin_data.write(8 * index, *entry);
        }
        let page = self.admin_data.address()?;
        let mut command = [0u32; 16];
        command[0] = u32::from(ADMIN_SET_FEATURES);
        command[6] = page as u32;
        command[7] = (page >> 32) as u32;
        command[10] = FEATURE_APST;
        command[11] = u32::from(enable);
        Ok(command)
    }

    /// Linux `nvme_alloc_host_mem_multi`: chunks of `chunk` bytes up to the
    /// preferred size and descriptor limit; `None` when not one chunk fits.
    pub(super) fn host_memory_chunks(
        request: HostMemoryRequest,
        preferred: u64,
        chunk: u64,
        source: &mut dyn HostMemorySource,
    ) -> Option<HostMemory> {
        let mut entries = preferred.div_ceil(chunk);
        if request.descriptors != 0 {
            entries = entries.min(u64::from(request.descriptors));
        }
        let mut descriptors = source
            .allocate(entries as usize * HOST_MEMORY_DESCRIPTOR, false)
            .ok()?;
        let mut chunks = Vec::new();
        let mut bytes = 0u64;
        while bytes < preferred && (chunks.len() as u64) < entries {
            let length = chunk.min(preferred - bytes);
            let Ok(buffer) = source.allocate(length as usize, true) else {
                break;
            };
            let at = chunks.len() * HOST_MEMORY_DESCRIPTOR;
            descriptors.write(at, buffer.address().ok()?);
            descriptors.write(at + 8, (length / PAGE as u64) as u32);
            chunks.push(buffer);
            bytes += length;
        }
        if bytes == 0 {
            return None;
        }
        descriptors.give().ok()?;
        for chunk in &mut chunks {
            chunk.give().ok()?;
        }
        Some(HostMemory {
            descriptors,
            chunks,
            bytes,
            returnable: false,
        })
    }

    /// Linux `nvme_setup_host_mem` at discovery: the preferred size, capped
    /// at `max_host_mem_size_mb`, from the largest chunks down to the
    /// minimum descriptor size. A controller works without one when it
    /// cannot be had; the bring-up hands it over.
    pub(super) fn allocate_host_memory(
        &mut self,
        request: HostMemoryRequest,
        source: &mut dyn HostMemorySource,
    ) {
        if request.preferred == 0 || request.minimum > HOST_MEMORY_MAXIMUM {
            return;
        }
        let preferred = request.preferred.min(HOST_MEMORY_MAXIMUM);
        let smallest = request.descriptor_minimum.max(2 * PAGE as u64);
        let mut chunk = preferred.min(HOST_MEMORY_CHUNK);
        while chunk >= smallest {
            if let Some(memory) = Self::host_memory_chunks(request, preferred, chunk, source) {
                if request.minimum == 0 || memory.bytes >= request.minimum {
                    self.host_memory = Some(memory);
                    return;
                }
                // nvme_free_host_mem: the controller never saw it.
                drop(memory);
            }
            chunk /= 2;
        }
    }

    /// Linux `nvme_set_host_mem` with EHM, and MR for a buffer the
    /// controller already filled.
    pub(super) fn host_memory_command(&self) -> Result<Option<[u32; 16]>, BlockError> {
        let Some(memory) = self.host_memory.as_ref() else {
            return Ok(None);
        };
        let mut bits = HOST_MEMORY_ENABLE;
        if memory.returnable {
            bits |= HOST_MEMORY_RETURN;
        }
        let list = memory.descriptors.address()?;
        let mut command = [0u32; 16];
        command[0] = u32::from(ADMIN_SET_FEATURES);
        command[10] = FEATURE_HOST_MEMORY;
        command[11] = bits;
        command[12] = (memory.bytes / PAGE as u64) as u32;
        command[13] = list as u32;
        command[14] = (list >> 32) as u32;
        command[15] = memory.chunks.len() as u32;
        Ok(Some(command))
    }

    /// The controller took the host memory buffer or refused it, which
    /// leaves it without one: the next bring-up offers it afresh
    /// (`nvme_free_host_mem`). Its pages stay reserved, never reused for
    /// anything else.
    pub(super) fn host_memory_answered(&mut self, accepted: bool) {
        self.host_memory_enabled = accepted;
        if let Some(memory) = self.host_memory.as_mut() {
            memory.returnable = accepted;
        }
    }

    /// Linux `nvme_set_queue_count` for the one I/O queue pair this driver
    /// runs. NVMe has the host set the Number of Queues before it creates
    /// an I/O queue, and Linux does after every reset; a controller may
    /// refuse a queue it was not asked for.
    pub(super) fn queue_count_command(&self) -> [u32; 16] {
        let mut command = [0u32; 16];
        command[0] = u32::from(ADMIN_SET_FEATURES);
        command[10] = FEATURE_QUEUE_COUNT;
        // One submission and one completion queue, each count zero-based.
        command[11] = 0;
        command
    }

    /// Linux `nvme_enable_aen`: enable the notices Linux handles.
    pub(super) fn events_command(&self) -> Option<[u32; 16]> {
        if self.events.supported == 0 {
            return None;
        }
        let mut command = [0u32; 16];
        command[0] = u32::from(ADMIN_SET_FEATURES);
        command[10] = FEATURE_ASYNC_EVENTS;
        command[11] = self.events.supported;
        Some(command)
    }

    /// Linux `nvme_pci_submit_async_event`.
    pub(super) fn request_event(&mut self) -> Result<(), BlockError> {
        let mut command = [0u32; 16];
        command[0] = u32::from(ADMIN_ASYNC_EVENT);
        let id = self.publish(false, command, Data::None)?;
        self.events.request = Some(id);
        Ok(())
    }

    /// Linux's interrupt-time `nvme_complete_async_event` and its work,
    /// between commands: act on a completed Asynchronous Event Request and
    /// request the next event. A failed request is not repeated; a
    /// persistent internal error resets the controller, which requests
    /// events again.
    pub(super) fn service_events(&mut self) -> Result<(), BlockError> {
        if self.events.request.is_some()
            && !self.busy()
            && let Err(error) = self.admin_completion(None)
        {
            return Err(self.poison(error));
        }
        let Some((result, status)) = self.events.completed.take() else {
            return Ok(());
        };
        if status != 0 {
            return Ok(());
        }
        match (result & 0x7, result >> 8 & 0xff) {
            (EVENT_NOTICE, NOTICE_NAMESPACES_CHANGED) => {
                // nvme_scan_work clears the Changed Namespace List first.
                self.read_log(LOG_CHANGED_NAMESPACES, PAGE)?;
                self.scans += 1;
            }
            (EVENT_NOTICE, NOTICE_FIRMWARE_ACTIVATION) => {
                // nvme_fw_act_work: commands wait while processing is paused,
                // up to MTFA or the admin timeout, then the Firmware Slot
                // Information log clears the event.
                let timeout = match self.events.activation_us {
                    0 => self.admin_timeout_us,
                    activation => activation,
                };
                match self.wait_status(timeout, |status| status & PROCESSING_PAUSED == 0) {
                    Ok(()) => self.read_log(LOG_FIRMWARE_SLOT, 512)?,
                    Err(BlockError::Timeout) => return self.reset_and_settle(),
                    Err(error) => return Err(self.poison(error)),
                }
            }
            (EVENT_ERROR, ERROR_PERSISTENT_INTERNAL) => return self.reset_and_settle(),
            _ => {}
        }
        self.request_event()
    }
}
