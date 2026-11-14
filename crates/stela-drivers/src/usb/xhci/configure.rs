//! Configuration of a device's endpoints: SET_CONFIGURATION and Configure
//! Endpoint (xHCI 4.6.6), validated before any context is published.
use super::*;

impl Controller {
    /// Cached device state, updated only by acknowledged standard requests.
    pub fn current_configuration(&mut self, id: impl DeviceRef) -> Result<Option<u8>, BlockError> {
        let slot = id.current_slot(self)?;
        self.current_configuration_owned(slot)
    }

    pub(super) fn current_configuration_owned(
        &mut self,
        slot: u8,
    ) -> Result<Option<u8>, BlockError> {
        self.live_slot(slot)?;
        self.ready()?;
        Ok(self.slot(slot)?.current_configuration)
    }

    /// Idempotent class-binding request. Repeating SET_CONFIGURATION, even with
    /// the same value, resets alternate settings and endpoint state (USB 9.4.7).
    pub fn set_configuration(&mut self, id: impl DeviceRef, value: u8) -> Result<(), BlockError> {
        let slot = id.current_slot(self)?;
        self.set_configuration_owned(slot, value)
    }

    pub(super) fn set_configuration_owned(
        &mut self,
        slot: u8,
        value: u8,
    ) -> Result<(), BlockError> {
        if self.current_configuration_owned(slot)? == Some(value) {
            return Ok(());
        }
        self.control_owned(
            slot,
            setup(0, SET_CONFIGURATION, u16::from(value), 0, 0),
            &[],
        )?;
        Ok(())
    }

    /// Adds `endpoints` to `slot` (Configure Endpoint, 4.6.6), with the hub
    /// fields when `hub` is `Some((ports, think_time, multi_tt))`.
    pub fn configure(
        &mut self,
        id: impl DeviceRef,
        endpoints: &[Endpoint],
        hub: Option<(u8, u8, bool)>,
        dma: &mut dyn DmaAllocator,
    ) -> Result<(), BlockError> {
        let slot = id.current_slot(self)?;
        self.configure_owned(slot, endpoints, hub, dma)
    }

    pub(super) fn configure_owned(
        &mut self,
        slot: u8,
        endpoints: &[Endpoint],
        hub: Option<(u8, u8, bool)>,
        dma: &mut dyn DmaAllocator,
    ) -> Result<(), BlockError> {
        self.configure_inner(slot, endpoints, hub, &[], 0, dma)
            .map(|_| ())
    }

    /// Check that every requested endpoint is valid and currently unclaimed
    /// before a class changes its device's alternate setting. This observes the
    /// current controller epoch; it does not reserve DCIs. The class must
    /// serialize the complete change and configure rechecks these same rules
    /// before recording or publishing any endpoint ownership.
    pub fn preflight_endpoints(
        &mut self,
        id: impl DeviceRef,
        endpoints: &[Endpoint],
    ) -> Result<(), BlockError> {
        let slot = id.current_slot(self)?;
        self.preflight_endpoints_owned(slot, endpoints)
    }

    pub(super) fn preflight_endpoints_owned(
        &mut self,
        slot: u8,
        endpoints: &[Endpoint],
    ) -> Result<(), BlockError> {
        self.endpoint_contexts(slot, endpoints).map(|_| ())
    }

    fn endpoint_contexts(
        &mut self,
        slot: u8,
        endpoints: &[Endpoint],
    ) -> Result<Vec<endpoint::Context>, BlockError> {
        self.live_slot(slot)?;
        self.ready()?;
        if !self
            .slot(slot)?
            .current_configuration
            .is_some_and(|value| value != 0)
        {
            return Err(BlockError::Unsupported(
                "USB device configuration is not acknowledged",
            ));
        }
        // Validate every endpoint and allocate every ring before publishing any
        // change. Existing rings may still be DMA-owned, even after a failure.
        let mut selected = 0u32;
        let speed = self.slot(slot)?.speed;
        let mut contexts = Vec::new();
        for endpoint in endpoints {
            let dci = endpoint.dci();
            if endpoint.address & 0x70 != 0
                || dci < 2
                || endpoint.max_packet & 0x7ff == 0
                || !matches!(endpoint.kind(), 1..=3)
                || selected & (1 << dci) != 0
            {
                return Err(BlockError::Unsupported(
                    "invalid or duplicate xHCI endpoint",
                ));
            }
            if let Some(transfer) = &self.slot(slot)?.endpoints[usize::from(dci)] {
                return Err(BlockError::Unsupported(if transfer.poisoned {
                    "xHCI endpoint cancellation failed"
                } else {
                    "xHCI endpoint replacement requires teardown"
                }));
            }
            let context = endpoint::Context::new(endpoint, speed)?;
            let context = if endpoint.kind() == 1 {
                self.iso_caps.context(context)
            } else {
                context
            };
            contexts.push(self.quirks.endpoint(context, speed));
            selected |= 1 << dci;
        }
        if endpoints.iter().any(|ep| ep.kind() == 1) {
            self.iso_caps.admit()?;
            let attachment = self.slot(slot)?.attachment;
            if attachment.tier > MAX_TIERS
                || attachment.route >> (4 * MAX_TIERS) != 0
                || (attachment.tier == 0
                    && (attachment.route != 0
                        || attachment.parent.is_some()
                        || attachment.translator.is_some()))
                || (attachment.tier != 0 && (attachment.route == 0 || attachment.parent.is_none()))
            {
                return Err(BlockError::Unsupported(
                    "invalid isochronous attachment route",
                ));
            }
        }
        Ok(contexts)
    }

    pub(super) fn configure_inner(
        &mut self,
        slot: u8,
        endpoints: &[Endpoint],
        hub: Option<(u8, u8, bool)>,
        stream_endpoints: &[u8],
        requested_streams: u16,
        dma: &mut dyn DmaAllocator,
    ) -> Result<u16, BlockError> {
        let contexts = self.endpoint_contexts(slot, endpoints)?;
        let speed = self.slot(slot)?.speed;
        let layout = if stream_endpoints.is_empty() {
            None
        } else {
            self.stream_policy.admit()?;
            Some(streams::Layout::new(
                self.max_primary_streams,
                speed,
                endpoints,
                stream_endpoints,
                requested_streams,
            )?)
        };
        self.retry_resources.preflight_endpoints(
            slot,
            endpoints,
            stream_endpoints,
            layout,
            self.stream_policy == streams::Policy::Cadence,
            &self.slots[usize::from(slot)]
                .as_ref()
                .unwrap()
                .retired_rings,
        )?;
        let mut rings = Vec::new();
        rings
            .try_reserve_exact(endpoints.len())
            .map_err(|_| BlockError::NoDmaMemory)?;
        for endpoint in endpoints {
            let allocation = (|| {
                if let Some(pair) = self.retry_resources.endpoint(slot, endpoint.dci()) {
                    return Ok(pair);
                }
                let streamed = stream_endpoints.contains(&endpoint.address);
                let stream = if streamed {
                    Some(streams::Streams::with_memory(
                        dma,
                        layout.unwrap(),
                        self.dma_profile,
                        self.stream_policy == streams::Policy::Cadence,
                        self.native_memory.as_ref(),
                    )?)
                } else {
                    None
                };
                let retained = if streamed {
                    None
                } else {
                    self.slot(slot)?.retired_rings[usize::from(endpoint.dci())].take()
                };
                let ring = match retained {
                    Some(ring) => ring,
                    None => self.allocate_ring(dma)?,
                };
                Ok((ring, stream))
            })();
            match allocation {
                Ok(pair) => rings.push(pair),
                Err(error) => {
                    // No input context or doorbell has been published. Keep
                    // both reclaimed and newly allocated ordinary rings if a
                    // subsequent allocation fails; a retry must not consume
                    // another slice of the boot-lived DMA reservation.
                    for (prior, (ring, stream)) in endpoints.iter().zip(rings) {
                        if stream.is_none() && !self.retry_resources.sealed(slot) {
                            self.slot(slot)?.retired_rings[usize::from(prior.dci())] = Some(ring);
                        } else {
                            self.retry_resources
                                .return_endpoint(slot, prior, ring, stream, speed);
                        }
                    }
                    return Err(error);
                }
            };
        }
        if let Err(error) = self.clear_input(slot) {
            for (endpoint, (ring, stream)) in endpoints.iter().zip(rings) {
                if stream.is_none() && !self.retry_resources.sealed(slot) {
                    self.slot(slot)?.retired_rings[usize::from(endpoint.dci())] = Some(ring);
                } else {
                    self.retry_resources
                        .return_endpoint(slot, endpoint, ring, stream, speed);
                }
            }
            return Err(error);
        }
        let context = self.context();
        let mut add = 1u32;
        let mut last = 1u8;
        for ((endpoint, (ring, stream)), recipe) in endpoints.iter().zip(rings).zip(contexts) {
            let dci = endpoint.dci();
            let state = self.slot(slot)?;
            let at = (usize::from(dci) + 1) * context;
            let (stream_info, dequeue) = stream
                .as_ref()
                .map_or((0, ring.dequeue_pointer()), |streams| streams.context());
            state.input.write(at, recipe.info | stream_info);
            state.input.write(at + 4, recipe.info2);
            state.input.write(at + 8, dequeue);
            state.input.write(at + 16, recipe.tx_info);
            state.endpoints[usize::from(dci)] = Some(Transfer {
                ring,
                streams: stream,
                iso: if endpoint.kind() == 1 {
                    Some(iso::Queue::new(iso::Recipe::new(endpoint, speed)?))
                } else {
                    None
                },
                is_in: endpoint.is_in(),
                poisoned: false,
                enabled: false,
                in_flight: 0,
                stopped: false,
            });
            add |= 1 << dci;
            last = last.max(dci);
        }
        let input_address = {
            let state = self.slot(slot)?;
            let mut words = state.context;
            let entries = (words[0] >> 27).max(u32::from(last));
            words[0] = words[0] & !(0x1f << 27) | entries << 27;
            if let Some((ports, think_time, multi)) = hub {
                words[0] |= 1 << 26 | u32::from(multi) << 25;
                words[1] = words[1] & 0x00ff_ffff | u32::from(ports) << 24;
                words[2] = words[2] & !(3 << 16) | u32::from(think_time & 3) << 16;
            }
            state.context = words;
            state.input.write(4, add);
            for (n, word) in words.iter().enumerate() {
                state.input.write(context + 4 * n, *word);
            }
            state.input.device_address()
        };
        let result = self.command(
            input_address,
            0,
            CONFIGURE_ENDPOINT << 10 | u32::from(slot) << 24,
        );
        // Keep rings on any error: a lost command completion may still leave
        // them reachable by the controller. Never expose an unacknowledged ring.
        for endpoint in endpoints {
            let transfer = self.slot(slot)?.endpoints[usize::from(endpoint.dci())]
                .as_mut()
                .unwrap();
            transfer.enabled = result.is_ok();
            transfer.poisoned = result.is_err();
        }
        result?;
        Ok(layout.map_or(0, |layout| layout.usable))
    }
}

#[cfg(test)]
mod tests;
