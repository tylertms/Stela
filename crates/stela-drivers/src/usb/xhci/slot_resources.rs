// SPDX-License-Identifier: GPL-2.0-only
//! Per-hardware-slot EP0 storage for unpublished enumeration retries.
//!
//! Only retirement's noncopyable proof can return storage. Neither a slot ID,
//! physical-port status nor the current ledger can manufacture that proof.
//! Published root-leaf storage requires retirement's physical-reset witness.
//! Storage never restores a generation or the permanent raw-slot fence.
use super::*;

pub(super) struct Resources {
    pub(super) output: memory::Buffer,
    pub(super) input: memory::Buffer,
    pub(super) control: memory::Buffer,
    pub(super) ring: Ring,
}

pub(super) struct Cache {
    // Keep the entire original owner until a new enabled grant and current
    // reservation exist. The old generation is never returned as permission.
    slots: Vec<Option<Slot>>,
    // Retired data/ISO/stream storage remains owned when EP0 is readdressed.
    endpoints: Vec<[Option<Transfer>; 32]>,
    sealed: Vec<bool>,
    rings: Vec<[Option<Ring>; 32]>,
    // Enable Slot may choose a different vacant numeric ID. The offered
    // complete owner follows that real grant, rather than allocating another set.
    offered: Option<u8>,
}

impl Cache {
    pub(super) fn new(slots: usize) -> Self {
        Self {
            slots: core::iter::repeat_with(|| None).take(slots).collect(),
            endpoints: core::iter::repeat_with(Default::default)
                .take(slots)
                .collect(),
            sealed: vec![false; slots],
            rings: core::iter::repeat_with(Default::default)
                .take(slots)
                .collect(),
            offered: None,
        }
    }

    pub(super) fn retain(&mut self, retired: retirement::Unpublished) {
        let (slot, state) = retired.into_slot();
        let retained = &mut self.slots[usize::from(slot)];
        assert!(retained.is_none(), "duplicate xHCI retry resource owner");
        *retained = Some(state);
    }

    pub(super) fn can_offer(&self, slot: u8, state: &Slot) -> bool {
        let index = usize::from(slot);
        self.offered.is_none()
            && self.slots[index].is_none()
            && (0..32).all(|dci| {
                (self.endpoints[index][dci].is_none() || state.endpoints[dci].is_none())
                    && (self.rings[index][dci].is_none() || state.retired_rings[dci].is_none())
            })
    }

    pub(super) fn offer(&mut self, retired: retirement::Published) {
        let (slot, mut state) = retired.into_slot();
        assert!(
            self.can_offer(slot, &state),
            "duplicate xHCI published resource owner"
        );
        let index = usize::from(slot);
        // Unconfigured leftovers already belong to this same pool.
        // Move them with the next grant even if its numeric ID differs.
        for dci in 2..32 {
            if let Some(endpoint) = self.endpoints[index][dci].take() {
                state.endpoints[dci] = Some(endpoint);
            }
            if let Some(ring) = self.rings[index][dci].take() {
                state.retired_rings[dci] = Some(ring);
            }
        }
        self.slots[index] = Some(state);
        self.offered = Some(slot);
    }

    pub(super) fn preflight_endpoints(
        &self,
        slot: u8,
        endpoints: &[Endpoint],
        selected: &[u8],
        layout: Option<streams::Layout>,
        cadence: bool,
        retired_rings: &[Option<Ring>; 32],
    ) -> Result<(), BlockError> {
        let index = usize::from(slot);
        for endpoint in endpoints {
            let dci = usize::from(endpoint.dci());
            let streamed = selected.contains(&endpoint.address);
            let Some(state) = self.endpoints[index][dci].as_ref() else {
                if self.sealed[index]
                    && (streamed
                        || self.rings[index][dci].is_none() && retired_rings[dci].is_none())
                {
                    return Err(BlockError::Unsupported(
                        "USB reusable endpoint geometry changed",
                    ));
                }
                continue;
            };
            if state.is_in != endpoint.is_in()
                || state.iso.is_some() != (endpoint.kind() == 1)
                || match (&state.streams, streamed) {
                    (Some(streams), true) => !streams.matches(layout.unwrap(), cadence),
                    (None, false) => false,
                    _ => true,
                }
            {
                return Err(BlockError::Unsupported(
                    "USB reusable stream or endpoint geometry changed",
                ));
            }
        }
        Ok(())
    }

    pub(super) fn sealed(&self, slot: u8) -> bool {
        self.sealed[usize::from(slot)]
    }

    pub(super) fn endpoint(
        &mut self,
        slot: u8,
        dci: u8,
    ) -> Option<(Ring, Option<streams::Streams>)> {
        let index = usize::from(slot);
        let dci = usize::from(dci);
        if let Some(state) = self.endpoints[index][dci].take() {
            Some((state.ring, state.streams))
        } else {
            self.rings[index][dci].take().map(|ring| (ring, None))
        }
    }

    pub(super) fn return_endpoint(
        &mut self,
        slot: u8,
        endpoint: &Endpoint,
        ring: Ring,
        streams: Option<streams::Streams>,
        speed: u8,
    ) {
        let retained = &mut self.endpoints[usize::from(slot)][usize::from(endpoint.dci())];
        assert!(retained.is_none());
        *retained = Some(Transfer {
            ring,
            streams,
            iso: (endpoint.kind() == 1).then(|| {
                iso::Queue::new(
                    iso::Recipe::new(endpoint, speed).expect("previously validated endpoint"),
                )
            }),
            is_in: endpoint.is_in(),
            poisoned: false,
            enabled: false,
            in_flight: 0,
            stopped: false,
        });
    }

    #[cfg(test)]
    pub(super) fn allocation_geometry(&self) -> Vec<(u64, usize)> {
        let mut all = Vec::new();
        for state in self.slots.iter().flatten() {
            all.extend([
                (state.input.device_address(), state.input.len()),
                (state._output.device_address(), state._output.len()),
                (state.control.device_address(), state.control.len()),
            ]);
            for ep in state.endpoints.iter().flatten() {
                all.push((ep.ring.buffer.device_address(), ep.ring.buffer.len()));
                if let Some(streams) = &ep.streams {
                    all.extend(streams.allocation_geometry());
                }
            }
            for ring in state.retired_rings.iter().flatten() {
                all.push((ring.buffer.device_address(), ring.buffer.len()));
            }
        }
        for endpoints in &self.endpoints {
            for ep in endpoints.iter().flatten() {
                all.push((ep.ring.buffer.device_address(), ep.ring.buffer.len()));
                if let Some(streams) = &ep.streams {
                    all.extend(streams.allocation_geometry());
                }
            }
        }
        for rings in &self.rings {
            for ring in rings.iter().flatten() {
                all.push((ring.buffer.device_address(), ring.buffer.len()));
            }
        }
        all.sort_unstable();
        all
    }

    fn take(
        &mut self,
        slot: u8,
        attachment: Attachment,
        parent: Option<SlotId>,
    ) -> Result<Option<Resources>, BlockError> {
        let index = usize::from(slot);
        let source = usize::from(self.offered.unwrap_or(slot));
        if self.offered.is_some()
            && self.slots[source]
                .as_ref()
                .is_none_or(|state| state.attachment != attachment || state.parent_id != parent)
        {
            return Err(BlockError::Unsupported(
                "USB reusable pool original attachment changed",
            ));
        }
        if self.offered.is_some()
            && source != index
            && (self.slots[index].is_some()
                || self.endpoints[index].iter().any(Option::is_some)
                || self.rings[index].iter().any(Option::is_some))
        {
            return Err(BlockError::Unsupported(
                "USB reusable grant has another retained resource pool",
            ));
        }
        let Some(state) = self.slots[source].take() else {
            return Ok(None);
        };
        let published = self.offered.take().is_some();
        let Slot {
            mut input,
            _output: mut output,
            mut control,
            mut endpoints,
            mut retired_rings,
            ..
        } = state;
        let mut ring = endpoints[1].take().unwrap().ring;
        output.bytes_mut().fill(0);
        input.bytes_mut().fill(0);
        control.bytes_mut().fill(0);
        ring.reinitialize();
        if published {
            for transfer in endpoints.iter_mut().flatten() {
                transfer.ring.reinitialize();
                if let Some(streams) = &mut transfer.streams {
                    streams.reinitialize();
                }
                // The old ISO/software ticket state cannot survive the new ID.
                transfer.poisoned = false;
                transfer.enabled = false;
                transfer.in_flight = 0;
            }
            for ring in retired_rings.iter_mut().flatten() {
                ring.reinitialize();
            }
            self.rings[index] = retired_rings;
            self.endpoints[index] = endpoints;
            self.sealed[source] = false;
            self.sealed[index] = true;
        }
        Ok(Some(Resources {
            output,
            input,
            control,
            ring,
        }))
    }
}

impl Controller {
    pub(super) fn address_resources(
        &mut self,
        slot: u8,
        attachment: Attachment,
        parent: Option<SlotId>,
        dma: &mut dyn DmaAllocator,
    ) -> Result<Resources, BlockError> {
        let index = usize::from(slot);
        // The fresh Enable Slot grant and successful current reservation are
        // established by address before entering here. A cache is storage, never
        // a substitute for either grant or the physical reset between retries.
        if slot == 0
            || self.slot_ownership.get(index) != Some(&reset::Ownership::Enabled)
            || self.slots.get(index).is_none_or(Option::is_some)
            || self.dcbaa.read::<u64>(index * 8) != 0
        {
            return Err(BlockError::Unsupported("invalid xHCI retry resource owner"));
        }
        if let Some(resources) = self.retry_resources.take(slot, attachment, parent)? {
            return Ok(resources);
        }
        let context = self.context();
        Ok(Resources {
            output: self.allocate(dma, 32 * context, 64)?,
            input: self.allocate(dma, 33 * context, 64)?,
            control: self.allocate(dma, CONTROL_BYTES, CONTROL_BYTES)?,
            ring: self.allocate_ring(dma)?,
        })
    }
}
