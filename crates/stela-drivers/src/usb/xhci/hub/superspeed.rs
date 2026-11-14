// SPDX-License-Identifier: GPL-2.0-only
//! SuperSpeed cold hub power/reset protocol on the real EP0 transport.
use super::{descriptor::*, *};

const SET_HUB_DEPTH: u8 = 12;
const PORT_RESET_ATTEMPTS: usize = 5;
// Linux hub_port_reset: TRSTRCY plus its ordinary 40-ms device margin.
const SS_RESET_RECOVERY_US: u64 = 50_000;
const HUB_EXT_PORT_STATUS: u16 = 2;
const BH_PORT_RESET: u16 = 28;
const CONNECTION: u16 = 1;
const ENABLE: u16 = 1 << 1;
const OVER_CURRENT: u16 = 1 << 3;
const RESET: u16 = 1 << 4;
const LINK_STATE: u16 = 15 << 5;
const POWER: u16 = 1 << 9;
const SPEED: u16 = 7 << 10;
const U0: u16 = 0;
const U3: u16 = 3 << 5;
const INACTIVE: u16 = 6 << 5;
const COMPLIANCE: u16 = 10 << 5;
const CHANGE_CONNECTION: u16 = 1;
const CHANGE_OVER_CURRENT: u16 = 1 << 3;
const CHANGE_CONFIG_ERROR: u16 = 1 << 7;
const CHANGES: &[(u16, u16)] = &[
    (1, 16),
    (1 << 3, 19),
    (1 << 4, 20),
    (1 << 5, 29),
    (1 << 6, 25),
    (1 << 7, 26),
];

pub(super) fn depth_request(depth: u8) -> [u8; 8] {
    setup(0x20, SET_HUB_DEPTH, u16::from(depth), 0, 0)
}
pub(super) fn status_request(port: u8, extended: bool) -> [u8; 8] {
    setup(
        0xa3,
        GET_STATUS,
        if extended { HUB_EXT_PORT_STATUS } else { 0 },
        u16::from(port),
        if extended { 8 } else { 4 },
    )
}

#[derive(Clone, Copy)]
pub(super) struct Port {
    status: u16,
    change: u16,
    extended: Option<u32>,
}
impl Port {
    pub fn decode(bytes: &[u8], extended: bool) -> Result<Self, BlockError> {
        if bytes.len() != if extended { 8 } else { 4 } {
            return Err(BlockError::Unsupported("short SuperSpeed hub port status"));
        }
        let status = u16::from_le_bytes([bytes[0], bytes[1]]);
        let change = u16::from_le_bytes([bytes[2], bytes[3]]);
        if status & !0x1ffb != 0
            || change & !0xf9 != 0
            || status & SPEED != 0
            || status & LINK_STATE > COMPLIANCE
        {
            return Err(BlockError::Unsupported(
                "reserved SuperSpeed hub port status",
            ));
        }
        Ok(Self {
            status,
            change,
            extended: extended.then(|| u32::from_le_bytes(bytes[4..8].try_into().unwrap())),
        })
    }
    pub fn warm(self) -> bool {
        matches!(self.status & LINK_STATE, INACTIVE | COMPLIANCE)
    }
    fn powered(self) -> Result<(), BlockError> {
        if self.status & POWER == 0 || self.status & OVER_CURRENT != 0 {
            Err(BlockError::Unsupported(
                "hub port power absent or over-current",
            ))
        } else {
            Ok(())
        }
    }
    fn ready(self) -> Result<(), BlockError> {
        self.powered()?;
        if self.status & (CONNECTION | ENABLE | RESET | LINK_STATE) != CONNECTION | ENABLE | U0
            || self.change != 0
        {
            return Err(BlockError::Unsupported(
                "hub port has no stable enabled U0 link",
            ));
        }
        Ok(())
    }
}

impl Controller {
    pub(in super::super) async fn superspeed_hub_with(
        a: &Access<'_>,
        device: &Device,
        attachment: Attachment,
        devices: &mut Vec<Device>,
        failures: &mut Vec<(String, BlockError)>,
    ) -> Result<(), BlockError> {
        if a.with(|c| {
            Ok::<_, BlockError>(
                c.validate_topology_id(device.id)? != device.slot
                    || c.slot(device.slot)?.attachment != attachment,
            )
        })? {
            return Err(BlockError::Unsupported(
                "USB SuperSpeed hub original attachment changed",
            ));
        }
        topology(attachment)?;
        let interfaces = interfaces(&device.configuration);
        let mut eligible = interfaces
            .iter()
            .filter_map(|interface| status_endpoint(device, interface));
        let endpoint = eligible.next().ok_or(BlockError::Unsupported(
            "SuperSpeed hub has no status endpoint",
        ))?;
        if eligible.next().is_some() {
            return Err(BlockError::Unsupported(
                "ambiguous SuperSpeed hub interfaces",
            ));
        }
        let slot = device.slot;
        let bytes = a
            .control(
                slot,
                setup(
                    0xa0,
                    GET_DESCRIPTOR,
                    u16::from(SS_HUB_DESCRIPTOR) << 8,
                    0,
                    SS_HUB_BYTES as u16,
                ),
                &[],
            )
            .await?;
        let descriptor = Descriptor::new(&bytes, &endpoint)?;
        let ssp = if u16::from_le_bytes([device.descriptor[2], device.descriptor[3]]) >= 0x0310 {
            Self::hub_ssp_for_with(a, device.id).await?
        } else {
            None
        };
        if attachment.speed == SUPER_SPEED_PLUS && ssp.is_none() {
            return Err(BlockError::Unsupported(
                "SuperSpeedPlus hub requires actual BOS sublink capabilities",
            ));
        }
        Self::set_configuration_with(a, slot, device.configuration_value()).await?;
        // Device status describes the configuration the device acknowledged.
        let self_powered = Self::hub_power_source_with(a, slot).await?;
        a.with(|c| {
            c.install_hub_current(
                slot,
                descriptor.ports,
                descriptor.controller_ma,
                self_powered,
            )
        })?;
        Self::observe_hub_current_with(a, slot).await?;
        // SuperSpeed has no USB2 transaction translator or MTT/TT think time.
        Self::configure_with(a, slot, &[endpoint], Some((descriptor.ports, 0, false))).await?;
        a.control(slot, depth_request(attachment.tier), &[]).await?;
        let mut powered_ports = Vec::new();
        for port in 1..=descriptor.ports {
            if let Err(error) = a.with(|c| c.reserve_port_current(slot, port)) {
                failures.push((
                    format!("{}.{port} reserve initial USB current", device.location),
                    error,
                ));
                continue;
            }
            a.control(
                slot,
                setup(0x23, SET_FEATURE, PORT_POWER, u16::from(port), 0),
                &[],
            )
            .await?;
            powered_ports.push(port);
        }
        a.minimum_wait(descriptor.power_good_us).await?;
        Self::observe_hub_current_with(a, slot).await?;
        for port in powered_ports {
            let location = format!("{}.{port}", device.location);
            crate::working(&location);
            let reset = enumerate::Reset::SuperSpeed {
                hub: device.id,
                port,
                parent: attachment,
                ssp: ssp.clone(),
            };
            Box::pin(Self::connect_with(
                a,
                location,
                Some(device.id),
                reset,
                devices,
                failures,
            ))
            .await;
            if a.with(|c| c.live_slot(slot)).is_err() {
                // connect already records the power observation failure.
                // Keep the inventory as historical identity, never as a live handle.
                break;
            }
        }
        Ok(())
    }

    pub(in super::super) async fn hub_power_source_with(
        a: &Access<'_>,
        slot: u8,
    ) -> Result<bool, BlockError> {
        let bytes = a
            .control(slot, setup(0x80, GET_STATUS, 0, 0, 2), &[])
            .await?;
        if bytes.len() != 2 {
            return Err(BlockError::Unsupported("short USB hub power source status"));
        }
        Ok(bytes[0] & 1 != 0)
    }

    pub(in super::super) fn hub_ssp_for(&mut self, id: SlotId) -> Result<Option<Ssp>, BlockError> {
        block_on(Self::hub_ssp_for_with(&Access::blocking(self, None), id))
    }

    async fn hub_ssp_for_with(a: &Access<'_>, id: SlotId) -> Result<Option<Ssp>, BlockError> {
        let slot = a.with(|c| {
            c.check_minimum()?;
            c.validate_topology_id(id)
        })?;
        let header = Self::descriptor_with(a, slot, BOS_DESCRIPTOR, 0, 5).await?;
        if header.len() != 5 || header[0] != 5 || header[1] != BOS_DESCRIPTOR {
            return Err(BlockError::Unsupported("invalid hub BOS header"));
        }
        let total = u16::from_le_bytes([header[2], header[3]]);
        if !(5..=CONTROL_BYTES as u16).contains(&total) {
            return Err(BlockError::Unsupported("oversized hub BOS descriptor"));
        }
        let bytes = Self::descriptor_with(a, slot, BOS_DESCRIPTOR, 0, total).await?;
        if bytes.get(..5) != Some(header.as_slice()) {
            return Err(BlockError::Unsupported("hub BOS changed during read"));
        }
        Ssp::bos(&bytes)
    }

    async fn superspeed_status_with(
        a: &Access<'_>,
        slot: u8,
        port: u8,
        extended: bool,
    ) -> Result<Port, BlockError> {
        let bytes = a.control(slot, status_request(port, extended), &[]).await?;
        Port::decode(&bytes, extended)
    }

    async fn clear_superspeed_changes_with(
        a: &Access<'_>,
        slot: u8,
        port: u8,
        change: u16,
    ) -> Result<(), BlockError> {
        for &(bit, selector) in CHANGES {
            if change & bit != 0 {
                a.control(
                    slot,
                    setup(0x23, CLEAR_FEATURE, selector, u16::from(port), 0),
                    &[],
                )
                .await?;
            }
        }
        Ok(())
    }

    pub(in super::super) fn superspeed_port_for(
        &mut self,
        id: SlotId,
        port: u8,
        ssp: Option<&Ssp>,
    ) -> Result<Option<u8>, BlockError> {
        block_on(Self::superspeed_port_for_with(
            &Access::blocking(self, None),
            id,
            port,
            ssp,
        ))
    }

    pub(in super::super) async fn superspeed_port_for_with(
        a: &Access<'_>,
        id: SlotId,
        port: u8,
        ssp: Option<&Ssp>,
    ) -> Result<Option<u8>, BlockError> {
        let slot = a.with(|c| {
            c.check_minimum()?;
            c.validate_topology_id(id)?;
            c.check_minimum()?;
            c.validate_topology_id(id)
        })?;
        // Check the same stable connection interval as USB2/root ports.
        // Clearing CSC once is not evidence that the downstream plug settled.
        let mut stable_state = None;
        let connected = super::super::enumerate::debounce_with(a, async || {
            let state = Self::superspeed_status_with(a, slot, port, ssp.is_some()).await?;
            state.powered()?;
            let changed = state.change & CHANGE_CONNECTION != 0;
            Self::clear_superspeed_changes_with(a, slot, port, state.change).await?;
            stable_state = Some(state);
            // Linux `hub_port_warm_reset_required`: a link in SS.Inactive or
            // Compliance Mode holds a device only a warm reset brings up,
            // whether or not the port shows a connection.
            Ok((state.status & CONNECTION != 0 || state.warm(), changed))
        })
        .await;
        let connected = a.with(|c| c.minimum_status(connected))?;
        if !connected {
            return Ok(None);
        }
        let mut state =
            stable_state.ok_or(BlockError::Unsupported("hub port has no stable status"))?;
        if state.status & LINK_STATE == U3 {
            return Err(BlockError::Unsupported(
                "hub port resume requires a separate lifecycle owner",
            ));
        }
        let mut warm = state.warm();
        // A failed hot reset can require a warm reset. Bound retries and never
        // admit a child on completion bits alone or on an upstream speed guess.
        for _ in 0..PORT_RESET_ATTEMPTS {
            let selector = if warm { BH_PORT_RESET } else { PORT_RESET };
            a.control(
                slot,
                setup(0x23, SET_FEATURE, selector, u16::from(port), 0),
                &[],
            )
            .await?;
            let deadline = a.with(|c| c.clock.deadline(PORT_RESET_US));
            loop {
                a.delay(1000).await;
                state = Self::superspeed_status_with(a, slot, port, ssp.is_some()).await?;
                state.powered()?;
                // SS connection is unknown during reset, and some devices
                // reconnect only after RESET falls. Do not interpret one CCS=0
                // sample or reset-induced CSC as a physical removal.
                if state.change & (CHANGE_OVER_CURRENT | CHANGE_CONFIG_ERROR) != 0 {
                    return Err(BlockError::Unsupported(
                        "hub port changed power/configuration during reset",
                    ));
                }
                if state.status & RESET == 0 && state.status & CONNECTION != 0 && state.warm() {
                    break;
                }
                if state.status & RESET == 0 && state.status & CONNECTION != 0 {
                    Self::clear_superspeed_changes_with(a, slot, port, state.change).await?;
                    a.minimum_wait(SS_RESET_RECOVERY_US).await?;
                    let mut final_state =
                        Self::superspeed_status_with(a, slot, port, ssp.is_some()).await?;
                    if final_state.change == 1 << 6 && final_state.status & LINK_STATE == U0 {
                        Self::clear_superspeed_changes_with(a, slot, port, final_state.change)
                            .await?;
                        final_state =
                            Self::superspeed_status_with(a, slot, port, ssp.is_some()).await?;
                    }
                    final_state.powered()?;
                    if final_state.change & (CHANGE_OVER_CURRENT | CHANGE_CONFIG_ERROR) != 0 {
                        return Err(BlockError::Unsupported(
                            "hub port power/configuration changed after reset",
                        ));
                    }
                    if final_state.ready().is_err() {
                        // Linux retries an acknowledged reset that completes
                        // disabled. Keep the retry budget for ordinary hot
                        // failures too; only Inactive/Compliance selects warm.
                        state = final_state;
                        break;
                    }
                    a.with(|c| {
                        let root = c.slot(slot)?.attachment.root_port;
                        c.current_hub_port_reset(id, port, root)
                    })?;
                    return match ssp {
                        Some(ssp) => ssp.speed(final_state.extended.unwrap()).map(Some),
                        None => Ok(Some(SUPER_SPEED)),
                    };
                }
                if deadline.passed() {
                    if state.status & RESET == 0 {
                        if state.warm() {
                            break;
                        }
                        if state.status & CONNECTION == 0 {
                            Self::clear_superspeed_changes_with(a, slot, port, state.change)
                                .await?;
                            return Ok(None);
                        }
                    }
                    return Err(BlockError::Timeout);
                }
            }
            warm |= state.warm();
            Self::clear_superspeed_changes_with(a, slot, port, state.change).await?;
        }
        Err(BlockError::Unsupported(
            "hub port failed bounded hot/warm resets",
        ))
    }
}
