// SPDX-License-Identifier: BSD-3-Clause-Clear
// Copyright (C) 2020-2023 MediaTek Inc.
//! One claimed MT7925 function taken from its registers to a managed station
//! link: `mt7925_pci_probe` identifies and boots it, `mt7925_regd_update`
//! programs its country, `mt7925_mac_init` and `mt7925_start` bring up its
//! MAC on mac80211's default channel, and `mt7925_add_interface` registers
//! its station interface, which a [`Station`] then keeps connected.
use super::{
    acpi::Sar,
    bootstrap::{
        self, Controller,
        client::{
            self, Client, Environment, LinkControlError, ManagedClient, ManagedLink, PolicyError,
        },
    },
    firmware::{
        LoadError, Requests,
        clc::{Environment as ClcEnvironment, Inputs as ClcInputs},
    },
    identity::{Chip, PciIdentity},
    mcu::device::Context,
    station::{Station, Wifi},
};
use crate::{
    Clock, Mac,
    bus::dma::{mapping::DmaDomain, memory::PageSource},
    firmware::Provider,
    wifi::{
        network::Network,
        regulatory::database::{self, Database},
    },
};
use alloc::sync::Arc;
use core::{any::Any, fmt, ptr::NonNull};
use stela_driver_base::entropy::Entropy;

/// The services and platform facts the attach borrows.
pub struct Inputs {
    pub firmware: &'static dyn Provider,
    pub pages: &'static dyn PageSource,
    pub clock: Clock,
    /// Whether `owner`'s function raises an MSI the caller has routed.
    pub vector: bool,
    /// `mt76_pci_aspm_supported`: whether the function's link enables ASPM.
    pub aspm: bool,
    /// regulatory.db, whose country the network names, or the world domain
    /// `00` cfg80211 starts from when it names none.
    pub database: Database,
    pub network: Network,
    pub sar: Sar,
    /// The source of the station's handshake nonces.
    pub entropy: Arc<dyn Entropy>,
}

/// Why the function did not reach a managed link. Every owner the device may
/// still reach stays held.
#[derive(Debug)]
pub enum AttachError {
    /// Only MT7925's regulatory programming is ported.
    UnsupportedChip(PciIdentity),
    /// No regulatory database loaded, so the radio may not transmit.
    NoRegulatoryDatabase,
    /// regulatory.db holds no usable rules for the network's country.
    Country([u8; 2], database::Error),
    Firmware(LoadError),
    Bootstrap(bootstrap::Error),
    /// The firmware reports no valid station address and no entropy gives a
    /// random one.
    NoAddress,
    /// The country enables no channel the radio may start on.
    NoChannel,
    Client(client::Error),
    Policy(PolicyError),
    Link(LinkControlError),
}

impl From<bootstrap::Error> for AttachError {
    fn from(error: bootstrap::Error) -> Self {
        Self::Bootstrap(error)
    }
}

impl fmt::Display for AttachError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Firmware(error) => write!(f, "{error}"),
            Self::Bootstrap(error) => write!(f, "{error}"),
            Self::Link(error) => write!(f, "{error}"),
            Self::NoRegulatoryDatabase => f.write_str("MT7925: no regulatory database"),
            Self::Country(alpha2, error) => write!(
                f,
                "MT7925: regulatory.db country {}: {error:?}",
                alpha2.escape_ascii()
            ),
            error => write!(f, "MT7925 attach: {error:?}"),
        }
    }
}

/// A kept station connection and what limits it without stopping it.
pub struct Attached<C: 'static> {
    pub station: Wifi<C>,
    /// The firmware offers its own regulatory domain
    /// (`MT7925_REGD_SUPPORTED`). The link programs and verifies
    /// regulatory.db's country instead, so the firmware's stays unused.
    pub firmware_domain_unused: bool,
    /// The firmware reported no valid station address, so the station took
    /// a random locally administered one, as `mt76_eeprom_override` does.
    pub random_address: bool,
}

/// `mt76_eeprom_override`: the firmware's station address when it is a
/// valid unicast address, else `eth_random_addr`'s random locally
/// administered unicast address; `None` only when no entropy is available.
fn station_address(reported: Option<Mac>, entropy: &dyn Entropy) -> Option<(Mac, bool)> {
    if let Some(mac) = reported.filter(|mac| !mac.is_multicast() && mac.0 != [0; 6]) {
        return Some((mac, false));
    }
    let mut bytes = [0; 6];
    entropy.fill(&mut bytes).ok()?;
    bytes[0] = (bytes[0] & 0xfe) | 0x02;
    Some((Mac(bytes), true))
}

/// Identify, boot and program the function `pci` names, then register its
/// station interface and keep it connected to `inputs.network`.
///
/// # Safety
/// [`Controller::new`]'s and [`Controller::install`]'s contracts hold for
/// `base..base + bytes`, `resource` and `domain`, with `inputs.pages` as the
/// page source: `resource` keeps the claimed function and its uncached BAR0
/// mapping, whose interrupts are off.
pub unsafe fn attach<C: 'static>(
    pci: PciIdentity,
    base: NonNull<u8>,
    bytes: usize,
    resource: Arc<C>,
    domain: Arc<dyn DmaDomain>,
    inputs: Inputs,
) -> Result<Attached<C>, AttachError> {
    if pci.chip() != Some(Chip::Mt7925) {
        return Err(AttachError::UnsupportedChip(pci));
    }
    let alpha2 = inputs.network.country.unwrap_or(*b"00");
    let country = inputs
        .database
        .select(alpha2)
        .map_err(|error| AttachError::Country(alpha2, error))?;
    let owner: Arc<dyn Any> = resource.clone();
    // SAFETY: forwarded from the caller.
    let identity =
        unsafe { bootstrap::identify(pci, base, bytes, owner.clone(), inputs.clock, inputs.aspm) }?;
    // Both images load before the controller claims the function.
    let prepared = Requests::new(&identity)
        .load(inputs.firmware)
        .map_err(AttachError::Firmware)?;
    // SAFETY: forwarded from the caller.
    let mut controller =
        unsafe { Controller::new(identity, base, bytes, owner, resource, inputs.clock) }?;
    // SAFETY: forwarded from the caller.
    unsafe { controller.install(inputs.pages, domain, inputs.aspm) }?;
    controller.boot(&prepared)?;
    let chip_capabilities = controller
        .capabilities()
        .and_then(|report| report.observed().chip_flags())
        .unwrap_or(0);
    // SAFETY: `resource` keeps the function's policy ownership. Connac3's
    // request carries none of Connac2's ACPI, power-node or MTCL fields, and
    // the CLC's environment is the OS's `country_ie_env`, ENVIRON_ANY.
    unsafe {
        controller.execute_clc(ClcInputs {
            alpha2,
            environment: ClcEnvironment::Any,
            chip_capabilities,
            power_limits_node: false,
            power_type: 0,
            acpi_flags: 0,
            mtcl_configuration: 0,
            world_mtcl_configuration: 0,
        })
    }?;
    let plan = controller.regulatory_plan(&country, &inputs.sar)?;
    let firmware_domain_unused = controller.firmware_regulatory_domain();
    // SAFETY: the plan resolves the CLC's restrictions, the country and the
    // platform's SAR, and `resource` keeps them.
    unsafe { controller.execute_remaining(plan.inputs()) }?;
    // SAFETY: no peer, WTBL or key user exists yet.
    unsafe { controller.initialize_mac() }?;
    let channel = plan.default_channel().ok_or(AttachError::NoChannel)?;
    // SAFETY: the programmed domain holds `channel`.
    unsafe { controller.start(channel.band, channel.number as u8, channel.frequency_mhz) }?;
    let (address, random_address) = station_address(
        controller
            .capabilities()
            .and_then(|report| report.observed().mac()),
        &*inputs.entropy,
    )
    .ok_or(AttachError::NoAddress)?;
    // `mt7925_add_interface`'s first interface: BSS, OMAC and WMM 0, no band
    // yet, and the broadcast WCID `MT792x_WTBL_RESERVED - 0`.
    let context = Context {
        bss_index: 0,
        omac_index: 0,
        band_index: 0xff,
        wmm_index: 0,
        link_index: 0,
        broadcast_wcid: 19,
        address,
    };
    // SAFETY: the first interface of a fresh controller.
    let interface = unsafe { controller.register_interface(context) }?;
    // SAFETY: the controller and interface are this requester's, after its
    // completed bootstrap and policy; `resource` keeps their permission.
    let client = unsafe { Client::new(controller, interface) }.map_err(AttachError::Client)?;
    // cfg80211 applies INDOOR_ONLY only to GO/AP initiation, never to a
    // managed station following its AP (chan.c cfg80211_ir_permissive_chan),
    // so a station on an indoor-only channel joins as on Linux. The model
    // tests of the managed link run under the same environment.
    let managed = ManagedClient::new(client, country.clone(), Environment::Indoor)
        .map_err(|error| AttachError::Policy(error.error))?;
    let link =
        ManagedLink::new(managed, inputs.pages).map_err(|error| AttachError::Link(error.error))?;
    let link = if inputs.vector {
        link.with_vector()
    } else {
        link
    };
    Ok(Attached {
        station: Station::new(
            link,
            context,
            country,
            inputs.network,
            inputs.entropy,
            inputs.clock,
        ),
        firmware_domain_unused,
        random_address,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use stela_driver_base::entropy::EntropyError;

    struct Fixed(u8);
    impl Entropy for Fixed {
        fn fill(&self, output: &mut [u8]) -> Result<(), EntropyError> {
            output.fill(self.0);
            Ok(())
        }
    }
    struct Empty;
    impl Entropy for Empty {
        fn fill(&self, _: &mut [u8]) -> Result<(), EntropyError> {
            Err(EntropyError::Unavailable)
        }
    }

    #[test]
    fn an_invalid_or_missing_address_becomes_a_random_local_one_as_mt76_does() {
        let good = Mac([0x00, 0x0a, 0xf5, 1, 2, 3]);
        assert_eq!(
            station_address(Some(good), &Fixed(0xff)),
            Some((good, false))
        );
        for reported in [None, Some(Mac([0; 6])), Some(Mac([0x01, 0, 0, 0, 0, 1]))] {
            let (mac, random) = station_address(reported, &Fixed(0xff)).unwrap();
            assert!(random);
            assert_eq!(mac.0, [0xfe, 0xff, 0xff, 0xff, 0xff, 0xff]);
            assert!(!mac.is_multicast() && mac.0[0] & 2 != 0);
        }
        assert_eq!(station_address(None, &Empty), None);
    }
}
