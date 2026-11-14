//! Binding each interface of a configured device to its class driver.
use super::*;

/// The advertised audio alternates and controls a bound function leaves
/// out, one unused entry each, so the boot reports every one once.
fn audio_unsupported<'a>(
    location: &'a str,
    unsupported: &'a [(u8, u8, &'static str)],
) -> impl Iterator<Item = (String, &'static str)> + 'a {
    unsupported.iter().map(move |&(interface, alternate, why)| {
        (
            format!("{location} interface {interface} alternate {alternate}"),
            why,
        )
    })
}

/// Binds every interface of `device` a driver here takes: one
/// configuration, its endpoints configured together. A Bulk-Only disk is
/// discovered after its settle time, waited for here.
#[cfg(test)]
pub(crate) fn bind(shared: &Shared, device: Device, dma: &mut dyn DmaAllocator, found: &mut Found) {
    bind_with_provider(
        shared,
        device,
        dma,
        found,
        &crate::firmware::HandoffProvider::new(None),
    );
    if !found.settling.is_empty() && crate::test_clock::read().is_some() {
        crate::test_clock::advance(shared.borrow().clock, settle::SETTLE_US);
    }
    found.settle(dma);
}

#[expect(
    clippy::result_large_err,
    reason = "The shared error preserves exact source and cleanup causes without allocating on propagation."
)]
pub(crate) fn bind_with_provider(
    shared: &Shared,
    device: Device,
    dma: &mut dyn DmaAllocator,
    found: &mut Found,
    firmware: &dyn crate::firmware::Provider,
) {
    if device.slot != device.id.hardware_slot() {
        found.failures.push((
            device.location.clone(),
            BlockError::Unsupported("USB bind descriptor slot identity mismatch"),
        ));
        return;
    }
    let all = interfaces(&device.configuration);
    let mut endpoints = Vec::new();
    let mut plans = Vec::new();
    let mut selected_settings = Vec::new();
    for interface in all.iter().filter(|i| configuration::primary(i)) {
        let plan =
            configuration::plan_with_controller(&device, interface, &all, Some(&shared.borrow()));
        match plan {
            Ok(plan) => {
                selected_settings.extend(plan.settings(interface.number));
                if !matches!(&plan, Plan::Storage(binding) if binding.uas()) {
                    endpoints.extend(plan.endpoints().iter().copied());
                }
                plans.push((interface.clone(), plan));
            }
            Err(why) => found.unused.push((
                format!(
                    "{} interface {} ({:02x}/{:02x}/{:02x})",
                    device.location,
                    interface.number,
                    interface.class,
                    interface.subclass,
                    interface.protocol
                ),
                why,
            )),
        }
    }
    if plans.is_empty() {
        return;
    }
    // Revalidate the plans immediately before SET_CONFIGURATION. A
    // controller error since inventory selection may now force UAS to BOT.
    if let Err(reason) = configuration::active_settings(&all, &selected_settings) {
        found
            .failures
            .push((device.location.clone(), BlockError::Unsupported(reason)));
        return;
    }
    if let Err(error) = shared.borrow_mut().claim_native(device.id) {
        found.failures.push((device.location.clone(), error));
        return;
    }
    let configured = {
        let mut controller = shared.borrow_mut();
        let value = device.configuration_value();
        during(
            format!("slot {} EP0 SET_CONFIGURATION value={value}", device.slot),
            controller
                .device(device.id)
                .and_then(|mut access| access.set_configuration(value)),
        )
        .and_then(|()| {
            if endpoints.is_empty() {
                Ok(())
            } else {
                during(
                    format!("slot {} Configure Endpoint", device.slot),
                    controller
                        .device(device.id)
                        .and_then(|mut access| access.configure(&endpoints, None, dma)),
                )
            }
        })
    };
    if let Err((operation, error)) = configured {
        found.failures.push((
            format!("{} {} {operation}", device.location, device.name()),
            error,
        ));
        return;
    }
    let configured = settle::configured(shared.borrow().clock);
    let mut hid_inputs = Vec::new();
    for (interface, plan) in plans {
        let location = format!("{} interface {}", device.location, interface.number);
        crate::working(&location);
        let result = match plan {
            // UAS has no settle time.
            Plan::Storage(binding) if binding.uas() => storage::discover_binding(
                shared.clone(),
                &device,
                binding,
                dma,
                &mut found.failures,
            )
            .map(|(owner, disks)| {
                found.storage.push(owner);
                found.disks.extend(disks);
            }),
            Plan::Storage(binding) => {
                let interface = settle::Configured::Xhci {
                    shared: shared.clone(),
                    device: device.clone(),
                    binding,
                };
                let entry = settle::Settling::new(configured, location.clone(), interface);
                found.settling.push(entry);
                Ok(())
            }
            Plan::Hid(endpoint, _) => {
                hid::UsbInput::new(shared.clone(), &device, &interface, endpoint, dma).map(
                    |input| match input {
                        Some(input) => hid_inputs.push(input),
                        None => found.unused.push((location.clone(), hid::NO_REPORTS)),
                    },
                )
            }
            Plan::Ethernet(plan) => cdc::UsbNet::new(shared.clone(), &device, plan, dma)
                .map(|link| found.links.push(link)),
            Plan::VendorEthernet(binding) => {
                match ethernet::UsbEthernet::new(shared.clone(), &device, binding, dma, firmware) {
                    Ok(link) => found.vendor_links.push(link),
                    Err(error) => found.vendor_failures.push((location.clone(), error)),
                }
                Ok(())
            }
            Plan::Audio(binding) => audio::UsbAudio::attach(shared.clone(), &device, binding, dma)
                .map(|owner| {
                    found.unused.extend(audio_unsupported(
                        &device.location,
                        &owner.binding().unsupported,
                    ));
                    found.audio.push(owner);
                }),
            Plan::BluetoothBootloader(binding) => {
                bluetooth::ath3k::UsbBootloader::attach(shared.clone(), binding, dma).map(
                    |mut owner| {
                        let _ = owner.initialize_with_provider(firmware);
                        found.bootloaders.push(owner);
                    },
                )
            }
            Plan::Bluetooth(binding) => {
                bluetooth::UsbHci::attach(shared.clone(), &device, binding, dma)
                    .map(|transport| found.bluetooth.push(transport))
            }
        };
        if let Err(error) = result {
            found.failures.push((location, error));
        }
    }
    if let Err(error) = hid::receiver::transport::attach_group(shared, &device, &mut hid_inputs) {
        found
            .failures
            .push((format!("{} HID++ receiver", device.location), error));
    }
    found.inputs.extend(hid_inputs);
}

/// What a class driver will use of an interface.
pub enum Plan {
    /// Audio function plan, initially at zero-bandwidth streaming alternates.
    Audio(audio::Plan),
    /// Bulk IN, bulk OUT.
    Storage(storage::Binding),
    /// The interrupt IN endpoint and any interrupt OUT endpoint.
    Hid(Endpoint, Option<Endpoint>),
    /// A CDC Ethernet adapter's interfaces and endpoints.
    Ethernet(cdc::Binding),
    /// A vendor Ethernet candidate whose constructor validates chip startup.
    VendorEthernet(ethernet::Binding),
    /// Legacy Bluetooth HCI interrupt and bulk endpoints.
    Bluetooth(bluetooth::Binding),
    /// Pre-HCI raw firmware downloader, terminal until real reenumeration.
    BluetoothBootloader(bluetooth::ath3k::Binding),
}

impl Plan {
    pub(super) fn settings(&self, interface: u8) -> Vec<(u8, u8)> {
        match self {
            Self::Audio(binding) => binding.settings(),
            Self::Storage(binding) => vec![binding.setting()],
            Self::VendorEthernet(binding) => vec![binding.setting()],
            Self::Ethernet(binding) => vec![(interface, 0), binding.data_setting()],
            _ => vec![(interface, 0)],
        }
    }
    pub(super) fn endpoints(&self) -> Vec<Endpoint> {
        match self {
            Self::Audio(_) => Vec::new(),
            Self::Storage(binding) => binding.endpoints(),
            Self::Hid(input, output) => [Some(*input), *output].into_iter().flatten().collect(),
            Self::Ethernet(binding) => binding.endpoints(),
            Self::VendorEthernet(binding) => binding.endpoints(),
            Self::Bluetooth(binding) => binding.endpoints(),
            Self::BluetoothBootloader(binding) => binding.endpoints(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn each_left_out_audio_alternate_is_one_unused_entry() {
        let unsupported = [
            (6, 11, "IEC 61937 Type III alternate"),
            (0, 0, "unit controls are not exposed"),
        ];
        let entries: Vec<_> =
            audio_unsupported("xhci 0000:0a:00.0 port 11", &unsupported).collect();
        assert_eq!(
            entries,
            [
                (
                    String::from("xhci 0000:0a:00.0 port 11 interface 6 alternate 11"),
                    "IEC 61937 Type III alternate"
                ),
                (
                    String::from("xhci 0000:0a:00.0 port 11 interface 0 alternate 0"),
                    "unit controls are not exposed"
                ),
            ]
        );
        assert_eq!(audio_unsupported("x", &[]).count(), 0);
    }
}
