//! `stela settings` without options on a terminal: ask for the Wi-Fi and
//! the network addresses an install that downloads macOS uses, starting
//! from what the stick has, and change only what the answers change.
use super::{Environment, Record, Request, addresses, ui, wizard};
use anyhow::Result;

/// Whether `request` names any setting to change.
pub(super) fn named(request: &Request) -> bool {
    request.wifi.is_some()
        || request.no_wifi
        || request.ipv4.is_some()
        || request.automatic_ipv4
        || request.dns.is_some()
        || request.automatic_dns
        || !request.usb4_approve.is_empty()
        || request.no_usb4_approve
        || request.default_install.is_some()
        || request.remove_install.is_some()
        || request.chooser_timeout.is_some()
        || request.profile
        || request.no_profile
}

/// Ask for the settings of `record`'s stick into `request`: `false` is a
/// no, and nothing changes.
pub(super) fn ask(
    environment: &Environment<'_>,
    request: &mut Request,
    record: Option<&Record>,
) -> Result<bool> {
    let ui = environment.ui;
    // What the stick has now, which the questions start from.
    let had_wifi = record.and_then(|r| r.wifi.as_ref()).map(|w| w.ssid.clone());
    let had = record.map(|r| r.addresses.clone()).unwrap_or_default();
    request.wifi = had_wifi.clone();
    request.ipv4 = had.ipv4.map(|ipv4| stela_boot_core::options::StaticIpv4 {
        address: ipv4.address,
        prefix: ipv4.prefix,
    });
    request.ipv4_gateway = had.ipv4.and_then(|ipv4| ipv4.gateway);
    request.dns = (!had.dns.is_empty()).then(|| had.dns.clone());
    let result = (|| -> Result<bool> {
        wizard::choose_wifi(environment, request)?;
        addresses::choose(ui, request)?;
        let rows = ui::rows(&[("Save", ""), ("Quit", "")]);
        Ok(ui.select("Save these settings?", &rows, 0)? == 0)
    })();
    let save = match result {
        Err(error) if ui::cancelled(&error) => false,
        result => result?,
    };
    if !save {
        ui.cancel("Nothing was changed.")?;
        return Ok(false);
    }
    // Choosing no Wi-Fi removes the network the stick had.
    request.no_wifi = request.wifi.is_none() && had_wifi.is_some();
    request.automatic_ipv4 = request.ipv4.is_none() && had.ipv4.is_some();
    request.automatic_dns = request.dns.is_none() && !had.dns.is_empty();
    Ok(true)
}
