//! A device the controller addressed, and its endpoints' xHCI contexts.
use super::*;

/// xHCI views of an endpoint.
impl Endpoint {
    pub(in crate::usb) fn validate_context(&self, speed: u8) -> Result<(), BlockError> {
        endpoint::Context::new(self, speed).map(|_| ())
    }
    /// The endpoint's Device Context Index.
    pub fn dci(&self) -> u8 {
        let number = self.address & 0xf;
        2 * number + u8::from(self.address & 0x80 != 0)
    }
}

/// A device the controller addressed, with what it said about itself.
#[derive(Clone, Debug)]
pub struct Device {
    /// Complete original PLA_TCR0 read; matching uses the donor's normalized
    /// version mask below while diagnostics keep every observed register bit.
    pub vendor_ethernet_tcr: Option<u32>,
    /// Read-only vendor Ethernet chip identity observed before configuration
    /// selection, matching Linux r8152's cfgselector preflight. None means no
    /// verified identity; product IDs alone never select a vendor backend.
    pub vendor_ethernet_revision: Option<u16>,
    /// Opaque ownership of this exact addressed device, including slot generation.
    pub id: super::SlotId,
    /// Hardware slot number for diagnostics; never an ownership credential.
    pub slot: u8,
    pub speed: u8,
    /// Where it is: the controller's PCI function, root port and hub ports.
    pub location: String,
    pub descriptor: [u8; 18],
    /// Every complete configuration descriptor in GET_DESCRIPTOR index order.
    /// The descriptor's bConfigurationValue is independent of that index.
    pub configurations: Vec<Vec<u8>>,
    /// Selected configuration bytes; selection alone does not configure hardware.
    pub configuration: Vec<u8>,
    pub manufacturer: Option<String>,
    pub product: Option<String>,
    pub serial: Option<String>,
    /// The language of the device's strings, when it has any.
    pub language: Option<u16>,
}

impl Device {
    pub fn configuration_value(&self) -> u8 {
        self.configuration.get(5).copied().unwrap_or(0)
    }
    /// A name for logs: the product string, or the IDs.
    pub fn name(&self) -> String {
        match self
            .product
            .as_deref()
            .map(str::trim)
            .filter(|name| !name.is_empty())
        {
            Some(product) => String::from(product),
            None => self.ids(),
        }
    }
    /// Whether any configuration offers the phone tethering interface
    /// Linux `ipheth` drives: Apple's vendor ID with the vendor-specific
    /// class ff, subclass fd, protocol 1. No driver here takes it
    /// ([`super::bind::TETHERING`]).
    pub fn offers_tethering(&self) -> bool {
        const APPLE: u16 = 0x05ac;
        u16::from_le_bytes([self.descriptor[8], self.descriptor[9]]) == APPLE
            && self.configurations.iter().any(|configuration| {
                interfaces(configuration).iter().any(|interface| {
                    (interface.class, interface.subclass, interface.protocol) == (0xff, 0xfd, 1)
                })
            })
    }
    /// The vendor and product IDs, as `vvvv:pppp`.
    fn ids(&self) -> String {
        format!(
            "{:04x}:{:04x}",
            u16::from_le_bytes([self.descriptor[8], self.descriptor[9]]),
            u16::from_le_bytes([self.descriptor[10], self.descriptor[11]])
        )
    }
    /// One line for the account of a failed disk search: where it is, its
    /// speed, IDs, product, and the class, subclass and protocol of each
    /// interface of its selected configuration, so a USB disk Stela did not
    /// take (UAS 08/06/62, bulk-only 08/06/50, a hub 09) can be told apart.
    /// No serial number.
    pub fn summary(&self) -> String {
        let speed = match self.speed {
            LOW_SPEED => "low speed",
            FULL_SPEED => "full speed",
            HIGH_SPEED => "high speed",
            SUPER_SPEED => "SuperSpeed",
            SUPER_SPEED_PLUS => "SuperSpeedPlus",
            _ => "unknown speed",
        };
        let mut line = format!("{}: {speed} {}", self.location, self.ids());
        if let Some(product) = self.product.as_deref().map(str::trim)
            && !product.is_empty()
        {
            line.push_str(&format!(" {product:?}"));
        }
        let mut classes: Vec<(u8, u8, u8)> = Vec::new();
        for interface in interfaces(&self.configuration) {
            let class = (interface.class, interface.subclass, interface.protocol);
            if !classes.contains(&class) {
                classes.push(class);
            }
        }
        if classes.is_empty() {
            line.push_str(&format!(" class {:02x}", self.descriptor[4]));
        } else {
            line.push_str(" interfaces");
            for (class, subclass, protocol) in classes {
                line.push_str(&format!(" {class:02x}/{subclass:02x}/{protocol:02x}"));
            }
        }
        line
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bus::dma::TestRegion;
    use alloc::collections::BTreeMap;

    #[test]
    fn configuration_descriptors_split_into_interfaces_with_their_endpoints() {
        let configuration = [
            9, 2, 32, 0, 1, 1, 0, 0x80, 50, // configuration
            9, 4, 0, 0, 2, 8, 6, 0x50, 0, // interface
            7, 5, 0x81, 2, 0, 2, 0, // bulk in
            7, 5, 0x02, 2, 0, 2, 0, // bulk out
        ];
        let found = interfaces(&configuration);
        assert_eq!(found.len(), 1);
        assert_eq!(
            (found[0].class, found[0].subclass, found[0].protocol),
            (8, 6, 0x50)
        );
        assert_eq!(found[0].endpoints[0].dci(), 3);
        assert_eq!(found[0].endpoints[1].dci(), 4);
        assert_eq!(found[0].endpoints[0].max_packet, 512);
    }

    /// The account of a failed disk search names each USB device by place,
    /// speed, IDs, product and interface classes, never by serial.
    #[test]
    fn a_device_summary_names_speed_ids_product_and_interface_classes() {
        let nodes = BTreeMap::from([(0, hub::model::Node::leaf(SUPER_SPEED))]);
        let mut dma = TestRegion::new(256 * 1024 * 8);
        let (mut controller, _) = hub::tests::controller(&mut dma, 32, nodes);
        let mut failures = Vec::new();
        let mut device = controller.enumerate(&mut dma, &mut failures).remove(0);
        device.location = String::from("00:14.0 port 3");
        device.descriptor[8..12].copy_from_slice(&[0x81, 0x07, 0x81, 0x55]);
        device.product = Some(String::from(" Flash Drive "));
        device.serial = Some(String::from("SERIAL123"));
        device.configuration = vec![
            9, 2, 41, 0, 1, 1, 0, 0x80, 50, //
            9, 4, 0, 0, 2, 8, 6, 0x50, 0, //
            7, 5, 0x81, 2, 0, 4, 0, //
            7, 5, 0x02, 2, 0, 4, 0, //
            9, 4, 0, 1, 2, 8, 6, 0x62, 0,
        ];
        assert_eq!(
            device.summary(),
            "00:14.0 port 3: SuperSpeed 0781:5581 \"Flash Drive\" interfaces 08/06/50 08/06/62"
        );
        device.product = None;
        device.configuration.clear();
        device.descriptor[4] = 9;
        device.speed = HIGH_SPEED;
        assert_eq!(
            device.summary(),
            "00:14.0 port 3: high speed 0781:5581 class 09"
        );
    }

    /// A phone's tethering interface is found in whichever configuration
    /// offers it, with the vendor and class triple Linux ipheth matches.
    #[test]
    fn a_phone_tethering_interface_is_found_in_any_configuration() {
        let nodes = BTreeMap::from([(0, hub::model::Node::leaf(HIGH_SPEED))]);
        let mut dma = TestRegion::new(256 * 1024 * 8);
        let (mut controller, _) = hub::tests::controller(&mut dma, 32, nodes);
        let mut failures = Vec::new();
        let mut device = controller.enumerate(&mut dma, &mut failures).remove(0);
        let still_image = vec![
            9, 2, 18, 0, 1, 1, 0, 0x80, 50, //
            9, 4, 0, 0, 0, 6, 1, 1, 0,
        ];
        let tethering = vec![
            9, 2, 27, 0, 2, 4, 0, 0x80, 50, //
            9, 4, 0, 0, 0, 6, 1, 1, 0, //
            9, 4, 1, 0, 0, 0xff, 0xfd, 1, 0,
        ];
        device.descriptor[8..10].copy_from_slice(&0x05acu16.to_le_bytes());
        device.configurations = vec![still_image.clone()];
        device.configuration = still_image.clone();
        assert!(!device.offers_tethering());
        device.configurations.push(tethering.clone());
        assert!(device.offers_tethering(), "a later configuration");
        // Another vendor's vendor-specific triple is not this interface.
        device.descriptor[8..10].copy_from_slice(&0x1234u16.to_le_bytes());
        assert!(!device.offers_tethering());
    }
}
