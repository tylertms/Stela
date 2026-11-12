//! Network adapters as raw Ethernet II links ([`Link`]): Intel 82574L and
//! PCH (I217–I219) [`e1000e`], discrete Intel Gigabit [`igb`], I225/I226
//! [`igc`], Realtek RTL8169/8168/8125/8126 [`realtek`], Qualcomm Atheros
//! AR816x/AR817x and Killer E2x00 [`alx`], Atheros AR813x/AR815x [`atl1c`]
//! and [`virtio`]-net.
//! USB Ethernet adapters are in the `stela-drivers` USB stack (`usb::cdc` and
//! `usb::ethernet`). [`wifi`] holds the shared 802.11 code and the Intel
//! and MediaTek MT7925 Wi-Fi drivers.
#![no_std]
#[macro_use]
extern crate alloc;

#[cfg(test)]
mod test_process;

#[cfg(test)]
use stela_driver_base::test_clock;
use stela_driver_base::{Clock, Deadline, Handoff, block, bus, registers};
use stela_driver_core::{bytes, firmware};

use alloc::{string::String, vec::Vec};
use core::fmt;

mod advanced;
pub mod alx;
pub mod aqtion;
pub mod atl1c;
pub mod e1000e;
pub mod igb;
pub mod igc;
pub mod realtek;
mod ring;
pub mod tg3;
pub mod virtio;
pub mod wifi;

/// The shortest frame without FCS; shorter frames are padded (IEEE 802.3).
pub const MIN_FRAME: usize = 60;
pub const MAX_FRAME: usize = 1514;
/// Destination, source and EtherType.
pub const HEADER: usize = 14;

/// Refuse what no Ethernet link carries: a frame without its header or
/// longer than the maximum.
pub fn check_frame(frame: &[u8]) -> Result<(), LinkError> {
    match frame.len() {
        length if length < HEADER => Err(LinkError::Invalid("frame shorter than its header")),
        length if length > MAX_FRAME => {
            Err(LinkError::Invalid("frame exceeds the Ethernet maximum"))
        }
        _ => Ok(()),
    }
}

/// An Ethernet MAC address.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub struct Mac(pub [u8; 6]);

impl Mac {
    pub const BROADCAST: Self = Self([0xff; 6]);

    pub fn is_multicast(&self) -> bool {
        self.0[0] & 1 != 0
    }
}

impl Mac {
    /// Linux `is_valid_ether_addr`: neither multicast nor all zeros.
    pub fn is_valid(&self) -> bool {
        !self.is_multicast() && self.0 != [0; 6]
    }

    /// Linux `eth_random_addr`, for an adapter whose programmed address is
    /// missing or invalid (`eth_hw_addr_random`): a locally administered
    /// unicast address, mixed from the time-stamp counter and `seed` (the
    /// adapter's location), so two such adapters get different ones.
    pub fn random(seed: &[u8]) -> Self {
        let mut mixed = Clock::ticks() ^ 0x9e37_79b9_7f4a_7c15;
        for &byte in seed {
            mixed = (mixed ^ u64::from(byte)).wrapping_mul(0x0100_0000_01b3);
        }
        mixed = (mixed ^ (mixed >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
        mixed = (mixed ^ (mixed >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
        mixed ^= mixed >> 31;
        let [a, b, c, d, e, f, _, _] = mixed.to_le_bytes();
        Self([(a & !1) | 2, b, c, d, e, f])
    }
}

impl fmt::Display for Mac {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let [a, b, c, d, e, g] = self.0;
        write!(f, "{a:02x}:{b:02x}:{c:02x}:{d:02x}:{e:02x}:{g:02x}")
    }
}

/// Why a link did not take or give a frame.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum LinkError {
    /// Exact PCI access/capability failure; the device owner remains retained.
    Pci(crate::block::BlockError),
    /// The adapter failed or refused a frame.
    Adapter(String),
    /// A frame no Ethernet link carries.
    Invalid(&'static str),
    /// The link is down: no frame can leave until it comes back. Frames the
    /// adapter held when it went down are discarded, as Linux does.
    NoCarrier,
    /// The transmit ring is full: the frame was not taken. The caller keeps
    /// it and tries again on a later poll, as Linux's stack requeues a
    /// `NETDEV_TX_BUSY` frame; no send waits for the controller.
    Busy,
}

/// How long a wired link may take from its start to carrier: copper
/// autonegotiation after a PHY reset or power-up, about 3 s at 1000BASE-T
/// and up to 5-7 s at 2.5G/5G/10GBASE-T, with margin.
pub const WIRED_CARRIER_WAIT_US: u64 = 10_000_000;

/// A raw Ethernet II link: whole frames without the FCS.
pub trait Link {
    /// The adapter's station address, which the stack sends from.
    fn mac(&self) -> Mac;
    /// Whether the adapter reports carrier; `true` when it cannot tell.
    fn carrier(&mut self) -> bool {
        true
    }
    /// How long after its start the link may take to report carrier before
    /// a missing one means no cable or no network. The default is a wired
    /// PHY's; a Wi-Fi station waits for its scan, join and handshake.
    fn carrier_wait_us(&self) -> u64 {
        WIRED_CARRIER_WAIT_US
    }
    /// Queues one frame (14-byte header and payload, at most 1514 bytes).
    fn transmit(&mut self, frame: &[u8]) -> Result<(), LinkError>;
    /// One frame the adapter received, if any, without blocking.
    ///
    /// A damaged frame (CRC, length, alignment or framing error, a failed
    /// bus transfer the adapter recovers from) is dropped and counted, as
    /// Linux counts `rx_errors`, and never returned as an error: the stack
    /// treats an `Err` as a device that is gone or wedged beyond its own
    /// recovery and stops using the link.
    fn receive(&mut self) -> Result<Option<Vec<u8>>, LinkError>;
    /// Receive one frame into caller-owned reusable storage. The buffer is
    /// unchanged if no frame is available or the adapter reports an error.
    /// Links that already own an allocated frame can hand it over directly.
    fn receive_reusing(&mut self, frame: &mut Vec<u8>) -> Result<bool, LinkError> {
        let Some(received) = self.receive()? else {
            return Ok(false);
        };
        *frame = received;
        Ok(true)
    }
    /// Whether the adapter sends its host's completion signal
    /// ([`crate::Handoff::completion`]) when a frame arrives or its link
    /// changes, waking the core that looks for frames, after
    /// [`Self::receive`] found none.
    fn signals(&self) -> bool {
        false
    }
    /// The clock's time at which a link that [`Self::signals`] next needs
    /// [`Self::receive`] for timers no signal ends, whatever arrives first.
    fn due_us(&self) -> Option<u64> {
        None
    }
    /// The next one-time diagnostic the adapter wants logged while it runs,
    /// such as an operation it declines or a connection attempt that
    /// failed; each is given once, and none is an error. Problems found
    /// while attaching stay with the probe's warnings.
    fn take_diagnostic(&mut self) -> Option<String> {
        None
    }
}

mod timing;

#[cfg(test)]
mod mac_tests {
    use super::Mac;

    #[test]
    fn a_random_address_is_locally_administered_unicast_and_differs_by_adapter() {
        let one = Mac::random(b"0000:03:00.0");
        let other = Mac::random(b"0000:04:00.0");
        for mac in [one, other] {
            assert!(mac.is_valid());
            assert_eq!(mac.0[0] & 3, 2, "{mac}");
        }
        assert_ne!(one, other);
        assert!(!Mac([0; 6]).is_valid() && !Mac([1, 0, 0, 0, 0, 1]).is_valid());
    }
}
