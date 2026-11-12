//! Wi-Fi chips Linux drives and no driver here does, named so a boot reports
//! each one by chip and Linux driver instead of a generic line. The IDs are
//! the PCI device tables of the Linux modules named, as their
//! `modules.alias` lists them (Linux 7.0).

/// What the boot says of a Wi-Fi function no driver here runs, or `None`
/// when a driver here takes it or no Linux driver lists it.
pub fn reason(vendor: u16, device: u16) -> Option<&'static str> {
    Some(match (vendor, device) {
        // iwlwifi, MVM op mode, pre-AX generations.
        (0x8086, 0x2526 | 0x271b | 0x271c | 0x30dc | 0x31dc | 0x9df0 | 0xa370) => {
            "Intel Wireless-AC 9000 series (Linux iwlwifi): this generation has no driver here"
        }
        (0x8086, 0x24f3 | 0x24f4 | 0x24f5 | 0x24f6 | 0x24fb | 0x24fd) => {
            "Intel Wireless 8000 series or 3168 (Linux iwlwifi): this generation has no driver here"
        }
        (0x8086, 0x08b1..=0x08b4 | 0x095a | 0x095b | 0x3165 | 0x3166) => {
            "Intel Wireless 7000 series (Linux iwlwifi): this generation has no driver here"
        }
        // iwlwifi, DVM op mode.
        (0x8086, 0x0082..=0x0091 | 0x0885..=0x0897 | 0x08ae | 0x08af | 0x422b..=0x423d) => {
            "Intel WiFi Link or Centrino 1000-6000 series (Linux iwlwifi): no driver here"
        }
        // mt7921e. MT7921 and MT7922 run a station here.
        (0x14c3, 0x7920) => "MediaTek MT7920 (Linux mt7921e): no driver here",
        (0x14c3, 0x7906 | 0x790a | 0x7915 | 0x7916) => {
            "MediaTek MT7915 or MT7916 (Linux mt7915e): no driver here"
        }
        (0x14c3, 0x7990..=0x7993 | 0x799a | 0x799b) => {
            "MediaTek MT7996 or MT7992 (Linux mt7996e): no driver here"
        }
        (0x14c3, 0x7611 | 0x7615 | 0x7663) => {
            "MediaTek MT7615 or MT7663 (Linux mt7615e): no driver here"
        }
        (0x14c3, 0x7603 | 0x7602 | 0x7612 | 0x7662 | 0x7610 | 0x7630 | 0x7650) => {
            "MediaTek MT76x0, MT76x2 or MT7603 (Linux mt76): no driver here"
        }
        // rtw88 and rtw89. Only RTL8822C runs a station here.
        (0x10ec, 0xb822) => "Realtek RTL8822BE (Linux rtw88_8822be): no driver here",
        (0x10ec, 0xb821 | 0xc821) => "Realtek RTL8821CE (Linux rtw88_8821ce): no driver here",
        (0x10ec, 0xd723) => "Realtek RTL8723DE (Linux rtw88_8723de): no driver here",
        (0x10ec, 0x8813) => "Realtek RTL8814AE (Linux rtw88_8814ae): no driver here",
        (0x10ec, 0x8852 | 0xa85a) => "Realtek RTL8852AE (Linux rtw89_8852ae): no driver here",
        (0x10ec, 0xb852 | 0xb85b) => {
            "Realtek RTL8852BE (Linux rtw89_8852be): firmware is parsed but no station driver here"
        }
        (0x10ec, 0xb520) => "Realtek RTL8852BE-VT (Linux rtw89_8852bte): no driver here",
        (0x10ec, 0xc852) => "Realtek RTL8852CE (Linux rtw89_8852ce): no driver here",
        (0x10ec, 0xb851) => "Realtek RTL8851BE (Linux rtw89_8851be): no driver here",
        (0x10ec, 0x8922 | 0x892b) => "Realtek RTL8922AE (Linux rtw89_8922ae): no driver here",
        // rtlwifi.
        (
            0x10ec,
            0x002b | 0x8171..=0x8179 | 0x818b | 0x8191..=0x8193 | 0x8723 | 0xb723 | 0x8812 | 0x8821,
        ) => {
            "Realtek rtlwifi-generation chip (Linux rtl8188ee, rtl8192*, rtl8723*, rtl8821ae): no driver here"
        }
        // Qualcomm Atheros.
        (0x17cb, 0x1101) => "Qualcomm QCA6390 (Linux ath11k_pci): no driver here",
        (0x17cb, 0x1103) => "Qualcomm WCN6855 (Linux ath11k_pci): no driver here",
        (0x17cb, 0x1104) => "Qualcomm QCN9074 (Linux ath11k_pci): no driver here",
        (0x17cb, 0x1107 | 0x1109 | 0x1112) => {
            "Qualcomm Wi-Fi 7 chip (Linux ath12k_wifi7): no driver here"
        }
        (0x168c, 0x003c | 0x003e | 0x0040..=0x0042 | 0x0046 | 0x0050 | 0x0056)
        | (0x0777, 0x11ac) => {
            "Qualcomm Atheros QCA6174, QCA9377 or QCA9xxx (Linux ath10k_pci): no driver here"
        }
        (0x168c, 0x0023..=0x0037) => "Qualcomm Atheros AR9xxx (Linux ath9k): no driver here",
        (0x168c, 0x0007 | 0x0011..=0x001d | 0x0207 | 0x1014 | 0xff16 | 0xff1a | 0xff1b) => {
            "Atheros AR5xxx (Linux ath5k): no driver here"
        }
        (0x17cb, 0x1201) | (0x1ae9, 0x0302 | 0x0310) => {
            "Qualcomm 60 GHz Wi-Fi (Linux wil6210): no driver here"
        }
        // Broadcom.
        (
            0x14e4,
            0x4354
            | 0x4355
            | 0x4365
            | 0x43a3
            | 0x43ba..=0x43bc
            | 0x43c3..=0x43c5
            | 0x43ca..=0x43cc
            | 0x43d3
            | 0x43d9
            | 0x43dc
            | 0x43e9
            | 0x43ec
            | 0x43ef
            | 0x440d
            | 0x4415
            | 0x4417
            | 0x4425
            | 0x4433
            | 0x4464
            | 0x4488
            | 0x449d
            | 0xaa31
            | 0xaa52,
        ) => "Broadcom FullMAC chip (Linux brcmfmac): no driver here",
        (
            0x14e4,
            0x0576
            | 0x4313
            | 0x4331
            | 0x4353
            | 0x4357..=0x4360
            | 0x43a0
            | 0x43a9
            | 0x43aa
            | 0x43b1
            | 0x4727
            | 0xa8d8
            | 0xa8db
            | 0xa8dc,
        ) => "Broadcom SoftMAC chip (Linux bcma with b43 or brcmsmac): no driver here",
        (
            0x14e4,
            0x4301
            | 0x4306
            | 0x4307
            | 0x4311
            | 0x4312
            | 0x4315
            | 0x4318..=0x4322
            | 0x4324
            | 0x4325
            | 0x4328
            | 0x4329
            | 0x432b
            | 0x432c
            | 0x4350
            | 0x4351
            | 0xa8d6,
        )
        | (0x14a4, 0x4318) => "Broadcom SoftMAC chip (Linux ssb with b43): no driver here",
        // Marvell.
        (0x11ab, 0x2b30 | 0x2b38 | 0x2b42) | (0x1b4b, 0x2b42) => {
            "Marvell 88W8766, 88W8897 or 88W8997 (Linux mwifiex_pcie): no driver here"
        }
        _ => return None,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn chips_a_driver_here_runs_have_no_reason_and_others_name_the_linux_driver() {
        // MT7921, MT7922, MT7925, RTL8822CE, and an Intel device the Intel
        // driver lists, which reports its own radio and step.
        for (vendor, device) in [
            (0x14c3, 0x7961),
            (0x14c3, 0x0608),
            (0x14c3, 0x7922),
            (0x14c3, 0x0616),
            (0x0b48, 0x7922),
            (0x14c3, 0x7925),
            (0x14c3, 0x0717),
            (0x10ec, 0xc822),
            (0x10ec, 0xc82f),
            (0x8086, 0x2725),
            (0x8086, 0x51f0),
            (0x8086, 0x272b),
        ] {
            assert_eq!(reason(vendor, device), None, "{vendor:04x}:{device:04x}");
        }
        for id in 0..=u16::MAX {
            if super::super::intel::identity::device(id).is_some() {
                assert_eq!(reason(0x8086, id), None, "{id:04x}");
            }
        }
        for (vendor, device, driver) in [
            (0x8086, 0x9df0, "iwlwifi"),
            (0x8086, 0x24fd, "iwlwifi"),
            (0x14c3, 0x7920, "mt7921e"),
            (0x14c3, 0x7990, "mt7996e"),
            (0x10ec, 0xb852, "rtw89_8852be"),
            (0x10ec, 0xc821, "rtw88_8821ce"),
            (0x17cb, 0x1103, "ath11k_pci"),
            (0x17cb, 0x1109, "ath12k"),
            (0x168c, 0x003e, "ath10k_pci"),
            (0x14e4, 0x4433, "brcmfmac"),
            (0x11ab, 0x2b38, "mwifiex_pcie"),
        ] {
            let text = reason(vendor, device).unwrap();
            assert!(text.contains(driver), "{text}");
        }
        assert_eq!(reason(0x1234, 0x5678), None);
    }
}
