// SPDX-License-Identifier: GPL-2.0-only
// Copyright (c) 2014 Realtek Semiconductor Corp. All rights reserved.
//! rtl8152_apply_firmware and MAC/PHY block executors from pinned r8152.c.
//! Upload is preparation, never proof that the device PHY/MAC is initialized.
use super::super::firmware::{Bank, Block, Firmware, Fixup, Mac as MacPatch, PhyNc, PhyUnion};
use super::*;

impl<C: Control> Chip<C> {
    /// Execute a completely validated image against its exact observed chip.
    /// `power_cut` may be true only when the revision-specific caller observed
    /// PHY_STAT_PWRDN/EXT_INIT, matching the donor's no-wait patch handshake.
    /// A failed upload poisons this chip until whole-device reset/reattachment.
    /// No unfinished revision becomes carrier-ready through this method.
    pub async fn upload_firmware(
        &mut self,
        image: &Firmware<'_>,
        power_cut: bool,
    ) -> Result<(), BlockError> {
        if image.revision() != self.revision || self.initialized || self.enabled || !self.accessible
        {
            return Err(BlockError::Unsupported(
                "RTL815x firmware upload requires exact unstarted chip",
            ));
        }
        if image
            .blocks()
            .iter()
            .any(|block| matches!(block, Block::Mac(patch) if patch.breakpoints().next().is_none()))
        {
            return Err(BlockError::Unsupported(
                "RTL815x firmware has no active MAC breakpoints",
            ));
        }
        if image.blocks().iter().any(|block| match block {
            Block::PhyNc(patch) => usize::from(patch.register()) + patch.payload().len() > 0x10000,
            Block::PhyUnion(patch) => {
                usize::from(patch.register()) + patch.payload().len() > 0x10000
            }
            _ => false,
        }) {
            return Err(BlockError::Unsupported(
                "RTL815x PHY firmware would wrap SRAM address space",
            ));
        }
        let result = self.upload(image, !power_cut).await;
        self.base = None;
        if result.is_err() {
            self.accessible = false;
        }
        result
    }

    async fn write_block(
        &mut self,
        bank: u16,
        address: u32,
        data: &[u8],
    ) -> Result<(), BlockError> {
        if data.is_empty()
            || !address.is_multiple_of(4)
            || !data.len().is_multiple_of(4)
            || address
                .checked_add(data.len() as u32)
                .is_none_or(|end| end > 0xffff)
        {
            return Err(BlockError::Unsupported(
                "RTL815x firmware OCP range/alignment",
            ));
        }
        for (index, chunk) in data.chunks(512).enumerate() {
            let at = address + index as u32 * 512;
            let request = super::super::super::descriptor::setup(
                0x40,
                5,
                at as u16,
                bank | 0xff,
                chunk.len() as u16,
            );
            self.transaction(request, chunk).await?;
        }
        Ok(())
    }

    pub(super) async fn sram_write(&mut self, address: u16, value: u16) -> Result<(), BlockError> {
        self.phy_write(OCP_SRAM_ADDR, u32::from(address)).await?;
        self.phy_write(OCP_SRAM_DATA, u32::from(value)).await
    }

    pub(super) async fn sram_read(&mut self, address: u16) -> Result<u16, BlockError> {
        self.phy_write(OCP_SRAM_ADDR, u32::from(address)).await?;
        let mapped = self.phy_address(OCP_SRAM_DATA).await?;
        Ok(self.read(PLA, mapped, 2).await? as u16)
    }

    async fn phy_stream(&mut self, address: u16, data: &[u8]) -> Result<(), BlockError> {
        self.phy_write(OCP_SRAM_ADDR, u32::from(address)).await?;
        for word in data.as_chunks::<2>().0 {
            self.phy_write(OCP_SRAM_DATA, u32::from(u16::from_le_bytes(*word)))
                .await?;
        }
        Ok(())
    }

    pub(super) async fn patch_request(
        &mut self,
        request: bool,
        wait: bool,
    ) -> Result<(), BlockError> {
        self.phy_modify(
            OCP_PHY_PATCH_CMD,
            PATCH_REQUEST,
            if request { PATCH_REQUEST } else { 0 },
        )
        .await?;
        if wait {
            self.minimum_wait(1000).await?;
            let mapped = self.phy_address(OCP_PHY_PATCH_STAT).await?;
            // The hardware changes PATCH_READY asynchronously. Even release
            // must acknowledge it clearing before normal PHY accesses resume.
            if let Err(error) = self
                .poll(
                    PLA,
                    mapped,
                    2,
                    PATCH_READY,
                    if request { PATCH_READY } else { 0 },
                    (5000, 1000),
                )
                .await
            {
                if request && self.accessible {
                    // Bounded best-effort request withdrawal on a live bus.
                    self.phy_modify(OCP_PHY_PATCH_CMD, PATCH_REQUEST, 0).await?;
                }
                return Err(error);
            }
        }
        Ok(())
    }

    async fn patch_key(&mut self, register: u16, key: u16) -> Result<(), BlockError> {
        if register == 0 {
            return Err(BlockError::Unsupported(
                "RTL815x missing PHY patch key register",
            ));
        }
        if key != 0 {
            self.sram_write(register, key).await?;
            self.sram_write(SRAM_PHY_LOCK as u16, PHY_PATCH_LOCK as u16)
                .await
        } else {
            self.sram_write(0, 0).await?;
            self.phy_modify(OCP_PHY_LOCK, PATCH_LOCK, 0).await?;
            self.sram_write(register, 0).await
        }
    }

    async fn clear_breakpoints(&mut self, bank: u16) -> Result<(), BlockError> {
        let extended = self.revision.0 == 0x6400
            || (bank == USB && matches!(self.revision.0, 0x6010 | 0x7030 | 0x7410 | 0x7420));
        if extended {
            self.write(bank, USB_BP2_EN, 2, 0).await?;
        } else {
            self.write(bank, PLA_BP_EN, 1, 0).await?;
        }
        self.write_block(bank, PLA_BP_0, if extended { &[0; 32] } else { &[0; 16] })
            .await?;
        self.minimum_wait(3000).await?;
        self.write(bank, PLA_BP_BA, 2, 0).await
    }

    async fn upload_mac(&mut self, patch: &MacPatch<'_>) -> Result<(), BlockError> {
        let bank = match patch.bank() {
            Bank::Pla => PLA,
            Bank::Usb => USB,
        };
        let (version_reg, version) = patch.version_register();
        if version_reg != 0
            && self.read(USB, u32::from(version_reg), 1).await? >= u32::from(version)
        {
            return Ok(());
        }
        self.clear_breakpoints(bank).await?;
        // RTL8153A revision 04 requires MACDBG backup/restore before PLA code.
        if self.revision.0 == 0x5c10
            && bank == PLA
            && self.read(PLA, PLA_MACDBG_POST, 2).await? & DEBUG_OE == 0
        {
            self.write(PLA, PLA_MACDBG_PRE, 2, DEBUG_LTSSM).await?;
            self.write(PLA, PLA_MACDBG_POST, 2, DEBUG_LTSSM).await?;
        }
        self.write_block(bank, u32::from(patch.register()), patch.payload())
            .await?;
        let (base_reg, base) = patch.breakpoint_base();
        self.write(bank, u32::from(base_reg), 2, u32::from(base))
            .await?;
        let mut breakpoints = [0u8; 32];
        let mut count: usize = 0;
        for (index, value) in patch.breakpoints().enumerate() {
            breakpoints[index * 2..index * 2 + 2].copy_from_slice(&value.to_le_bytes());
            count += 2;
        }
        // The donor calls generic_ocp_write even for zero breakpoints; that
        // call explicitly rejects zero length rather than silently doing nothing.
        if count == 0 {
            return Err(BlockError::Unsupported(
                "RTL815x firmware has no active MAC breakpoints",
            ));
        }
        self.write_block(
            bank,
            u32::from(patch.breakpoint_start()),
            &breakpoints[..count.next_multiple_of(4)],
        )
        .await?;
        let (enable_reg, enable) = patch.breakpoint_enable();
        if enable_reg != 0 {
            self.write(bank, u32::from(enable_reg), 2, u32::from(enable))
                .await?;
        }
        if version_reg != 0 {
            self.write(USB, u32::from(version_reg), 1, u32::from(version))
                .await?;
        }
        Ok(())
    }

    async fn upload_nc(&mut self, patch: &PhyNc<'_>) -> Result<(), BlockError> {
        self.base = None;
        let (mode, before, after) = patch.mode();
        self.sram_write(mode, before).await?;
        let (base, value) = patch.base();
        self.sram_write(base, value).await?;
        self.phy_stream(patch.register(), patch.payload()).await?;
        let (enable, value) = patch.patch_enable();
        self.sram_write(enable, value).await?;
        for (index, value) in patch.breakpoints().enumerate() {
            self.sram_write(patch.breakpoint_start() + index as u16 * 2, value)
                .await?;
        }
        self.sram_write(mode, after).await
    }

    async fn upload_union(&mut self, patch: &PhyUnion<'_>) -> Result<(), BlockError> {
        self.base = None;
        for (register, value) in patch.presets() {
            self.sram_write(register, value).await?;
        }
        self.phy_stream(patch.register(), patch.payload()).await?;
        for (register, value) in patch.breakpoints() {
            self.sram_write(register, value).await?;
        }
        if let Some((register, value)) = patch.breakpoint_enable() {
            self.sram_write(register, value).await?;
        }
        Ok(())
    }

    async fn upload_speedup(
        &mut self,
        register: u16,
        version: u16,
        data: &[u8],
        wait: bool,
    ) -> Result<(), BlockError> {
        self.base = None;
        if self.sram_read(SRAM_GPHY_FW_VER as u16).await? >= version {
            return Ok(());
        }
        self.patch_request(true, wait).await?;
        let transferred: Result<(), BlockError> = async {
            for chunk in data.chunks(2048) {
                self.modify(USB, USB_GPHY_CTRL, 2, 0, GPHY_PATCH_DONE | BACKUP_RESTRORE)
                    .await?;
                self.write_block(USB, u32::from(register), chunk).await?;
                self.modify(PLA, PLA_POL_GPIO_CTRL, 2, 0, POL_GPHY_PATCH)
                    .await?;
                self.poll(PLA, PLA_POL_GPIO_CTRL, 2, POL_GPHY_PATCH, 0, (1000, 0))
                    .await?;
            }
            Ok(())
        }
        .await;
        self.base = None;
        // The donor exits the chunk loop then releases the patch request even
        // after a GPIO deadline. A failed USB access forbids further register
        // writes; a still-accessible device must acknowledge request release.
        if self.accessible {
            self.patch_request(false, wait).await?;
        }
        transferred?;
        if self.sram_read(SRAM_GPHY_FW_VER as u16).await? != version {
            return Err(BlockError::Unsupported(
                "RTL815x PHY firmware version did not acknowledge upload",
            ));
        }
        Ok(())
    }

    async fn upload(&mut self, image: &Firmware<'_>, wait: bool) -> Result<(), BlockError> {
        self.pre_firmware().await?;
        let mut patch_phy = true;
        let mut key_register = None;
        for block in image.blocks() {
            match block {
                Block::Mac(patch) => self.upload_mac(patch).await?,
                Block::PhyStart {
                    key_register: register,
                    key,
                } if patch_phy => {
                    self.patch_request(true, wait).await?;
                    self.patch_key(*register, *key).await?;
                    key_register = Some(*register);
                }
                Block::PhyStop if patch_phy => {
                    self.patch_key(
                        key_register.ok_or(BlockError::Unsupported(
                            "RTL815x missing active PHY patch key",
                        ))?,
                        0,
                    )
                    .await?;
                    self.patch_request(false, wait).await?;
                }
                Block::PhyNc(patch) => self.upload_nc(patch).await?,
                Block::PhyUnion(patch) if patch_phy => self.upload_union(patch).await?,
                Block::PhyFixup {
                    register,
                    value,
                    operation,
                } if patch_phy => {
                    self.base = None;
                    let mapped = self.phy_address(u32::from(*register)).await?;
                    let old = self.read(PLA, mapped, 2).await? as u16;
                    let new = match operation {
                        Fixup::And => old & value,
                        Fixup::Or => old | value,
                        Fixup::Clear => old & !value,
                        Fixup::Xor => old ^ value,
                    };
                    self.write(PLA, mapped, 2, u32::from(new)).await?;
                }
                Block::PhySpeedUp {
                    register,
                    version,
                    payload,
                } => {
                    self.upload_speedup(*register, *version, payload, wait)
                        .await?
                }
                Block::PhyVersion { register, version } => {
                    self.base = None;
                    patch_phy = self.sram_read(*register).await? < *version;
                    if patch_phy {
                        self.sram_write(*register, *version).await?;
                    }
                }
                Block::PhyStart { .. }
                | Block::PhyStop
                | Block::PhyUnion(_)
                | Block::PhyFixup { .. } => {}
            }
        }
        self.post_firmware().await
    }

    async fn pre_firmware(&mut self) -> Result<(), BlockError> {
        match self.revision.0 {
            0x5c10 | 0x5c20 => {
                self.poll(USB, USB_WDT1_CTRL, 1, WTD1_EN, 0, (104, 1000))
                    .await?;
                if self.revision.0 == 0x5c20 {
                    self.modify(USB, USB_FW_FIX_EN0, 2, FW_FIX_SUSPEND, 0)
                        .await?;
                }
            }
            0x6010 | 0x6400 => {
                self.write(USB, USB_FC_TIMER, 2, CTRL_TIMER_EN | (1000 / 8))
                    .await?
            }
            _ => {}
        }
        Ok(())
    }

    async fn post_firmware(&mut self) -> Result<(), BlockError> {
        match self.revision.0 {
            0x5c10 => {
                if self.read(USB, USB_CSTMR, 1).await? & FORCE_SUPER != 0 {
                    self.write(USB, PLA_BP_4, 2, BP4_SUPER_ONLY).await?;
                }
                self.write(PLA, PLA_UPHY_TIMER, 2, 36000 / 16).await?;
            }
            0x5c20 => {
                if self.read(USB, USB_CSTMR, 1).await? & FORCE_SUPER != 0 {
                    self.modify(PLA, PLA_BP_EN, 2, 0, 1).await?;
                }
                self.write(PLA, PLA_UPHY_TIMER, 2, 36000 / 16).await?;
                self.write(PLA, PLA_EXTRA_STATUS, 2, U3P3_CHECK_EN | 4)
                    .await?;
                self.modify(USB, USB_FW_FIX_EN0, 2, 0, FW_FIX_SUSPEND)
                    .await?;
                self.modify(USB, USB_USB2PHY, 1, 0, USB2PHY_L1 | USB2PHY_SUSPEND)
                    .await?;
            }
            0x5c30 => {
                self.modify(USB, USB_USB2PHY, 1, 0, USB2PHY_L1 | USB2PHY_SUSPEND)
                    .await?;
                self.modify(USB, USB_FW_FIX_EN1, 2, 0, FW_IP_RESET_EN)
                    .await?;
            }
            0x6010 => {
                if self.read(USB, USB_MISC_1, 1).await? & BND_MASK != 0 {
                    self.modify(PLA, PLA_BP_EN, 2, 0, 1).await?;
                }
                self.modify(USB, USB_FW_CTRL, 2, 0, FLOW_CTRL_PATCH_OPT)
                    .await?;
                self.modify(USB, USB_FW_TASK, 2, 0, FC_PATCH_TASK).await?;
                self.modify(USB, USB_FW_FIX_EN1, 2, 0, FW_IP_RESET_EN)
                    .await?;
            }
            0x6400 => {
                self.modify(USB, USB_FW_CTRL, 2, 0, FLOW_CTRL_PATCH_2)
                    .await?;
                self.modify(USB, USB_FW_TASK, 2, 0, FC_PATCH_TASK).await?;
            }
            0x7030 => {
                self.modify(USB, USB_FW_FIX_EN1, 2, 0, FW_IP_RESET_EN)
                    .await?;
                self.write(USB, USB_UPHY3_MDCMDIO, 4, 0x4026_840e).await?;
                self.write(USB, USB_UPHY3_MDCMDIO, 4, 0x4001_acc9).await?;
            }
            0x7410 | 0x7420 => {}
            _ => {
                return Err(BlockError::Unsupported(
                    "RTL815x firmware hooks for revision not implemented",
                ));
            }
        }
        Ok(())
    }
}

#[cfg(test)]
pub(crate) mod tests;
