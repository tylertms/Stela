mod fallible;
use super::*;
use crate::bus::pci::regs;

#[test]
fn an_address_reads_back_from_its_written_form_and_nothing_else() {
    let address = Address {
        segment: 1,
        bus: 0x85,
        device: 0x1f,
        function: 7,
    };
    assert_eq!(Address::parse(&alloc::format!("{address}")), Some(address));
    assert_eq!(
        Address::parse("0000:0a:00.3"),
        Some(Address {
            segment: 0,
            bus: 0x0a,
            device: 0,
            function: 3
        })
    );
    for text in [
        "",
        "0000:0a:00",
        "0000:0a:20.0",
        "0000:0a:00.8",
        "0a:00.0",
        "x:0a:00.0",
    ] {
        assert_eq!(Address::parse(text), None, "{text}");
    }
}

#[test]
fn borrowed_owned_transport_keeps_original_mapping_and_refuses_owner_recovery() {
    use alloc::boxed::Box;
    use core::sync::atomic::Ordering;
    let address = Address {
        segment: 0,
        bus: 0,
        device: 2,
        function: 0,
    };
    let (owner, counters) = owned::test_support::fixture(address);
    // Whole-boot owner, exactly as the production
    // LegacyControllerOwner will keep its original config and BAR leases.
    let owner = Box::leak(Box::new(owner));
    let mut config = EcamConfig::from_retained(owner);
    config.write32(0x40, 0xa5c3_5a7e).unwrap();
    config.write8(0x41, 0x12).unwrap();
    assert_eq!(config.read32(0x40).unwrap(), 0xa5c3_127e);
    config.write16(0x42, 0x4b36).unwrap();
    assert_eq!(config.read16(0x42).unwrap(), 0x4b36);
    let config = match config.into_owned() {
        Ok(_) => panic!("borrow is not recoverable ownership"),
        Err(config) => config,
    };
    let before = counters.contexts.load(Ordering::Relaxed);
    counters
        .fail_context_at
        .store(before + 1, Ordering::Relaxed);
    assert_eq!(
        config.read8(0),
        Err(ConfigError::before_effect(ConfigFailure::Mapping(
            crate::bus::mmio::owned::Error::Unavailable
        ))),
    );
    counters.fail_context_at.store(0, Ordering::Relaxed);
    assert_eq!(
        config.read32(4094),
        Err(ConfigError::before_effect(ConfigFailure::Access {
            offset: 4094,
            width: 4
        })),
    );
    drop(config);
    assert_eq!(counters.unmaps.load(Ordering::Relaxed), 0);
    assert_eq!(counters.resource_drops.load(Ordering::Relaxed), 0);
    assert_eq!(counters.platform_drops.load(Ordering::Relaxed), 0);
}

#[test]
#[expect(
    clippy::result_large_err,
    reason = "inventory release refusal returns the same original observed function without allocating"
)]
fn owned_inventory_releases_every_present_config_before_returning_data() {
    use alloc::rc::Rc;
    use core::cell::Cell;
    struct Observed {
        words: [u32; 64],
        live: Rc<Cell<usize>>,
    }
    impl Drop for Observed {
        fn drop(&mut self) {
            self.live.set(self.live.get() - 1);
        }
    }
    impl ConfigSpace for Observed {
        fn read32(&self, offset: u16) -> Result<u32, crate::bus::pci::ConfigError> {
            Ok(self.words[usize::from(offset) / 4])
        }
        fn write32(&mut self, _: u16, _: u32) -> Result<(), crate::bus::pci::ConfigError> {
            panic!("discovery must not write configuration");
        }

        fn read8(&self, offset: u16) -> Result<u8, crate::bus::pci::ConfigError> {
            Ok((self.read32(offset & !3)? >> ((offset & 3) * 8)) as u8)
        }

        fn write8(&mut self, offset: u16, value: u8) -> Result<(), crate::bus::pci::ConfigError> {
            let shift = (offset & 3) * 8;
            let word = self.read32(offset & !3)? & !(0xff << shift) | u32::from(value) << shift;
            self.write32(offset & !3, word)
        }

        fn read16(&self, offset: u16) -> Result<u16, crate::bus::pci::ConfigError> {
            Ok((self.read32(offset & !3)? >> ((offset & 2) * 8)) as u16)
        }

        fn write16(&mut self, offset: u16, value: u16) -> Result<(), crate::bus::pci::ConfigError> {
            let shift = (offset & 2) * 8;
            let word = self.read32(offset & !3)? & !(0xffff << shift) | u32::from(value) << shift;
            self.write32(offset & !3, word)
        }
    }
    let live = Rc::new(Cell::new(0));
    let released = Cell::new(0);
    let window = EcamWindow {
        base: 0xe000_0000,
        segment: 0,
        start_bus: 0,
        end_bus: 0,
    };
    let found = Function::<Observed>::inventory_with(
        &window,
        |address| -> Result<_, ()> {
            assert_eq!(
                live.get(),
                0,
                "last actual owner must retire before another opens"
            );
            if address.device != 2 || address.function != 0 {
                return Ok(None);
            }
            let mut words = [0; 64];
            words[0] = 0x11e8_1234;
            words[2] = 0xff00_0000;
            live.set(live.get() + 1);
            Ok(Some(
                Function::new(
                    address,
                    Observed {
                        words,
                        live: live.clone(),
                    },
                )
                .unwrap(),
            ))
        },
        |function| {
            assert_eq!(live.get(), 1);
            released.set(released.get() + 1);
            drop(function);
            Ok(())
        },
    )
    .unwrap();
    assert_eq!(live.get(), 0);
    assert_eq!(released.get(), 1);
    assert_eq!(found.len(), 1);
    assert_eq!(
        found[0],
        Identity {
            address: Address {
                segment: 0,
                bus: 0,
                device: 2,
                function: 0
            },
            vendor: 0x1234,
            device: 0x11e8,
            class: [0xff, 0, 0]
        }
    );
}

#[test]
fn native_ecam_narrow_stores_preserve_adjacent_status_and_vendor_bytes() {
    let mut words = [0xa5c3_5a7eu32; 1024];
    // SAFETY: this test exclusively owns the aligned backing allocation.
    let mut config = unsafe { EcamConfig::raw(words.as_mut_ptr() as u64) };
    config.write8(0x44, 0x78).unwrap();
    assert_eq!(config.read32(0x44).unwrap(), 0xa5c3_5a78);
    config.write8(0x47, 0x12).unwrap();
    assert_eq!(config.read32(0x44).unwrap(), 0x12c3_5a78);
    config.write16(regs::COMMAND, 0x0406).unwrap();
    assert_eq!(config.read32(regs::COMMAND).unwrap(), 0xa5c3_0406);
}

/// A configuration space whose BAR registers keep only their writable
/// bits, as hardware does.
struct Fake {
    words: [u32; 64],
    bar_masks: [u32; 6],
    writes: alloc::rc::Rc<core::cell::Cell<usize>>,
}

impl ConfigSpace for Fake {
    fn read32(&self, offset: u16) -> Result<u32, crate::bus::pci::ConfigError> {
        Ok(self.words[usize::from(offset / 4)])
    }

    fn write32(&mut self, offset: u16, value: u32) -> Result<(), crate::bus::pci::ConfigError> {
        self.writes.set(self.writes.get() + 1);
        let index = usize::from(offset / 4);
        self.words[index] = match index.checked_sub(4) {
            Some(bar) if bar < 6 => {
                let mask = self.bar_masks[bar];
                value & mask | self.words[index] & !mask
            }
            _ => value,
        };
        Ok(())
    }

    fn write16(&mut self, offset: u16, value: u16) -> Result<(), crate::bus::pci::ConfigError> {
        self.writes.set(self.writes.get() + 1);
        let shift = (offset & 2) * 8;
        let word = &mut self.words[usize::from(offset / 4)];
        *word = *word & !(0xffff << shift) | u32::from(value) << shift;
        Ok(())
    }

    fn write8(&mut self, offset: u16, value: u8) -> Result<(), crate::bus::pci::ConfigError> {
        self.writes.set(self.writes.get() + 1);
        let shift = (offset & 3) * 8;
        let word = &mut self.words[usize::from(offset / 4)];
        *word = *word & !(0xff << shift) | u32::from(value) << shift;
        Ok(())
    }

    fn read8(&self, offset: u16) -> Result<u8, crate::bus::pci::ConfigError> {
        Ok((self.read32(offset & !3)? >> ((offset & 3) * 8)) as u8)
    }

    fn read16(&self, offset: u16) -> Result<u16, crate::bus::pci::ConfigError> {
        Ok((self.read32(offset & !3)? >> ((offset & 2) * 8)) as u16)
    }
}

fn fake() -> Fake {
    let mut words = [0; 64];
    words[0] = 0x1042_1af4;
    words[1] = 0x0010_0007; // capabilities; IO, memory and master on
    words[2] = 0x0108_0200;
    words[13] = 0x40;
    // MSI-X at 0x40: 4 vectors, table in BAR 4 at 0, PBA in BAR 4 at 0x800.
    words[16] = 0x0003_5011;
    words[17] = 0x0000_0004;
    words[18] = 0x0000_0804;
    // MSI at 0x50: 64-bit, up to 8 vectors, end of list.
    words[20] = 0x0086_0005;
    Fake {
        words,
        bar_masks: [0; 6],
        writes: alloc::rc::Rc::new(core::cell::Cell::new(0)),
    }
}

#[test]
fn bars_decode_and_size_every_kind() {
    let mut config = fake();
    // regs::BASE_ADDRESS_0: 16 KiB 32-bit memory; BAR1: 256-byte I/O; BAR2+3: 64-bit
    // prefetchable 1 MiB above 4 GiB; BAR4 unimplemented; BAR5 4 KiB.
    config.words[4] = 0xfebf_0000;
    config.bar_masks[0] = 0xffff_c000;
    config.words[5] = 0xc001;
    config.bar_masks[1] = 0xffff_ff00;
    config.words[6] = 0x0010_000c;
    config.bar_masks[2] = 0xfff0_0000;
    config.words[7] = 0x80;
    config.bar_masks[3] = 0xffff_ffff;
    config.words[9] = 0xfeb0_0000;
    config.bar_masks[5] = 0xffff_f000;
    let mut function = Function::new(Address::default_for_tests(), config).unwrap();
    let bars = function.bars().unwrap();
    assert_eq!(
        bars[0],
        Some(Bar::Memory {
            address: 0xfebf_0000,
            size: 0x4000,
            prefetchable: false,
            wide: false
        })
    );
    assert_eq!(
        bars[1],
        Some(Bar::Io {
            port: 0xc000,
            size: 0x100
        })
    );
    assert_eq!(
        bars[2],
        Some(Bar::Memory {
            address: 0x80_0010_0000,
            size: 0x10_0000,
            prefetchable: true,
            wide: true
        })
    );
    assert_eq!(bars[3], None);
    assert_eq!(bars[4], None);
    assert_eq!(
        bars[5].and_then(|bar| bar.memory()),
        Some(0xfeb0_0000..0xfeb0_1000)
    );
    // Sizing restores every BAR and the command register.
    assert_eq!(function.config().words[4], 0xfebf_0000);
    assert_eq!(function.config().words[7], 0x80);
    assert_eq!(function.config().read16(regs::COMMAND).unwrap(), 0x0007);
    assert_eq!(function.class, [1, 8, 2]);
}

#[test]
fn interrupts_are_discovered_and_left_off() {
    let mut function = Function::new(Address::default_for_tests(), fake()).unwrap();
    function.config().words[16] |= 1 << 31; // MSI-X enabled by firmware
    function.config().words[20] |= 1 << 16; // MSI enabled by firmware
    let interrupts = function.interrupts().unwrap();
    assert_eq!(
        interrupts.msix,
        Some(MsiX {
            offset: 0x40,
            vectors: 4,
            table_bar: 4,
            table_offset: 0,
            pending_bar: 4,
            pending_offset: 0x800
        })
    );
    assert_eq!(
        interrupts.msi,
        Some(Msi {
            offset: 0x50,
            vectors: 8,
            wide: true
        })
    );
    function.enable_polled().unwrap();
    assert_eq!(function.config().words[16] >> 31, 0);
    assert_eq!(function.config().words[20] >> 16 & 1, 0);
    let command = function.config().read16(regs::COMMAND).unwrap();
    assert_eq!(command & 0x406, 0x406);
}

/// MSI-X opens only once its table is written. A failed writer retains the
/// acknowledged Function Mask and original cause without cleanup writes.
#[test]
fn msix_opens_only_after_its_table_is_written() {
    let mut function = Function::new(Address::default_for_tests(), fake()).unwrap();
    let msix = function.interrupts().unwrap().msix.unwrap();
    function.enable_polled().unwrap();
    function.enable_msix(msix, || Ok(())).unwrap();
    assert_eq!(function.config().words[16] >> 30, 0b10, "enabled, unmasked");
    let writes = function.config().writes.clone();
    let failed_at = core::cell::Cell::new(0);
    assert_eq!(
        function.enable_msix(msix, || {
            failed_at.set(writes.get());
            Err::<(), _>(BlockError::Timeout)
        }),
        Err(BlockError::Timeout)
    );
    assert_eq!(
        function.config().words[16] >> 30,
        0b11,
        "enabled under Function Mask"
    );
    assert_eq!(
        writes.get(),
        failed_at.get(),
        "cleanup store followed failed table writer"
    );
}

/// MSI opens with one message once its address and data are written.
#[test]
fn msi_opens_one_message_after_its_address_and_data() {
    let mut config = fake();
    config.words[20] |= 0b011 << 20;
    let mut function = Function::new(Address::default_for_tests(), config).unwrap();
    let msi = function.interrupts().unwrap().msi.unwrap();
    function.enable_polled().unwrap();
    let message = crate::bus::irq::MsiMessage {
        address: 0x1_fee0_0000,
        data: 0x4041,
    };
    function.enable_msi(msi, message).unwrap();
    let words = &function.config().words;
    assert_eq!(words[21], 0xfee0_0000);
    assert_eq!(words[22], 1);
    assert_eq!(words[23] & 0xffff, 0x4041);
    assert_eq!(words[20] >> 16 & 1, 1, "enabled");
    assert_eq!(words[20] >> 20 & 0b111, 0, "one message");
}

/// Missing table resources are explicit failures, not MSI-X capability absence.
#[test]
fn missing_msix_table_refuses_without_enabling_msi() {
    let mut function = Function::new(Address::default_for_tests(), fake()).unwrap();
    function.enable_polled().unwrap();
    let handoff = crate::Handoff {
        firmware: None,
        completion: None,
        ecam: Vec::new(),
        mapped: Vec::new(),
        dma: 0..0,
        clock: crate::Clock {
            ticks_per_microsecond: 1,
        },
    };
    let message = crate::bus::irq::MsiMessage {
        address: 0xfee0_0000,
        data: 0x41,
    };
    let before = function.config().words;
    let writes = function.config().writes.get();
    assert_eq!(
        function.enable_message(&handoff, &[None; 6], message),
        Err(BlockError::Unsupported("register BAR is not memory"))
    );
    let words = &function.config().words;
    assert_eq!(words[16] >> 31, 0, "MSI-X off");
    assert_eq!(words[20] >> 16 & 1, 0, "MSI stayed off");
    assert_eq!(*words, before, "resource refusal changed original config");
    assert_eq!(
        function.config().writes.get(),
        writes,
        "resource refusal issued a store"
    );
}

#[test]
fn memory_enable_does_not_choose_an_interrupt_policy() {
    for intx in [0, regs::COMMAND_INTX_DISABLE] {
        let mut config = fake();
        config.words[1] = (config.words[1] & !0xffff) | u32::from(intx);
        config.words[16] |= 1 << 31;
        config.words[20] |= 1 << 16;
        let mut function = Function::new(Address::default_for_tests(), config).unwrap();
        function.enable_memory_and_bus_master().unwrap();
        let config = function.config();
        assert_eq!(
            config.read16(regs::COMMAND).unwrap(),
            intx | regs::COMMAND_MEMORY | regs::COMMAND_MASTER
        );
        assert_eq!(config.words[16] >> 31, 1);
        assert_eq!(config.words[20] >> 16 & 1, 1);
    }
}

#[test]
fn a_looping_capability_list_stops_at_the_loop() {
    let mut config = fake();
    config.words[20] = 0x0086_4005; // MSI points back at MSI-X
    let function = Function::new(Address::default_for_tests(), config).unwrap();
    assert_eq!(
        function.capabilities(),
        Err(CapabilityError::Cycle { offset: 0x40 })
    );
}

#[test]
fn bar_sizes_come_from_the_lowest_writable_address_bit() {
    let mut config = fake();
    // regs::BASE_ADDRESS_0+1: 64-bit 1 MiB whose upper dword is read-only zero; BAR2:
    // 32-bit range ending at 4 GiB; BAR5: 64-bit type with no upper register.
    config.words[4] = 0xfe00_0004;
    config.bar_masks[0] = 0xfff0_0000;
    config.words[6] = 0xffff_f000;
    config.bar_masks[2] = 0xffff_f000;
    config.words[9] = 0xfd00_0004;
    config.bar_masks[5] = 0xffff_f000;
    let mut function = Function::new(Address::default_for_tests(), config).unwrap();
    let bars = function.bars().unwrap();
    assert_eq!(
        bars[0],
        Some(Bar::Memory {
            address: 0xfe00_0000,
            size: 0x10_0000,
            prefetchable: false,
            wide: true
        })
    );
    assert_eq!(
        bars[2].and_then(|bar| bar.memory()),
        Some(0xffff_f000..0x1_0000_0000)
    );
    assert_eq!(bars[5], None);
    let overflowing = Bar::Memory {
        address: u64::MAX - 0xfff,
        size: 0x2000,
        prefetchable: false,
        wide: true,
    };
    assert_eq!(overflowing.memory(), None);
}

#[test]
fn the_capability_walk_stops_at_an_all_ones_id() {
    let mut config = fake();
    config.words[16] = 0x0003_50ff; // first capability reads as all ones
    let function = Function::new(Address::default_for_tests(), config).unwrap();
    assert_eq!(
        function.capabilities(),
        Err(CapabilityError::Malformed { offset: 0x40 })
    );
}

/// A function whose D3hot exit performs the internal reset that
/// No_Soft_Reset clear advertises: its BARs read back as zero.
struct SoftReset(Fake);

impl ConfigSpace for SoftReset {
    fn read32(&self, offset: u16) -> Result<u32, crate::bus::pci::ConfigError> {
        self.0.read32(offset)
    }

    fn write32(&mut self, offset: u16, value: u32) -> Result<(), crate::bus::pci::ConfigError> {
        self.0.write32(offset, value)?;
        Ok(())
    }

    fn write16(&mut self, offset: u16, value: u16) -> Result<(), crate::bus::pci::ConfigError> {
        let before = self.0.read16(offset)?;
        self.0.write16(offset, value)?;
        if offset == 0x64 && before & PM_STATE == D3HOT && before & PM_NO_SOFT_RESET == 0 {
            self.0.words[4..10].fill(0);
        }
        Ok(())
    }

    fn read8(&self, offset: u16) -> Result<u8, crate::bus::pci::ConfigError> {
        Ok((self.read32(offset & !3)? >> ((offset & 3) * 8)) as u8)
    }

    fn write8(&mut self, offset: u16, value: u8) -> Result<(), crate::bus::pci::ConfigError> {
        let shift = (offset & 3) * 8;
        let word = self.read32(offset & !3)? & !(0xff << shift) | u32::from(value) << shift;
        self.write32(offset & !3, word)
    }

    fn read16(&self, offset: u16) -> Result<u16, crate::bus::pci::ConfigError> {
        Ok((self.read32(offset & !3)? >> ((offset & 2) * 8)) as u16)
    }
}

#[test]
fn power_up_leaves_d3hot_and_restores_soft_reset_bars() {
    for no_soft_reset in [0, PM_NO_SOFT_RESET] {
        let mut config = fake();
        config.words[20] = 0x0086_6005; // MSI, next capability at 0x60
        config.words[24] = 0x0003_0001; // power management, end of list
        config.words[25] = u32::from(D3HOT | no_soft_reset | 1 << 15);
        config.words[4] = 0xfebf_0000;
        config.bar_masks[0] = 0xffff_c000;
        let mut function = Function::new(Address::default_for_tests(), SoftReset(config)).unwrap();
        // This model checks PMCSR/BAR effects, not real time. Capture
        // the requested wait instead of passing an invalid Clock.
        let mut delays = Vec::new();
        function
            .power_up_with(|us| {
                delays.push(us);
                Ok(())
            })
            .unwrap();
        assert_eq!(delays, [D3HOT_DELAY_US]);
        assert_eq!(function.config().read16(0x64).unwrap(), 0);
        assert_eq!(
            function.config().read32(regs::BASE_ADDRESS_0).unwrap(),
            0xfebf_0000
        );
    }
}

#[test]
fn power_up_reports_a_function_that_stays_in_d3hot() {
    let mut config = fake();
    config.words[20] = 0x0086_6005;
    config.words[24] = 0x0003_0001;
    config.words[25] = u32::from(D3HOT);
    struct Stuck(Fake);
    impl ConfigSpace for Stuck {
        fn read32(&self, offset: u16) -> Result<u32, crate::bus::pci::ConfigError> {
            self.0.read32(offset)
        }
        fn write32(&mut self, offset: u16, value: u32) -> Result<(), crate::bus::pci::ConfigError> {
            self.0.write32(offset, value)?;
            Ok(())
        }
        fn write16(&mut self, offset: u16, value: u16) -> Result<(), crate::bus::pci::ConfigError> {
            if offset != 0x64 {
                self.0.write16(offset, value)?
            }
            Ok(())
        }

        fn read8(&self, offset: u16) -> Result<u8, crate::bus::pci::ConfigError> {
            Ok((self.read32(offset & !3)? >> ((offset & 3) * 8)) as u8)
        }

        fn write8(&mut self, offset: u16, value: u8) -> Result<(), crate::bus::pci::ConfigError> {
            let shift = (offset & 3) * 8;
            let word = self.read32(offset & !3)? & !(0xff << shift) | u32::from(value) << shift;
            self.write32(offset & !3, word)
        }

        fn read16(&self, offset: u16) -> Result<u16, crate::bus::pci::ConfigError> {
            Ok((self.read32(offset & !3)? >> ((offset & 2) * 8)) as u16)
        }
    }
    let mut function = Function::new(Address::default_for_tests(), Stuck(config)).unwrap();
    let mut delays = Vec::new();
    assert_eq!(
        function.power_up_with(|us| {
            delays.push(us);
            Ok(())
        }),
        Err(BlockError::Unsupported("PCI function did not enter D0"))
    );
    assert_eq!(delays, [D3HOT_DELAY_US]);
}

#[test]
fn power_up_refuses_invalid_clock_before_any_configuration_access() {
    struct NoAccess;
    impl ConfigSpace for NoAccess {
        fn read32(&self, _: u16) -> Result<u32, crate::bus::pci::ConfigError> {
            panic!("config access before clock admission")
        }
        fn write32(&mut self, _: u16, _: u32) -> Result<(), crate::bus::pci::ConfigError> {
            panic!("config effect before clock admission")
        }

        fn read8(&self, offset: u16) -> Result<u8, crate::bus::pci::ConfigError> {
            Ok((self.read32(offset & !3)? >> ((offset & 3) * 8)) as u8)
        }

        fn write8(&mut self, offset: u16, value: u8) -> Result<(), crate::bus::pci::ConfigError> {
            let shift = (offset & 3) * 8;
            let word = self.read32(offset & !3)? & !(0xff << shift) | u32::from(value) << shift;
            self.write32(offset & !3, word)
        }

        fn read16(&self, offset: u16) -> Result<u16, crate::bus::pci::ConfigError> {
            Ok((self.read32(offset & !3)? >> ((offset & 2) * 8)) as u16)
        }

        fn write16(&mut self, offset: u16, value: u16) -> Result<(), crate::bus::pci::ConfigError> {
            let shift = (offset & 2) * 8;
            let word = self.read32(offset & !3)? & !(0xffff << shift) | u32::from(value) << shift;
            self.write32(offset & !3, word)
        }
    }
    struct MissingMinimum {
        _nominal_rate: u64,
    }
    // SAFETY: this diagnostic provider never grants a successful minimum wait.
    unsafe impl power::Timer for MissingMinimum {
        fn check(&self) -> Result<(), power::Error> {
            Err(power::Error::Clock)
        }
        fn wait_us(&self, _: u64) -> Result<(), power::Error> {
            panic!("wait after missing minimum-delay refusal")
        }
    }
    for rate in [0, u64::MAX, u64::MAX / 2 / D3HOT_DELAY_US + 1] {
        let mut function = Function {
            address: Address::default_for_tests(),
            vendor: 0x1af4,
            device: 0x1042,
            class: [1, 8, 2],
            config: NoAccess,
        };
        // Nominal frequency data cannot supply the missing physical provider.
        let minimum = MissingMinimum {
            _nominal_rate: rate,
        };
        assert_eq!(
            function.power_up(&minimum),
            Err(BlockError::PciPower(power::Error::Clock))
        );
        assert_eq!(
            function.function_level_reset(&minimum),
            Err(BlockError::PciPower(power::Error::Clock))
        );
    }
}

/// An ECAM window over buses 0 and 1, every function absent.
struct Ecam(Vec<u32>);

impl Ecam {
    fn new() -> Self {
        Self(alloc::vec![u32::MAX; 2 << 18])
    }
    fn word(&mut self, bus: usize, device: usize, function: usize, offset: usize) -> &mut u32 {
        &mut self.0[(bus << 18) | (device << 13) | (function << 10) | (offset / 4)]
    }
    /// A function of header type `header`, with no capabilities.
    fn present(&mut self, bus: usize, device: usize, function: usize, header: u32) {
        *self.word(bus, device, function, 0) = 0x1234_8086;
        *self.word(bus, device, function, 0x04) = 0;
        *self.word(bus, device, function, 0x08) = 0;
        *self.word(bus, device, function, 0x0c) = header << 16;
        *self.word(bus, device, function, 0x100) = 0;
    }
    /// A PCI Express port of `kind` on 0:1.0 with bus 1 below it.
    fn port(&mut self, kind: u16, control_2: u16) {
        self.present(0, 1, 0, 1);
        *self.word(0, 1, 0, 0x04) = 1 << 20; // Status: capabilities list
        *self.word(0, 1, 0, 0x18) = 1 << 8;
        *self.word(0, 1, 0, 0x34) = 0x40;
        *self.word(0, 1, 0, 0x40) =
            u32::from(CAPABILITY_EXPRESS) | u32::from(kind << EXPRESS_TYPE_SHIFT) << 16;
        *self.word(0, 1, 0, 0x40 + usize::from(EXPRESS_DEVICE_CONTROL_2)) = u32::from(control_2);
    }
    fn found(&self) -> Vec<(u8, u8, u8)> {
        let window = EcamWindow {
            base: self.0.as_ptr() as u64,
            segment: 0,
            start_bus: 0,
            end_bus: 1,
        };
        // SAFETY: this test exclusively owns the backing allocation.
        unsafe { Function::enumerate(&window) }
            .unwrap()
            .iter()
            .map(|function| {
                let address = function.address;
                (address.bus, address.device, address.function)
            })
            .collect()
    }
}

#[test]
fn below_a_root_port_only_device_zero_is_scanned() {
    let mut ecam = Ecam::new();
    ecam.port(EXPRESS_ROOT_PORT, 0);
    // A device that answers at every device number below the port.
    for device in 0..32 {
        ecam.present(1, device, 0, 0);
    }
    // A function still initializing is not a function.
    ecam.present(0, 2, 0, 0);
    *ecam.word(0, 2, 0, 0) = 0xffff_0001;
    assert_eq!(ecam.found(), [(0, 1, 0), (1, 0, 0)]);
}

#[test]
fn below_a_port_forwarding_ari_the_functions_follow_their_chain() {
    let mut ecam = Ecam::new();
    ecam.port(EXPRESS_DOWNSTREAM, ARI_FORWARDING);
    for (number, next) in [(0usize, 9u32), (9, 0)] {
        ecam.present(1, number >> 3, number & 7, 0);
        // PCI Express, which alone has the extended list ARI is in.
        *ecam.word(1, number >> 3, number & 7, 0x04) = 1 << 20;
        *ecam.word(1, number >> 3, number & 7, 0x34) = 0x40;
        *ecam.word(1, number >> 3, number & 7, 0x40) = u32::from(CAPABILITY_EXPRESS) | 2 << 16;
        *ecam.word(1, number >> 3, number & 7, 0x100) = u32::from(EXTENDED_ARI);
        *ecam.word(1, number >> 3, number & 7, 0x104) = next << 8;
    }
    // Not in the chain.
    ecam.present(1, 0, 1, 0);
    assert_eq!(ecam.found(), [(0, 1, 0), (1, 0, 0), (1, 1, 1)]);
}

/// A laptop's slot 3: function 0 has the multifunction bit, and functions
/// 2 to 4, which firmware names, read all zeros.
#[test]
fn functions_that_read_all_zeros_are_not_functions() {
    let mut ecam = Ecam::new();
    ecam.present(0, 3, 0, 0x80);
    for function in 2..=4 {
        for offset in [0, 0x04, 0x08, 0x0c] {
            *ecam.word(0, 3, function, offset) = 0;
        }
    }
    assert_eq!(ecam.found(), [(0, 3, 0)]);
}

#[test]
fn below_a_conventional_bridge_every_device_is_scanned() {
    let mut ecam = Ecam::new();
    ecam.present(0, 1, 0, 1);
    *ecam.word(0, 1, 0, 0x18) = 1 << 8;
    ecam.present(1, 0, 0, 0);
    ecam.present(1, 5, 0, 0);
    assert_eq!(ecam.found(), [(0, 1, 0), (1, 0, 0), (1, 5, 0)]);
}

#[test]
fn a_bus_no_bridge_leads_to_is_not_scanned() {
    let mut ecam = Ecam::new();
    ecam.present(0, 3, 0, 0);
    // Bus 1 answers, but nothing on the root bus leads to it.
    ecam.present(1, 0, 0, 0);
    assert_eq!(ecam.found(), [(0, 3, 0)]);
}

#[test]
fn a_bridge_whose_capabilities_cycle_hides_nothing_behind_or_after_it() {
    let mut ecam = Ecam::new();
    ecam.port(EXPRESS_ROOT_PORT, 0);
    // A vendor capability at 0x50 whose next pointer leads back to itself,
    // ahead of the Express capability: the port type cannot be read.
    *ecam.word(0, 1, 0, 0x34) = 0x50;
    *ecam.word(0, 1, 0, 0x50) = 0x09 | 0x50 << 8 | 4 << 16;
    // Without the port type, every device number below it is scanned.
    ecam.present(1, 0, 0, 0);
    ecam.present(1, 4, 0, 0);
    // A function after the bridge in scan order.
    ecam.present(0, 2, 0, 0);
    assert_eq!(ecam.found(), [(0, 1, 0), (0, 2, 0), (1, 0, 0), (1, 4, 0)]);
}

#[test]
fn a_bridge_with_its_express_capability_before_a_bad_link_keeps_its_port_type() {
    let mut ecam = Ecam::new();
    ecam.port(EXPRESS_ROOT_PORT, 0);
    // The Express capability's next pointer is misaligned.
    *ecam.word(0, 1, 0, 0x40) |= 0x53 << 8;
    for device in 0..32 {
        ecam.present(1, device, 0, 0);
    }
    assert_eq!(ecam.found(), [(0, 1, 0), (1, 0, 0)]);
}

#[test]
fn windows_narrow_to_each_root_bridge_from_its_root_bus() {
    let window = |segment, start_bus, end_bus| EcamWindow {
        base: 0xe000_0000,
        segment,
        start_bus,
        end_bus,
    };
    let mcfg = [window(0, 0, 0xff), window(1, 0, 0x3f)];
    assert_eq!(
        rooted(
            &mcfg,
            &[
                (0, 0..=0x7f),
                (0, 0x80..=0xff),
                (1, 0x10..=0xff),
                (2, 0..=0xff)
            ]
        ),
        [
            window(0, 0, 0x7f),
            window(0, 0x80, 0xff),
            window(1, 0x10, 0x3f)
        ]
    );
}

#[test]
fn ecam_ranges_cover_the_decoded_buses() {
    let window = EcamWindow {
        base: 0xb000_0000,
        segment: 0,
        start_bus: 2,
        end_bus: 3,
    };
    assert_eq!(window.range(), 0xb020_0000..0xb040_0000);
}

#[test]
fn copied_identity_performs_no_additional_configuration_reads() {
    use core::cell::Cell;
    struct Counted {
        reads: Cell<usize>,
    }
    impl ConfigSpace for Counted {
        fn read32(&self, offset: u16) -> Result<u32, crate::bus::pci::ConfigError> {
            Ok({
                self.reads.set(self.reads.get() + 1);
                match offset {
                    0 => 0x1001_1af4,
                    8 => 0x0100_0000,
                    _ => 0,
                }
            })
        }
        fn write32(
            &mut self,
            _offset: u16,
            _value: u32,
        ) -> Result<(), crate::bus::pci::ConfigError> {
            panic!("identity observation must not write configuration");
        }

        fn read8(&self, offset: u16) -> Result<u8, crate::bus::pci::ConfigError> {
            Ok((self.read32(offset & !3)? >> ((offset & 3) * 8)) as u8)
        }

        fn write8(&mut self, offset: u16, value: u8) -> Result<(), crate::bus::pci::ConfigError> {
            let shift = (offset & 3) * 8;
            let word = self.read32(offset & !3)? & !(0xff << shift) | u32::from(value) << shift;
            self.write32(offset & !3, word)
        }

        fn read16(&self, offset: u16) -> Result<u16, crate::bus::pci::ConfigError> {
            Ok((self.read32(offset & !3)? >> ((offset & 2) * 8)) as u16)
        }

        fn write16(&mut self, offset: u16, value: u16) -> Result<(), crate::bus::pci::ConfigError> {
            let shift = (offset & 2) * 8;
            let word = self.read32(offset & !3)? & !(0xffff << shift) | u32::from(value) << shift;
            self.write32(offset & !3, word)
        }
    }
    let function = Function::new(
        Address::default_for_tests(),
        Counted {
            reads: Cell::new(0),
        },
    )
    .unwrap();
    let before = function.config.reads.get();
    let observed = function.identity();
    assert_eq!(function.config.reads.get(), before);
    assert_eq!(observed.address, function.address);
    assert_eq!(observed.vendor, function.vendor);
    assert_eq!(observed.device, function.device);
    assert_eq!(observed.class, function.class);
}

impl Address {
    fn default_for_tests() -> Self {
        Self {
            segment: 0,
            bus: 0,
            device: 3,
            function: 0,
        }
    }
}

/// A function a reset empties, silent for `silence` reads after one.
struct Resettable {
    words: [u32; 64],
    silence: u32,
    busy: core::cell::Cell<u32>,
    resets: u32,
}

impl ConfigSpace for Resettable {
    fn read32(&self, offset: u16) -> Result<u32, crate::bus::pci::ConfigError> {
        Ok({
            if offset == 0 && self.busy.get() != 0 {
                self.busy.set(self.busy.get() - 1);
                return Ok(u32::MAX);
            }
            self.words[usize::from(offset / 4)]
        })
    }

    fn write32(&mut self, offset: u16, value: u32) -> Result<(), crate::bus::pci::ConfigError> {
        let before = self.words[usize::from(offset / 4)];
        self.words[usize::from(offset / 4)] = value;
        let soft_reset = offset == 0x84
            && before & 3 == 3
            && value & 3 == 0
            && before & u32::from(PM_NO_SOFT_RESET) == 0;
        if soft_reset {
            self.words[0x84 / 4] = 0;
        }
        if soft_reset || offset == 0x48 && value & 1 << 15 != 0 {
            self.words[0x48 / 4] = 0;
            self.words[1] = 0x0010_0000;
            self.words[4] = 0;
            self.words[0x50 / 4] = 0;
            self.words[0x68 / 4] = 0;
            self.words[0x70 / 4] = 0;
            self.busy.set(self.silence);
            self.resets += 1;
        }
        Ok(())
    }

    fn read8(&self, offset: u16) -> Result<u8, crate::bus::pci::ConfigError> {
        Ok((self.read32(offset & !3)? >> ((offset & 3) * 8)) as u8)
    }

    fn write8(&mut self, offset: u16, value: u8) -> Result<(), crate::bus::pci::ConfigError> {
        let shift = (offset & 3) * 8;
        let word = self.read32(offset & !3)? & !(0xff << shift) | u32::from(value) << shift;
        self.write32(offset & !3, word)
    }

    fn read16(&self, offset: u16) -> Result<u16, crate::bus::pci::ConfigError> {
        Ok((self.read32(offset & !3)? >> ((offset & 2) * 8)) as u16)
    }

    fn write16(&mut self, offset: u16, value: u16) -> Result<(), crate::bus::pci::ConfigError> {
        let shift = (offset & 2) * 8;
        let word = self.read32(offset & !3)? & !(0xffff << shift) | u32::from(value) << shift;
        self.write32(offset & !3, word)
    }
}

fn resettable(capabilities: u32, silence: u32) -> Function<Resettable> {
    let mut words = [0; 64];
    words[0] = 0x1234_8086;
    words[1] = 0x0010_0006;
    words[4] = 0xfe00_0000;
    words[0x34 / 4] = 0x40;
    words[0x40 / 4] = 0x0002_8010;
    words[0x44 / 4] = capabilities;
    words[0x48 / 4] = 0x0020_2810;
    words[0x50 / 4] = 0x0143;
    words[0x68 / 4] = 0x0400;
    words[0x70 / 4] = 3;
    words[0x80 / 4] = 0x0003_0001;
    Function {
        address: Address::default_for_tests(),
        vendor: 0x8086,
        device: 0x1234,
        class: [2, 0x80, 0],
        config: Resettable {
            words,
            silence,
            busy: core::cell::Cell::new(0),
            resets: 0,
        },
    }
}

#[test]
fn a_function_reset_waits_for_requests_and_the_function_and_puts_its_setup_back() {
    let mut function = resettable(1 << 28, 2);
    let mut waits = Vec::new();
    let reset = function.function_level_reset_with(|microseconds| {
        waits.push(microseconds);
        Ok(())
    });
    assert_eq!(reset, Ok(true));
    // A request stands through all its waits, and the function stays silent
    // through two.
    assert_eq!(waits, [100_000, 200_000, 400_000, 100_000, 1_000, 2_000]);
    let config = &function.config;
    assert_eq!(config.resets, 1);
    assert_eq!(config.words[1] & 0xffff, 6);
    assert_eq!(config.words[4], 0xfe00_0000);
    assert_eq!(config.words[0x48 / 4] & 0xffff, 0x2810);
    assert_eq!(config.words[0x50 / 4], 0x0143);
    assert_eq!(config.words[0x68 / 4], 0x0400);
    assert_eq!(config.words[0x70 / 4], 3);
}

#[test]
fn a_function_without_a_reset_is_left_alone_and_one_that_stays_silent_is_an_error() {
    let mut function = resettable(0, 0);
    assert_eq!(function.function_level_reset_with(|_| Ok(())), Ok(false));
    assert_eq!(function.config.resets, 0);
    let mut function = resettable(1 << 28, u32::MAX);
    assert_eq!(
        function.function_level_reset_with(|_| Ok(())),
        Err(BlockError::Unsupported(
            "PCI function did not return from its reset"
        ))
    );
}

#[test]
fn a_function_without_flr_is_reset_through_d3hot_when_its_exit_resets_it() {
    let mut function = resettable(0, 1);
    assert_eq!(
        function.reset_method(),
        Ok(Some(ResetMethod::PowerManagement))
    );
    let mut waits = Vec::new();
    let reset = function.reset_with(|microseconds| {
        waits.push(microseconds);
        Ok(())
    });
    assert_eq!(reset, Ok(Some(ResetMethod::PowerManagement)));
    assert_eq!(waits, [10_000, 10_000, 1_000]);
    let config = &function.config;
    assert_eq!(config.resets, 1);
    assert_eq!(config.words[0x84 / 4] & 3, 0);
    assert_eq!(config.words[1] & 0xffff, 6);
    assert_eq!(config.words[4], 0xfe00_0000);
    assert_eq!(config.words[0x48 / 4] & 0xffff, 0x2810);
    assert_eq!(config.words[0x70 / 4], 3);
}

#[test]
fn flr_is_preferred_and_no_soft_reset_or_a_low_power_state_offers_no_reset() {
    let mut function = resettable(1 << 28, 0);
    function.config.words[0x84 / 4] = u32::from(PM_NO_SOFT_RESET);
    assert_eq!(
        function.reset_method(),
        Ok(Some(ResetMethod::FunctionLevel))
    );
    assert_eq!(
        function.reset_with(|_| Ok(())),
        Ok(Some(ResetMethod::FunctionLevel))
    );
    for control in [u32::from(PM_NO_SOFT_RESET), 3] {
        let mut function = resettable(0, 0);
        function.config.words[0x84 / 4] = control;
        assert_eq!(function.reset_method(), Ok(None));
        assert_eq!(function.reset_with(|_| Ok(())), Ok(None));
        assert_eq!(function.function_level_reset_with(|_| Ok(())), Ok(false));
        assert_eq!(function.config.resets, 0);
        assert_eq!(function.config.words[0x84 / 4], control);
    }
}

#[test]
fn an_unanswering_capability_is_an_error_not_a_reset() {
    let mut function = resettable(u32::MAX, 0);
    assert_eq!(
        function.reset_with(|_| Ok(())),
        Err(BlockError::Unsupported("PCI function does not answer"))
    );
    assert_eq!(function.config.resets, 0);
}
