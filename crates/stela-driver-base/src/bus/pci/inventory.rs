//! Fallible discovery keeps failed observations with their original owners.
use super::*;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum EnumerationError {
    Config {
        address: Address,
        error: ConfigError,
    },
    Capability {
        address: Address,
        error: CapabilityError,
    },
}
impl EnumerationError {
    pub fn address(&self) -> Address {
        match *self {
            Self::Config { address, .. } | Self::Capability { address, .. } => address,
        }
    }
}

/// A plain account for the boot log and screen.
impl core::fmt::Display for EnumerationError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Config { address, error } => write!(f, "{address}: {error}"),
            Self::Capability { address, error } => write!(f, "{address}: {error}"),
        }
    }
}
pub struct EnumerationFailure {
    pub error: EnumerationError,
    pub functions: Vec<Function>,
    pub refused: Option<FunctionRefusal<EcamConfig>>,
}
impl EnumerationFailure {
    pub fn address(&self) -> Address {
        self.error.address()
    }
}
impl fmt::Debug for EnumerationFailure {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("EnumerationFailure")
            .field("error", &self.error)
            .field("kept_functions", &self.functions.len())
            .finish_non_exhaustive()
    }
}
#[derive(Debug)]
pub enum InventoryError<E> {
    Open(E),
    Release(E),
    Configuration(EnumerationError),
}
pub struct InventoryFailure<C, E> {
    pub address: Address,
    pub error: InventoryError<E>,
    pub retained: Vec<Function<C>>,
    pub observed: Vec<Identity>,
}
impl<C, E: fmt::Debug> fmt::Debug for InventoryFailure<C, E> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("InventoryFailure")
            .field("address", &self.address)
            .field("error", &self.error)
            .field("kept_functions", &self.retained.len())
            .finish_non_exhaustive()
    }
}
pub struct InventoryReleaseFailure<C, E> {
    pub error: E,
    pub function: Function<C>,
}

type Metadata = (bool, u8, Option<(u8, BusScan)>);
fn metadata<C: ConfigSpace>(
    function: &Function<C>,
    scan: BusScan,
) -> Result<Metadata, EnumerationError> {
    let address = function.address;
    let access = |error| EnumerationError::Config { address, error };
    let capability = |error| EnumerationError::Capability { address, error };
    let multi = function.config.read8(regs::HEADER_TYPE).map_err(access)? & 0x80 != 0;
    let next = if scan == BusScan::Ari {
        match function
            .extended_capability(EXTENDED_ARI)
            .map_err(capability)?
        {
            Some(ari) => (function.config.read32(ari + 4).map_err(access)? >> 8) as u8,
            None => 0,
        }
    } else {
        0
    };
    Ok((multi, next, function.secondary().map_err(capability)?))
}
impl<C: ConfigSpace> Function<C> {
    /// The callbacks keep all physical/root ownership. A release refusal must
    /// return the same actual observation; transport denial is never absence.
    pub fn inventory_with<E>(
        window: &EcamWindow,
        mut open: impl FnMut(Address) -> Result<Option<Self>, E>,
        mut release: impl FnMut(Self) -> Result<(), InventoryReleaseFailure<C, E>>,
    ) -> Result<Vec<Identity>, InventoryFailure<C, E>> {
        let mut scans = [None; 256];
        scans[usize::from(window.start_bus)] = Some(BusScan::All);
        let mut observed = Vec::new();
        for bus in window.start_bus..=window.end_bus {
            let Some(scan) = scans[usize::from(bus)] else {
                continue;
            };
            let mut visit = |address| -> Result<Option<(bool, u8)>, InventoryFailure<C, E>> {
                let function = match open(address) {
                    Ok(Some(function)) => function,
                    Ok(None) => return Ok(None),
                    Err(error) => {
                        return Err(InventoryFailure {
                            address,
                            error: InventoryError::Open(error),
                            retained: Vec::new(),
                            observed: core::mem::take(&mut observed),
                        });
                    }
                };
                let mut retained = kept::Kept::new(function);
                let (multi, next, secondary) = match metadata(retained.get_mut(), scan) {
                    Ok(value) => value,
                    Err(error) => {
                        return Err(InventoryFailure {
                            address,
                            error: InventoryError::Configuration(error),
                            retained: alloc::vec![retained.take()],
                            observed: core::mem::take(&mut observed),
                        });
                    }
                };
                let function = retained.take();
                let identity = function.identity();
                if let Err(failure) = release(function) {
                    return Err(InventoryFailure {
                        address,
                        error: InventoryError::Release(failure.error),
                        retained: alloc::vec![failure.function],
                        observed: core::mem::take(&mut observed),
                    });
                }
                if let Some((secondary, scan)) = secondary
                    && secondary > bus
                {
                    scans[usize::from(secondary)] = Some(scan);
                }
                observed.push(identity);
                Ok(Some((multi, next)))
            };
            if scan == BusScan::Ari {
                let mut number = 0;
                loop {
                    let address = Address {
                        segment: window.segment,
                        bus,
                        device: number >> 3,
                        function: number & 7,
                    };
                    let Some((_, next)) = visit(address)? else {
                        break;
                    };
                    if next <= number {
                        break;
                    }
                    number = next;
                }
            } else {
                for device in 0..if scan == BusScan::OneChild { 1 } else { 32 } {
                    for function in 0..8 {
                        let address = Address {
                            segment: window.segment,
                            bus,
                            device,
                            function,
                        };
                        match visit(address)? {
                            None if function == 0 => break,
                            None => continue,
                            Some((multi, _)) if function == 0 && !multi => break,
                            Some(_) => {}
                        }
                    }
                }
            }
        }
        Ok(observed)
    }
}
impl Function {
    /// # Safety
    /// The entire original ECAM/root assignment stays mapped UC, exclusively
    /// controlled and retained by the caller. These raw observations grant no
    /// native function/resource owner; real native discovery uses its producer.
    #[expect(
        clippy::result_large_err,
        reason = "enumeration failures return every original observation owner without failure-path allocation"
    )]
    pub unsafe fn enumerate(window: &EcamWindow) -> Result<Vec<Self>, EnumerationFailure> {
        let mut scans = [None; 256];
        scans[usize::from(window.start_bus)] = Some(BusScan::All);
        let mut functions = Vec::new();
        for bus in window.start_bus..=window.end_bus {
            let Some(scan) = scans[usize::from(bus)] else {
                continue;
            };
            let mut visit = |address: Address| -> Result<Option<(bool, u8)>, EnumerationFailure> {
                // SAFETY: this method's original mapped/excluded root contract.
                let config =
                    unsafe { window.config(address.bus, address.device, address.function) };
                let function = match Self::new(address, config) {
                    // Vendor ID 0000 is no vendor's: as Linux reads an
                    // all-zero identity, it is no function. Firmware names
                    // such functions, as ACPI does a multifunction device's
                    // absent ones, and the native census leaves them out.
                    Ok(function) if function.vendor == 0 => {
                        drop(function);
                        return Ok(None);
                    }
                    Ok(function) => function,
                    Err(refused) => match refused.reason {
                        FunctionError::Absent | FunctionError::ConfigurationRetry => {
                            // Only this raw observation view ends. Its original
                            // physical/root owner remains with the unsafe caller.
                            drop(refused.config);
                            return Ok(None);
                        }
                        FunctionError::Access(error) => {
                            return Err(EnumerationFailure {
                                error: EnumerationError::Config { address, error },
                                functions: core::mem::take(&mut functions),
                                refused: Some(refused),
                            });
                        }
                    },
                };
                let data = metadata(&function, scan);
                functions.push(function);
                let (multi, next, secondary) = match data {
                    Ok(data) => data,
                    Err(error) => {
                        return Err(EnumerationFailure {
                            error,
                            functions: core::mem::take(&mut functions),
                            refused: None,
                        });
                    }
                };
                if let Some((secondary, scan)) = secondary
                    && secondary > bus
                {
                    scans[usize::from(secondary)] = Some(scan);
                }
                Ok(Some((multi, next)))
            };
            if scan == BusScan::Ari {
                let mut number = 0;
                loop {
                    let address = Address {
                        segment: window.segment,
                        bus,
                        device: number >> 3,
                        function: number & 7,
                    };
                    let Some((_, next)) = visit(address)? else {
                        break;
                    };
                    if next <= number {
                        break;
                    }
                    number = next;
                }
            } else {
                for device in 0..if scan == BusScan::OneChild { 1 } else { 32 } {
                    for function in 0..8 {
                        let address = Address {
                            segment: window.segment,
                            bus,
                            device,
                            function,
                        };
                        match visit(address)? {
                            None if function == 0 => break,
                            None => continue,
                            Some((multi, _)) if function == 0 && !multi => break,
                            Some(_) => {}
                        }
                    }
                }
            }
        }
        Ok(functions)
    }
}
