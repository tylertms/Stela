// SPDX-License-Identifier: GPL-2.0-only
// PCI encodings follow Linux include/uapi/linux/pci_regs.h, copyright
// 1994 Drew Eckhardt and 1997-1999 Martin Mares; see NOTICE.
//! Immutable firmware topology, exclusive child config and ordered Command I/O.
use super::{capabilities::Capabilities, *};
use crate::bus::pci::regs;

const DECODE: u16 = regs::COMMAND_IO | regs::COMMAND_MEMORY;

#[derive(Clone)]
struct Function {
    child: Child,
    capabilities: Capabilities,
    bars: Vec<u32>,
    bridge: Option<Bridge>,
}
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct Bridge {
    buses: u32,
    memory: u32,
    prefetch: u32,
    upper_base: u32,
    upper_limit: u32,
    control: u16,
}
impl Bridge {
    fn secondary(self) -> u8 {
        (self.buses >> 8) as u8
    }
    fn subordinate(self) -> u8 {
        (self.buses >> 16) as u8
    }
    fn contains_bus(self, bus: u8) -> bool {
        (self.secondary()..=self.subordinate()).contains(&bus)
    }
    fn routes(self, base: u64, bytes: u64, prefetch: bool) -> Result<(), Error> {
        let (first, last) = if prefetch {
            let low = self.prefetch as u16;
            let high = (self.prefetch >> 16) as u16;
            if low & 15 != high & 15 || !matches!(low & 15, 0 | 1) {
                return Err(Error::InvalidResource);
            }
            let (upper_base, upper_limit) = if low & 15 == 1 {
                (
                    u64::from(self.upper_base) << 32,
                    u64::from(self.upper_limit) << 32,
                )
            } else {
                (0, 0)
            };
            (
                upper_base | u64::from(low & 0xfff0) << 16,
                upper_limit | u64::from(high & 0xfff0) << 16 | 0x000f_ffff,
            )
        } else {
            if self.memory & 0x000f_000f != 0 {
                return Err(Error::InvalidResource);
            }
            (
                u64::from(self.memory & 0xfff0) << 16,
                u64::from((self.memory >> 16) & 0xfff0) << 16 | 0x000f_ffff,
            )
        };
        let end = base.checked_add(bytes).ok_or(Error::Address)?;
        if first > last || base < first || end == 0 || end - 1 > last {
            return Err(Error::InvalidResource);
        }
        Ok(())
    }
}

/// One endpoint function this hierarchy admitted from its firmware BAR:
/// the bridges above it, the BAR they must route, and where its lifecycle is.
struct Selected {
    index: usize,
    path: Vec<usize>,
    bar_base: u64,
    bar_bytes: u64,
    prefetch: bool,
    /// Powered with memory decode along its path.
    prepared: bool,
    /// Bus mastering along its path.
    dma: bool,
    /// Its driver retired it, or it was abandoned before any effect.
    released: bool,
}

pub(super) struct Hierarchy {
    domain: u32,
    children: Vec<Function>,
    /// Every endpoint admitted, in admission order; an index is its id.
    endpoints: Vec<Selected>,
    windows: [Window; 2],
    buses: (u8, u8),
    first_bus: u8,
    parent_id: u32,
    parent_bars: [u32; 6],
}

fn config<R: Registers, T>(
    host: &mut Host<R>,
    address: Address,
    effect: impl FnOnce(&mut ChildConfig<'_, R>) -> Result<T, Error>,
) -> Result<T, Error> {
    if address.domain != host.domain {
        return Err(Error::InvalidResource);
    }
    effect(&mut host.config(address.bus, address.device, address.function)?)
}
fn word<R: Registers>(config: &mut ChildConfig<'_, R>, offset: u16) -> Result<u16, Error> {
    let value = config.read(offset, 2)? as u16;
    (value != u16::MAX)
        .then_some(value)
        .ok_or(Error::ReadFailed)
}
fn parent_command<R: Registers>(host: &mut Host<R>) -> Result<u16, Error> {
    let value = host.io().parent_read(regs::COMMAND, 2)? as u16;
    (value != u16::MAX)
        .then_some(value)
        .ok_or(Error::ReadFailed)
}
/// The current Host preparation profile requires its parent already in D0.
/// The parent's capability list is validated again before its PM control is read.
pub(super) fn parent_d0<R: Registers>(host: &mut Host<R>) -> Result<(), Error> {
    if host.io().parent_read(0, 4)? != host.identity {
        return Err(Error::UnsupportedDevice);
    }
    let status = host.io().parent_read(6, 2)?;
    if status == u32::from(u16::MAX) {
        return Err(Error::ReadFailed);
    }
    if status & 0x10 == 0 {
        return Ok(());
    }
    for entry in host.parent_capabilities()? {
        if entry.id == 1 {
            let control = host.io().parent_read(entry.offset + 4, 2)?;
            if control == u32::from(u16::MAX) {
                return Err(Error::ReadFailed);
            }
            if control & 3 != 0 {
                return Err(Error::Suspended);
            }
        }
    }
    Ok(())
}
fn child_command<R: Registers>(
    host: &mut Host<R>,
    address: Address,
    value: u16,
) -> Result<(), Error> {
    host.check_live()?;
    let base = host
        .config(address.bus, address.device, address.function)?
        .base;
    // This private operation is reachable only through the consumed hierarchy
    // lifecycle's real-domain grant. Ordinary ChildConfig still rejects BM.
    let result = (|| {
        host.io()
            .child_write(base + usize::from(regs::COMMAND), 2, u32::from(value))?;
        if host.io().child_read(base + usize::from(regs::COMMAND), 2)? != u32::from(value) {
            return Err(Error::Readback);
        }
        Ok(())
    })();
    if result.is_err() {
        host.poisoned = true;
    }
    result
}
fn capture<R: Registers>(host: &mut Host<R>, child: Child) -> Result<Function, Error> {
    config(host, child.address, |config| {
        let bridge = child.header == 1;
        if !matches!(child.header, 0 | 1) || (bridge && child.class != [6, 4, 0]) {
            return Err(Error::UnsupportedDevice);
        }
        if word(config, regs::COMMAND)? & (regs::COMMAND_MASTER | regs::COMMAND_INTX_DISABLE)
            != regs::COMMAND_INTX_DISABLE
        {
            return Err(Error::ActiveChild);
        }
        let capabilities = Capabilities::inspect(config, bridge)?;
        let mut bars = Vec::new();
        for i in 0..if bridge { 2 } else { 6 } {
            bars.push(config.read(0x10 + i * 4, 4)?);
        }
        let bridge = if bridge {
            let control = word(config, 0x3e)?;
            if control & (1 << 6) != 0 {
                return Err(Error::ActiveChild);
            }
            Some(Bridge {
                buses: config.read(0x18, 4)?,
                memory: config.read(0x20, 4)?,
                prefetch: config.read(0x24, 4)?,
                upper_base: config.read(0x28, 4)?,
                upper_limit: config.read(0x2c, 4)?,
                control,
            })
        } else {
            None
        };
        Ok(Function {
            child,
            capabilities,
            bars,
            bridge,
        })
    })
}
impl Hierarchy {
    /// Stop and capture every function below `host`, then admit `bar`'s
    /// function as the first endpoint (id 0).
    pub(super) fn discover<R: Registers>(
        host: &mut Host<R>,
        bar: &FirmwareBar,
        class: [u8; 3],
    ) -> Result<Self, Error> {
        if host.domain != bar.domain {
            return Err(Error::ActiveChild);
        }
        let mut hierarchy = Self::capture(host)?;
        hierarchy.admit(bar, class)?;
        Ok(hierarchy)
    }

    /// Stop every function firmware left mastering or signalling, while the
    /// parent's own Bus Master is still clear, and capture the topology.
    fn capture<R: Registers>(host: &mut Host<R>) -> Result<Self, Error> {
        host.check_live()?;
        parent_d0(host)?;
        if host.programmed_vectors.is_some() || parent_command(host)? & regs::COMMAND_MASTER != 0 {
            return Err(Error::ActiveChild);
        }
        let mut children = Vec::new();
        for child in host.scan()? {
            host.stop_child(child.address)?;
            children.push(capture(host, child)?);
        }
        for function in &children {
            if let Some(bridge) = function.bridge
                && (bridge.buses as u8 != function.child.address.bus
                    || bridge.secondary() <= function.child.address.bus
                    || bridge.secondary() > bridge.subordinate()
                    || bridge.subordinate() > host.last_bus
                    || !function.capabilities.is_pcie())
            {
                return Err(Error::InvalidResource);
            }
        }
        // Every enumerated bus must have one real path. This rejects stray
        // unowned functions, overlapping sibling ranges and cycles rather than
        // selecting whichever bridge happened to be enumerated first.
        for (index, function) in children.iter().enumerate() {
            let ancestors = Self::path(&children, host.first_bus, function.child.address.bus)?;
            if let Some(bridge) = function.bridge {
                for ancestor in ancestors {
                    if bridge.subordinate()
                        > children[ancestor].bridge.expect("ancestor").subordinate()
                    {
                        return Err(Error::InvalidResource);
                    }
                }
                if children[..index].iter().any(|other| {
                    other.child.address.bus == function.child.address.bus
                        && other.bridge.is_some_and(|other| {
                            bridge.secondary() <= other.subordinate()
                                && other.secondary() <= bridge.subordinate()
                        })
                }) {
                    return Err(Error::InvalidResource);
                }
            }
        }
        let parent_id = host.io().parent_read(0, 4)?;
        let mut parent_bars = [0; 6];
        for (i, value) in parent_bars.iter_mut().enumerate() {
            *value = host.io().parent_read(0x10 + i as u16 * 4, 4)?;
        }
        Ok(Self {
            domain: host.domain,
            children,
            endpoints: Vec::new(),
            windows: host.windows,
            buses: (host.first_bus, host.last_bus),
            first_bus: host.first_bus,
            parent_id,
            parent_bars,
        })
    }

    /// Admit `bar`'s function, which must be of `class`, as another
    /// endpoint, as Linux's VMD driver enumerates every NVMe controller below
    /// one host: each keeps its own firmware BAR inside the parent's windows,
    /// routed by every bridge above it. Nothing is written; its lifecycle
    /// starts at [`Self::prepare`]. Every function no endpoint is admitted
    /// for stays stopped, without decode or bus mastering.
    pub(super) fn admit(&mut self, bar: &FirmwareBar, class: [u8; 3]) -> Result<usize, Error> {
        if bar.domain != self.domain {
            return Err(Error::ActiveChild);
        }
        let index = self
            .children
            .iter()
            .position(|function| function.child.address == bar.child)
            .ok_or(Error::InvalidResource)?;
        let endpoint = &self.children[index];
        if endpoint.child.class != class {
            return Err(Error::UnsupportedDevice);
        }
        if endpoint.child.header != 0
            || !endpoint.capabilities.is_pcie()
            || endpoint.bars[0] != bar.low
            || (bar.low & 6 == 4 && endpoint.bars[1] != bar.high)
        {
            return Err(Error::InvalidResource);
        }
        // One endpoint per function, and no two firmware BARs overlap.
        let end = bar.bus_start.checked_add(bar.bytes).ok_or(Error::Address)?;
        if self.endpoints.iter().any(|other| {
            other.index == index
                || (bar.bus_start < other.bar_base + other.bar_bytes && other.bar_base < end)
        }) {
            return Err(Error::InvalidResource);
        }
        let path = Self::path(&self.children, self.first_bus, endpoint.child.address.bus)?;
        let (_, prefetch) = bar_address(bar.low, bar.high, bar.bytes)?;
        for &index in &path {
            self.children[index].bridge.expect("path bridge").routes(
                bar.bus_start,
                bar.bytes,
                prefetch,
            )?;
        }
        self.endpoints.push(Selected {
            index,
            path,
            bar_base: bar.bus_start,
            bar_bytes: bar.bytes,
            prefetch,
            prepared: false,
            dma: false,
            released: false,
        });
        Ok(self.endpoints.len() - 1)
    }

    fn path(children: &[Function], first: u8, target: u8) -> Result<Vec<usize>, Error> {
        let mut current = first;
        let mut path = Vec::new();
        while current != target {
            let mut candidates = children.iter().enumerate().filter(|(_, function)| {
                function.child.address.bus == current
                    && function
                        .bridge
                        .is_some_and(|bridge| bridge.contains_bus(target))
            });
            let (index, function) = candidates.next().ok_or(Error::InvalidResource)?;
            if candidates.next().is_some() {
                return Err(Error::InvalidResource);
            }
            let next = function.bridge.expect("selected bridge").secondary();
            if next <= current || next > target {
                return Err(Error::InvalidResource);
            }
            path.push(index);
            current = next;
        }
        Ok(path)
    }

    fn selected(&self, id: usize) -> Result<&Selected, Error> {
        self.endpoints
            .get(id)
            .filter(|endpoint| !endpoint.released)
            .ok_or(Error::InvalidResource)
    }

    /// The bridges above endpoint `id`, then the endpoint itself.
    fn order(&self, id: usize) -> Result<Vec<usize>, Error> {
        let endpoint = self.selected(id)?;
        let mut order = endpoint.path.clone();
        order.push(endpoint.index);
        Ok(order)
    }

    /// Whether `index` is a member of a path, endpoint included, of an
    /// endpoint other than `except` that `wanted` selects.
    fn shared(&self, index: usize, except: usize, wanted: impl Fn(&Selected) -> bool) -> bool {
        self.endpoints.iter().enumerate().any(|(id, endpoint)| {
            id != except
                && !endpoint.released
                && wanted(endpoint)
                && (endpoint.index == index || endpoint.path.contains(&index))
        })
    }

    /// Whether `index` masters for some endpoint now.
    fn mastering(&self, index: usize) -> bool {
        self.shared(index, usize::MAX, |endpoint| endpoint.dma)
    }

    /// Whether each admitted endpoint other than `id` has retired.
    pub(super) fn endpoints_except(&self, id: usize) -> impl Iterator<Item = bool> + '_ {
        self.endpoints
            .iter()
            .enumerate()
            .filter(move |(other, _)| *other != id)
            .map(|(_, endpoint)| endpoint.released)
    }

    /// Endpoint `id` retired with the host itself.
    pub(super) fn release_last(&mut self, id: usize) {
        if let Some(endpoint) = self.endpoints.get_mut(id) {
            endpoint.prepared = false;
            endpoint.dma = false;
            endpoint.released = true;
        }
    }

    /// Give up endpoint `id` before its lifecycle made any effect.
    pub(super) fn abandon(&mut self, id: usize) {
        if let Some(endpoint) = self.endpoints.get_mut(id)
            && !endpoint.prepared
            && !endpoint.dma
        {
            endpoint.released = true;
        }
    }

    pub(super) fn recheck<R: Registers>(&self, host: &mut Host<R>) -> Result<(), Error> {
        host.check_live()?;
        parent_d0(host)?;
        if host.domain != self.domain || host.io().parent_read(0, 4)? != self.parent_id {
            return Err(Error::InvalidResource);
        }
        for (index, expected) in self.parent_bars.iter().enumerate() {
            if host.io().parent_read(0x10 + index as u16 * 4, 4)? != *expected {
                return Err(Error::InvalidResource);
            }
        }
        // Re-read shadow/config bus restrictions, rather than comparing
        // two cached copies while hardware routing has changed underneath.
        host.first_bus = 0;
        host.discover()?;
        if host.windows != self.windows || (host.first_bus, host.last_bus) != self.buses {
            return Err(Error::InvalidResource);
        }
        let enabled = self.endpoints.iter().any(|endpoint| endpoint.dma);
        let parent = parent_command(host)?;
        if parent & regs::COMMAND_MASTER != if enabled { regs::COMMAND_MASTER } else { 0 }
            || parent & regs::COMMAND_INTX_DISABLE == 0
        {
            return Err(Error::ActiveChild);
        }
        let children = host.scan()?;
        if children.len() != self.children.len() {
            return Err(Error::InvalidResource);
        }
        for (index, (actual, saved)) in children.iter().zip(&self.children).enumerate() {
            if *actual != saved.child {
                return Err(Error::InvalidResource);
            }
            config(host, saved.child.address, |config| {
                let command = word(config, regs::COMMAND)?;
                let dma = self.mastering(index);
                if command & (regs::COMMAND_MASTER | regs::COMMAND_INTX_DISABLE)
                    != (regs::COMMAND_INTX_DISABLE | if dma { regs::COMMAND_MASTER } else { 0 })
                {
                    return Err(Error::ActiveChild);
                }
                if Capabilities::inspect(config, saved.bridge.is_some())? != saved.capabilities {
                    return Err(Error::Capability);
                }
                if dma {
                    saved.capabilities.check_d0(config)?;
                }
                for (i, value) in saved.bars.iter().enumerate() {
                    if config.read(0x10 + i as u16 * 4, 4)? != *value {
                        return Err(Error::InvalidResource);
                    }
                }
                if let Some(bridge) = saved.bridge {
                    let current = Bridge {
                        buses: config.read(0x18, 4)?,
                        memory: config.read(0x20, 4)?,
                        prefetch: config.read(0x24, 4)?,
                        upper_base: config.read(0x28, 4)?,
                        upper_limit: config.read(0x2c, 4)?,
                        control: word(config, 0x3e)?,
                    };
                    if current != bridge {
                        return Err(Error::InvalidResource);
                    }
                    for endpoint in &self.endpoints {
                        if endpoint.path.contains(&index) {
                            current.routes(
                                endpoint.bar_base,
                                endpoint.bar_bytes,
                                endpoint.prefetch,
                            )?;
                        }
                    }
                }
                Ok(())
            })?;
        }
        Ok(())
    }

    /// Bring endpoint `id`'s path to D0 with memory decode. The first
    /// endpoint prepared also quiesces every other function and enables the
    /// parent's decode; a later one leaves the functions an earlier endpoint
    /// runs on as they are.
    pub(super) fn prepare<R: Registers>(
        &mut self,
        host: &mut Host<R>,
        minimum: &(impl crate::bus::pci::power::Timer + ?Sized),
        id: usize,
    ) -> Result<(), Error> {
        minimum.check()?;
        let order = self.order(id)?;
        if self.endpoints[id].prepared {
            return Err(Error::ActiveChild);
        }
        self.recheck(host)?;
        let first = !self.endpoints.iter().any(|endpoint| endpoint.prepared);
        let live = |hierarchy: &Self, index: usize| {
            hierarchy.shared(index, id, |endpoint| endpoint.prepared)
        };
        // This initial attachment profile checked parent D0 before child
        // discovery; the grant excludes outside PM changes.
        // Power/decode bridge ancestors before touching downstream memory.
        // A member an earlier endpoint runs on is already in D0 and decoding.
        // Child config is always CFGBAR I/O.
        for &index in &order {
            if live(self, index) {
                continue;
            }
            let function = &self.children[index];
            config(host, function.child.address, |config| {
                function.capabilities.power_up(config, minimum)?;
                function.capabilities.check_d0(config)
            })?;
            // A bridge's D3hot transition can reset its bus/window registers.
            // Detect that before writing PM controls of any downstream child.
            self.recheck(host)?;
        }
        for (index, function) in self.children.iter().enumerate() {
            let member = order.contains(&index);
            if !first && (!member || live(self, index)) {
                continue;
            }
            let command = config(host, function.child.address, |config| {
                word(config, regs::COMMAND)
            })?;
            let decode = if member { regs::COMMAND_MEMORY } else { 0 };
            child_command(
                host,
                function.child.address,
                (command | regs::COMMAND_INTX_DISABLE) & !(DECODE | regs::COMMAND_MASTER) | decode,
            )?;
        }
        let command = parent_command(host)?;
        if first {
            host.parent_write_checked(
                regs::COMMAND,
                (command | regs::COMMAND_MEMORY | regs::COMMAND_INTX_DISABLE)
                    & !regs::COMMAND_MASTER,
            )?;
        } else if command & regs::COMMAND_MEMORY == 0 {
            return Err(Error::ActiveChild);
        }
        self.endpoints[id].prepared = true;
        self.recheck(host)
    }

    /// Enable bus mastering along endpoint `id`'s path, the parent's first.
    pub(super) fn enable<R: Registers>(
        &mut self,
        host: &mut Host<R>,
        id: usize,
    ) -> Result<(), Error> {
        let order = self.order(id)?;
        if !self.endpoints[id].prepared || self.endpoints[id].dma {
            return Err(Error::ActiveChild);
        }
        self.recheck(host)?;
        let command = parent_command(host)?;
        if command & regs::COMMAND_MEMORY == 0 {
            return Err(Error::ActiveChild);
        }
        if command & regs::COMMAND_MASTER == 0 {
            host.parent_write_checked(regs::COMMAND, command | regs::COMMAND_MASTER)?;
        }
        for index in order {
            if self.mastering(index) {
                continue;
            }
            let address = self.children[index].child.address;
            let command = config(host, address, |config| word(config, regs::COMMAND))?;
            if command & regs::COMMAND_MEMORY == 0 {
                return Err(Error::ActiveChild);
            }
            child_command(host, address, command | regs::COMMAND_MASTER)?;
        }
        self.endpoints[id].dma = true;
        self.recheck(host)
    }

    /// Clear bus mastering on endpoint `id` and on each member of its path,
    /// and the parent, no other endpoint still masters through. This also
    /// runs for an endpoint never enabled: attach can fail before that.
    pub(super) fn disable<R: Registers>(
        &mut self,
        host: &mut Host<R>,
        id: usize,
    ) -> Result<(), Error> {
        let order = self.order(id)?;
        // Identity/assignment checks still precede every containment.
        self.recheck(host)?;
        self.endpoints[id].dma = false;
        for &index in order.iter().rev() {
            if self.mastering(index) {
                continue;
            }
            let address = self.children[index].child.address;
            let command = config(host, address, |config| word(config, regs::COMMAND))?;
            child_command(
                host,
                address,
                (command | regs::COMMAND_INTX_DISABLE) & !regs::COMMAND_MASTER,
            )?;
        }
        if !self.endpoints.iter().any(|endpoint| endpoint.dma) {
            let command = parent_command(host)?;
            host.parent_write_checked(
                regs::COMMAND,
                (command | regs::COMMAND_INTX_DISABLE) & !regs::COMMAND_MASTER,
            )?;
        }
        self.recheck(host)
    }

    /// Retire endpoint `id`, whose driver stopped and disabled it, while
    /// another endpoint still runs: its decode, and that of each member of
    /// its path no other endpoint runs on, goes off.
    pub(super) fn retire<R: Registers>(
        &mut self,
        host: &mut Host<R>,
        id: usize,
    ) -> Result<(), Error> {
        let order = self.order(id)?;
        if self.endpoints[id].dma {
            return Err(Error::ActiveChild);
        }
        self.recheck(host)?;
        self.endpoints[id].prepared = false;
        self.endpoints[id].released = true;
        for &index in order.iter().rev() {
            if self.shared(index, id, |endpoint| endpoint.prepared) {
                continue;
            }
            let address = self.children[index].child.address;
            let command = config(host, address, |config| word(config, regs::COMMAND))?;
            child_command(
                host,
                address,
                (command | regs::COMMAND_INTX_DISABLE) & !(DECODE | regs::COMMAND_MASTER),
            )?;
        }
        self.recheck(host)
    }
}
