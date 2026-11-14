//! Validate the captured PCI bus tree before a source can assign its slots.
use super::{RootWindow, census};
use alloc::vec::Vec;
use stela_drivers::bus::pci::Address;

/// A configuration observation, with the original Type 1 bus dword.
#[derive(Clone, Copy)]
pub struct Function {
    pub address: Address,
    pub bridge_buses: Option<u32>,
}

/// Where the immediate bus comes from. Root indices identify entries in the
/// same captured root slice. These descriptions confer no ownership.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Parent {
    Root(usize),
    Bridge(Address),
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Error {
    Exhausted,
    DuplicateFunction(Address),
    MissingRoot(Address),
    AmbiguousRoot(Address),
    InvalidBridge(Address),
    DuplicateSecondary(Address),
    OverlappingBridges(Address),
    MissingParent(Address),
    /// A function with a CardBus or unknown header has Bus Master set or
    /// asserts INTx, so it cannot be left undriven.
    ActiveUnsupportedHeader(Address),
    /// A function reading vendor ID 0000 has Bus Master set or asserts
    /// INTx, so it cannot be treated as absent.
    ActiveWithoutVendor(Address),
}

struct Route {
    address: Address,
    root: usize,
    secondary: u8,
    subordinate: u8,
}

fn root(roots: &[RootWindow], address: Address) -> Result<usize, Error> {
    let mut matching = roots.iter().enumerate().filter(|(_, window)| {
        window.segment == u32::from(address.segment)
            && window
                .buses()
                .is_some_and(|buses| buses.contains(&address.bus))
    });
    let (root, _) = matching.next().ok_or(Error::MissingRoot(address))?;
    if matching.next().is_some() {
        return Err(Error::AmbiguousRoot(address));
    }
    Ok(root)
}

/// The secondary bus a bridge at `address` forwards to, from its Type 1 bus
/// dword, or None when firmware assigned it no buses. Whether functions
/// sit there does not matter: an empty port forwards as well.
pub fn secondary_bus(address: Address, buses: u32) -> Option<u8> {
    let [_, secondary, subordinate, _] = buses.to_le_bytes();
    (secondary > address.bus && subordinate >= secondary).then_some(secondary)
}

/// Whether turning on forwarding through `bridge`, which forwards to
/// `secondary`, lets DMA from `address` through: the bridge itself and each
/// function on its secondary bus. A bridge on that bus whose own forwarding
/// is off still holds back the buses below it.
pub fn exposed_by(bridge: Address, secondary: u8, address: Address) -> bool {
    address == bridge || (address.segment == bridge.segment && address.bus == secondary)
}

/// Return the direct parent of every function in BDF order. An occupied
/// subordinate bus must be the exact secondary bus of its immediate bridge.
/// The rest of a subordinate range can be unpopulated. A function on a bus
/// that no bridge forwards to, such as uncore devices on a high bus number
/// or a bus behind a bridge the census left out, is placed under its root,
/// which decodes every bus it grants.
pub fn resolve(
    roots: &[RootWindow],
    functions: &[Function],
) -> Result<Vec<(Address, Parent)>, Error> {
    let mut routes = Vec::new();
    let mut observed = Vec::new();
    observed
        .try_reserve_exact(functions.len())
        .map_err(|_| Error::Exhausted)?;
    for function in functions {
        let address = function.address;
        let root = root(roots, address)?;
        observed.push((address, root));
        let Some(buses) = function.bridge_buses else {
            continue;
        };
        // As in Linux, the primary field routes nothing and is not checked.
        let (secondary, subordinate) = match census::route(address.bus, buses) {
            census::Route::Unconfigured => continue,
            census::Route::Forwards { buses, .. } => (*buses.start(), *buses.end()),
            census::Route::Invalid => return Err(Error::InvalidBridge(address)),
        };
        let grant = roots[root].buses().expect("validated root bus grant");
        if !grant.contains(&subordinate) {
            return Err(Error::InvalidBridge(address));
        }
        routes.try_reserve(1).map_err(|_| Error::Exhausted)?;
        routes.push(Route {
            address,
            root,
            secondary,
            subordinate,
        });
    }

    observed.sort_unstable_by_key(|&(address, _)| address);
    if let Some(pair) = observed.windows(2).find(|pair| pair[0].0 == pair[1].0) {
        return Err(Error::DuplicateFunction(pair[0].0));
    }
    routes.sort_unstable_by_key(|route| (route.address.segment, route.secondary));
    if let Some(pair) = routes.windows(2).find(|pair| {
        pair[0].address.segment == pair[1].address.segment && pair[0].secondary == pair[1].secondary
    }) {
        return Err(Error::DuplicateSecondary(pair[1].address));
    }

    // No bridge forwards to this bus by its secondary bus number.
    let orphan = |address: Address| {
        routes
            .binary_search_by_key(&(address.segment, address.bus), |route| {
                (route.address.segment, route.secondary)
            })
            .is_err()
    };
    let mut nesting: Vec<&Route> = Vec::new();
    nesting
        .try_reserve(routes.len().min(256))
        .map_err(|_| Error::Exhausted)?;
    for route in &routes {
        while nesting.last().is_some_and(|parent| {
            parent.address.segment != route.address.segment || parent.subordinate < route.secondary
        }) {
            nesting.pop();
        }
        if let Some(parent) = nesting.last() {
            if parent.root != route.root
                || route.subordinate > parent.subordinate
                || (route.address.bus != parent.secondary && !orphan(route.address))
            {
                return Err(Error::OverlappingBridges(route.address));
            }
        } else if roots[route.root].minimum != u64::from(route.address.bus)
            && !orphan(route.address)
        {
            return Err(Error::MissingParent(route.address));
        }
        nesting.push(route);
    }

    let mut parents = Vec::new();
    parents
        .try_reserve_exact(observed.len())
        .map_err(|_| Error::Exhausted)?;
    for (address, root) in observed {
        let key = (address.segment, address.bus);
        let parent = match routes
            .binary_search_by_key(&key, |route| (route.address.segment, route.secondary))
        {
            Ok(index) if routes[index].root == root => Parent::Bridge(routes[index].address),
            Err(_) => Parent::Root(root),
            Ok(_) => return Err(Error::MissingParent(address)),
        };
        parents.push((address, parent));
    }
    Ok(parents)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn root(first: u8, last: u8) -> RootWindow {
        RootWindow {
            space: 2,
            minimum: u64::from(first),
            maximum: u64::from(last),
            length: u64::from(last) - u64::from(first) + 1,
            ..RootWindow::default()
        }
    }

    fn node(bus: u8, device: u8, route: Option<(u8, u8)>) -> Function {
        Function {
            address: Address {
                segment: 0,
                bus,
                device,
                function: 0,
            },
            bridge_buses: route.map(|(secondary, subordinate)| {
                u32::from_le_bytes([bus, secondary, subordinate, 0])
            }),
        }
    }

    #[test]
    fn empty_ports_forward_and_unassigned_bridges_do_not() {
        let port = node(4, 8, Some((6, 6)));
        assert_eq!(
            secondary_bus(port.address, port.bridge_buses.unwrap()),
            Some(6)
        );
        let unassigned = node(4, 9, Some((0, 0)));
        assert_eq!(
            secondary_bus(unassigned.address, unassigned.bridge_buses.unwrap()),
            None
        );
    }

    #[test]
    fn a_bridge_exposes_itself_and_its_secondary_bus_only() {
        let bridge = node(2, 1, Some((5, 7))).address;
        assert!(exposed_by(bridge, 5, bridge));
        assert!(exposed_by(bridge, 5, node(5, 3, None).address));
        // Below a closed bridge on bus 5, beside it, and its parent bus.
        assert!(!exposed_by(bridge, 5, node(6, 0, None).address));
        assert!(!exposed_by(bridge, 5, node(2, 2, None).address));
        assert!(!exposed_by(bridge, 5, node(0, 1, None).address));
        let other_segment = Address {
            segment: 1,
            ..node(5, 3, None).address
        };
        assert!(!exposed_by(bridge, 5, other_segment));
    }

    #[test]
    fn nested_bridges_own_only_their_direct_secondary_bus() {
        let upstream = node(0, 1, Some((2, 8)));
        let downstream = node(2, 1, Some((5, 7)));
        let endpoint = node(5, 1, None);
        assert_eq!(
            resolve(&[root(0, 9)], &[endpoint, downstream, upstream]).unwrap(),
            [
                (upstream.address, Parent::Root(0)),
                (downstream.address, Parent::Bridge(upstream.address)),
                (endpoint.address, Parent::Bridge(downstream.address)),
            ]
        );
    }

    #[test]
    fn a_bus_no_bridge_forwards_to_is_placed_under_its_root() {
        let upstream = node(0, 1, Some((2, 8)));
        // Inside the upstream range, with no bridge on bus 2 forwarding to
        // it, as behind a bridge the census left out.
        let inside = node(6, 1, None);
        let inside_bridge = node(6, 2, Some((7, 7)));
        let below = node(7, 0, None);
        // Uncore devices on the root's last bus.
        let uncore = node(0xff, 0x1e, None);
        let uncore_bridge = node(0xff, 0x1f, Some((0xfe, 0xfe)));
        assert_eq!(
            resolve(
                &[root(0, 0xff)],
                &[upstream, inside, inside_bridge, below, uncore]
            )
            .unwrap(),
            [
                (upstream.address, Parent::Root(0)),
                (inside.address, Parent::Root(0)),
                (inside_bridge.address, Parent::Root(0)),
                (below.address, Parent::Bridge(inside_bridge.address)),
                (uncore.address, Parent::Root(0)),
            ]
        );
        // A bridge's secondary bus never sits at or below its own.
        assert_eq!(
            resolve(&[root(0, 0xff)], &[uncore_bridge]),
            Err(Error::InvalidBridge(uncore_bridge.address))
        );
        // A bus that a bridge does forward to keeps that bridge as parent.
        let beside = node(2, 0, None);
        assert_eq!(
            resolve(&[root(0, 9)], &[upstream, beside]).unwrap()[1],
            (beside.address, Parent::Bridge(upstream.address))
        );
    }

    #[test]
    fn sibling_bridges_cannot_overlap_or_alias_a_secondary_bus() {
        let left = node(0, 1, Some((2, 7)));
        let right = node(0, 2, Some((5, 9)));
        assert_eq!(
            resolve(&[root(0, 9)], &[left, right]),
            Err(Error::OverlappingBridges(right.address))
        );
        let duplicate = node(0, 3, Some((2, 8)));
        assert!(matches!(
            resolve(&[root(0, 9)], &[left, duplicate]),
            Err(Error::DuplicateSecondary(_))
        ));
        let cross_root = node(0, 1, Some((2, 8)));
        assert_eq!(
            resolve(&[root(0, 4), root(5, 9)], &[cross_root]),
            Err(Error::InvalidBridge(cross_root.address))
        );
    }

    #[test]
    fn a_wrong_primary_bus_field_keeps_the_bridge_route() {
        let mut upstream = node(0, 1, Some((2, 8)));
        upstream.bridge_buses = upstream.bridge_buses.map(|buses| buses & !0xff | 0x07);
        let endpoint = node(2, 0, None);
        assert_eq!(
            resolve(&[root(0, 9)], &[upstream, endpoint]).unwrap(),
            [
                (upstream.address, Parent::Root(0)),
                (endpoint.address, Parent::Bridge(upstream.address)),
            ]
        );
        let backwards = node(3, 1, Some((3, 4)));
        assert_eq!(
            resolve(&[root(0, 9)], &[backwards]),
            Err(Error::InvalidBridge(backwards.address))
        );
    }
}
