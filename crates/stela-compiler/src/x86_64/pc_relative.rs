//! ADR and ADRP as MOV of the computed guest address.
use super::store;
use crate::aarch64::{Register, pc_relative::Operand};
use crate::x86_64::asm::Code;

pub(super) fn emit(code: &mut Code, operand: &Operand) {
    if operand.destination == Register::Zero {
        // Keep each guest instruction's native source-map interval nonempty.
        code.asm().nop();
        return;
    }
    // MOVABS leaves guest NZCV untouched.
    super::guest_immediate(code, operand.address);
    store(code, operand.destination);
}

#[cfg(test)]
mod tests {
    use super::*;

    use alloc::vec::Vec;

    #[test]
    fn address_is_full_width_and_discarded_destination_keeps_map_interval() {
        let mut code = Code::new();
        emit(
            &mut code,
            &Operand {
                destination: Register::Zero,
                address: crate::GuestAddress(u64::MAX),
            },
        );
        assert_eq!(code, [0x90]);
        code.clear();
        emit(
            &mut code,
            &Operand {
                destination: Register::X(30),
                address: crate::GuestAddress(0xfedcba9876543210),
            },
        );
        assert_eq!(&code[..2], &[0x48, 0xb8]);
        assert_eq!(&code[2..10], &0xfedcba9876543210u64.to_le_bytes());
        assert_eq!(&code[10..], &[0x48, 0x89, 0x87, 240, 0, 0, 0]);
        let source: Vec<_> = [0x1000001fu32, 0xd65f03c0]
            .into_iter()
            .flat_map(u32::to_le_bytes)
            .collect();
        let leaf = crate::testing::leaf(&source, crate::GuestAddress(0x100ffc)).unwrap();
        let block = crate::testing::block(&source, crate::GuestAddress(0x100ffc)).unwrap();
        for native in [&leaf, &block.native] {
            assert_eq!(
                native.source_map,
                [
                    (crate::GuestAddress(0x100ffc), 0),
                    (crate::GuestAddress(0x101000), 1)
                ]
            );
        }
    }

    #[test]
    fn addresses_take_their_shortest_move() {
        let store = [0x48, 0x89, 0x87, 8, 0, 0, 0];
        for (address, mov) in [
            (0x1234_5678_u64, &[0xb8, 0x78, 0x56, 0x34, 0x12][..]),
            (
                0xffff_ffff_8000_0000,
                &[0x48, 0xc7, 0xc0, 0, 0, 0, 0x80][..],
            ),
            (0x1_8000_0000, &[0x48, 0xb8, 0, 0, 0, 0x80, 1, 0, 0, 0][..]),
        ] {
            let mut code = Code::new();
            emit(
                &mut code,
                &Operand {
                    destination: Register::X(1),
                    address,
                },
            );
            assert_eq!(&code[..mov.len()], mov, "{address:x}");
            assert_eq!(&code[mov.len()..], store, "{address:x}");
        }
    }
}
