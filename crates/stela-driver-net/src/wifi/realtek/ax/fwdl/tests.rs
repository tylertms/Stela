extern crate std;
use super::*;
use crate::wifi::realtek::ax::pci::{Rings, pre_init};
use crate::wifi::realtek::ax::tests::model::{Op, RegisterModel};
use crate::wifi::realtek::beacon::tests::{Domain, Model, Source};
use crate::wifi::realtek::firmware::{Ce, Rtw89};
use alloc::{boxed::Box, string::ToString, sync::Arc, vec, vec::Vec};
use core::sync::atomic::{AtomicU8, Ordering};
use std::sync::Mutex;

/// A v0 image (no container) with a 4-byte dynamic header and two sections:
/// type 2 at raw 0x8810_0000 with 4500 bytes, and type 1 at raw
/// 0x2021_B000 with 3 bytes plus an 8-byte checksum.
pub(crate) fn image_bytes() -> Vec<u8> {
    let mut b = vec![0u8; 32 + 2 * 16];
    b[4..8].copy_from_slice(&[0, 27, 66, 1]);
    // W3[23:16]: header length with the dynamic part, 32 + 32 + 4.
    b[12..16].copy_from_slice(&(68u32 << 16).to_le_bytes());
    b[24..28].copy_from_slice(&(2u32 << 8).to_le_bytes());
    // W7: idmem share mode 1, dynamic header, stored part size 0x1234.
    b[28..32].copy_from_slice(&(1u32 << 18 | 1 << 16 | 0x1234).to_le_bytes());
    for (i, (address, flags)) in [
        (0x8810_0000u32, 4500u32 | 2 << 24),
        (0x2021_B000, 3 | 1 << 24 | 1 << 28),
    ]
    .into_iter()
    .enumerate()
    {
        let at = 32 + i * 16;
        b[at..at + 4].copy_from_slice(&address.to_le_bytes());
        b[at + 4..at + 8].copy_from_slice(&flags.to_le_bytes());
    }
    b.extend_from_slice(&[4, 0, 0, 0]);
    b.extend((0..4500u32).map(|i| (i * 7) as u8));
    b.extend_from_slice(&[0x5A; 11]);
    b
}

fn le32(v: u32) -> [u8; 4] {
    v.to_le_bytes()
}

/// A 16-byte FWCMD descriptor with only dword0 set.
fn descriptor(dword0: u32) -> Vec<u8> {
    let mut d = le32(dword0).to_vec();
    d.extend_from_slice(&[0; 12]);
    d
}

#[test]
fn header_packet_bytes() {
    let bytes = image_bytes();
    let fw = Rtw89::parse(&bytes, 2, Ce::Never).unwrap();
    for seq in [0u8, 0x5C] {
        let p = header_packet(&fw.image, seq);
        let header = fw.image.header_packet();
        assert_eq!(header.len(), 64, "dynamic header removed");
        // rtw89_build_txwd_fwcmd0_v1: RPKT_LEN [13:0] = skb->len (the FWCMD
        // header plus the image header), RPKT_TYPE [27:24] = 13 (H2C).
        assert_eq!(p[..16], descriptor((8 + 64) | 13 << 24)[..]);
        // rtw89_h2c_pkt_set_hdr_fwdl: CAT 1 [1:0], CLASS 3 [7:2], FUNC 0
        // [15:8], DEL_TYPE 0 [19:16], SEQ [31:24]; TOTAL_LEN = len + 8.
        assert_eq!(p[16..20], le32(1 | 3 << 2 | u32::from(seq) << 24));
        assert_eq!(p[20..24], le32(8 + 64));
        assert_eq!(p[24..], header[..]);
        // The payload is the image header with W7[15:0] = 2020.
        let mut want = bytes[..64].to_vec();
        want[28..30].copy_from_slice(&2020u16.to_le_bytes());
        assert_eq!(p[24..], want[..]);
        assert_eq!(p.len(), 16 + 8 + 64);
    }
}

#[test]
fn chunks_are_2020_with_type_14() {
    let data: Vec<u8> = (0..4500u32).map(|i| (i ^ (i >> 8)) as u8).collect();
    let section = AxSection {
        address: 0x0010_0000,
        kind: 2,
        checksum: false,
        bytes: &data,
    };
    let packets: Vec<Vec<u8>> = chunk_packets(&section).collect();
    assert_eq!(packets.len(), 3);
    let mut at = 0;
    for (packet, len) in packets.iter().zip([2020usize, 2020, 460]) {
        assert_eq!(packet.len(), 16 + len);
        // RPKT_TYPE 14 (FWDL), no FWCMD header.
        assert_eq!(packet[..16], descriptor(len as u32 | 14 << 24)[..]);
        assert_eq!(packet[16..], data[at..at + len]);
        at += len;
    }
    assert_eq!(CHUNK, 2020);
    // Exactly one chunk, and nothing at all for an empty section.
    let one = AxSection {
        bytes: &data[..2020],
        ..section
    };
    assert_eq!(chunk_packets(&one).count(), 1);
    let none = AxSection {
        bytes: &[],
        ..section
    };
    assert_eq!(chunk_packets(&none).count(), 0);
}

type Captured = Arc<Mutex<Vec<Vec<u8>>>>;

/// What the device would read at `iova`, through the model's mapping.
fn device_read(m: &Model, iova: u64, len: usize) -> Vec<u8> {
    let (base, (pages, cpu)) = m.mappings.range(..=iova).next_back().unwrap();
    assert!(iova + len as u64 <= base + pages.bytes() as u64);
    let at = cpu + (iova - base) as usize;
    // SAFETY: the range lies inside a live pinned allocation this test's
    // model holds, and nothing writes it concurrently.
    unsafe { core::slice::from_raw_parts(at as *const u8, len) }.to_vec()
}

/// Rings installed and CH12 started on a fresh model. The CH12 engine
/// consumes every kicked entry at once and records the packet it read
/// through the TXBD, as the device would see it.
fn running() -> (Arc<Mutex<Model>>, Rings, RegisterModel, Captured) {
    let m = Arc::new(Mutex::new(Model::default()));
    // SAFETY: the model domain isolates only the modeled requester behind
    // this test's `RegisterModel`; its pages are plain kept test RAM with no
    // real device, and this test is the queue's only worker.
    let rings = unsafe { Rings::install(&Source, Arc::new(Domain(m.clone()))) }.unwrap();
    let mut io = RegisterModel::new();
    // The BD RAM reset self-clears.
    io.on_read(
        0x1000,
        Box::new(|r: &mut RegisterModel| r.get32(0x1000) & !(1 << 3)),
    );
    // SAFETY: `io` is the modeled requester that `rings`' model domain
    // isolates, under the same model-only contract.
    unsafe { pre_init(&mut io, 2, &rings) }.unwrap();
    io.trace.clear();
    let captured: Captured = Arc::new(Mutex::new(Vec::new()));
    let (dma, sent) = (m.clone(), captured.clone());
    io.on_read(
        0x1080,
        Box::new(move |r: &mut RegisterModel| {
            let host = r.get32(0x1080) & 0xFFF;
            let slot = (host + 15) % 16;
            let ring = u64::from(r.get32(0x1280));
            let state = dma.lock().unwrap();
            let bd = device_read(&state, ring + u64::from(slot) * 8, 8);
            let len = u16::from_le_bytes([bd[0], bd[1]]) as usize;
            let at = u32::from_le_bytes([bd[4], bd[5], bd[6], bd[7]]);
            sent.lock()
                .unwrap()
                .push(device_read(&state, u64::from(at), len));
            host | host << 16
        }),
    );
    (m, rings, io, captured)
}

/// The WCPU as the firmware drives 0x01E0: H2C path ready (bit1) on the
/// second sample after the WCPU is enabled, FWDL path ready (bit2) once the
/// header packet was kicked, and status `ready` in [7:5] once every one of
/// `packets` was kicked.
fn wcpu(io: &mut RegisterModel, packets: u32, ready: Arc<AtomicU8>) {
    let mut h2c_samples = 0;
    io.on_read(
        0x01E0,
        Box::new(move |r: &mut RegisterModel| {
            let mut v = r.get32(0x01E0);
            let kicked = r.get32(0x1080) & 0xFFF;
            if r.get32(0x0088) & 2 != 0 {
                h2c_samples += 1;
                if h2c_samples >= 2 {
                    v |= 1 << 1;
                }
            }
            if kicked >= 1 {
                v |= 1 << 2;
            }
            if kicked == packets {
                v = (v & !0xE0) | u32::from(ready.load(Ordering::Relaxed)) << 5;
            }
            v
        }),
    );
}

/// The WDT status that `rtw89_mac_mem_read` sees through the indirect
/// window when 0x0C04 selects CPU_LOCAL + R_AX_WDT_STATUS.
fn watchdog(io: &mut RegisterModel) {
    io.on_read(
        0x4_0000,
        Box::new(|r: &mut RegisterModel| {
            if r.get32(0x0C04) == 0x1800_3044 {
                0x0000_0011
            } else {
                r.get32(0x4_0000)
            }
        }),
    );
}

fn run(
    rings: &mut Rings,
    io: &mut RegisterModel,
    image: &crate::wifi::realtek::firmware::AxImage<'_>,
) -> Result<(), AxError> {
    // SAFETY: `io` is the modeled requester that `running` programmed with
    // these rings; no real device can reach the model's IOVAs, and this test
    // is the only worker.
    unsafe { download(io, &mut rings.ch12(), image) }
}

/// The ops one `download` performs before its first packet, for registers
/// preset as in `download_sequence`.
fn cpu_restart() -> Vec<Op> {
    vec![
        // rtw89_mac_disable_cpu_ax (mac.c:4146):
        // write32_clr(R_AX_PLATFORM_ENABLE, B_AX_WCPU_EN).
        Op::R32(0x0088),
        Op::W32(0x0088, 0x0000_0001),
        // write32_clr(R_AX_WCPU_FW_CTRL, FWDL_EN | H2C_PATH_RDY | FWDL_PATH_RDY).
        Op::R32(0x01E0),
        Op::W32(0x01E0, 0x0000_00E0),
        // write32_clr(R_AX_SYS_CLK_CTRL, B_AX_CPU_CLK_EN).
        Op::R32(0x0008),
        Op::W32(0x0008, 0x0000_0001),
        // rtw89_disable_fw_watchdog (mac.c:4127), not 885xB:
        // rtw89_mac_mem_write(R_AX_WDT_CTRL, WDT_CTRL_ALL_DIS, CPU_LOCAL)
        // = 0x0C04 <- 0x18003000 + 0x40, then 0x40000 <- 0 (mac.c:42).
        Op::W32(0x0C04, 0x1800_3040),
        Op::W32(0x4_0000, 0),
        // rtw89_mac_mem_read(R_AX_WDT_STATUS) (mac.c:52).
        Op::W32(0x0C04, 0x1800_3044),
        Op::R32(0x4_0000),
        // |= B_AX_FS_WDT_INT (bit8), &= ~B_AX_FS_WDT_INT_MSK (bit0), then
        // rtw89_mac_mem_write re-selects the address before the store.
        Op::W32(0x0C04, 0x1800_3044),
        Op::W32(0x4_0000, 0x0000_0110),
        // write32_clr then write32_set(R_AX_PLATFORM_ENABLE, B_AX_PLATFORM_EN).
        Op::R32(0x0088),
        Op::W32(0x0088, 0x0000_0000),
        Op::R32(0x0088),
        Op::W32(0x0088, 0x0000_0001),
        // rtw89_mac_enable_cpu_ax(boot_reason 0, dlfw true) (mac.c:4161):
        // refuse if B_AX_WCPU_EN is set.
        Op::R32(0x0088),
        // UDM1, UDM2, HALT_H2C_CTRL, HALT_C2H_CTRL, HALT_H2C, HALT_C2H <- 0.
        Op::W32(0x01F4, 0),
        Op::W32(0x01F8, 0),
        Op::W32(0x0160, 0),
        Op::W32(0x0164, 0),
        Op::W32(0x0168, 0),
        Op::W32(0x016C, 0),
        // write32_set(R_AX_SYS_CLK_CTRL, B_AX_CPU_CLK_EN).
        Op::R32(0x0008),
        Op::W32(0x0008, 0x0000_4001),
        // One RMW of R_AX_WCPU_FW_CTRL: clear [2:0], status [7:5] =
        // RTW89_FWDL_INITIAL_STATE, set B_AX_WCPU_FWDL_EN.
        Op::R32(0x01E0),
        Op::W32(0x01E0, 0x0000_0001),
        // write16_mask(R_AX_BOOT_REASON, [2:0], 0).
        Op::R16(0x01E6),
        Op::W16(0x01E6, 0x00F8),
        // write32_set(R_AX_PLATFORM_ENABLE, B_AX_WCPU_EN).
        Op::R32(0x0088),
        Op::W32(0x0088, 0x0000_0003),
        // rtw89_fwdl_secure_idmem_share_mode_ax (mac.c:7437) returns at
        // once: secure boot is off, so nothing is written.
        // fwdl_check_path_ready(h2c) (mac.c:7419): read8 & B_AX_H2C_PATH_RDY,
        // 1 us apart; ready on the second sample.
        Op::R8(0x01E0),
        Op::Delay(1),
        Op::R8(0x01E0),
    ]
}

fn preset(io: &mut RegisterModel) {
    // A previous attempt left the WCPU running with every 0x01E0 flag set.
    io.set32(0x0088, 0x0000_0003);
    io.set32(0x01E0, 0x0000_00E7);
    io.set32(0x0008, 0x0000_4001);
    io.set32(0x01E4, 0x00FF_0000);
}

#[test]
fn download_sequence() {
    let bytes = image_bytes();
    let fw = Rtw89::parse(&bytes, 2, Ce::Never).unwrap();
    let (_m, mut rings, mut io, sent) = running();
    preset(&mut io);
    wcpu(&mut io, 5, Arc::new(AtomicU8::new(7)));
    watchdog(&mut io);

    run(&mut rings, &mut io, &fw.image).unwrap();

    let mut want = cpu_restart();
    want.extend([
        // __rtw89_fw_download_hdr: the header packet on CH12 (slot 0).
        Op::W16(0x1080, 1),
        Op::R32(0x1080),
        // fwdl_check_path_ready(fwdl): read8 & B_AX_FWDL_PATH_RDY.
        Op::R8(0x01E0),
        // rtw89_fw_download_hdr (fw.c:1804): HALT_H2C_CTRL, HALT_C2H_CTRL.
        Op::W32(0x0160, 0),
        Op::W32(0x0164, 0),
        // __rtw89_fw_download_main: section 0 in 2020, 2020, 460 bytes,
        Op::W16(0x1080, 2),
        Op::R32(0x1080),
        Op::W16(0x1080, 3),
        Op::R32(0x1080),
        Op::W16(0x1080, 4),
        Op::R32(0x1080),
        // section 1 (3 bytes plus its 8-byte checksum) in one packet.
        Op::W16(0x1080, 5),
        Op::R32(0x1080),
        // __rtw89_fw_download (fw.c:2028): mdelay(5).
        Op::Delay(5000),
        // rtw89_fw_check_rdy -> rtw89_fw_get_rdy_ax: read8, [7:5] == 7.
        Op::R8(0x01E0),
    ]);
    assert_eq!(io.trace, want);

    let sent = sent.lock().unwrap().clone();
    let mut header = descriptor((8 + 64) | 13 << 24);
    header.extend_from_slice(&le32(1 | 3 << 2));
    header.extend_from_slice(&le32(8 + 64));
    let mut image_header = bytes[..64].to_vec();
    image_header[28..30].copy_from_slice(&2020u16.to_le_bytes());
    header.extend_from_slice(&image_header);
    let body = &bytes[68..68 + 4500];
    let data = |range: core::ops::Range<usize>, src: &[u8]| {
        let mut p = descriptor(range.len() as u32 | 14 << 24);
        p.extend_from_slice(&src[range]);
        p
    };
    assert_eq!(
        sent,
        vec![
            header,
            data(0..2020, body),
            data(2020..4040, body),
            data(4040..4500, body),
            data(0..11, &[0x5A; 11]),
        ]
    );
    // Secure boot is off: no idmem share mode record or valid bit.
    assert_eq!(io.get32(0x01E0) & 0x0F80_0000, 0);
}

#[test]
fn ready_error_codes() {
    let bytes = image_bytes();
    let fw = Rtw89::parse(&bytes, 2, Ce::Never).unwrap();
    for (status, what) in [
        (2, "checksum failed"),
        (3, "security check failed"),
        (4, "cut mismatch"),
        (1, "not ready"),
        (6, "not ready"),
    ] {
        let (_m, mut rings, mut io, _sent) = running();
        wcpu(&mut io, 5, Arc::new(AtomicU8::new(status)));
        watchdog(&mut io);
        // The error carries the whole 0x01E0, read once after the wait:
        // FWDL_EN, both path bits and the status in [7:5].
        assert_eq!(
            run(&mut rings, &mut io, &fw.image),
            Err(AxError::FirmwareStatus {
                what,
                fw_ctrl: 0x07 | u32::from(status) << 5,
            }),
            "status {status}"
        );
        assert_eq!(
            io.trace[io.trace.len() - 2..],
            [Op::R8(0x01E0), Op::R32(0x01E0)]
        );
        // rtw89_fw_check_rdy waits for ready for FWDL_WAIT_CNT (400 ms) of
        // `read_poll_timeout_atomic` budget (iopoll.h: 399_602 samples, so
        // 399_601 1 us waits, with free reads) and only then decodes the
        // last status.
        let delay = io.trace.iter().position(|op| *op == Op::Delay(5000));
        let polls = io.trace[delay.unwrap()..]
            .iter()
            .filter(|op| **op == Op::Delay(1))
            .count();
        assert_eq!(polls, 399_601, "status {status}");
    }
}

#[test]
fn retry_after_ready_failure_starts_over() {
    let bytes = image_bytes();
    let fw = Rtw89::parse(&bytes, 2, Ce::Never).unwrap();
    let (_m, mut rings, mut io, sent) = running();
    let status = Arc::new(AtomicU8::new(2));
    wcpu(&mut io, 5, status.clone());
    watchdog(&mut io);
    assert_eq!(
        run(&mut rings, &mut io, &fw.image),
        Err(AxError::FirmwareStatus {
            what: "checksum failed",
            fw_ctrl: 0x0000_0047,
        })
    );
    // Linux's next __rtw89_fw_download starts with disable_cpu; the queue
    // continues at slot 5 and the WCPU reports ready this time.
    io.trace.clear();
    status.store(7, Ordering::Relaxed);
    wcpu(&mut io, 10, status);
    run(&mut rings, &mut io, &fw.image).unwrap();
    assert_eq!(io.trace[0], Op::R32(0x0088));
    assert_eq!(io.trace[1], Op::W32(0x0088, 0x0000_0001));
    assert_eq!(io.trace.last(), Some(&Op::R8(0x01E0)));
    let sent = sent.lock().unwrap();
    assert_eq!(sent.len(), 10);
    assert_eq!(sent[..5], sent[5..]);
}

#[test]
fn refuses_when_wcpu_already_enabled() {
    let bytes = image_bytes();
    let fw = Rtw89::parse(&bytes, 2, Ce::Never).unwrap();
    let (_m, mut rings, mut io, sent) = running();
    // B_AX_WCPU_EN does not clear.
    io.on_read(
        0x0088,
        Box::new(|r: &mut RegisterModel| r.get32(0x0088) | 1 << 1),
    );
    watchdog(&mut io);
    assert_eq!(
        run(&mut rings, &mut io, &fw.image),
        Err(AxError::Firmware("WCPU already enabled"))
    );
    // The disable sequence ran, then the enable check read 0x0088 and
    // stopped (rtw89_mac_enable_cpu_ax returns -EFAULT before any write).
    let check = io.trace.iter().rposition(|op| *op == Op::R32(0x0088));
    assert_eq!(check, Some(io.trace.len() - 1));
    assert!(!io.trace.contains(&Op::W32(0x01F4, 0)));
    assert!(sent.lock().unwrap().is_empty());
}

#[test]
fn path_ready_timeout_sends_nothing() {
    let bytes = image_bytes();
    let fw = Rtw89::parse(&bytes, 2, Ce::Never).unwrap();
    let (_m, mut rings, mut io, sent) = running();
    watchdog(&mut io);
    let began = io.now;
    // FWDL_EN alone, as rtw89_mac_enable_cpu_ax left it.
    assert_eq!(
        run(&mut rings, &mut io, &fw.image),
        Err(AxError::FirmwareStatus {
            what: "H2C path not ready",
            fw_ctrl: 0x0000_0001,
        })
    );
    // read_poll_timeout_atomic: 399_601 waits of 1 us with free reads.
    assert_eq!(io.now - began, 399_601);
    assert!(sent.lock().unwrap().is_empty());
    assert!(!io.trace.iter().any(|op| matches!(op, Op::W16(0x1080, _))));
}

#[test]
fn all_ones_status_byte_is_checked_against_the_register() {
    let bytes = image_bytes();
    let fw = Rtw89::parse(&bytes, 2, Ce::Never).unwrap();
    // A gone device reads all ones: 0xFF would pass every 8-bit check.
    let (_m, mut rings, mut io, sent) = running();
    io.on_read(0x01E0, Box::new(|_: &mut RegisterModel| u32::MAX));
    watchdog(&mut io);
    assert_eq!(
        run(&mut rings, &mut io, &fw.image),
        Err(AxError::AllOnes { register: 0x01E0 })
    );
    assert!(sent.lock().unwrap().is_empty());
    assert_eq!(
        io.trace[io.trace.len() - 2..],
        [Op::R8(0x01E0), Op::R32(0x01E0)]
    );
    // A live 0xFF byte (bits 3 and 4 set too) is a real value.
    let (_m, mut rings, mut io, sent) = running();
    io.on_read(0x01E0, Box::new(|_: &mut RegisterModel| 0x0000_00FF));
    watchdog(&mut io);
    run(&mut rings, &mut io, &fw.image).unwrap();
    assert_eq!(sent.lock().unwrap().len(), 5);
}

#[test]
fn submit_failure_stops_the_download() {
    let bytes = image_bytes();
    let fw = Rtw89::parse(&bytes, 2, Ce::Never).unwrap();
    let (_m, mut rings, mut io, _sent) = running();
    wcpu(&mut io, 5, Arc::new(AtomicU8::new(7)));
    watchdog(&mut io);
    // CH12 never consumes the header packet.
    io.on_read(
        0x1080,
        Box::new(|r: &mut RegisterModel| r.get32(0x1080) & 0xFFF),
    );
    assert_eq!(
        run(&mut rings, &mut io, &fw.image),
        Err(AxError::Timeout {
            register: 0x1080,
            mask: 0x0FFF_0000
        })
    );
    assert_eq!(
        io.trace
            .iter()
            .filter(|op| matches!(op, Op::W16(0x1080, _)))
            .count(),
        1
    );
    assert!(!io.trace.contains(&Op::Delay(5000)));
}

#[test]
fn an_expired_wait_reads_the_whole_status_register_once() {
    let mut m = RegisterModel::new();
    m.set32(0x01E0, 0x0000_0C01);
    assert_eq!(
        path_ready(&mut m, 1 << 2),
        Err(AxError::FirmwareStatus {
            what: "FWDL path not ready",
            fw_ctrl: 0x0000_0C01,
        })
    );
    assert_eq!(m.trace.last(), Some(&Op::R32(0x01E0)));
    assert_eq!(
        m.trace.iter().filter(|op| **op == Op::R32(0x01E0)).count(),
        1
    );
    assert_eq!(
        AxError::FirmwareStatus {
            what: "FWDL path not ready",
            fw_ctrl: 0x0000_0C01,
        }
        .to_string(),
        "FWDL path not ready (0x01E0=0x00000c01)"
    );
    // A device gone by then reads all ones there: that is the error.
    let mut m = RegisterModel::new();
    let mut reads = 0;
    m.on_read(
        0x01E0,
        Box::new(move |_: &mut RegisterModel| {
            reads += 1;
            if reads > 399_602 { u32::MAX } else { 0 }
        }),
    );
    assert_eq!(
        check_ready(&mut m),
        Err(AxError::AllOnes { register: 0x01E0 })
    );
}

/// Reads of `R_AX_WCPU_FW_CTRL` (the 8-bit sample).
fn ctrl_reads(m: &RegisterModel) -> usize {
    m.trace
        .iter()
        .filter(|o| **o == Op::R8(R_AX_WCPU_FW_CTRL))
        .count()
}

/// `read_poll_timeout_atomic(.., 1, FWDL_WAIT_CNT, false)` (`mac.c:7431`,
/// `fw.c:112`): left_ns = 400_000_000, cost 1001 per failure, so the
/// expired sample is k = 400_000_000 / 1001 + 1 = 399_601 and the poll
/// makes 399_602 samples, where wall time with a 1 us read ends near 200_000.
#[test]
fn fwdl_ready_polls_spend_the_linux_iteration_budget() {
    const SAMPLES: usize = 399_602;
    let mut m = RegisterModel::new();
    m.read_cost_us = 1;
    assert!(path_ready(&mut m, 0x02).is_err());
    assert_eq!(ctrl_reads(&m), SAMPLES);
    let mut m = RegisterModel::new();
    m.read_cost_us = 1;
    assert!(check_ready(&mut m).is_err());
    assert_eq!(ctrl_reads(&m), SAMPLES);
}
