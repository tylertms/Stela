# Run Stela in a Proxmox VM

Stela can boot in a Proxmox VE (QEMU/KVM) virtual machine with an NVIDIA GPU passed through. This page describes the VM we test with:

- **Host:** an i9-13900 running Proxmox VE, with an RTX 3060 (GA106) passed through.
- **Guest:** macOS 27.2 (26B123), set up as a Mac mini M2 (Mac14,3).

The examples use VM ID `117`. Replace it, the disk names and the USB IDs with your own.

## What you need

- **A GPU Stela supports, passed through to the VM.** The host needs IOMMU and VFIO passthrough working. Pass only the VGA function (`01:00.0`), not the GPU's audio function.
- **A USB keyboard and mouse, or their receiver, passed through as USB 3 devices (`usb3=1`).** Stela drives xHCI controllers only, not UHCI or EHCI.
- **A raw disk of at least 32 GB for the Stela USB stick.** A logical volume or a raw image file both work. We use 40 GiB.
- **A disk for macOS.** We use 128 GB.

## Make the stick

Stela boots from a USB stick. In a VM, the stick is a raw disk attached as a USB disk on its own xHCI controller (see `args` below).

`stela setup` has to see that disk as a USB stick. We did this from a small Linux install inside the same VM:

1. Install Linux on a second VM disk.
2. Boot that Linux with the stick disk attached.
3. Run `sudo ./stela setup` there and choose the stick.

Keep that Linux disk. `stela setup update stela` and `stela setup boot-log` need the same access later.

## VM settings

Set these with `qm set` or in the web UI:

```bash
qm set 117 --machine q35 --bios ovmf --cpu host --sockets 1 --cores 8 --numa 0
```

```bash
qm set 117 --memory 32768 --balloon 0 --vga none
```

```bash
qm set 117 --hostpci0 0000:01:00.0,pcie=1 --usb0 host=413c:301c,usb3=1 --serial0 socket
```

The `--usb0` ID above is our Dell keyboard and mouse receiver. Find yours with `lsusb`.

The EFI disk must have Secure Boot off. Either create the disk with `pre-enrolled-keys=0`, or turn Secure Boot off in the OVMF setup menu.

The stick and the macOS disk go in `args`, because the web UI cannot set either way of attaching them. In `/etc/pve/qemu-server/117.conf`, everything on one line:

```
args: -cpu host,+invtsc -machine i8042=off -drive file=/dev/pve/vm-117-disk-5,if=none,id=stela-stick,format=raw,cache=none -device qemu-xhci,id=stelaxhci -device usb-storage,bus=stelaxhci.0,drive=stela-stick,removable=on,bootindex=0 -drive file=/dev/pve/vm-117-disk-2,if=none,id=macos,format=raw,cache=writeback,discard=unmap -device nvme,drive=macos,serial=123456DRIVE
```

| Part | Why |
|---|---|
| `-cpu host,+invtsc` | Stela's code needs the host's instruction set: SSSE3 and POPCNT failed under QEMU's default `kvm64` CPU. Stela also expects an invariant TSC. |
| `-machine i8042=off` | Removes QEMU's emulated PS/2 controller. Input comes from the USB devices passed through instead. |
| `qemu-xhci,id=stelaxhci` and `bootindex=0` | Puts the stick on its own xHCI controller, so Stela can take that controller without disturbing the passed-through input devices, and OVMF boots from it first. |
| `-device nvme ... serial=...` | Puts the macOS disk on an emulated NVMe drive. The serial is required. virtio-blk does not work, because the firmware reports no serial for it. Do not attach the same disk a second time as `sata0` or `scsi0`. |

You need no vIOMMU (`viommu=`) or `intel-iommu` options.

**Network:** use a `virtio` NIC, for example `--net0 virtio,bridge=vmbr0`. Stela passes it to macOS as a USB Ethernet adapter.

**Optional settings:**
- **Pin the vCPUs one per physical core.** On a 13th-gen Intel CPU, that means the P-cores' first threads: `--affinity 0,2,4,6,8,10,12,14`. Mac14,3 has at most 8 cores.
- **Capture the serial console:**
  ```bash
  script -f /root/stela-serial.log -c "qm terminal 117"
  ```
  Stela writes its boot log there.

## Boot options

Stela reads its boot menu from `EFI/stela/choices.cfg` on the stick's EFI partition. Each entry is a name, a tab, then the options. To edit the file from the Proxmox host, first stop the VM, then run each command in turn:

```bash
L=$(losetup -fP --show /dev/pve/vm-117-disk-5); partx -u $L; udevadm settle
```

```bash
mkdir -p /mnt/stela-esp && mount ${L}p1 /mnt/stela-esp
```

```bash
nano /mnt/stela-esp/EFI/stela/choices.cfg
```

```bash
umount /mnt/stela-esp; losetup -d $L
```

**First boot, through Setup Assistant:**
- Make sure the macOS line has `--input`, so Stela forwards the passed-through keyboard and mouse to the guest.
- Also add `--no-guest-bluetooth`. A VM has no Bluetooth radio, and with Stela's modelled controller present, Setup Assistant stalls on a black screen.

**Once setup is done:** remove `--no-guest-bluetooth` again. Without a Bluetooth controller, macOS restarts `bluetoothd` about every 10 seconds.

## Known issues

- **Shutting down right after boot:** wait a few minutes after the desktop appears. A shutdown about 4 minutes after boot ended in a "Halt/Restart Timed Out" panic, and macOS restarted instead of powering off.
- **First Parsec connection:** the first Parsec connection after a boot can time out while macOS starts screen capture. The second attempt works.
