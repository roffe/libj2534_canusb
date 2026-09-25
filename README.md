# libj2534_canusb

SAE J2534-1 (v04.04) PassThru driver for the **LAWICEL CANUSB** and the
canusb2 clone, for **Windows** (32 and 64 bit DLL) and
**Linux** (`.so`), written in plain C.

Supported: raw **CAN** (protocol 5) and **ISO15765** (protocol 6), 11 and
29 bit IDs, pass/block filters (CAN) and flow control filters (ISO15765),
periodic messages, bitrates 10k–1M plus 615.4k (Trionic 5) and 83.3k
(GMLAN single wire preset).

The ISO15765 (ISO-TP) layer runs entirely in the driver on top of raw frames:
segmented transmit paced by the peer's BS/STmin, receive reassembly with
automatic flow controls (`ISO15765_BS`/`ISO15765_STMIN` config, default
BS=0 STmin=0), `ISO15765_FRAME_PAD`, first-frame and TxDone indications.
Normal addressing only. Extended/mixed addressing and channels on both
protocols at once are not supported.

> ⚠️ **This software talks to vehicle control modules.** Reading, writing, or
> reflashing an ECU can render it inoperable. Provided **as is, with no
> warranty**. You are solely responsible for how you use it.

## Building

Both platforms build from Linux; Windows is cross-built with mingw-w64:

    make            # everything below
    make linux      # build/libj2534_canusb.so
    make windows    # build/{x86,x64}/libj2534_canusb.dll + build/libj2534_canusb_setup.exe
    make test       # host-side test of the slcan encode/decode, ISO-TP + filter logic

## Linux

The dongle's FTDI chip (or the canusb2's FT232R emulation) is bound by the
in-tree `ftdi_sio` driver as `/dev/ttyUSB*`, and the driver talks to that
tty. **No libftd2xx, no libusb, no udev rule.**

Requirements:

- Linux with `ftdi_sio` (standard on every desktop distro).
- Permission to open the tty. Be in the `uucp` or `dialout` group:
  ```sh
  sudo usermod -aG uucp $USER   # or dialout, depending on your distro; log out/in
  ```

Install:

    make install    # -> ~/.passthru/libj2534_canusb.so + ~/.passthru/canusb.json

This follows the de-facto Linux J2534 convention: apps that scan
`~/.passthru/*.json` find the driver via the descriptor's `FUNCTION_LIB`.
Nothing is installed system-wide and no root is needed. The `PassThru*`
exports are plain cdecl with the 32-bit `unsigned long` ABI.

`PassThruOpen` picks the device in this order:

1. `pName`, if it is a `/dev/...` path;
2. `$CANUSB_PORT`;
3. the `/dev/serial/by-id/` entry naming a CANUSB, preferring the one whose
   serial number is `pName` (e.g. `LWSY2WXO`), as on Windows.

```sh
CANUSB_PORT=/dev/ttyUSB2 ./your-app
```

The port is opened exclusively; a second opener gets `ERR_DEVICE_IN_USE`.

## Windows

The DLL talks to the dongle directly over the FTDI **D2XX** driver
(`ftd2xx.dll`), so it works with the factory configuration: VCP disabled,
no COM port. The FTDI D2XX driver (CDM package) must be installed.

Run `build/libj2534_canusb_setup.exe` (it asks for admin). It carries both
DLLs inside it, extracts them to `C:\Program Files\libj2534_canusb\` and
registers the `PassThruSupport.04.04` keys in both registry views. Remove
everything again with `libj2534_canusb_setup.exe uninstall` (or the
interactive menu).

Manual alternative: copy the DLLs yourself and import
`windows/libj2534_canusb.reg` (paths inside match the installer's layout;
adjust them if you deviate).

Applications then see a **LAWICEL - CANUSB** J2534 device. `PassThruOpen`
with a `pName` string selects a specific dongle by FTDI serial number
(e.g. `LWSXKNIO`); NULL opens the first device whose description is CANUSB.

## Testing on hardware

    make testexes   # build/test_device (linux), build/{x86,x64}/test_device.exe (windows)
    build/test_device [bitrate] [--send id hexbytes]

It opens the device, prints versions, connects CAN (default 500000), sets a
pass-all filter and dumps received frames for 5 seconds. `--send` transmits
one standard-ID frame first. `make winetest` loads both DLLs under wine and
calls through the stdcall boundary.

## Notes

- Transmits are synchronous: each frame waits for the dongle's `z`/`Z` ack
  (~1 USB round trip per frame). Fine for diagnostics and logging; if bulk
  flashing throughput ever matters, a windowed-credit scheme is the upgrade.
- Per the J2534 spec a freshly connected channel delivers **nothing** until
  the application sets a `PASS_FILTER`.
- The driver never sends the Lawicel `X` command. On canusb2 firmware that
  reboots the dongle into the DFU bootloader.

## Project layout

| Path | Purpose |
|---|---|
| `common/canusb.c` | The J2534 core: slcan command channel, filters, ISO-TP, periodic messages, the 14 exports |
| `common/proto.c` | Pure slcan / ISO-TP / filter logic (`test_proto.c` tests it) |
| `common/port.h` | Platform layer the core runs on: byte stream, threads, locks, events, clocks |
| `common/j2534.h` | J2534 types and prototypes (32-bit ABI) |
| `common/test_device.c` | On-hardware exerciser for either platform |
| `linux/port.c` | `ftdi_sio` tty + pthreads |
| `linux/canusb.json` | `~/.passthru` device descriptor |
| `windows/port.c` | FTDI D2XX + Win32 threads |
| `windows/setup.c` | Single-file installer/uninstaller with both DLLs embedded |
| `windows/libj2534_canusb.def` / `.reg` | DLL exports / manual registry entries |
| `Makefile` | Builds both platforms |
