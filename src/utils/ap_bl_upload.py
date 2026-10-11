#!/usr/bin/env python3
"""
Upload a Wingflight XIP firmware through an ArduPilot boot loader.

Boards like the FrSky Aegis Lite (STM32H730) keep an ArduPilot boot loader in the internal flash. It maps the
external OCTOSPI flash into memory and starts the firmware from there. This script writes a firmware built for
such a board (TARGET=STM32H730, linked at 0x90000000) to the external flash, using the boot loader's serial
protocol (see ArduPilot's Tools/AP_Bootloader/bl_protocol.cpp and Tools/scripts/uploader.py).

The custom defaults (a wingflight-targets .config file) can be embedded into the image, the same way the
Configurator does it for the other targets.

Usage:
    ap_bl_upload.py --port COM20 --config FRSK-AEGISLITE.config obj/wingflight_4.6.0_STM32H730.hex

If the board runs Wingflight or ArduPilot, it is rebooted into the boot loader first.
"""

import argparse
import struct
import sys
import time
import zlib

import serial
import serial.tools.list_ports

XIP_BASE = 0x90000000
CUSTOM_DEFAULTS_POINTER_OFFSET = 0x2800
CUSTOM_DEFAULTS_HEADER = "## Wingflight Custom Defaults\n"

ARDUPILOT_USB_VID = 0x1209

# Boot loader protocol
INSYNC = 0x12
EOC = 0x20
OK = 0x10
FAILED = 0x11
INVALID = 0x13

GET_SYNC = 0x21
GET_DEVICE = 0x22
BOOT = 0x30
EXTF_ERASE = 0x34
EXTF_PROG_MULTI = 0x35
EXTF_GET_CRC = 0x37

INFO_BL_REV = 1
INFO_BOARD_ID = 2
INFO_BOARD_REV = 3
INFO_FW_SIZE = 4
INFO_EXTF_SIZE = 6

PROG_MULTI_MAX = 252


def crc32_ardupilot(data):
    """CRC32 as computed by the boot loader: reflected 0xEDB88320, initial value 0, no final XOR"""
    return zlib.crc32(data, 0xFFFFFFFF) ^ 0xFFFFFFFF


def load_hex(path):
    """Load an Intel HEX file into (base address, bytearray), gaps filled with 0xFF"""
    chunks = {}
    upper = 0
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line.startswith(":"):
                continue
            record = bytes.fromhex(line[1:])
            length, address, kind = record[0], (record[1] << 8) | record[2], record[3]
            data = record[4:4 + length]
            if kind == 0x00:
                chunks[upper + address] = data
            elif kind == 0x04:
                upper = ((data[0] << 8) | data[1]) << 16
            elif kind == 0x02:
                upper = ((data[0] << 8) | data[1]) << 4
            elif kind == 0x01:
                break
    base = min(chunks)
    end = max(address + len(data) for address, data in chunks.items())
    image = bytearray(b"\xFF" * (end - base))
    for address, data in chunks.items():
        image[address - base:address - base + len(data)] = data
    return base, image


def load_firmware(path):
    if path.lower().endswith(".hex"):
        base, image = load_hex(path)
    else:
        base, image = XIP_BASE, bytearray(open(path, "rb").read())
    if base != XIP_BASE:
        sys.exit(f"Firmware starts at 0x{base:08x}, expected 0x{XIP_BASE:08x}. Was it built with TARGET=STM32H730?")
    while len(image) % 4:
        image += b"\xFF"
    return image


def insert_custom_defaults(image, config_path):
    start, end = struct.unpack_from("<II", image, CUSTOM_DEFAULTS_POINTER_OFFSET)
    if not (XIP_BASE <= start < end <= XIP_BASE + len(image) + 0x10000):
        sys.exit(f"No custom defaults area found in the firmware (0x{start:08x}-0x{end:08x})")

    with open(config_path, newline="") as f:
        config = f.read().replace("\r\n", "\n")
    text = (CUSTOM_DEFAULTS_HEADER + config + "\0").encode("ascii")
    if len(text) > end - start:
        sys.exit(f"Custom defaults too large: {len(text)} bytes, {end - start} available")

    offset = start - XIP_BASE
    if offset + len(text) > len(image):
        image += b"\xFF" * (offset + len(text) - len(image))
    image[offset:offset + len(text)] = text
    while len(image) % 4:
        image += b"\xFF"
    print(f"Custom defaults from {config_path} inserted at 0x{start:08x} ({len(text)} bytes)")


def mavlink_reboot_to_bootloader():
    """MAVLink v1 COMMAND_LONG MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN with param1 = 3 (reboot into boot loader)"""
    payload = struct.pack("<7fHBBB", 3.0, 0, 0, 0, 0, 0, 0, 246, 1, 1, 0)
    msg_id, crc_extra = 76, 152
    frame = bytes([len(payload), 0, 255, 0, msg_id]) + payload

    crc = 0xFFFF
    for byte in frame + bytes([crc_extra]):
        tmp = (byte ^ crc) & 0xFF
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return b"\xFE" + frame + struct.pack("<H", crc)


class BootLoader:
    def __init__(self, port):
        self.port = serial.Serial(port, 115200, timeout=0.5)

    def close(self):
        self.port.close()

    def send(self, data):
        self.port.write(bytes(data))

    def recv(self, count=1, timeout=None):
        if timeout is not None:
            self.port.timeout = timeout
        data = self.port.read(count)
        self.port.timeout = 0.5
        if len(data) != count:
            raise RuntimeError("Timeout waiting for the boot loader")
        return data

    def get_sync(self, timeout=None):
        insync, status = self.recv(2, timeout)
        if insync != INSYNC:
            raise RuntimeError(f"Unexpected 0x{insync:02x} instead of INSYNC")
        if status == INVALID:
            raise RuntimeError("Boot loader reports INVALID OPERATION")
        if status == FAILED:
            raise RuntimeError("Boot loader reports OPERATION FAILED")
        if status != OK:
            raise RuntimeError(f"Unexpected 0x{status:02x} instead of OK")

    def sync(self):
        self.port.reset_input_buffer()
        self.send([GET_SYNC, EOC])
        self.get_sync()

    def get_info(self, param):
        self.send([GET_DEVICE, param, EOC])
        value = struct.unpack("<I", self.recv(4))[0]
        self.get_sync()
        return value

    def erase_extf(self, size):
        self.send(bytes([EXTF_ERASE]) + struct.pack("<I", size) + bytes([EOC]))
        self.get_sync(timeout=5)
        # One percentage byte per poll, then INSYNC OK when done
        previous = None
        while True:
            byte = self.recv(1, timeout=20)[0]
            if byte == INSYNC and previous is not None and previous >= 90:
                status = self.recv(1)[0]
                if status == OK:
                    break
                raise RuntimeError(f"Erase failed (0x{status:02x})")
            previous = byte
            print(f"\rErasing   {min(byte, 100):3d}%", end="", flush=True)
        print("\rErasing   100%")

    def program_extf(self, image):
        for offset in range(0, len(image), PROG_MULTI_MAX):
            chunk = image[offset:offset + PROG_MULTI_MAX]
            self.send(bytes([EXTF_PROG_MULTI, len(chunk)]) + chunk + bytes([EOC]))
            self.get_sync(timeout=2)
            if offset % (PROG_MULTI_MAX * 32) == 0:
                print(f"\rWriting   {100 * offset // len(image):3d}%", end="", flush=True)
        print("\rWriting   100%")

    def verify_extf(self, image):
        self.send(bytes([EXTF_GET_CRC]) + struct.pack("<I", len(image)) + bytes([EOC]))
        crc = struct.unpack("<I", self.recv(4, timeout=10))[0]
        self.get_sync()
        expected = crc32_ardupilot(bytes(image))
        if crc != expected:
            raise RuntimeError(f"CRC mismatch: expected 0x{expected:08x}, read 0x{crc:08x}")
        print(f"Verified  CRC 0x{crc:08x}")

    def boot(self):
        self.send([BOOT, EOC])
        self.get_sync(timeout=2)


def ardupilot_ports():
    return [p.device for p in serial.tools.list_ports.comports() if p.vid == ARDUPILOT_USB_VID]


def try_bootloader(port):
    try:
        bl = BootLoader(port)
    except serial.SerialException:
        return None
    try:
        for _ in range(3):
            try:
                bl.sync()
                return bl
            except RuntimeError:
                pass
    except serial.SerialException:
        pass
    bl.close()
    return None


def request_bootloader(port):
    """Ask the firmware on the port to reboot into the boot loader"""
    try:
        with serial.Serial(port, 115200, timeout=0.5) as p:
            # Wingflight: enter the CLI and reboot into the flash boot loader. The line break ends the
            # line in case the CLI is already active, where "#" starts a comment.
            p.write(b"#\r\n")
            time.sleep(0.3)
            p.write(b"bl\r\n")
            time.sleep(0.3)
            # ArduPilot: MAVLink reboot into the boot loader
            p.write(mavlink_reboot_to_bootloader())
            time.sleep(0.3)
    except serial.SerialException:
        pass


def find_bootloader(port, timeout):
    bl = try_bootloader(port) if port else None
    if bl:
        return bl

    if port:
        print(f"Requesting the boot loader on {port}")
        request_bootloader(port)

    deadline = time.time() + timeout
    while time.time() < deadline:
        time.sleep(0.5)
        for candidate in ([port] if port else []) + ardupilot_ports():
            bl = try_bootloader(candidate)
            if bl:
                print(f"Boot loader found on {candidate}")
                return bl
    sys.exit("No boot loader found. Replug the board and run again within 5 seconds.")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("firmware", help="firmware .hex or .bin built for an XIP target")
    parser.add_argument("--port", help="serial port of the board (default: search for the boot loader)")
    parser.add_argument("--config", help="custom defaults (wingflight-targets .config) to embed")
    parser.add_argument("--board-id", type=int, default=7200, help="expected APJ board ID (default: 7200, Aegis Lite)")
    parser.add_argument("--force", action="store_true", help="flash even if the board ID does not match")
    parser.add_argument("--timeout", type=float, default=15, help="seconds to wait for the boot loader")
    args = parser.parse_args()

    image = load_firmware(args.firmware)
    if args.config:
        insert_custom_defaults(image, args.config)

    bl = find_bootloader(args.port, args.timeout)
    try:
        bl_rev = bl.get_info(INFO_BL_REV)
        board_id = bl.get_info(INFO_BOARD_ID)
        board_rev = bl.get_info(INFO_BOARD_REV)
        fw_size = bl.get_info(INFO_FW_SIZE)
        extf_size = bl.get_info(INFO_EXTF_SIZE)
        print(f"Boot loader rev {bl_rev}, board ID {board_id} rev {board_rev}, "
              f"internal flash {fw_size} bytes, external flash {extf_size} bytes")

        if board_id != args.board_id and not args.force:
            sys.exit(f"Board ID {board_id} does not match {args.board_id}, use --force to flash anyway")
        if len(image) > extf_size:
            sys.exit(f"Firmware ({len(image)} bytes) does not fit into the external flash ({extf_size} bytes)")

        bl.erase_extf(len(image))
        bl.program_extf(image)
        bl.verify_extf(image)
        bl.boot()
        print("Done, starting the firmware")
    finally:
        bl.close()


if __name__ == "__main__":
    main()
