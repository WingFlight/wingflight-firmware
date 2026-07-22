"""Wire protocol encode/decode for the HIL sensor injection link.

Mirrors src/main/io/hil_sensor.c on the firmware side. See
hitl/bridge/PROTOCOL.md for the authoritative wire format spec -
keep this module and the C parser in sync if the protocol changes.
"""

from __future__ import annotations

import dataclasses
import struct

SYNC1 = 0x48  # 'H'
SYNC2 = 0x53  # 'S'
PROTO_VERSION = 1
TYPE_CORE_V1 = 0x01

# uint32 seq, float gyro[3] (deg/s), float accel[3] (g), int32 baro_pressure_pa,
# int32 baro_temp_centic
_CORE_V1_STRUCT = struct.Struct("<I6fii")
CORE_V1_PAYLOAD_LEN = _CORE_V1_STRUCT.size  # 36
assert CORE_V1_PAYLOAD_LEN == 36

_CRC_POLY = 0x1021
_HEADER_STRUCT = struct.Struct("<BBBBB")  # sync1, sync2, version, type, len
_CRC_STRUCT = struct.Struct(">H")  # big-endian, matches firmware parser


def crc16_ccitt(crc: int, data: bytes) -> int:
    """Byte-at-a-time CRC16-CCITT (poly 0x1021), matching
    src/main/common/crc.c::crc16_calc() with the same polynomial."""
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ _CRC_POLY) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc & 0xFFFF


@dataclasses.dataclass
class HilSensorCoreV1:
    """One HIL_SENSOR_CORE_V1 packet's worth of sensor data.

    gyro_dps / accel_g are expressed in the FC's canonical body frame:
    X front/roll+, Y right/pitch+, Z down/yaw+ (see PROTOCOL.md - this is
    the frame *after* the firmware's own per-board sensor alignment, so
    the bridge does not need per-board alignment knowledge).
    """

    seq: int
    gyro_dps: tuple[float, float, float]
    accel_g: tuple[float, float, float]
    baro_pressure_pa: int
    baro_temp_centic: int

    def encode(self) -> bytes:
        payload = _CORE_V1_STRUCT.pack(
            self.seq & 0xFFFFFFFF,
            *self.gyro_dps,
            *self.accel_g,
            self.baro_pressure_pa,
            self.baro_temp_centic,
        )
        header_tail = _HEADER_STRUCT.pack(
            SYNC1, SYNC2, PROTO_VERSION, TYPE_CORE_V1, len(payload)
        )
        crc = crc16_ccitt(0, header_tail[2:] + payload)
        return header_tail + payload + _CRC_STRUCT.pack(crc)

    @classmethod
    def decode(cls, frame: bytes) -> "HilSensorCoreV1":
        """Decodes a full frame (as produced by encode()). Raises ValueError
        on any framing/CRC/type mismatch. Mainly useful for tests and for a
        loopback sanity check of the serial link."""
        if len(frame) < _HEADER_STRUCT.size + CORE_V1_PAYLOAD_LEN + _CRC_STRUCT.size:
            raise ValueError("frame too short")

        sync1, sync2, version, ptype, length = _HEADER_STRUCT.unpack_from(frame, 0)
        if sync1 != SYNC1 or sync2 != SYNC2:
            raise ValueError("bad sync bytes")
        if version != PROTO_VERSION:
            raise ValueError(f"unsupported version {version}")
        if ptype != TYPE_CORE_V1 or length != CORE_V1_PAYLOAD_LEN:
            raise ValueError(f"unexpected type/len {ptype}/{length}")

        payload_start = _HEADER_STRUCT.size
        payload_end = payload_start + length
        payload = frame[payload_start:payload_end]

        (crc_rx,) = _CRC_STRUCT.unpack_from(frame, payload_end)
        crc_calc = crc16_ccitt(0, frame[2:payload_end])
        if crc_rx != crc_calc:
            raise ValueError(f"CRC mismatch: rx={crc_rx:04x} calc={crc_calc:04x}")

        seq, gx, gy, gz, ax, ay, az, baro_p, baro_t = _CORE_V1_STRUCT.unpack(payload)
        return cls(
            seq=seq,
            gyro_dps=(gx, gy, gz),
            accel_g=(ax, ay, az),
            baro_pressure_pa=baro_p,
            baro_temp_centic=baro_t,
        )


def frame_length() -> int:
    return _HEADER_STRUCT.size + CORE_V1_PAYLOAD_LEN + _CRC_STRUCT.size
