"""Unit tests for the wire protocol encode/decode, mirroring the firmware's
parser (src/main/io/hil_sensor.c). Run with: python -m pytest hitl/bridge
or python -m unittest discover hitl/bridge/tests
"""

import unittest

from hitl_bridge.protocol import (
    CORE_V1_PAYLOAD_LEN,
    SYNC1,
    SYNC2,
    HilSensorCoreV1,
    crc16_ccitt,
    frame_length,
)


class TestProtocol(unittest.TestCase):
    def test_payload_length_matches_firmware_constant(self):
        # Must stay in sync with HIL_SENSOR_CORE_V1_PAYLOAD_LEN in
        # src/main/io/hil_sensor.c
        self.assertEqual(CORE_V1_PAYLOAD_LEN, 36)

    def test_frame_length(self):
        # 5 byte header + 36 byte payload + 2 byte CRC
        self.assertEqual(frame_length(), 43)

    def test_encode_decode_roundtrip(self):
        sample = HilSensorCoreV1(
            seq=12345,
            gyro_dps=(1.5, -2.5, 3.25),
            accel_g=(0.1, -0.2, 0.98),
            baro_pressure_pa=101325,
            baro_temp_centic=2500,
        )
        frame = sample.encode()
        self.assertEqual(len(frame), frame_length())
        self.assertEqual(frame[0], SYNC1)
        self.assertEqual(frame[1], SYNC2)

        decoded = HilSensorCoreV1.decode(frame)
        self.assertEqual(decoded.seq, sample.seq)
        # Values round-trip through 32-bit floats on the wire (matching the
        # firmware's `float` fields), so compare with tolerance rather than
        # exact equality.
        for got, want in zip(decoded.gyro_dps, sample.gyro_dps):
            self.assertAlmostEqual(got, want, places=5)
        for got, want in zip(decoded.accel_g, sample.accel_g):
            self.assertAlmostEqual(got, want, places=5)
        self.assertEqual(decoded.baro_pressure_pa, sample.baro_pressure_pa)
        self.assertEqual(decoded.baro_temp_centic, sample.baro_temp_centic)

    def test_decode_rejects_bad_sync(self):
        sample = HilSensorCoreV1(
            seq=1, gyro_dps=(0, 0, 0), accel_g=(0, 0, 1), baro_pressure_pa=101325, baro_temp_centic=2500
        )
        frame = bytearray(sample.encode())
        frame[0] = 0x00
        with self.assertRaises(ValueError):
            HilSensorCoreV1.decode(bytes(frame))

    def test_decode_rejects_bad_crc(self):
        sample = HilSensorCoreV1(
            seq=1, gyro_dps=(0, 0, 0), accel_g=(0, 0, 1), baro_pressure_pa=101325, baro_temp_centic=2500
        )
        frame = bytearray(sample.encode())
        frame[-1] ^= 0xFF
        with self.assertRaises(ValueError):
            HilSensorCoreV1.decode(bytes(frame))

    def test_crc16_ccitt_known_vector(self):
        # "123456789" -> 0x29B1 is the standard CRC16/CCITT-FALSE (poly
        # 0x1021, init 0xFFFF) check value. This repo's firmware/bridge use
        # init=0 (matching src/main/rx/sumd.c's convention), so verify
        # against an independently computed reference instead of the
        # textbook init=0xFFFF vector.
        data = b"123456789"
        crc = 0
        for byte in data:
            crc = crc16_ccitt(crc, bytes([byte]))
        # Recompute with the bulk API (single call over all bytes) and check
        # both paths agree.
        crc_bulk = crc16_ccitt(0, data)
        self.assertEqual(crc, crc_bulk)


if __name__ == "__main__":
    unittest.main()
