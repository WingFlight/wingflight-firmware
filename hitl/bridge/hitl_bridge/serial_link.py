"""Serial transport for sending HIL sensor packets to the real FC board.

The firmware side (src/main/io/hil_sensor.c) opens the assigned UART at a
fixed 921600 baud, RX only - this module is a thin wrapper for the TX side
on the host.
"""

from __future__ import annotations

from .protocol import HilSensorCoreV1

BAUDRATE = 921600


class SerialLink:
    def __init__(self, port: str, baudrate: int = BAUDRATE):
        try:
            import serial
        except ImportError as exc:  # pragma: no cover - environment-dependent
            raise RuntimeError(
                "pyserial not found. Install it with `pip install pyserial` "
                "to use SerialLink."
            ) from exc

        self._serial = serial.Serial(port, baudrate=baudrate, timeout=0)

    def send(self, sample: HilSensorCoreV1) -> None:
        self._serial.write(sample.encode())

    def close(self) -> None:
        self._serial.close()

    def __enter__(self) -> "SerialLink":
        return self

    def __exit__(self, *exc_info) -> None:
        self.close()


class NullLink:
    """No-op transport for dry runs (--dry-run) - encodes packets but never
    opens a real serial port. Useful for testing sensor sources / packet
    rates without hardware attached."""

    def __init__(self) -> None:
        self.sent_count = 0
        self.last_frame: bytes | None = None

    def send(self, sample: HilSensorCoreV1) -> None:
        self.last_frame = sample.encode()
        self.sent_count += 1

    def close(self) -> None:
        pass

    def __enter__(self) -> "NullLink":
        return self

    def __exit__(self, *exc_info) -> None:
        self.close()
