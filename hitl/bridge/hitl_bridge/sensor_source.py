"""Sensor data sources for the HIL bridge.

SensorSource is the abstraction the bridge (bridge.py) pulls sensor state
from. Two implementations:

- SyntheticSensorSource: generates simple deterministic test data with no
  Gazebo dependency at all. Useful for bringing up/testing the serial link
  and firmware injection independently of Phase A (Gazebo world setup).
- GazeboSensorSource: subscribes to the real Gazebo topics. Requires
  `gz-transport` Python bindings (package `gz-transport13` /
  `python3-gz-transport13` on the Harmonic release used by this plan) and
  the exact topic names / message types published by the PX4 fixed-wing
  "plane" model's IMU, air_pressure and (later) navsat sensor plugins -
  these are NOT hardcoded with confidence here since Phase A (world setup,
  `gz topic -l` verification) has not been done yet. Confirm and adjust the
  constants at the top of this file once the world is running.
"""

from __future__ import annotations

import abc
import math
import time

from .protocol import HilSensorCoreV1

# --- GazeboSensorSource: topic names, TO BE CONFIRMED (Phase A step 3) ---
# Run `gz topic -l` against the running world and update these before use.
GZ_IMU_TOPIC = "/world/wingflight/model/plane/link/base_link/sensor/imu_sensor/imu"
GZ_AIR_PRESSURE_TOPIC = (
    "/world/wingflight/model/plane/link/base_link/sensor/air_pressure_sensor/air_pressure"
)


class SensorSource(abc.ABC):
    """Produces one HilSensorCoreV1 sample per read() call, at whatever rate
    the underlying source updates (the bridge polls this at a fixed loop
    rate - see bridge.py)."""

    @abc.abstractmethod
    def read(self) -> HilSensorCoreV1 | None:
        """Returns the latest sample, or None if nothing new is available
        since the last call."""

    def close(self) -> None:
        pass


class SyntheticSensorSource(SensorSource):
    """No-Gazebo-required test data source: constant 1g level-flight
    baseline plus an optional slow roll-rate sinusoid, so the serial link
    and firmware injection path can be exercised on the bench before Phase A
    (Gazebo world) is wired up.

    This deliberately does NOT try to be a flight-dynamics model - it only
    exists to produce well-formed, plausible-looking packets.
    """

    def __init__(self, roll_rate_amplitude_dps: float = 20.0, roll_rate_period_s: float = 4.0):
        self._roll_rate_amplitude_dps = roll_rate_amplitude_dps
        self._roll_rate_period_s = roll_rate_period_s
        self._seq = 0
        self._t0 = time.monotonic()

    def read(self) -> HilSensorCoreV1 | None:
        t = time.monotonic() - self._t0
        gyro_x = self._roll_rate_amplitude_dps * math.sin(2 * math.pi * t / self._roll_rate_period_s)

        sample = HilSensorCoreV1(
            seq=self._seq,
            gyro_dps=(gyro_x, 0.0, 0.0),
            accel_g=(0.0, 0.0, 1.0),  # level, wings-flat, 1g down
            baro_pressure_pa=101325,
            baro_temp_centic=2500,
        )
        self._seq += 1
        return sample


class GazeboSensorSource(SensorSource):
    """Subscribes to Gazebo IMU/air_pressure topics via gz-transport and
    converts them into HilSensorCoreV1 samples.

    NOT YET WIRED UP: message field names/units below are best-effort based
    on the gz-transport Python bindings tutorial
    (https://gazebosim.org/api/transport/13/python.html) and the general
    shape of gz-sim's IMU/AirPressure sensor plugins, but have not been
    verified against a running world (Phase A). Treat the message-type
    imports and field access below as a starting point to adjust once
    `gz topic -l` / `gz topic -info -t <topic>` confirm the actual types.
    """

    def __init__(self, imu_topic: str = GZ_IMU_TOPIC, air_pressure_topic: str = GZ_AIR_PRESSURE_TOPIC):
        try:
            from gz.transport13 import Node
        except ImportError as exc:  # pragma: no cover - environment-dependent
            raise RuntimeError(
                "gz-transport Python bindings not found. Install the "
                "gz-transport13 Python package matching your Gazebo Harmonic "
                "install (e.g. `sudo apt install python3-gz-transport13`) to "
                "use GazeboSensorSource."
            ) from exc

        self._node = Node()
        self._seq = 0
        self._latest_gyro_dps = (0.0, 0.0, 0.0)
        self._latest_accel_g = (0.0, 0.0, 1.0)
        self._latest_baro_pressure_pa = 101325
        self._latest_baro_temp_centic = 2500
        self._have_new_sample = False

        if not self._node.subscribe(self._imu_msg_type(), imu_topic, self._on_imu):
            raise RuntimeError(f"failed to subscribe to IMU topic {imu_topic!r}")
        if not self._node.subscribe(
            self._air_pressure_msg_type(), air_pressure_topic, self._on_air_pressure
        ):
            raise RuntimeError(
                f"failed to subscribe to air pressure topic {air_pressure_topic!r}"
            )

    @staticmethod
    def _imu_msg_type():
        # TODO(Phase A): confirm this is the correct protobuf type for the
        # plane model's IMU sensor topic (commonly gz.msgs.IMU).
        from gz.msgs10.imu_pb2 import IMU

        return IMU

    @staticmethod
    def _air_pressure_msg_type():
        # TODO(Phase A): confirm this is the correct protobuf type
        # (commonly gz.msgs.FluidPressure for the air_pressure sensor
        # plugin).
        from gz.msgs10.fluid_pressure_pb2 import FluidPressure

        return FluidPressure

    def _on_imu(self, msg) -> None:
        # TODO(Phase A): confirm field names/units against the real message.
        # gz-sim IMU messages typically carry angular_velocity (rad/s) and
        # linear_acceleration (m/s^2) in the sensor/body frame.
        RAD_TO_DEG = 180.0 / math.pi
        G = 9.80665

        self._latest_gyro_dps = (
            msg.angular_velocity.x * RAD_TO_DEG,
            msg.angular_velocity.y * RAD_TO_DEG,
            msg.angular_velocity.z * RAD_TO_DEG,
        )
        self._latest_accel_g = (
            msg.linear_acceleration.x / G,
            msg.linear_acceleration.y / G,
            msg.linear_acceleration.z / G,
        )
        self._have_new_sample = True

    def _on_air_pressure(self, msg) -> None:
        # TODO(Phase A): confirm field name/units (commonly Pascal) and
        # decide on a temperature source (FluidPressure has no temperature
        # field in gz-msgs - may need a separate topic or a fixed value).
        self._latest_baro_pressure_pa = int(round(msg.pressure))

    def read(self) -> HilSensorCoreV1 | None:
        if not self._have_new_sample:
            return None
        self._have_new_sample = False

        sample = HilSensorCoreV1(
            seq=self._seq,
            gyro_dps=self._latest_gyro_dps,
            accel_g=self._latest_accel_g,
            baro_pressure_pa=self._latest_baro_pressure_pa,
            baro_temp_centic=self._latest_baro_temp_centic,
        )
        self._seq += 1
        return sample
