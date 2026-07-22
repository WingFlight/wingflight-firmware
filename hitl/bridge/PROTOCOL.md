# HIL Sensor Injection Protocol (v1)

Wire format for the dedicated-UART link between the HIL bridge (host, driven
by Gazebo) and real Wingflight firmware built with `USE_HIL_SENSOR_OVERRIDE`.
This is a **separate link from MSP** (see `hitl_plan.txt` Decisions section).

Scope of v1: gyro, accelerometer and barometer injection only. Magnetometer,
GPS and airspeed are intentionally deferred — they are not scheduler/timing
critical the way gyro/accel are, and adding them cleanly requires additional
firmware plumbing (a `MAG_FAKE` hardware option, a simulated GPS parser).
Adding them later is a new packet `TYPE`, not a breaking change to this one.

## Transport

- One dedicated UART, assigned via the CLI `serial` command to the new
  `FUNCTION_HIL_SENSOR` port function (see `src/main/io/serial.h`).
- Fixed baud rate: 921600 8N1, RX only on the firmware side (bridge → FC).
- The bridge sends one frame per simulation sensor-update tick. Target rate:
  a few hundred Hz up to ~1kHz (see `hitl_plan.txt` Further Considerations #1
  for the zero-order-hold rate/bandwidth trade-off).

## Framing

```
Offset  Size  Field
0       1     SYNC1        = 0x48 ('H')
1       1     SYNC2        = 0x53 ('S')
2       1     VERSION      = 0x01
3       1     TYPE         = 0x01 (HIL_SENSOR_CORE_V1)
4       1     LEN          payload length in bytes (36 for TYPE 0x01)
5..N    LEN   PAYLOAD      little-endian, see below
N+1     1     CRC_HI       CRC16-CCITT (poly 0x1021, init 0), big-endian
N+2     1     CRC_LO
```

The CRC is computed over `VERSION, TYPE, LEN, PAYLOAD` (i.e. everything
after the two sync bytes), one byte at a time via `crc16_ccitt()`
(`src/main/common/crc.h`), matching the convention already used by
`src/main/rx/sumd.c`.

## Payload — TYPE 0x01 (HIL_SENSOR_CORE_V1), 36 bytes

| Offset | Size | Type    | Field              | Units                                   |
| ------ | ---- | ------- | ------------------ | ---------------------------------------- |
| 0      | 4    | uint32  | seq                | monotonically increasing, for gap detection |
| 4      | 4    | float32 | gyro_x             | deg/s, FC body frame (see Axis convention) |
| 8      | 4    | float32 | gyro_y             | deg/s                                    |
| 12     | 4    | float32 | gyro_z             | deg/s                                    |
| 16     | 4    | float32 | accel_x            | g (9.80665 m/s^2), FC body frame          |
| 20     | 4    | float32 | accel_y            | g                                         |
| 24     | 4    | float32 | accel_z            | g                                         |
| 28     | 4    | int32   | baro_pressure_pa   | Pascal                                    |
| 32     | 4    | int32   | baro_temp_centiC   | 0.01 °C                                   |

Total payload = 36 bytes. Total frame = 5 (header) + 36 (payload) + 2 (crc)
= 43 bytes.

## Axis convention (important)

Injection happens **after** the firmware's own per-board sensor alignment
step (`gyroAlign`/`accAlign`, e.g. `CW270_DEG`), not before it. This means
the bridge does **not** need to know the target board's physical chip
orientation — it always sends values in the FC's canonical body frame:

- X: forward / roll axis (positive = nose right roll rate / forward accel)
- Y: right / pitch axis (positive = nose down pitch rate)
- Z: down / yaw axis (positive = nose right yaw rate / down accel)

This matches the axis convention already used by `src/main/common/axis.h`
and the SITL FDM packet (`src/main/target/SITL/udplink.h`), so the bridge's
Gazebo→FC axis conversion code can be shared/ported from the SITL bridge.

## Firmware behavior

- Gyro/accel: the real sensor chip's `readFn()` still runs and still gates
  on real EXTI/interrupt timing (scheduler `GYRO_LOCK_COUNT` is untouched).
  Only the *sampled value* is substituted, immediately after the firmware's
  own alignment/rotation step, using zero-order hold of the latest received
  packet. See `hilSensorOverrideGyro()` / `hilSensorOverrideAcc()` in
  `src/main/io/hil_sensor.c`.
- Barometer: no real chip patch needed. `USE_HIL_SENSOR_OVERRIDE` also
  compiles in the existing `USE_FAKE_BARO` driver
  (`src/main/drivers/barometer/barometer_fake.c`) and adds a `BARO_FAKE`
  hardware option; injection calls `fakeBaroSet()` directly on packet
  receipt (same mechanism the legacy Gazebo-classic SITL setup already used).
- The attitude estimator is never short-circuited — only raw gyro/accel/baro
  values are injected, so the real IMU fusion + PID stack runs unmodified
  (`hitl_plan.txt` Decision #10).

## Safety / fail-safe behavior

- Requires the build-time flag `USE_HIL_SENSOR_OVERRIDE` (never defined by
  default for any target/board — opt-in only via
  `make TARGET=STM32F7X2 EXTRA_FLAGS=-DUSE_HIL_SENSOR_OVERRIDE`).
- Requires the explicit CLI/runtime step of assigning a UART to
  `FUNCTION_HIL_SENSOR` via the `serial` command (same convention as
  `FUNCTION_ESC_SENSOR`/`FUNCTION_GPS` — nothing is active on an
  unconfigured port).
- Heartbeat/staleness: if no valid packet has been received in the last
  250ms (`HIL_SENSOR_TIMEOUT_MS`), `hilSensorIsActive()` returns false and
  override stops being applied — the real gyro/accel raw values (from
  whatever chip is physically wired) pass through unmodified instead.
  Barometer has no automatic fallback (there is no real chip once
  `BARO_FAKE` is selected), so a bridge disconnect will freeze the last
  injected baro reading — acceptable for bench testing, called out here so
  it isn't a surprise.

## Not yet implemented (deferred, tracked in `hitl_plan.txt`)

- Magnetometer injection (`MAG_FAKE` hardware option, mirroring `BARO_FAKE`).
- GPS injection (needs a simulated GPS backend; no fake GPS parser exists
  today).
- Airspeed injection (no airspeed sensor abstraction exists in this repo
  today).
- A push-style actuator telemetry channel (Phase D) — v1 relies on
  MSP_SERVO/MSP_MOTOR polling as originally planned.
