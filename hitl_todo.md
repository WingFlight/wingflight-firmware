# Wingflight HITL — Open / Done TODOs

Companion to `hitl_plan.txt` (the full plan/design doc). This file is the
running task tracker — update it as work happens instead of editing prose
status blocks in the plan.

## Done

- [x] Plan drafted and researched (`hitl_plan.txt`).
- [x] Protocol spec v1 documented: `hitl/bridge/PROTOCOL.md` (gyro,
      accel, baro; mag/GPS/airspeed deferred).
- [x] Firmware: `FUNCTION_HIL_SENSOR` serial port function
      (`src/main/io/serial.h`, `docs/Serial.md`).
- [x] Firmware: `src/main/io/hil_sensor.c/.h` — packet parser, CRC16-CCITT,
      250ms staleness timeout, `TASK_HIL_SENSOR` @ 500Hz.
- [x] Firmware: gyro/accel raw-value override in `sensors/gyro.c`
      (`gyroUpdateSensor`) / `sensors/acceleration.c` (`accUpdate`), applied
      after real driver read + per-board alignment (real EXTI/scheduler
      timing untouched).
- [x] Firmware: `BARO_FAKE` hardware option (`pg/barometer.h`,
      `sensors/barometer.c`, `cli/settings.c`, `docs/Cli.md`), reuses
      existing `USE_FAKE_BARO` driver, enabled only under
      `USE_HIL_SENSOR_OVERRIDE` (`STM32_UNIFIED/target.h`).
- [x] Build verification: `TARGET=STM32F7X2` compiles/links both with and
      without `USE_HIL_SENSOR_OVERRIDE` (opt-in via
      `EXTRA_FLAGS=-DUSE_HIL_SENSOR_OVERRIDE`); default build size delta is
      ~1KB text, attributable entirely to the new module.
- [x] Python bridge skeleton: `hitl/bridge/hitl_bridge/`
      (`protocol.py`, `sensor_source.py`, `serial_link.py`, `bridge.py`,
      `__main__.py`) + unit tests (`tests/test_protocol.py`, all passing)
      + `README.md`.
- [x] `SyntheticSensorSource` — no-Gazebo-required test data source, usable
      today for bench-testing the serial link end-to-end.
- [x] Relocated all HITL tooling from `tools/` to a new top-level `hitl/`
      directory — the root `.gitignore` blanket-ignores `/tools/` (reserved
      for downloaded ARM toolchain artifacts via `make/tools.mk`), so
      anything placed there was silently untracked by git. `hitl/bridge/`
      and `hitl/gazebo/` are confirmed NOT gitignored
      (`git check-ignore` returns nothing for either).
- [x] Phase A scaffolding: `hitl/gazebo/Dockerfile` (installs `gz-harmonic`
      + `python3-gz-transport13`/`python3-gz-msgs10` inside a container,
      commands verified against gazebosim.org/docs/harmonic/install_ubuntu),
      `docker-compose.yml`, `install.sh` (Docker by default, `--native` for
      a direct host install), `scripts/fetch_px4_models.sh` (sparse/shallow
      clone of PX4/PX4-gazebo-models `models/`+`worlds/` into a repo-local
      `.cache/`, gitignored), `scripts/verify_topics.sh` (`gz sim -s -r` +
      `gz topic -l`), and a draft `worlds/wingflight_plane.sdf`.

## Open

### Phase A — Gazebo environment (scripts written, NOT yet executed/verified)
- [ ] Actually run `hitl/gazebo/install.sh` on a real Linux/Docker host (not
      done in this pass - no Gazebo/Docker available in the environment
      this was written in) and confirm the image builds and `gz sim` runs.
- [ ] Run `scripts/fetch_px4_models.sh` and confirm the real PX4 fixed-wing
      model folder name under `.cache/px4-gazebo-models/models/` (candidates
      per docs.px4.io: `rc_cessna`, `advanced_plane` — not yet confirmed
      against an actual checkout).
- [ ] Fill in the commented-out `<include>` in
      `hitl/gazebo/worlds/wingflight_plane.sdf` with the confirmed model
      name/pose.
- [ ] Run `scripts/verify_topics.sh` against the completed world and
      confirm real topic names/message types via `gz topic -l` /
      `gz topic -info -t <topic>`; update the placeholder constants + TODOs
      in `hitl/bridge/hitl_bridge/sensor_source.py` (`GazeboSensorSource`,
      `GZ_IMU_TOPIC`, `GZ_AIR_PRESSURE_TOPIC`) to match.
- [ ] Decide on GUI passthrough (X11/WSLg) if a visual view of the sim is
      wanted — not addressed yet, headless-only so far.

### Phase B — Bridge hardening (skeleton exists, needs Phase A to finish)
- [ ] Wire up and test `GazeboSensorSource` against a running world.
- [ ] Decide/implement a baro temperature source (gz-sim `FluidPressure`
      has no temperature field — may need a second topic or a fixed value).
- [ ] Time-base handling: sim-time vs wall-clock (reference `simRate` in
      `src/main/target/SITL/target.c`).

### Phase C — Firmware (v1 done; follow-ups)
- [ ] Mag injection: add `MAG_FAKE` hardware option mirroring `BARO_FAKE`
      (`pg/compass.h`, `sensors/compass.c`, `cli/settings.c` lookup table).
- [ ] GPS injection: needs a new simulated GPS backend (no fake GPS parser
      exists today) — bigger task, not started.
- [ ] Airspeed injection: no airspeed sensor abstraction exists in this
      repo today — not started.
- [ ] Real-hardware validation on an actual STM32F7X2-family board (bench
      test only done via build/link so far, not on real hardware/EXTI).

### Phase D — Actuator telemetry read-back
- [ ] Bridge-side MSP_SERVO/MSP_MOTOR polling client.
- [ ] Measure achieved rate/latency; only add a push-style channel on the
      dedicated UART if polling proves insufficient.

### Phase E — RC input paths
- [ ] Manual: real RX wiring + `serialrx_provider` CLI config (no firmware
      change expected — verify on real hardware).
- [ ] Automatic: MSP_SET_RAW_RC injection client in the bridge
      (`RX_PROVIDER_MSP`, already compiled in generically).

### Phase F — Pattern & evaluation tooling
- [ ] Pattern runner (`hitl/bridge/patterns/*.yaml`) — not started.
- [ ] Data capture: blackbox + bridge MSP telemetry to CSV — not started.
- [ ] `evaluate.py` — pass/fail comparison vs Gazebo ground truth — not
      started.

### Phase G — Stability testing harness
- [ ] Doublet/step-input maneuver set + control-theory metrics (rise time,
      overshoot, settling time, steady-state error) — not started.
- [ ] Optional wind-gust/disturbance injection via gz-sim wind plugin — not
      started.

## Known open decisions

- Zero-order-hold gyro/accel injection reduces effective control bandwidth
  vs a real sensor's native sample rate (bridge realistically sustains
  ~hundreds of Hz to ~1kHz over serial). Plan's recommendation (keep real
  chip for scheduler timing, override only the sampled value) has been
  implemented; whether the resulting bandwidth is acceptable for fixed-wing
  test goals should be confirmed once real hardware is on the bench.
- Which specific physical board to use for the first real-hardware bench
  test (affects which UART is free to dedicate to `FUNCTION_HIL_SENSOR`).
