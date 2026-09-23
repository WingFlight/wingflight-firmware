# Changes in Rotorflight Firmware

This file is collecting the changes in the firmware that are affecting
the APIs or flight performance.


## Flight Performance

Airborne detection now requires at least 10% pilot roll or pitch input with
at least 15 degrees/second gyro response in the same direction for 250 ms
continuously. Each axis and direction qualifies independently. Arming alone,
static tilt, yaw steering, motor output and altitude do not establish flight.
Once detected, flight is latched until disarm, preserving full attitude
correction through hands-off flight and motor-off glides. Ground reduction
therefore returns on disarm, not automatically after landing while armed.
The existing armed GPS-rescue/failsafe override and hands-on detection remain.
Normal rate/manual behavior and TRAINER are unchanged. ANGLE/HORIZON, ATT HOLD,
TV hold and AUTO HOVER retain their existing airborne-based correction policy.
Debug AIRBORNE indexes 3 and 5 now show roll/pitch response duration in ms.
These initial thresholds need bench/flight validation: hand movement can
imitate a response, and flight without a qualifying input remains undetected.
See `src/main/flight/airborne.c`.

TRAINER now keeps normal rate-mode I-term decay independently on roll and pitch
until its envelope limiter changes that axis's rate command. Stronger pilot
input back into the envelope also retains normal decay. Active limiting still
suspends decay to sustain correction (`src/main/flight/trainer.c`,
`src/main/flight/pid.c`). Mode exit and profile reload clear limiter state.


ANGLE and TRAINER now support independent roll and pitch limits per PID profile
(`src/main/flight/leveling.c`, `src/main/flight/trainer.c`). Explicit roll limits
use 10–90° and pitch limits 10–75°, matching SAFE's documented configuration
ranges. ANGLE commands attitude and self-levels; TRAINER retains rate control
with envelope limiting and no self-leveling. This does not change the trainer's
prediction algorithm or airborne detection, and is not stall protection.

New CLI settings: `angle_roll_limit`, `angle_pitch_limit`,
`acro_trainer_roll_limit`, `acro_trainer_pitch_limit`. Zero inherits the existing
`angle_level_limit` / `acro_trainer_angle_limit`; positive values below 10 are
effectively 10. Inherited legacy values remain unchanged, including pitch limits
above 75°. The new PG_ATTITUDE_LIMITS array preserves existing PID-profile
storage and participates in profile copy/reset. MSP API 22.4 appends four U8
values to PID_PROFILE in ANGLE roll/pitch, TRAINER roll/pitch order. Older writes
leave them intact; updated clients expose the independent controls. The ANGLE
limits also apply to the shared leveling path used by HORIZON and GPS navigation.


Fixed-wing cross-axis relax is added for normal stabilization. When enabled,
yaw/rudder command can attenuate roll and/or pitch P and D feedback, and slow
the I accumulation (the I output itself is not scaled), so rudder-induced
coupling is not held artificially flat. The feature is
configured by `cross_axis_relax_strength`,
`cross_axis_relax_pitch_strength`, `cross_axis_relax_level`, and
`cross_axis_relax_cutoff`, and defaults to off.

The decimator is changed to use a Bessel filter (#287). It should give more
consistent D-term reaction on transients.

PID Mode 4 is introduced for testing new features (#293). The current default
PID Mode 3 is maintained for backward compatibility.

`nav_loiter_direction` now orbits the way it says. Loiter and RTH steered the
wrong way round the target (`CW` orbited anticlockwise and the other way
round). If you set the opposite value to get the direction you wanted, swap it
back after updating.

## Configuration Changes

Added airborne re-arm grace settings `rearm_grace_seconds` and
`rearm_min_armed_seconds`. After the aircraft has been armed for the minimum
time and has latched in-flight state, an accidental disarm opens a short re-arm
window where only throttle and angle arming checks are ignored; all hard safety
checks still apply.

The fixed-wing I-term decay settings are renamed from
`error_decay_time_cyclic` / `error_decay_limit_cyclic` to
`iterm_decay_time` / `iterm_decay_limit`. The old CLI names remain accepted
as aliases for compatibility with existing dumps. MSP byte layout is unchanged.

Added `cross_axis_relax_strength`, `cross_axis_relax_pitch_strength`,
`cross_axis_relax_level`, and `cross_axis_relax_cutoff` PID profile settings.

`master_gain` now scales only the stabilizing P/I/D terms. Feedforward (`F`)
is no longer scaled by `master_gain`, keeping F as a separate command-response
tuning parameter while master gain remains focused on loop authority.

Fixed-wing throttle PID attenuation (`fw_tpa_breakpoint` / `fw_tpa_rate`) is
replaced by `fw_tpa_gain` and `fw_tpa_curve`, mirroring `master_gain` /
`gain_curve`: `fw_tpa_gain` is a baseline percent scale (100 = unscaled), and
`fw_tpa_curve` is an optional index (0=off, 1..`GAIN_CURVE_COUNT`) into the
same shared gain-curve pool used by per-axis `gain_curve`, further scaling
`fw_tpa_gain` by throttle instead of |stick deflection|. This allows a
non-linear attenuation shape instead of the previous fixed linear ramp.
Existing PID profiles are reset to defaults on upgrade.

Added `model_type` to `mixerConfig_t` (`REGULAR_AIRPLANE` / `FLYING_WING` /
`V_TAIL_AIRPLANE` / `DELTA_WING` / `RUDDER_ELEVATOR_TRAINER` / `CUSTOM`).
This is descriptive metadata only -- the firmware does not read it or change
mixer behavior based on it. It exists so the configurator can show a
simplified mixer view for named airframes and reserve the full raw rule
editor for `CUSTOM`. Defaults to `REGULAR_AIRPLANE` on upgrade, matching the
default mixer rule set.

Removed `tail_rotor_mode` (and the `TAIL_MODE_VARIABLE` / `TAIL_MODE_MOTORIZED`
/ `TAIL_MODE_BIDIRECTIONAL` enum) from `mixerConfig_t`. This was a legacy
helicopter tail-rotor setting with no meaning for wingflight's fixed-wing
mixer. The RPM filter and motor code that used to key off it (selecting the
"tail motor" for RPM-based notch filtering, and deriving a synthetic tail
frequency from the main motor via a gear ratio when there was no separate
tail motor) now just checks whether a second motor is configured
(`getMotorCount() > 1`), so genuine dual-motor airframes keep independent
per-motor RPM notch filtering without a manual mode switch. Because removing
the field shifts `model_type` into `tail_rotor_mode`'s old byte offset,
`mixerConfig` PG version is bumped (1 -> 2) this time, so `model_type` resets
to its default on upgrade instead of being overlaid with the old
`tail_rotor_mode` byte.

Renamed the "main/tail" (main rotor / tail rotor) naming leftover from this
firmware's helicopter origins to "motor1/motor2" throughout, since it's
generic per-motor RPM/gear-ratio handling now, not heli-specific:

- CLI: `main_rotor_gear_ratio` -> `motor1_gear_ratio`,
  `tail_rotor_gear_ratio` -> `motor2_gear_ratio`. MSP wire layout (2x U16
  pairs in `MSP_MOTOR_CONFIG`/`MSP_SET_MOTOR_CONFIG`) is unchanged, only the
  field names changed -- not a breaking wire change.
- Internal: `getMainGearRatio()`/`getTailGearRatio()` ->
  `getMotor1GearRatio()`/`getMotor2GearRatio()`; `getHeadSpeed()`/
  `getHeadSpeedf()`/`getTailSpeed()`/`getTailSpeedf()` ->
  `getMotor1Speed()`/`getMotor1Speedf()`/`getMotor2Speed()`/
  `getMotor2Speedf()`.
- Telemetry sensor identifiers: `TELEM_HEADSPEED`/`TELEM_TAILSPEED` ->
  `TELEM_MOTOR1SPEED`/`TELEM_MOTOR2SPEED` (numeric sensor IDs 60/61
  unchanged). CRSF and S.Port app IDs (`0x10C0`/`0x10C1`,
  `0x0500`/`0x0501`) are unchanged, so radio-side sensor names for those
  protocols are unaffected here -- they're defined by the receiving
  Lua/radio scripts, not transmitted by the firmware. Jeti EX Bus *does*
  send its sensor name as text: `"Headspeed"`/`"Tailspeed"` ->
  `"Motor1Speed"`/`"Motor2Speed"`, so Jeti radios will show the new name.
- Blackbox log fields: `headspeed`/`tailspeed` -> `motor1speed`/
  `motor2speed`. This changes the log field/column names, so blackbox log
  tooling needs to follow.

Renamed the RC adjustment function `ADJUSTMENT_GOV_HEADSPEED` ->
`ADJUSTMENT_GOV_RPM` (numeric id 80 unchanged) for the same reason -- the
governor now targets a plain RPM value (`governor_rpm`), not a
helicopter-specific "headspeed". This id has no `get_`/`set_` implementation
in `rc_adjustments.c` yet, so it's an identifier-only rename with no wire or
runtime effect.


## MSP Changes

### Heli-only placeholder bytes removed (MSP API 22.3)

The always-zero bytes left over from Rotorflight are dropped from the wire, so
every field after each one moves up. This is a breaking layout change for the
messages below, and the MSP API version is bumped from 22.2 to 22.3 so clients
can tell the two layouts apart. Read and write use the same new layout.

- `MSP_RC_TUNING` / `MSP_SET_RC_TUNING`: `rates_type` (first byte), the collective
  block (`rcRates`, `rcExpo`, `sRates`, `response_time` U8 each, `accel_limit` U16),
  the collective `setpoint_boost_gain`/`cutoff` pair, and the trailing
  `cyclic_ring`/`cyclic_polar`. 11 bytes.
- `MSP_PID_PROFILE` / `MSP_SET_PID_PROFILE`: the three error-decay placeholders,
  `error_rotation`, the eight yaw stop-gain/precomp/collective-FF placeholders,
  the three cyclic cross-coupling bytes, and the two yaw inertia precomp bytes.
  17 bytes.
- `MSP_PID_TUNING` / `MSP_SET_PID_TUNING`: the trailing two U16 `O` terms. 4 bytes.
- `MSP_SETPOINT`: the trailing collective S16. 2 bytes.
- `MSP_TELEMETRY_CONFIG` / `MSP_SET_TELEMETRY_CONFIG`: the U32 `enableSensors`
  after `halfDuplex`. 4 bytes.
- `MSP_ESC_SENSOR_CONFIG` / `MSP_SET_ESC_SENSOR_CONFIG`: the U32 HW4 parameters
  after `current_offset`. 4 bytes.
- `MSP_RC_CONFIG` / `MSP_SET_RC_CONFIG`: the U16 `rc_arm_throttle` after
  `rc_deflection`. 2 bytes.
- `MSP_SENSOR_CONFIG` / `MSP_SET_SENSOR_CONFIG`: the U16 `gyro_offset_yaw` after
  `gyroCalibrationDuration`. 2 bytes.

`MSP_STATUS` still carries its two compat placeholder bytes.

### MSP_PID_PROFILE

- appended `cross_axis_relax_strength`, `cross_axis_relax_level`,
  `cross_axis_relax_cutoff`, and `cross_axis_relax_pitch_strength`.
- `error_rotation` parameter is unused (#294)
- `fw_tpa_breakpoint` and `fw_tpa_rate` are replaced by `fw_tpa_gain` (U8,
  baseline percent scale) and `fw_tpa_curve` (U8, gain-curve index).

### MSP_SET_PID_PROFILE

- accepts optional appended `cross_axis_relax_strength`,
  `cross_axis_relax_level`, `cross_axis_relax_cutoff`, and
  `cross_axis_relax_pitch_strength`.
- `error_rotation` parameter is unused (#294)
- `fw_tpa_breakpoint` and `fw_tpa_rate` are replaced by `fw_tpa_gain` (U8,
  baseline percent scale) and `fw_tpa_curve` (U8, gain-curve index).

### MSP_PILOT_CONFIG

- added `modelFlags` parameter (#317)

### MSP_FLIGHT_STATS

- added `MSP_FLIGHT_STATS` and `MSP_SET_FLIGHT_STATS` (#317)

### MSP_SET_MOTOR_OVERRIDE

A 1.0s timeout is added to the override. The MSP call must be repeated
at least once per second to keep the override active (#304).

### MSP_SET_MOTOR

This legacy MSP call is disabled, as it does not have a timeout (#304).

### MSP_SET_4WIF_ESC_FWD_PROG

New MSP command (244) to select an ESC for forward programming over the 4-way interface. Payload: U8 ESC id; values in `0..MAX_SUPPORTED_MOTORS-1` select that ESC, while values `>= MAX_SUPPORTED_MOTORS` (for example `0xFF`) are treated as deselect/exit and return success. The only error conditions for command `244` are: the system is armed, the payload length is not exactly 1 byte, or the 4-way selection fails.

When a 4-way ESC is selected, `MSP_ESC_PARAMETERS` / `MSP_SET_ESC_PARAMETERS` now expose the detected target EEPROM payload for both AM32 and BLHeli_S SiLabs targets. AM32 continues to use the compact 48-byte payload; BLHeli_S uses the 0x70-byte BLHeli_S EEPROM layout and erases the containing settings page before writes.

### MSP_SET_PID_PROFILE

The `pid_mode` parameter can be now changed.

### MSP_GOVERNOR_PROFILE

Multiple changes (#314) (#353).

### MSP_GOVERNOR_CONFIG

Multiple changes (#314) (#353).

### MSP_MIXER_CONFIG

- `tail_rotor_mode` (U8) is removed. Payload is now just `model_type` (U8,
  descriptive-only airframe type, see Configuration Changes). This is a
  breaking wire change -- older configurator builds that expect the old
  2-byte payload will misread `model_type` as `tail_rotor_mode`.

### MSP_SET_MIXER_CONFIG

- `tail_rotor_mode` (U8) is removed. Payload is now just `model_type` (U8).
  Breaking wire change, same as `MSP_MIXER_CONFIG` above.

### MSP_SERVO_TRIM

New MSP command (233) returning the live, runtime-only servo trim in us set by
continuous (mapped) `SERVO_TRIM_*` adjustments. It is never saved and starts from
zero at boot, so a client can show that a trim is in effect even though the servo
center is unchanged. Returns: U8 count, then one S16 per servo, in the same order
and count as `MSP_SERVO_CONFIGURATIONS`. Read-only, and small enough to fit the
MSP response buffers at any servo count. The API version is not bumped: firmware
without it answers the command as unsupported, which is how a client tells.

### MSP_BUS_SERVO_CONFIG

New MSP command (152) to retrieve BUS servo source configuration (18 channels).

### MSP_SET_BUS_SERVO_CONFIG

New MSP command (153) to configure individual BUS servo source settings. Payload: U8 index (0-17) + U8 sourceType (0=MIXER, 1=RX).

### MSP_GET_BUS_SERVO_CONFIG

New MSP command (157) to retrieve individual BUS servo source configuration. Payload: U8 index (0-17). Returns: U8 sourceType.

### MSP_SET_SERVO_CONFIG

New MSP command (124) to configure individual servo settings (PWM and BUS servos). Payload: U8 index + 8x U16 fields (mid, min, max, rneg, rpos, rate, speed, flags).

### MSP_GET_SERVO_CONFIG

New MSP command (125) to retrieve individual servo configuration. Payload: U8 index. Returns: 8x U16 fields (mid, min, max, rneg, rpos, rate, speed, flags).

### MSP_SET_SERVO_OVERRIDE_ALL

New MSP command (196) to set servo overrides for all servos in one call. Payload: U16 value (0=enable/center, 2001=disable).

### MSP_SET_SERVO_CENTER

New MSP command (213) to set just the servo center point. Payload: U8 index + U16 mid.

### MSP_GET_MIXER_INPUT

Add msp call to allow retrieving a single mixer line at a time (#361)

### MSP_GET_ADJUSTMENT_RANGE

Add msp call to allow retrieving a single adjustment line at a time (#362)

### MSP_SERVO

Modified to support bus servos:

- Condition: when bus servos are configured — e.g. the `SBUS_OUT` or `FBUS_MASTER` serial function is enabled.
- Returns: configured PWM servo outputs `S1–Sn` (up to `getServoCount()`), followed by all bus servo outputs `S9–S26` (starting at `BUS_SERVO_OFFSET`).
- Skipping behavior: any unconfigured PWM servos between `getServoCount()` and `BUS_SERVO_OFFSET` are skipped.

### MSP_SERVO_CONFIGURATIONS

Modified to support bus servos. When bus servos are configured:

- Mapping (indices → servo outputs):
	- indices `0` to `getServoCount()-1` → PWM servos `S1–S{getServoCount()}`
	- indices `getServoCount()` to `getServoCount()+17` → bus servos `S9–S26`

- Note: `getServoCount()` is used to determine how many PWM servos are present; any unconfigured PWM servos between `getServoCount()` and `BUS_SERVO_OFFSET` are skipped when assembling the returned list.

### MSP_SET_SERVO_CONFIGURATION

Modified to support bus servos. When bus servos are configured, the index parameter is mapped: indices 0 to (getServoCount()-1) map to PWM servos S1-Sn, indices getServoCount() to (getServoCount()+17) map to bus servos S9-S26.

### MSP_RC_TUNING

The `cyclic_ring` parameter is added (#345).

The `cyclic_polar` parameter is added (#426).

### MSP_GET_ADJUSTMENT_FUNCTION_IDS

Added a call to deliver the function id in use per slot (#398)

### MSP_BATTERY_STATE

The `batteryProfile` field is added. (#415)

### MSP_BATTERY_CONFIG

The `batteryCapacity` array is added. (#415)

The `batteryCellCount`, `vbatmincellvoltage`, `vbatmaxcellvoltage`,
`vbatfullcellvoltage` and `vbatwarningcellvoltage` arrays are added, one value
per battery profile. The legacy single-value fields report the active profile.

### MSP_SET_BATTERY_CONFIG

The `batteryCapacity` array is added. (#415)

The `batteryCellCount`, `vbatmincellvoltage`, `vbatmaxcellvoltage`,
`vbatfullcellvoltage` and `vbatwarningcellvoltage` arrays are added (optional).
The legacy single-value fields are stored into the active profile.

### MSP_BATTERY_PROFILE

New MSP command to get the active battery profile. (#415)

### MSP_SET_BATTERY_PROFILE

New MSP command to set the active battery profile. (#415)

### MSP_SETPOINT

New MSP command to get the current setpoint. (#443)

### MSP2_GET_SMARTFUEL_CONFIG

New Rotorflight MSPv2 command (`0x4000`) to get the SmartFuel configuration.

### MSP2_SET_SMARTFUEL_CONFIG

New Rotorflight MSPv2 command (`0x4001`) to set the SmartFuel configuration.


## CLI Changes

`pid_process_denom` is a divider for the PID loop speed vs. the gyro
output data rate (ODR). With #291 the output rate is halved, dropping
the PID loop rate to half too.

`error_rotation` parameter is removed in #294.

`model_set_name` parameter added (ON/OFF). Corresponds with bit 0 of `pilotConfig_t.modelFlags` and is used to indicate whether the Lua scripts should set the name of the model on the radio.

`model_tell_capacity` parameter added (ON/OFF). Corresponds with bit 1 of `pilotConfig_t.modelFlags` and is used to indicate whether the Lua scripts should announce the remaining capacity of the battery.

`board_name`, `board_design`, and `manufacturer_id` now display a detailed
incompatible-configuration warning and halt the system when an attempt is made
to change them after they have been set. Previously only an error was shown.

`deadband` parameter maximum value is changed from 32 to 100 (#327).

`rc_arm_throttle` parameter is removed (#332).

`rc_min_throttle` and `rc_max_throttle` parameters default to zero, indicating that
the actual values are calculated automatically (#332).

`gov_mode` now accepts values `OFF`, `LIMIT`, `DIRECT`, `ELECTRIC`, `NITRO`.

`gov_throttle_type` is added, with possible values `NORMAL`, `SWITCH`, `FUNCTION`.

`gov_spooldown_time` is added. Value in 1/10s increments.

`gov_idle_throttle` is added. Value in 0%..25%, with 0.1% steps.

`gov_auto_throttle` is added. Value in 0%..25%, with 0.1% steps.

`gov_bypass_throttle` is added. Value array of 9, with values in 0..200. Step is 0.5%.

`gov_use_<xyz>` flags have been added. Value is `OFF` or `ON`.

`gov_fallback_drop` is added. Value in 0..50%.

`gov_dyn_min_throttle` is added. Value in 0..100%.

`gov_collective_curve` is added. Value in 5..40.

`gov_autorotation_bailout_time` is removed.

`gov_autorotation_min_entry_time` is removed.

`gov_lost_headspeed_timeout` is removed.

`gov_spoolup_min_throttle` is removed.

`blackbox_log_governor` flag is added.

`bus_servo_source_type` parameter added. Array of 18 uint8 values where element `bus_servo_source_type[i]` selects the source type for BUS servo channel `i` (array indices 0–17 map to channel numbers 0–17). Value selects the source type: `MIXER` = 0, `RX` = 1. The "source index" is equal to the channel number — i.e. when `RX` is selected the RX input index equals the channel number, and when `MIXER` is selected the mixer channel index equals the channel number.

`fbus_master_frame_rate` parameter added. Value in 25..550 Hz, controls the FBUS Master output frame rate.

`fbus_master_pinswap` parameter added (ON/OFF). Swaps TX/RX pins on the FBUS Master serial port.

`fbus_master_inverted` parameter added (ON/OFF). Controls electrical inversion of the FBUS Master UART output.

`freq_input_minhz` parameter added. Value in 1..100 Hz, sets the minimum accepted frequency on the input sensor.

`rates_type` accepts `ROTORFLIGHT` (#345).

`cyclic_ring` meaning is changed. The value indicates % of the max rate.

`resource GYRO_CLK` added. Sets the pin for the gyro synchronisation clock, if supported.

`gyro_offset_yaw` is removed (#391).

`bat_capacity` parameter changed from a single value to an array of 6 values (one for each battery profile).

`bat_profile` parameter added. Value in 0-5, selects the active battery profile.

`battery_cell_count`, `vbat_max_cell_voltage`, `vbat_full_cell_voltage`,
`vbat_min_cell_voltage` and `vbat_warning_cell_voltage` changed from a single
value to an array of 6 values (one for each battery profile). This allows
profiles with different cell counts (e.g. 3S and 4S) and chemistries
(e.g. LiPo and LiHV). A single value in an old `diff` only sets profile 0.
Each profile must be ordered `min` <= `warning` <= `full` <= `max`; a profile
that is not ordered is reset to the defaults.

`pid_gyro_filter_type` and `yaw_precomp_filter_type` parameters are removed (#414).

`serialrx_provider` extended to include `IBUS2` as a protocol option

`smartfuel` parameter added (`OFF`/`VOLTAGE`/`CURRENT`/`COMBINED`). Selects the SmartFuel charge estimator mode.

`smartfuel_voltage_drop_rate` is in **millivolts per second** (mV/s): maximum downward slew of the internal filtered **per-cell** voltage used by the estimator. Range 0..250, default 10.

`smartfuel_charge_drop_rate` is in **0.01% units per second** (maximum drop rate of the displayed percentage once armed). Range 0..250, default 50.

`smartfuel_sag_gain` scales sag compensation from cyclic and collective stick load while airborne. Range 0..100, default 40.

`bus_servo_clone_pwm` parameter added (ON/OFF). When ON, bus servo channel N mirrors PWM servo output S(N+1) one-to-one for every bus channel that has a physical PWM counterpart, so a PWM-only mixer setup (e.g. a named model type built via the configurator wizard) drives bus servos too without separate mixer rules. Bus channels beyond the physical PWM servo count are unaffected and always use their own mixer rule. `MSP_MIXER_CONFIG`/`MSP_SET_MIXER_CONFIG` gain a second byte carrying this value.


## Defaults

`cbat_alert_percent` changed from 10 to 35 to better reflect heli usage.

`rescue_flip` default is changed from OFF to ON.

`deadband` and `yaw_deadband` defaults changed to 5.

`rc_min_throttle` and `rc_max_throttle` defaults are changed to 0.

`motor_poles` default is changed to 0,0,0,0.

`bus_servo_source_type` defaults to MIXER (0) for first 8 channels, RX (1) for channels 8-17.

`fbus_master_frame_rate` defaults to 500 Hz.

`fbus_master_pinswap` defaults to OFF (0).

`fbus_master_inverted` defaults to ON (SERIAL_INVERTED), which is the standard for FBUS receivers.

`rates_type` default is changed to `ROTORFLIGHT`.

`cyclic_ring` default is changed to 150%.

`rc_threshold` default for collective (4th element) is changed from 50 to 100 (5% to 10% stick) for airborne/hands-on detection.

`gyro_decimation_hz` default is changed to 500Hz (#405).

`blackbox_mode` default is changed to ARMED (#412).

`blackbox_log_governor` default is changed to ON (#412).

`blackbox_rolling_erase` default is changed to ON (#412).

`bus_servo_clone_pwm` defaults to ON.

`roll_srate` default is changed to 12 (#413).

`pitch_srate` default is changed to 12 (#413).

`yaw_srate` default is changed to 12 (#413).

`collective_srate` default is changed to 12 (#413).

`error_limit` default is changed to 45,45,60 (#425).

`offset_limit` default is changed to 90,90 (#425).

`stats_min_armed_time_s` default is changed to 15 (#460).


## CRSF Custom Telemetry

`BATTERY_PROFILE` sensor 0x1214 type U8 is added.


## Features

### Drop gyro ODR to 4k on F4 and F7 (#291)

The gyro output data rate is changed from 8k to 4k on F4 and F7.
This lowers the real-time load considerably, and gives more headroom for
other functions. It should not affect performance.

### Use Bessel filter in the decimator (#287)

Using a Bessel filter in decimator should give better phase response
near the cutoff frequency. This should give more consistent D-term
reaction to fast movements.

### Motor Override (#304)

A safety mechanism is added to the Motor Override that will turn off the throttle
if the override command is not repeated continuously. This guarantees that
the motor is not left running if the connection to the FC is interrupted.

### ELRS Custom Telemetry maximum frame size (#323)

The maximum size of custom telemetry frames is reduced to 32.
This will improve telemetry reception in poor radio conditions.

### ELRS RPM and Temperature telemetry frame types (#326)

The native ELRS telemetry can now send RPM and temperature data.
The RPM frame (0x0C) supports sending headspeed and tail speed.
The temperature frame (0x0D) supports MCU and ESC temperatures.

Two new RF Telemetry sensors are added: RPM (108) and TEMP (109).

### Throttle Range calculated automatically (#332)

The input throttle range is now calculated automatically from `rc_deflection`.
It can be still set by the user with `rc_min_throttle` and `rc_max_throttle`.
The parameter `rc_arm_throttle` is removed, and arming is allowed when
input throttle is well below `rc_min_throttle`.

### Motor Pole Count (#333)

The default pole count is now zero, which is effectively disabling the RPM input.
This forces the user to enter the correct number, before the RPM input can be used.

### PID Mode 4 (#293)

All new features and changes to the PID controller are done in the new PID mode 4.
The current PID Mode 3 will be kept as-is for backward compatibility.

**Changes:**
- axis_error changed from actual angle to I-term units
- I-gains, O-gains and F-gains forced to be the same for roll & pitch
- I-term decay forced if I-gain is zero (Rate Mode)
- Pitch B-gain scaled x10
- Roll B-gain and D-gain scaled /5
- Yaw precomp cutoff scaled by /10

### Governor Refactoring (#314) (#343) (#353)

The Governor has been refactored to accomodate I.C./nitro and other new features.

### Rotorflight Rates (#345)

A new Rates systems is added for helicopter applications. `ROTORFLIGHT` rates is
controlled by three parameters: maximum rate, expo, and shape.

### Gyro calibration (#391)

The previous gyro calibration (sum/variance and sample counting) is replaced
with a filter-based flow: raw data is passed through a Bessel noise filter,
then split into DC (bias) and high-frequency components via PT filters.
When the minimum sample count (from `gyro_calib_duration`) is reached and
the smoothed high-frequency envelope is below `gyro_calib_noise_limit` on
all axes, the DC estimate is stored as the gyro zero and calibration
completes. The existing CLI parameters are unchanged.

### RPM Filter Presets (#406)

Minor changes introduced to all three presets for better match to common
use cases.

### Servo/Mixer Override disables arming (#431)

A new arming disabled flag `OVERRIDE` was added. It is activated if either
servo or mixer override is active.

### SmartFuel Battery Charge Estimator (#463)

A new battery charge estimator is added that provides a monotonically
non-increasing remaining-charge percentage suitable for telemetry, audible
warnings, and Lua scripts.

The estimator runs at the voltage task rate and combines:
- a slew-rate-limited per-cell voltage (`smartfuel_voltage_drop_rate`),
- a stick-load voltage-sag compensation while airborne, scaled by
  `smartfuel_sag_gain` (cyclic + collective deflection),
- a sigmoid voltage-to-charge curve clamped between `vbat_min_cell` and
  `vbat_full_cell`,
- optional coulomb counting from the measured current and the active
  `bat_capacity` profile,
- a slew-rate limit on the displayed charge level while armed
  (`smartfuel_charge_drop_rate`), so the reported percentage cannot
  bounce back up during flight.

The mode is selected by the `smartfuel` parameter (`OFF` / `VOLTAGE` /
`CURRENT` / `COMBINED`). When enabled, `getBatteryChargeLevel()` returns
the SmartFuel estimate instead of the legacy capacity-based percentage,
and the value is always reported as available regardless of whether
`bat_capacity` is configured.

## Receiver Protocols

### IBUS 2 Support (#424)

Support for the IBUS2 protocol for control link and basic telemetry using the ibus hub protocol.

## Bug Fixes

### Continuous servo trim no longer changes the saved servo center

A continuous ("Absolute") `SERVO_TRIM_ROLL/PITCH/YAW` adjustment, where a pot or
channel position is the trim, used to be written into the servo center and saved.
After a reboot the pot applied itself again on top of its own saved result, and a
bad reading (e.g. a channel that was not valid yet at boot) left a wrong center
behind.

It is now a runtime-only offset added at the servo output. It starts from zero at
boot, follows the pot, is never saved, and is limited to 20% of the servo's scale
(the larger of `rneg`/`rpos`). Switch-stepped adjustments and `BOXAUTOTRIM` still
edit the servo center as before, and auto trim leaves the pot's part out of the
center it saves. No MSP or configuration changes.

### Servo trim (SERVO_TRIM_*) could snap on a boot-time or reacquired RX link

The mapped/continuous ("Absolute") in-flight adjustment mode had no debounce
of its own, unlike the stepped mode: it wrote whatever the adjustment channel
read straight to the servo center on every tick. A single garbage or
not-yet-settled frame right at boot, or immediately after the RX link was
reacquired following a brief dropout, could snap a servo's trimmed center
before the pilot had any control over it.

`SERVO_TRIM_ROLL/PITCH/YAW` adjustments now require the RX link to have been
continuously valid for 1000 ms (was 300 ms) before they are evaluated at all, and the
continuous/mapped mode now uses the same +-2 / 100 ms channel-stability
debounce that stepped mode already had. Other adjustment functions (PID
gains, rates, etc.) are unaffected.

### Servo trim (SERVO_TRIM_*) could still snap in continuous/"Absolute" mode, and was sluggish to use

The boot/reacquisition debounce added above narrowed the window but didn't
close it: continuous ("Absolute") mode maps a channel position straight to
the servo center with no per-tick increment of its own (unlike stepped
mode's step size), so once a reading cleared the settle/stability checks it
could still move the physical servo center by the whole adjustment range in
a single tick. Reports of the original snap still occurring, specifically
on channel-mapped (not switch-stepped) trim setups, confirmed this path.
The same +-2 / 100 ms stability debounce also meant a continuously-moving
pot/channel never updated at all while it was moving -- it only caught up
100 ms after the pilot stopped and held still, making live trimming feel
sluggish and coarse.

Continuous-mode `SERVO_TRIM_ROLL/PITCH/YAW` now tracks the channel live on
every tick (the stability debounce is gone for this mode), but the applied
value slews toward the mapped target instead of jumping straight to it,
capped to roughly 200us/sec (the full +-200 range takes ~2s to traverse, in
line with the auto-trim capture window). A bad reading can now only nudge
the servo center a little before the next good one corrects it back --
never a snap -- while legitimate movement is followed in real time instead
of waiting for the pilot to stop and hold still. Stepped mode is
unaffected -- it already couldn't snap and was never subject to the
stability debounce's live-tracking issue.

### S.PORT telemetry Scaling for attitude sensors

The attitiude sensors where found to be out by a factor of 10.  The scaling
in the firmware has been adjusted to set these correctly. (#313)

### Servo `speed` limiting no longer couples independent wing surfaces

`servoUpdate()` (PWM) and the SBUS-out mixer path shared a Rotorflight-heritage
"cyclic" mode: any servo fed by stabilized/RC roll or pitch was synchronized
with every other such servo, so that one overrunning its `speed` limit scaled
*all* of them down together to keep a helicopter swashplate in plane. On a
fixed-wing airframe roll and pitch feed independent, unrelated surfaces (e.g.
left aileron and elevator), so a slow aileron servo incorrectly slowed the
elevator too. `DEFAULT_SERVO_SPEED` is 0, so this only showed up once a pilot
configured a nonzero `servo_speed`. (#109)

Each servo (and each SBUS-out channel) is now speed-limited independently, in
both `src/main/flight/servos.c` and `src/main/drivers/sbus_output.c`. The
swashplate-coupling logic, `mixerIsCyclicServo()` and the mixer's
`cyclicMapping` bitmap, is removed entirely -- Wingflight targets fixed-wing
airframes only and has no cyclic/swashplate output to keep synchronized. No
MSP or CLI changes; `servo_speed` behaves the same for servos that were never
coupled, and now behaves correctly for the roll/pitch-fed ones that were.

### NaN/Inf can no longer reach the servo output or the mixer

A NaN reaching a comparison-based clamp (`if (x > max) ... else if (x < min)
... else x`) fails every comparison and falls through untouched, since NaN is
"unordered" -- this pattern was used, unguarded, at the servo hardware
boundary (`servoSetOutput()`'s `lrintf(pos * resolution)`, written straight
into a timer compare register -- `lrintf()` of a NaN/Inf is undefined
behaviour), in the mixer's per-input clamp (`mixerApplyInputLimit()`) and
per-rule output (`mixerUpdateRules()`), and in the shared `constrainf()`/
`limitf()` helpers used throughout the codebase. Once a NaN reached a
speed-limited servo's or mixer rule's own slew state (`servoInput[]`,
`sbusServoInput[]`, `mixer.ruleOutput[]`), it stayed there permanently. (#107)

This is defense in depth rather than a currently-reachable flight-time bug
via any first-party input path, with one exception: `MSP_SET_RC_CONFIG`
writes `rc_deflection`/`rc_deadband` with no cross-validation at all (unlike
the CLI's own independent range checks on each), and applies live on the next
`MSP_EEPROM_WRITE` with no reboot required. A deadband at or beyond deflection
zeroed or inverted `rc.range[]` in `initRcProcessing()`
(`src/main/fc/rc.c`), and `data / rc.range[axis]` in `updateRcCommands()`
could then divide by zero (a 0/0 is a NaN) or go negative. `rc.range[]` is
now floored to a minimum of 1.

Fixed by adding `isfinitef()` to `common/maths.h` -- checking the IEEE-754
exponent bits directly, since this firmware is built with `-ffast-math`
(`OPTIMISATION_BASE` in the Makefile), which lets the compiler assume every
float is finite and fold `isnan()`/`isfinite()`/the `x != x` idiom away as
dead code returning a constant (verified against this project's actual
arm-none-eabi-gcc invocation -- a naive `isnan()`/`isfinite()` guard would
silently compile to nothing). `constrainf()` and `limitf()` now use it, and
the same pattern is applied at each hand-rolled equivalent: `limitTravel()`
and `servoSetOutput()` in `servos.c`, `sbusLimitTravel()` and the input stage
of `sbusOutGetValueMixer()` in `sbus_output.c`, and `mixerApplyInputLimit()`
and `mixerUpdateRules()` in `mixer.c`. Existing `+-Inf` clamping (already
correct, since ordered comparisons are unaffected by `-ffast-math`) is
unchanged; only the NaN fallthrough is now defined, resolving to the clamp's
low bound or, at the servo/mixer state-protecting boundaries, to zero. No MSP
or CLI changes.

### Flight-controller failsafe stage 2 is re-enabled and adapted for fixed-wing

`failsafeStartMonitoring()` had its body commented out ("RTFL: Keep disabled until code
refactored"), so `failsafeIsMonitoring()` was always false and `failsafeUpdateState()` (run every
10ms) returned immediately without ever executing. In practice, signal loss was handled entirely
by `rx.c`'s per-channel RX fallback (hold last value 300ms, then configured fallback), never
`FAILSAFE_MODE` -- no staged landing, no disarm-on-loss, no GPS rescue, and every
`failsafe_*` setting was dead. This is documented as finding H-1 in the [Flight Dynamics tech
reference](https://doc.wingflight.org/contributing/tech/flight-dynamics/).

Monitoring is re-enabled (`failsafeStartMonitoring()`), and each of the three
`failsafe_procedure` options now does something real and appropriate for a plane:

- **AUTO-LAND** / **DROP** already worked correctly once monitoring was on, with no changes
  needed: `FAILSAFE_MODE` was already wired into the self-leveling pipeline
  (`pid.c`/`leveling.c`), and disarming a plane only cuts the motor (`motors.c` gates motor
  output on `ARMING_FLAG(ARMED)` independently) -- control surfaces keep self-leveling
  regardless of arm state. AUTO-LAND self-levels for `failsafe_off_delay` before cutting; DROP
  cuts immediately.
- **GPS-RESCUE** previously drove `gps_rescue.c`, a multirotor/heli hover-throttle-learning
  descent algorithm -- neither meaningful nor safe on a fixed-wing airframe. It, and the
  independent `BOXGPSRESCUE` pilot switch that triggered the same thing (already live today,
  regardless of this fix -- see below), now both drive the existing fixed-wing-native
  `navRthStart()`/`updateGpsNav()` controller (`gps_nav.c`, previously only reachable via
  `BOXRTH`) via `RTH_MODE` instead: fly home and orbit at the configured loiter radius/altitude,
  for up to `failsafe_off_delay`, then hand off to the same motor-off self-level glide-down
  AUTO-LAND/DROP already use. If GPS isn't healthy or no home position was ever recorded, it
  falls back to AUTO-LAND's behaviour outright, rather than flying toward `GPS_home == {0,0}`.
  **This is deliberately a bounded, first-cut rescue**: one fixed configured cruise throttle
  (`failsafe_throttle`, below), no altitude-managed pitch-to-throttle correction, no stall/min-
  speed protection, no autoland/flare. A proper altitude-managed autoland (see iNav's
  `navigation_fixedwing.c` for prior art) is a substantial, separate, sensor-dependent
  (baro/airspeed) undertaking, tracked as a follow-up rather than attempted here.
- `failsafe_throttle` (CLI/MSP setting, PWM 1000-2000, documented "throttle level used for
  landing") existed but was never read anywhere in the flight code -- it's now wired into
  `mixer.c`'s throttle input, applied whenever any failsafe procedure is active. Default (1000 =
  off) preserves today's motor-cut behaviour for existing configs; raise it to fly the AUTO-LAND
  glide or the GPS-RESCUE fly-home leg under power. `failsafe_stick_threshold` ("stick deflection
  to exit GPS Rescue") is likewise still declared but unread -- left as a follow-up, same as the
  altitude-managed rescue above.

Two related correctness fixes, found while re-enabling this:

- `BOXGPSRESCUE` ("GPS RESCUE") was already live and independent of this fix -- a pilot switch
  that triggered the heli `gps_rescue.c` algorithm on any armed, GPS-fixed aircraft, switch or no
  failsafe involved. Retargeting it to `RTH_MODE` (above) fixes this too, rather than just the
  failsafe-triggered path. `GPS_RESCUE_MODE` is no longer set anywhere on this fork;
  `gps_rescue.c` stays compiled in but is now a guaranteed no-op (its own internal
  `if (!FLIGHT_MODE(GPS_RESCUE_MODE)) rescueStop();` check), and is a reasonable candidate for a
  follow-up pruning pass, matching how #109 removed `mixerIsCyclicServo()`.
- `BOXPASSTHROUGH`/`BOXMANUAL` bypass PID/leveling entirely at the mixer level, using whatever the
  RC channel currently reads. Since aux/mode channels hold their last value through a real signal
  loss (`RX_FAILSAFE_MODE_HOLD`, the default), a switch that happened to be left engaged the
  moment the link dropped would otherwise keep commanding raw/stale stick position straight to
  the surfaces for as long as failsafe was active -- silently defeating the self-leveling
  `FAILSAFE_MODE` is specifically meant to provide, for all three procedures. `mixer.c` now defers
  to failsafe whenever it's genuinely active, regardless of switch state.

Also: `processRxModes()` (TASK_RX, ~33Hz) and `failsafeUpdateState()` (the 10ms scheduler path)
are independently scheduled with no guaranteed ordering. `core.c`'s per-cycle `BOXRTH`/
`BOXGPSRESCUE` re-evaluation would otherwise clobber `RTH_MODE` back off on any cycle where the
pilot's switch itself isn't engaged (the normal case during a real signal loss) -- `core.c` now
explicitly defers to failsafe.c for the duration of its own GPS-rescue phase.

No unit test coverage exists for failsafe, leveling, airborne detection, or GPS navigation --
verified with a full build and by tracing the state machine for each `failsafe_procedure`/
`failsafe_switch_mode` combination; bench-test (props off) before trusting this in the field.

### GPS RTH/loiter altitude hold was commanding the wrong pitch direction

`updateGpsNav()`'s altitude term (`gps_nav.c`) computed `pitchDdeg = altitudeKp * (targetAltitude -
currentAltitude)` and fed it straight into the pitch target, positive when below target. Pitch in
this codebase's convention is positive **nose-down** (bench-confirmed in `autohover.c`: `+900`
drives the elevator toward nose-down, `-900` is the physically-vertical nose-up target -- the same
convention `attitude.raw[]`/`navAngle[]` use throughout, see `leveling.c`'s
`calcLevelErrorAngle()`). So being below target altitude commanded nose-**down**, diving further
away from it, and being above target commanded nose-up, climbing further away -- actively
divergent, not just ineffective. This affected both `BOXRTH`/`BOXLOITER` (already live) and the
new GPS-rescue failsafe procedure above, which now depends on this same code with no pilot able to
intervene. The roll/bearing term was unaffected (already fixed in #139); only the pitch/altitude
term was inverted. Fixed by negating it. Added `GpsNavAltitudeTest` to `gps_nav_unittest.cc`
(below/above/at target, and clamping to `maxPitchAngleDeg`); confirmed the new tests fail without
the fix and pass with it.

### GPS RTH/Loiter could navigate on a stale position, and wouldn't resume after a fix drop

Two bugs found in code review of the GPS Navigation work above, both in `gps_nav.c`:

- `navIsHealthy()` checked `gpsIsHealthy()` (GPS frames are being received) and satellite count,
  but not `STATE(GPS_FIX)`. `gpsIsHealthy()` says nothing about whether the last frame was
  actually a fix, and UBLOX PVT sets `gpsSol.numSat` from the frame's `numSV` field
  unconditionally, independent of `fixType`/`NAV_STATUS_FIX_VALID` -- so a receiver could report a
  healthy satellite count with no valid fix at all, and RTH/Loiter would keep commanding bank/pitch
  toward `gpsSol.llh.lat/lon` using that stale or invalid position. Fixed by requiring
  `STATE(GPS_FIX)` too.
- When GPS did become unhealthy, `updateGpsNav()` called `navStop()`, but `fc/core.c`'s
  `wasRthActive`/`wasLoiterActive` switch-latch only calls `navRthStart()`/`navLoiterStart()`
  again on the mode switch's off->on edge -- so once `nav.active` was cleared, it stayed cleared
  (RTH_MODE/LOITER_MODE remained flagged "on", but nav commanded nothing) until the pilot cycled
  the switch, even after GPS recovered. Fixed by zeroing the commanded bank/pitch on a health loss
  instead of stopping nav outright, so it resumes toward the original target the moment health
  returns, with no pilot action needed.

Added `GpsNavHealthTest` to `gps_nav_unittest.cc` (healthy satellite count with no fix commands
nothing; nav resumes toward the original target after a fix is lost and reacquired, without
restarting).

Also: `src/test/Makefile` unconditionally passed a few clang-only warning flags
(`-Wno-c99-extensions`, `-Wno-reorder`, `-Wno-error=unused-command-line-argument`) that GCC
rejects as unrecognized under `-Werror`, breaking `make test CC=gcc CXX=g++` on a machine without
clang installed (the comment right above them already said they were clang-specific; they just
weren't gated). Now gated behind `ifeq ($(CC),clang)`. Also fixed a latent MinGW portability gap
in `gps_nav_unittest.cc`'s own test stub: `M_PI` isn't standard C++ and needs a platform-specific
feature-test macro that MinGW's `<cmath>` doesn't define by default -- replaced with a local
literal.

### MSP_FAILSAFE_CONFIG / MSP_SET_FAILSAFE_CONFIG gain `failsafe_recovery_delay`

Found while wiring up Configurator/Lua-suite UI for the failsafe stage-2 work above: this MSP
pair only ever carried six of the seven `failsafeConfig_t` fields (`failsafe_delay`,
`failsafe_off_delay`, `failsafe_throttle`, `failsafe_switch_mode`, `failsafe_throttle_low_delay`,
`failsafe_procedure`) -- `failsafe_recovery_delay` (how long a recovered RX link must stay good
before re-arming is allowed, `failsafeState.rxDataRecoveryPeriod`) was CLI-only, with no MSP
path at all. Appended as a 7th field (U16) on both messages. `MSP_SET_FAILSAFE_CONFIG` only reads
it when present (`sbufBytesRemaining(src) >= 2`, same pattern used elsewhere in `msp.c`, e.g.
`MSP_SET_TELEMETRY_CONFIG`), so older clients that only send the original six fields are
unaffected.

### New MSP command: `gpsNavConfig` (BOXRTH/BOXLOITER/GPS-rescue tuning)

`gpsNavConfig_t` (`PG_GPS_NAV` -- `nav_rth_altitude`, `nav_loiter_radius`, `nav_loiter_direction`,
`nav_max_bank_angle`, `nav_max_pitch_angle`, `nav_min_sats`, `nav_bearing_kp`, `nav_altitude_kp`)
had no MSP command at all -- CLI-only, so `BOXRTH`/`BOXLOITER` and, since the change above,
`FAILSAFE_PROCEDURE_GPS_RESCUE`, were all untunable from the Configurator or the Lua suite even
though they were flyable. New MSPv2 pair, `MSP2_WING_GPS_NAV_CONFIG`/
`MSP2_WING_SET_GPS_NAV_CONFIG` (`0x5F16`/`0x5F17`, next free ID after
`MSP2_WING_CRSF_SENSORS_STATUS`), one flat 12-byte record matching `gpsNavConfig_t`'s field order
exactly. No existing MSP surface changed.

### New custom-telemetry sensor: GPS fix type

The only existing way to see GPS fix state over CRSF/S.Port was `armdisableflags`' GPS bit
(`ARMING_DISABLED_GPS`) -- and that bit permanently clears the first time the model is ever armed
(`WAS_EVER_ARMED`), so it stops reflecting reality right when GPS Rescue/RTH actually needs a
live answer. Added `TELEM_GPS_FIX_TYPE` (custom-telemetry sensor id 119, next free id after
`TELEM_TV_PROFILE`; ids 92-94 stay reserved, heli rescue/governor removed, not reused): `0` = no
fix, `1` = fix, `2` = fix + home captured, read straight off `STATE(GPS_FIX)`/`STATE(GPS_FIX_HOME)`
every time it's polled. Registered in both `crsf.c` (appId `0x112C` -- `0x112B` is taken by the Lua
suite's synthetic GPS Longitude sensor split out of `0x1125` GPS Coord -- after the
existing GPS block `0x1121`-`0x112A`) and `smartport.c` (appId `0x5124`, next free id after
`ARMING_DISABLE_FLAGS`, which also covers FPort/FPort2 -- `smartport.c` already serves
`FSSP_MSPC_FRAME_FPORT` frames on the same sensor table). Not added to any other telemetry
protocol.

### GPS RESCUE switch removed (was a redundant alias for GPS RTH)

`BOXGPSRESCUE` and `BOXRTH` drove the identical fixed-wing RTH controller
(`navRthStart()`/`gps_nav.c`) -- the old multirotor hover-descent GPS Rescue algorithm was
disconnected from both switches earlier (see "Flight-controller failsafe stage 2" above). Two
switches for the same behavior was confusing, and the prearm GPS-fix check only recognized the
legacy `BOXGPSRESCUE`/failsafe-procedure trigger, so a craft wired with only `BOXRTH` -- the
natural fixed-wing choice -- got no GPS-fix protection at all (already fixed separately).

`BOXGPSRESCUE` is now retired: `msp_box.c` no longer registers it (permanentId 46 reserved, not
reused), so it no longer appears in the Modes tab or any MSP box list. The `boxId_e` enum slot is
kept and marked reserved (`fc/rc_modes.h`), matching this codebase's existing convention for
retired boxes (`BOXRESCUE`, `BOXOSD`, `BOXVTXPITMODE`, etc.) -- no renumbering, so existing saved
configs aren't affected. Any leftover `BOXGPSRESCUE` mode-activation-condition from an older
config is now cleared unconditionally on load.

The Failsafe tab's Stage 2 "GPS Rescue" *procedure* (`FAILSAFE_PROCEDURE_GPS_RESCUE`) is
unaffected -- it's a separate config value that also drives the RTH controller automatically on
signal loss, and still works exactly as before.
