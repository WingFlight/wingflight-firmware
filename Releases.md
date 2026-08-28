# 0.0.16

Add XACT servo programming support: multi-servo discovery over FBUS, per-servo background reads, and a field set aligned to FrSky's own configuration tool.
Fix XACT parameter reads hanging forever on a field an unsupported servo doesn't answer.
Fix XACT writes silently reprogramming the wrong servo when two servos share an App ID.
Fix XACT servo tracking losing a servo after its Physical ID is renamed.
Fix SRXL2 telemetry seqlock read.
Make the governor/idle-up switch a hard motor interlock when a governor mode is configured, instead of only refining the bottom of the throttle curve.

# 0.0.15

Add support for Spektrum SRXL2 ESC (throttle and telemetry over a single bidirectional bus).
Support Spektrum full-size receivers (e.g. AR6610T) by sending an early SRXL2 handshake at boot.

# 0.0.14

Add debounce for servo trim adjustments to prevent snapping on RX link reacquisition.

# 0.0.13

Fix servo trim ignoring mixer-rule-based servo reversal, causing mirrored servos (e.g. dual ailerons) to trim the same direction.
Fix servo trim also ignoring a negative (Inverted) rate on the driving mixer input, causing trim to apply backwards on models using axis invert.

# 0.0.12

Fix BOXTHRUSTVECTOR/BOXLOITER permanentId collision.
Fix governor bypassing RX-loss failsafe throttle cut.
Clamp m/3s telemetry to 0 when out of range.
Update current sensor FBUS refresh to accept timestamp and adjust capacity calculation.

# 0.0.11

Add basic GPS Loiter and RTH flight modes.
Add Thrust Vector PID control and blackbox logging.
Add MSP commands for FBUS/S.Port sensor diagnostics and forwarding.
Add airborne re-arm grace settings for improved in-flight safety.
Improve HoTT telemetry responsiveness and GPS accuracy.
Use governor idle throttle as the RPM idle-hold floor to avoid ESC cut on rapid throttle chop.
Fix blackbox MSC mass storage startup.
Remove OSD and CMS.

# 0.0.10

Remove legacy heli tail_rotor_mode; rename main/tail naming to motor1/motor2 throughout (gear ratios, RPM/speed accessors, telemetry sensor IDs, blackbox log fields).
Add descriptive model_type field to mixer config.

# 0.0.9

Add servo trim adjustments for roll/pitch/yaw, with resync/commit support.
Add governor RPM max limiter.
Add FrSKY RPM S.Port sensor support.
Add runtime effective PID gain MSP endpoint.
Allow old and new mode for XDFLY/ZTW/OMPHOBBY ESC telemetry.
Fix S.Port master detection when checking forwarded FBUS sensor telemetry.
Fix mixer/gain curve point count validation from MSP.
Widen master_gain range for finer PID gain resolution.

# 0.0.8

Rework fixed-wing throttle attenuation (TPA) to use the shared gain-curve pool instead of a fixed linear ramp.

# 0.0.7

Fix stabilization break when mounting-surface trim is set without board alignment.

# 0.0.6

Add Mounting-surface trim (manual entry and auto-detect wizard).
Add throttle range governor.
Add ready-to-arm surface wiggle.
Add auto trim bit to flight mode sensors.
Rework ATTHOLD mode with a new implementation.
Fix Board Auto-Align sign-blind scoring; gate on accelerometer calibration.
Reduce gyro calibration sensitivity to cope with windy days.

# 0.0.5

Add AUTO TRIM flight mode (servo-center capture, ported from iNav's BOXAUTOTRIM).
Fix auto-hover fighting aileron input, allowing free rotation in hover.

# 0.0.4

Fix missing channel slot handling.
Fix pitch-up direction when enabling auto-hover.
Add distinct manual and passthrough modes.
Add auto-hover flight mode support.
Add cross-axis relax behavior.
Remove atthold mode.
Update default servo rate.

# 0.0.3

ESC Programing
Improve telemetry conditions
IdleUP governor
Added fixed-wing cross-axis relax tuning for rudder-to-roll coupling in normal stabilization.

# 0.0.2

Board Alignment
Remove collective from channel maps
Added in ability to set master gains for RPY via msp and adjustment functions
New flight mode sensor
Reset mixer rules so stale saved throttle mixes are rebuilt with M1 as the default motor output.

# 0.0.1

This is the first _development snapshot_ of the Wingflight firmware.

## Notes

Wingflight is a fork of Rotorflight, refocused exclusively on fixed-wing 3D
and aerobatic aircraft. This is the first release under the Wingflight name,
starting a fresh release history independent of Rotorflight.

This version is intended to be used for beta-testing only. It is not fully
working nor stable, and should not be used by end-users.

For more information, please join the [Wingflight Discord](https://discord.gg/aEyyAJTXRw/) chat.

## Downloads

The download locations are:

- [Wingflight Configurator](https://github.com/WingFlight/wingflight-configurator/releases/tag/snapshot/0.0.1)
- [Wingflight Lua Suite for FrSky Ethos](https://github.com/WingFlight/wingflight-lua-ethos-suite/releases/tag/snapshot/0.0.1)
