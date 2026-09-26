# 0.0.28

Add two packed status telemetry sensors over CRSF and S.Port/FPort/FPort2: SYSTEM_STATUS (120) with armed/airborne state, main and backup RX link, failsafe phase, GPS fix and health, LOITER/RTH blocked, battery state, control saturation, gyro overflow, autotrim and Blackbox state; and SYSTEM_CONFIG (121) with the PID/rates/battery/TV profile numbers, unsaved settings, reboot required and sensors present. The 0.0.28 Lua suites read their FC status from them. This is a hard cut: the arming flags (90), PID/rates/battery/LED profile (95-98), TV profile (118) and GPS fix type (119) sensors are removed, and FLIGHT_MODE (89) no longer carries the GPS-unavailable bit 15 (now NAV_BLOCKED in SYSTEM_STATUS). Jeti EX Bus is unchanged. Use the 0.0.28 Configurator and Lua suites with this firmware.
IMPORTANT: updating keeps your saved telemetry_sensors list, which does not include the new sensors 120/121 that the Lua suites now need. After flashing, run this in the CLI and save (or press Default on the Ethos suite's Telemetry page, or select the sensors in the Configurator):
set telemetry_sensors = 3,4,5,6,15,43,50,52,58,59,60,89,91,99,120,121
Change the defaults for new and reset configs: telemetry is enabled with custom CRSF telemetry (set crsf_telemetry_mode = NATIVE for native CRSF), telemetry_sensors is the list above, and smartfuel = CURRENT (falls back to the voltage estimate without a current sensor or pack capacity). Existing configs keep their values.
Fix the SmartFuel mode being cleared to OFF on save when no battery voltage source was configured yet.
Move S.Port DEBUG_7 from 0x52F8 to 0x52F7, so debug 0-7 use 0x52F0-0x52F7.

Change the MSP API to 22.5: receive 24 RC channels and drive 24 bus servos (S1-S32). Existing mixer numbers stay put: CH19-24 are mixer inputs 30-35 and bus servos 19-24 are outputs 31-36. Use the 0.0.28 Configurator and Lua suites with this firmware.
Add per-output bus channel counts: fbus_master_channels (8/12/16/24, default 24; 24 uses the 24-channel F.Bus frame) and sbus_out_channels (8/12/16, default 16), reported in MSP_MIXER_CONFIG. Channels past the count are sent at center. Fix the 16-channel F.Bus frame reading CH17-18 from past the end of its channel array.
Accept 8-, 16- and 24-channel F.Bus/F.Port2 frames on the backup receiver. On takeover, channels the backup frame doesn't carry go to stick center instead of minimum.
Change the default bus servo scale to 500, so full mixer output gives full travel instead of reaching it at half stick. Saved configs keep their scale.
Fix bus servos with a speed limit moving far slower than configured (about 20x at 50 Hz SBUS), and give SBUS and F.Bus output separate speed-limit state.
Read the satellite count from FBUS GPS sensors that send it. Older FBUS GPS sensors report 0 satellites so they no longer pass the RTH/Loiter/GPS Rescue minimum-satellite checks; the CLI-only gps_fbus_assumed_sats sets an assumed count for them.
Stop compiling in the parallel PWM receiver. Configs with it enabled have it cleared at boot.

# 0.0.27

Split the shared roll/pitch stick deadband into separate roll and pitch deadbands (CLI roll_deadband, pitch_deadband, yaw_deadband). This is a breaking MSP_RC_CONFIG/MSP_SET_RC_CONFIG layout change with no compatibility path, and the MSP API version stays 22.4, so an older client still connects but reads and writes the wrong deadband fields: use the 0.0.27 Configurator and Lua suites with this firmware. Updating resets the RC controls settings (stick center, deflection, throttle range, deadbands, smoothing) to defaults, so re-check them after flashing. The blackbox header's deadband is replaced by roll_deadband and pitch_deadband.
Report GPS Loiter/RTH as unavailable in flight-mode telemetry when its switch is on but the mode can't fly (disarmed, no accelerometer, no healthy fix, or no home for RTH), so the radio can announce it. Flight behaviour is unchanged.
Stop compiling in the GHOST and CPPM receivers and the FrSky Hub, MAVLink and LTM telemetry protocols. Feature and serial-function IDs are unchanged.
Add a simulated barometer and blackbox logging to onboard flash in SITL, and fix the SITL build.

# 0.0.26

Rework GPS Loiter/RTH after a flight log showed oversized full-bank orbits and a steady descent. Fix the altitude gain being applied 10x too weak: nav_altitude_kp now gives the documented degrees of pitch per meter (divide it by 10 if you raised it to compensate). Add climb-rate damping (nav_altitude_kd), steer smoothly onto the loiter circle with a speed-based bank feedforward and a 45°/s bank slew limit, and feed rudder into nav turns instead of holding it against them (nav_turn_coordination). Default nav_loiter_radius is now 100 m; existing configs keep their value.
Add nav_throttle (default 60%): GPS Loiter, GPS RTH and the failsafe GPS Rescue fly at it once airborne, instead of wherever the throttle stick was. failsafe_throttle now applies only to failsafe landing. Block arming while the GPS Loiter switch is on. The new settings are appended to MSP2_WING_GPS_NAV_CONFIG; edit them with the 0.0.26 Configurator or Lua suite.
Log the GPS navigation settings in the blackbox header.

Change the MSP API to 22.4: add independent bank (10-90°) and pitch (10-75°) limits for ANGLE and TRAINER. Use the 0.0.26 Configurator and Lua suites with this firmware; older clients keep working against the shared-limit fields.
Re-enable failsafe stage 2 for fixed-wing, which had been dead code since this fork's rewrite: signal loss now actually drives AUTO-LAND/DROP or GPS Rescue instead of falling through to per-channel RX fallback only. GPS Rescue and the GPS RTH switch both fly home and orbit via the fixed-wing RTH controller (not the old multirotor hover-descent algorithm), then hand off to a motor-off self-level glide-down. The separate GPS RESCUE switch is retired as a redundant alias for GPS RTH; anyone using it should rewire to GPS RTH.
Fix an inverted pitch sign in GPS RTH/Loiter altitude hold that commanded away from the target altitude instead of toward it, and fix GPS RTH/Loiter navigating on a stale GPS fix and not resuming after the fix recovers.
Fix the loiter direction being inverted; anyone who set the opposite value to get the direction they wanted needs to swap it back.
Add a GPS Navigation tab's worth of MSP support (nav_rth_altitude, nav_loiter_radius/direction, nav_max_bank/pitch_angle, nav_min_sats, nav_bearing/altitude_kp) and a failsafe_recovery_delay field on MSP_FAILSAFE_CONFIG, both previously CLI-only.
Add a GPS fix-type telemetry sensor over CRSF and S.Port/FPort/FPort2, and clear the GPS fix state when CRSF- or MSP-fed GPS telemetry goes stale instead of latching the last-known fix forever.
Block arming when GPS RTH is wired only to the GPS RTH switch and has no GPS fix, closing a gap where such a setup got no prearm protection at all.
Change airborne detection to require sustained roll/pitch stick and gyro response (or altitude gain), so ANGLE/HORIZON/ATT HOLD/TV hold/AUTO HOVER keep full authority through hands-off flight and motor-off glides instead of dropping to ground-level authority after about a second of level cruising; authority now stays latched until disarm.
Fix servo speed limiting coupling independent wing surfaces together (e.g. a slow aileron also slowing the elevator) when servo_speed is set to a nonzero value.
Apply cross-axis relax to the I-term accumulation only, not its output, removing a jump/drop in I contribution when rudder is applied and released. Default strength is 0, so default configurations are unaffected.
Rename the Governor Headspeed adjustment function to Governor RPM.
Reduce SmartPort and CRSF telemetry memory usage.
Guard the mixer and servo output paths against NaN/Inf reaching the hardware, including flooring rc.range to a minimum of 1 so an unchecked MSP_SET_RC_CONFIG deadband/deflection can no longer divide by zero.

# 0.0.25

Change the MSP API to 22.3, stripping the always-zero heli placeholder bytes from eight MSP messages (MSP_RC_TUNING, MSP_PID_PROFILE, MSP_PID_TUNING, MSP_SETPOINT, MSP_TELEMETRY_CONFIG, MSP_ESC_SENSOR_CONFIG, MSP_RC_CONFIG, MSP_SENSOR_CONFIG). This is a breaking layout change: use the 0.0.25 Configurator and Lua suites with this firmware.
Add CRSF Sensors support: decode third-party CRSF GPS, battery, barometer, cells and RPM telemetry, with a battery source select and a diagnostic MSP status command.
Add cell count and cell voltages to battery profiles, so a model can switch between e.g. 3S and 4S or LiPo and LiHV packs.
Make the flap compensation and diff thrust yaw adjustments signed (-1000..1000), so compensation can be driven negative without clobbering a rule's Reverse.
Rework SmartFuel sag compensation to follow motor throttle output instead of roll/pitch stick load, and give motorless models none. smartfuel_sag_gain is now volts per cell (x100) at full throttle.
Gate the AUTOHOVER throttle assist on the throttle stick and RX signal, so it no longer spins the motor at idle or keeps assisting after link loss.
Remove the AUTOHOVER roll hold, so roll is a free stick pass-through again on entry and in the hover (autohover_roll_deadband is kept for compatibility but no longer does anything), and lower the default autohover max_rate from 300 to 120 deg/s so engaging in forward flight makes a wider turn.
Bleed wound-up I-term quickly after an ATT HOLD/Thrust Vector hold stall re-capture, instead of taking ~15 s to re-center the surface.
Log the hold stall state in the ATTHOLD and TVHOLD blackbox debug modes.
Fix SITL providing only 4 of 8 servo outputs (an existing SITL setup needs a defaults reset to see the extra channels).
Use integer maths for ADC current capacity accumulation, saving about 2.6 KB of flash on STM32F7X2.
Remove the VTX, camera control and rangefinder code, none of which was built into any target. MSP_SONAR_ALTITUDE is now unsupported; wire IDs stay reserved.
Move the remaining user and developer documentation to wingflight-docs, leaving only design notes in the firmware repo.

# 0.0.24

Make continuous (pot/channel-mapped) SERVO_TRIM_* adjustments runtime-only: they no longer rewrite and save the servo center, so a trim can't re-apply itself on top of its own saved result after a reboot or leave a wrong center behind.
Add MSP_SERVO_TRIM (command 233) to report the live runtime servo trim per servo. The MSP API version stays at 22.2.
Lengthen the servo trim link-settle delay from 300ms to 1000ms, fixing trim channels that come online later than the RX link driving a servo center far off position after boot.
Add a channel deadband to continuous adjustments so a noisy pot no longer flips the value by 1 every tick, causing repeated config writes, beeps, blackbox events and wandering PID values.
Make ATT HOLD fly like normal mode until it holds: the hold target is captured once the axis stops rotating instead of the instant the stick centers, so releasing mid-rotation no longer snaps the aircraft back.
Let ATT HOLD re-center when nothing is happening: I-term bleeds slowly on a holding axis, and a frozen axis pinned against something it can't move (e.g. tilted on the bench) re-captures its target after a stall timeout.
Apply the same settle-then-capture and per-axis I-term decay to AUTOHOVER roll, fixing its release snap-back.
Run the Thrust Vector hold on the same shared hold engine as ATT HOLD, picking up all of its fixes (per-axis tracking, reduced ground authority, clamped deadband, settle-then-capture, stall timeout), and drop its engaged state while a safety mode has priority.

# 0.0.23

Add optional throttle assist to AUTOHOVER for underthrottled hover, ramping throttle when pitch correction stays pinned at max rate.
Add per-servo balance curve for matching multi-servo control surfaces (e.g. dual ailerons).
Add SITL (Software-In-The-Loop) simulator target for testing firmware behavior without hardware.
Give ATT HOLD/AUTOHOVER real pre-airborne correction authority instead of none, and stop I-term decay eroding a sustained hold once rate error settles.
Fix ATT HOLD holding all axes as a single gated group instead of tracking/freezing each axis independently.
Fix AUTOHOVER/ATTHOLD retaining a stale hold target after a safety mode (GPS rescue/failsafe/RTH/loiter/angle) preempts them, unclamped MSP deadbands freezing hold correction, and a throttle assist trigger that could fire with max_rate at 0.
Fix blackbox flightModeFlags truncating box IDs past bit 31, so modes like AUTOHOVER never showed as engaged in logs.
Fix MANUAL mode's throw being scaled against the rate profile's maximum ceiling instead of the tune's actual feedforward.
Fix blackbox mass-storage log filenames still using the legacy "rtfl" prefix.
Remove legacy Matek/Nucleo board targets (superseded by unified STM32 targets).

# 0.0.22

Add a descriptive role tag to mixer rules, with per-role adjustment functions (ADJUSTMENT_FLAP_COMPENSATION_GAIN, ADJUSTMENT_DIFF_THRUST_YAW_GAIN) and fixes for role-adjusted rules losing their Reverse flag.
Add TRADITIONAL flight mode: stabilized with I-term forced to zero.
Block first arm until a configured backup RX has linked.
Hold Auto Hover's roll axis against disturbance drift (e.g. torque roll) once the stick centers, instead of leaving it a bare pass-through the whole time.
Change default servo refresh rate to a safe 50Hz.

# 0.0.21

Tag CLI errors raised while replaying a target's embedded custom-defaults blob (e.g. on `defaults`/`defaults nosave`) with "(custom defaults)", so they're no longer indistinguishable from an error in whatever command the user actually typed.

# 0.0.20

Fix MANUAL mode losing its rates/expo shaping on fast stick moves and ignoring the configured rate profile's authority, both of which pushed it toward full-deflection passthrough.

# 0.0.19

Add RX and ESC telemetry serial wiring auto-detect trial modes to help diagnose signal-inversion and pin-swap mismatches.
Fix RX/TX invert and pin-swap settings sticking across UART reopens.
Fix Spektrum satellite bind pin selection to honor serial pin-swap.

# 0.0.18

Add Thrust Vector profiles (6 independently-switchable profiles).
Generalize the backup RX from SBUS-only into a provider-selectable input, adding FBUS, FPort, FPort2, Jeti EX Bus, and CRSF support alongside SBUS.

# 0.0.17

Add Thrust Vector Attitude Hold.
Add SBUS-In Fallback Receiver (Serial Rx (Backup, SBUS)).
Fix servo-center snap when continuously slewing a SERVO_TRIM_* adjustment.
Add bus servo PWM-clone mode (`bus_servo_clone_pwm`, default ON) so bus servo channels mirror the matching PWM output without separate mixer rules.
Fix an init glitch on the Spektrum satellite receiver bind pin.

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
