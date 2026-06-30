# AGENTS Guide: Wingflight Firmware

## Project Identity

Wingflight is a fixed-wing fork of Rotorflight (itself based on Betaflight 4.3) and is intended for winged aircraft only.

Practical positioning for contributors and agents:

- Wingflight is a stabilizer-focused firmware for line-of-sight wing flying.
- Primary airframes include 3D airplanes, jets, gliders, and other fixed-wing models.
- The codebase intentionally shifts away from helicopter-specific assumptions.

Reference: [README.md](README.md)

## What Changed vs Rotorflight (Code-Backed)

This section captures behavior already present in this repository, not roadmap items.

### 1) Fixed-Wing Build Identity Is Explicit

- The macro `WINGFLIGHT_AIRPLANE` is enabled in common target configuration.
- Runtime docs in code state this macro is the fork marker for fixed-wing paths.

References:

- [src/main/target/common_pre.h](src/main/target/common_pre.h)
- [src/main/fc/runtime_config.h](src/main/fc/runtime_config.h)

### 2) Mixer Defaults Are Aircraft-Oriented

Default mixer rules are a standard fixed-wing layout:

- S1: left aileron from stabilized roll
- S2: right aileron from stabilized roll (opposite sign)
- S3: elevator from stabilized pitch
- S4: rudder from stabilized yaw
- M1: throttle from RC throttle command

Also:

- Stabilized collective input is kept at zero-rate by default (unused on wing aircraft).
- Curve slots default to neutral passthrough lines.

Reference: [src/main/pg/mixer.c](src/main/pg/mixer.c)

### 3) Passthrough Flight Mode Exists for Wing Workflows

- `BOXPASSTHROUGH` is an exposed mode.
- When active, stabilized roll/pitch/yaw mixer inputs are replaced by raw RC channel inputs.
- Throttle path remains direct.

References:

- [src/main/fc/rc_modes.h](src/main/fc/rc_modes.h)
- [src/main/msp/msp_box.c](src/main/msp/msp_box.c)
- [src/main/flight/mixer.c](src/main/flight/mixer.c)

### 4) New Attitude-Hold Mode (ATTHOLD)

- `BOXATTHOLD` is available and mapped to flight mode flags.
- ATTHOLD logic tracks current attitude per axis when pilot input exceeds deadband, then applies correction when sticks return near center.
- Mode behavior uses airborne state to avoid applying hold logic before flight.

References:

- [src/main/fc/rc_modes.h](src/main/fc/rc_modes.h)
- [src/main/fc/runtime_config.h](src/main/fc/runtime_config.h)
- [src/main/flight/atthold.c](src/main/flight/atthold.c)
- [src/main/flight/pid.c](src/main/flight/pid.c)
- [src/main/pg/pid.h](src/main/pg/pid.h)
- [src/main/pg/pid.c](src/main/pg/pid.c)

### 5) PID Path Reflects Fixed-Wing Dynamics

- PID mode `1` is implemented as fixed-wing rate PID.
- A throttle-based gain attenuation path (`fw_tpa_*`) is used for control-surface authority management.
- I-term decay naming and behavior have been adjusted for fixed-wing semantics (`iterm_decay_*`, no cyclic naming).

References:

- [src/main/flight/pid.c](src/main/flight/pid.c)
- [src/main/pg/pid.h](src/main/pg/pid.h)
- [Changes.md](Changes.md)

### 6) Heli-Specific Slots Are Pruned but ID Compatibility Is Preserved

- Some helicopter-specific boxes/flags remain as reserved placeholders to avoid ID renumbering and protocol breakage.

References:

- [src/main/fc/rc_modes.h](src/main/fc/rc_modes.h)
- [src/main/fc/runtime_config.h](src/main/fc/runtime_config.h)

## Guidance for Future Agents

When implementing features or reviewing PRs, assume fixed-wing-first behavior unless explicitly documented otherwise.

### Keep Stable Interfaces Stable

- Do not renumber BOX permanent IDs unless an explicit migration plan exists.
- Preserve MSP and CLI backward compatibility where possible.

### Mixer and Mode Work Is Safety-Critical

- Treat changes in [src/main/flight/mixer.c](src/main/flight/mixer.c), [src/main/pg/mixer.c](src/main/pg/mixer.c), and [src/main/flight/pid.c](src/main/flight/pid.c) as high risk.
- Validate arming interactions and override behavior in [src/main/fc/core.c](src/main/fc/core.c).

### Document Behavior, Not Only Intent

- For new wing-specific behavior, add concise notes to release docs and point to exact source files.
- If a setting is renamed, keep aliases where practical and record it in [Changes.md](Changes.md).