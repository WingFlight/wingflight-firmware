# AGENTS Guide: Wingflight Firmware

## Project Identity

Wingflight is a fixed-wing fork of Rotorflight (itself based on Betaflight 4.3) and is intended for winged aircraft only.

Practical positioning for contributors and agents:

- Wingflight is a stabilizer-focused firmware for line-of-sight wing flying.
- Primary airframes include 3D airplanes, jets, gliders, and other fixed-wing models.
- The codebase intentionally shifts away from helicopter-specific assumptions.

Reference: [README.md](README.md)

## Project Components

Wingflight is split across several repositories under <https://github.com/WingFlight>:

| Repository | Purpose |
| --- | --- |
| [wingflight-firmware](https://github.com/WingFlight/wingflight-firmware) | Flight controller firmware (this repository) |
| [wingflight-targets](https://github.com/WingFlight/wingflight-targets) | Board configurations (custom defaults) applied on top of the unified targets |
| [wingflight-configurator](https://github.com/WingFlight/wingflight-configurator) | Desktop/web app for flashing and configuring the FC over MSP |
| [wingflight-blackbox](https://github.com/WingFlight/wingflight-blackbox) | Blackbox Explorer for analysing flight logs |
| [wingflight-lua-edgetx](https://github.com/WingFlight/wingflight-lua-edgetx) | Transmitter Lua scripts for EdgeTX |
| [wingflight-lua-edgetx-updater](https://github.com/WingFlight/wingflight-lua-edgetx-updater) | Updater for the EdgeTX Lua scripts |
| [wingflight-lua-ethos-suite](https://github.com/WingFlight/wingflight-lua-ethos-suite) | Lua suite for FrSky Ethos |
| [wingflight-lua-ethos-suite-updater](https://github.com/WingFlight/wingflight-lua-ethos-suite-updater) | Updater for the Ethos Lua suite |
| [wingflight-msp-ble-bridge](https://github.com/WingFlight/wingflight-msp-ble-bridge) | ESP32-C3 Bluetooth LE bridge for configuring the FC over MSP |
| [wingflight-artifacts](https://github.com/WingFlight/wingflight-artifacts) | Firmware builds mirrored for browser downloads by the web configurator |
| [wingflight-docs](https://github.com/WingFlight/wingflight-docs) | Documentation website (<https://doc.wingflight.org>) |
| [wingflight-driver-fixer](https://github.com/WingFlight/wingflight-driver-fixer) | Windows tool that installs the USB bootloader/DFU driver for WebUSB flashing |
| [rotorflight-to-wingflight](https://github.com/WingFlight/rotorflight-to-wingflight) | Windows app that converts a Rotorflight flight controller to Wingflight |
| [wingflight-landing-page](https://github.com/WingFlight/wingflight-landing-page) | Project landing page |

The Configurator, the Lua scripts and the BLE bridge talk to the firmware over MSP, so MSP changes here affect all
of them.

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
- A throttle-based gain attenuation path (`fw_tpa_gain` / `fw_tpa_curve`) is used for control-surface authority management, mirroring `master_gain`/`gain_curve`: a baseline scale further shaped by an optional curve from the same shared gain-curve pool/evaluator.
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

### Never Bump the MSP API or Firmware Version

Agents must not change these. The maintainers bump them manually when a new version is released, which keeps version changes to a minimum:

- MSP API version: `API_VERSION_MAJOR` / `API_VERSION_MINOR` in [src/main/msp/msp_protocol.h](src/main/msp/msp_protocol.h).
- Firmware version: `FC_VERSION_MAJOR` / `FC_VERSION_MINOR` / `FC_VERSION_PATCH_LEVEL` in [src/main/build/version.h](src/main/build/version.h).

MSP changes made between releases belong to the upcoming API version. List them in the PR under **Compatibility** so they are covered by the release bump and the Configurator can pick them up.

Parameter groups are the exception: bump the PG version (last argument of `PG_REGISTER*`, e.g. `PG_REGISTER_WITH_RESET_TEMPLATE(pidConfig_t, pidConfig, PG_PID_CONFIG, 3)`) in the same change whenever the parameter group changes: fields added, removed, reordered or resized, array lengths, default values or value ranges changed. This makes stored settings from older builds reset to the current defaults instead of being misread or keeping stale defaults. The version is 4 bits (0–15).

### Mixer and Mode Work Is Safety-Critical

- Treat changes in [src/main/flight/mixer.c](src/main/flight/mixer.c), [src/main/pg/mixer.c](src/main/pg/mixer.c), and [src/main/flight/pid.c](src/main/flight/pid.c) as high risk.
- Validate arming interactions and override behavior in [src/main/fc/core.c](src/main/fc/core.c).

### Document Behavior, Not Only Intent

- For new wing-specific behavior, add concise notes to release docs and point to exact source files.
- If a setting is renamed, keep aliases where practical and record it in [Changes.md](Changes.md).

## Where Documentation Lives

- **User-facing documentation** (setup, Configurator tabs, flight modes, CLI reference) lives in the
  [wingflight-docs](https://github.com/WingFlight/wingflight-docs) repository, published at
  <https://doc.wingflight.org>. It is the source of truth. Do not add or update user documentation in this
  repository's `docs/`. Change wingflight-docs instead, and note in the PR when a behaviour change needs a
  matching docs page update.
- **Developer and internals documentation** (code guidelines, coding style, parameter groups, configuration and
  Blackbox formats, SmartFuel internals, hardware debugging) is in the Technical Reference under Contributing on
  the same site: <https://doc.wingflight.org/contributing/tech/>.
- **`docs/` in this repository** holds only a README that points at the docs site. Do not add documents there; add
  them to wingflight-docs, and link to them by URL from source comments.

## Build and Test

The ARM toolchain does not need to be installed by hand: `make arm_sdk_install` downloads the version the
Makefile expects into `tools/`.

Wingflight builds **unified targets**, one firmware per MCU family: `STM32F405`, `STM32F411`, `STM32F7X2`,
`STM32F745`, `STM32G47X`, `STM32H743` (see [make/targets_list.mk](make/targets_list.mk)). Board-specific pin
mappings and defaults are not in this repository; they live in
[wingflight-targets](https://github.com/WingFlight/wingflight-targets) and are applied as a config on top of
the unified firmware.

Prefer `STM32F7X2` for test builds.

```
make arm_sdk_install          # once: installs the ARM toolchain under tools/
make TARGET=STM32F7X2         # preferred test target; `make unified` builds all
make test                     # unit tests in src/test/unit
make TARGET=SITL              # native simulator build, see src/main/target/SITL/README.md
```

`make help` lists the options. Extra defines go in `OPTIONS="USE_SOMETHING"`. CI runs the GitHub Actions
workflows in `.github/workflows`.

Feature branch dev builds embed `FC_VER_SUFFIX` from the `feature/<name>` branch slug plus the short SHA.
`src/main/build/version.h` requires `FC_VERSION_STRING` to stay under 30 bytes, so long branch names can make
the workflow fail with `fc_version_string_too_long`. This is a real compile failure, not a superseded run; either
keep feature branch names short or update `.github/workflows/dev-build.yml` to truncate the generated suffix.

Unit tests exist for PID, setpoint, curves, maths and the acro trainer. The hold engine, ATTHOLD,
the thrust-vector loop, leveling, airborne detection, servos and GPS navigation have none, so changes there
need extra care and, where possible, a new test.

## Before Changing Flight-Control Code

Read [Flight Dynamics](https://doc.wingflight.org/contributing/tech/flight-dynamics/) in the Technical
Reference. It records the signal chain, the design rationale behind each stage, and a review of known defects. Two things to keep in mind:

- Sign conventions are easy to get wrong. Attitude pitch is **positive nose-down**, RC yaw is negated once in
  `setpoint.c`, and PID output is a unitless surface command where 1.0 is full travel. See section 1 of that
  document.
- Some behaviour that looks intentional is a known defect or a disabled path (for example, the
  flight-controller failsafe stage 2 is disabled, and the "airborne" state is read from stick and tilt only).
  Section 4 lists them. Check it before building on, or "fixing", the surrounding code.

## Issues and Pull Requests

Maintainers and users read these, so keep them short and precise. No filler, no restating the diff.

**Issues**

- Title: the symptom in one line.
- Body: firmware version and target, steps to reproduce, expected vs actual behaviour. Attach CLI `diff all` or a
  Blackbox log when relevant.

**Pull requests**

- Title: imperative, under ~70 characters (e.g. "Fix ATTHOLD drift after stick release").
- Body, a few lines each:
  - **What**: the change and why it is needed. Link the issue (`Fixes #123`).
  - **Compatibility**: MSP, CLI or setting changes, and whether the Configurator or wingflight-docs need updating.
  - **Testing**: what was built and run (unit tests, SITL, bench, flight).
- One topic per PR. Leave out file-by-file change lists and generated summaries.
