# SITL → JSBSim → FlightGear Integration Plan

Status: **Phase 0 and Phase 1 implemented and validated** (native MinGW-w64 toolchain
vendored under `tools/mingw64`, `pwmOutConfig`/servo link gap fixed, `make TARGET=SITL`
builds and the resulting binary runs). Phases 2-4 (JSBSim bridge, FlightGear, docs) are
still unimplemented — see §5 for per-phase checklists.

## 1. Executive Summary

Wingflight inherits a Betaflight/Rotorflight-era `SITL` target that talks to **Gazebo 8**
over a small custom UDP protocol (`fdm_packet` in / `servo_packet` out). There is no
JSBSim or FlightGear integration today, and the native-Windows build of `SITL` is not
currently verified working on this machine (no C toolchain capable of building it is
installed, and a real code gap was found — see §2.3 — that likely breaks the link step
for a fixed-wing (servo-enabled) build).

Goal: get `TARGET=SITL` building and running natively on Windows with a toolchain kept
under `tools/` (same philosophy as `make arm_sdk_install`), then drive it from **JSBSim**
(flight dynamics) and visualize with **FlightGear**, on Windows, with wing control
surfaces (S1–S4) and throttle (M1) actually connected end-to-end.

## 2. Current State (as found in this repo, 2026-08-04)

### 2.1 What exists

- `src/main/target/SITL/` — the SITL target:
  - [target.c](../../src/main/target/SITL/target.c): fakes IMU (`fakeAccSet`/`fakeGyroSet`),
    runs a UDP thread receiving `fdm_packet` (timestamp, angular velocity, linear
    acceleration, orientation quaternion, velocity, position) and a TCP (dyad-based) MSP
    server. Motor output is fed back via `servo_packet` (`float motor_speed[4]`).
  - [udplink.c/.h](../../src/main/target/SITL/udplink.h): thin cross-platform UDP socket
    wrapper (already has a Winsock2 branch for native Windows builds).
  - [target.mk](../../src/main/target/SITL/target.mk), [pg.ld](../../src/main/target/SITL/pg.ld),
    [README.md](../../src/main/target/SITL/README.md) — the README documents the
    **Gazebo 8 + ArduCopterPlugin** workflow only (Ubuntu 16.04), ports
    `udp://127.0.0.1:9002` (out) / `9003` (in).
  - [make/mcu/SITL.mk](../../make/mcu/SITL.mk): already has real native-Windows support
    (MinGW `-mno-ms-bitfields`, `-lws2_32`, no `-lc -lrt` on Windows) — this was clearly
    added specifically for Wingflight, not inherited as-is.
- [scripts/sitl-rc-check.ps1](../../scripts/sitl-rc-check.ps1): a substantial PowerShell
  MSP smoke/sweep/stress test script that can build, launch, and RC-inject into
  `wingflight_SITL.exe` over MSP TCP. This is Windows-first and already assumes a
  `wingflight_SITL.exe`/`.elf` binary naming convention.
- **No** references to JSBSim or FlightGear anywhere in the repo (`grep` for
  `JSBSim|FlightGear` returned nothing). **No** prior planning doc existed.

### 2.2 Toolchain status (RESOLVED — Phase 0 done)

- `tools/gcc-arm-none-eabi-9-2020-q2-update/` is the **ARM** cross-compiler used for
  real flight-controller hardware targets; irrelevant to `TARGET=SITL`.
- A `mingw_sdk_install` target was added to [make/tools.mk](../../make/tools.mk),
  mirroring `arm_sdk_install` exactly (marker file + conditional download/extract).
  It vendors a standalone [WinLibs](https://winlibs.com/) MinGW-w64 build into
  `tools/mingw64/`, no admin rights or PATH/registry changes required.
- `make/mcu/SITL.mk` now resolves `ARM_SDK_PREFIX` to `tools/mingw64/bin/` on Windows
  when present, falling back to a system `gcc` on `PATH` otherwise (or on Linux/macOS).
- **Pinned version note**: the WinLibs "latest" release at the time of writing (GCC
  16.1.0 + mingw-w64 14.0.0, UCRT, release 4) has a winpthreads header bug — duplicate/
  incompatible `struct _timespec64` declarations between `<time.h>` and `<pthread.h>` —
  that breaks the build under `-Werror`. `MINGW_SDK_URL` is pinned instead to a mature,
  known-good combo: **GCC 13.3.0 (POSIX threads) + mingw-w64 11.0.1 (UCRT), release 1**.
  Do not bump this to a "latest" WinLibs URL without first confirming the new
  GCC/mingw-w64 pairing builds cleanly.
- Verified end-to-end: `make mingw_sdk_install` then `make TARGET=SITL` produces
  `obj/main/wingflight_SITL.elf` (a native PE executable despite the `.elf` extension),
  which runs its main loop without crashing.
- **Runtime DLL dependency fix**: the default (dynamic) link against winpthreads made
  the binary fail to start (`STATUS_DLL_NOT_FOUND`, exit code `0xC0000135`) unless
  `tools/mingw64/bin` happened to be on `PATH` at run time (e.g. running the built exe
  from a different working directory, or via `scripts/sitl-rc-check.ps1`, which doesn't
  add it to `PATH`). Fixed by adding `-static` to the Windows `LD_FLAGS` in
  [make/mcu/SITL.mk](../../make/mcu/SITL.mk), so `wingflight_SITL.elf` is fully
  self-contained and runs from any directory without needing the toolchain on `PATH`.

### 2.3 The pwmOutConfig/servo link gap (RESOLVED — Phase 1 done)

- Confirmed (once a toolchain existed): `flight/servos.c`'s `servoInit()` calls
  `pwmOutConfig()` (line ~288), which only exists in `drivers/pwm_output.c` — a file
  `make/mcu/SITL.mk` excludes from the SITL build. `servoInit()` is called
  unconditionally under `#ifdef USE_SERVOS` from `fc/init.c`, which is globally
  defined in `common_pre.h`, so this was a genuine link-time failure
  (`undefined reference to 'pwmOutConfig'`), not a runtime-only issue.
- **Fix implemented** in [src/main/target/SITL/target.c](../../src/main/target/SITL/target.c):
  - A SITL-only `pwmOutConfig()` stub satisfies the link requirement. It sets
    `channel->ccr = NULL` (and `tim = NULL`), which is safe because `flight/servos.c`
    only writes through `ccr` when it is non-NULL.
  - Servo positions are instead read back via the existing public API
    `getServoCount()`/`getServoOutput()` (declared in `flight/servos.h`), independent of
    the `ccr` mechanism.
  - `pwmCompleteMotorUpdate()` now also populates a new `servo[8]` array (S1-S8, in
    microseconds) in the outgoing `servo_packet`, alongside the existing
    `motor_speed[4]`.
  - `servo_packet` (in [target.h](../../src/main/target/SITL/target.h)) was extended
    with `float servo[8];` to carry these values over the UDP link. This changes the
    packet size/shape relative to the legacy Gazebo ArduCopterPlugin format (see §2.4).
- Wingflight's fixed-wing SITL build now links and runs; S1-S4 control-surface values
  are available on the simulator UDP link for Phase 2's JSBSim bridge to consume.

### 2.4 Protocol limitations relative to fixed-wing / JSBSim / FlightGear

- `servo_packet` only carries `motor_speed[4]` — no servo/control-surface channels.
- `fdm_packet`/`servo_packet` are Gazebo-ArduCopterPlugin-shaped, not JSBSim- or
  FlightGear-native-FDM-shaped. JSBSim and FlightGear have their own well-known wire
  formats (JSBSim `<output>`/`<input>` UDP directives; FlightGear's `native-fdm`/generic
  protocol), so a bridge or protocol extension is needed either way.

## 3. Goals / Non-Goals

**Goals**

1. `make TARGET=SITL` builds and runs a Wingflight fixed-wing SITL executable natively
   on Windows, with a toolchain that lives under `tools/` and installs with one command
   (like `make arm_sdk_install`).
2. JSBSim drives the flight dynamics (physics), fed by Wingflight's mixer outputs
   (S1–S4 servos + M1 motor), and returns IMU/attitude/position state to Wingflight.
3. FlightGear is usable as a visualization front-end for the same JSBSim run.
4. Documented, reproducible setup: a developer can go from a clean clone to a flying
   SITL session using only documented commands/manual installer steps.

**Non-goals (for this plan)**

- Fully automating/silently-installing JSBSim or FlightGear. **Per user decision, these
  two remain manual installs using their official Windows installer `.exe` files** —
  only the MinGW-w64 native compiler toolchain (which ships as a clean portable zip) is
  vendored/automated like the ARM SDK.
- Reworking the Gazebo workflow described in the existing SITL README — it can stay
  documented as a legacy/alternative path, not removed.
- Any change to real-hardware targets' PWM/servo code paths.

## 4. Proposed Toolchain Setup (Phase 0)

| Component | How it's obtained | Where it lives | Automation |
|---|---|---|---|
| Native C/C++ compiler (MinGW-w64 GCC, for `TARGET=SITL`) | [WinLibs](https://winlibs.com/) standalone GCC+MinGW-w64 zip (no installer, no admin, disk-location independent) | `tools/mingw64/` | New `make mingw_sdk_install` target in `make/tools.mk`, mirroring `arm_sdk_install` (download zip to `downloads/`, unzip to `tools/`, no PATH/registry changes; `make/mcu/SITL.mk`/root `Makefile` picks it up when `TARGET=SITL` similarly to how `ARM_SDK_PREFIX` is resolved) |
| JSBSim (flight dynamics engine) | Official Windows installer `JSBSim-<ver>-setup.exe` from the [JSBSim releases page](https://github.com/JSBSim-Team/jsbsim/releases) (no portable Windows zip is published) | Wherever the user installs it (default `%LOCALAPPDATA%`) | **Manual**, per user decision. Plan documents the required version and how to point our scripts at the install path. |
| FlightGear (visualization) | Official Windows installer `flightgear-<ver>-windows-amd64.exe` from [flightgear.org/download](https://www.flightgear.org/download/) (no portable Windows build is published) | Default FlightGear install location | **Manual**, per user decision. Plan documents launch flags/protocol config needed. |

Rationale for MinGW-w64 being the one piece we vendor: it's the only one of the three
that ships as a genuine no-installer/no-admin zip, so it fits the existing
`arm_sdk_install` pattern cleanly. JSBSim and FlightGear only ship Windows installer
`.exe`s upstream — forcing those into a portable layout would mean silently mucking
with NSIS installer internals, which is fragile and explicitly out of scope per the
user's decision above.

## 5. Phased Plan

### Phase 0 — Local native toolchain (blocking everything else) — DONE
- [x] Add `mingw_sdk_install` / `mingw_sdk_download` / `mingw_sdk_clean` targets to
  `make/tools.mk`, following the exact shape of the existing `arm_sdk_*` targets.
- [x] Wire `TARGET=SITL` native builds to prefer `tools/mingw64/bin` when present
  (similar to how `ARM_SDK_DIR`/`ARM_SDK_PREFIX` resolution works today), without
  breaking users who already have a system MinGW/MSYS2 toolchain on `PATH`.
- [x] Run `make TARGET=SITL` from a clean state and record whether it builds —
  confirmed the §2.3 hypothesis; see §2.3 for the fix.

### Phase 1 — Fix native SITL for fixed-wing (Wingflight-specific) — DONE
- [x] Fix the `pwmOutConfig`/servo link gap (§2.3): a SITL-only stub satisfies the
  linker; actual servo values are read back via `getServoCount()`/`getServoOutput()`
  instead of the (unused, NULL) `ccr` register mechanism.
- [x] Extend the simulator wire protocol so the S1–S4 servo outputs (not just M1
  motor) reach whatever is on the other end of the UDP link (`servo_packet.servo[8]`).
- [ ] Re-validate with [scripts/sitl-rc-check.ps1](../../scripts/sitl-rc-check.ps1)
  (smoke/sweep) now that the new toolchain + fix are in place \u2014 **partially done**: the
  script now connects, builds/runs SITL, and gets a valid `MSP_API_VERSION`/feature
  mask response (confirms the toolchain + link fix work end-to-end). However
  `rcInjectOk`/`controlResponsive` come back `false` \u2014 `MSP_SET_RAW_RC` doesn't appear
  to get ack'd on a fresh default config. Not yet root-caused; may be a pre-existing
  RX/MSP config quirk unrelated to the servo fix (untouched by this session's changes).
  Needs follow-up before relying on RC-injection results.

### Phase 2 — JSBSim as the FDM
- [ ] Choose an existing JSBSim fixed-wing aircraft model (or adapt one) whose control
  surfaces map onto Wingflight's default mixer (S1 left aileron, S2 right aileron
  (opposite sign), S3 elevator, S4 rudder, M1 throttle — see
  [AGENTS.md](../../AGENTS.md)).
- [ ] Build a small bridge (a Python script using the official `jsbsim` PyPI package,
  `pip install jsbsim`, run inside a local venv under `tools/`) that:
  - receives Wingflight's actuator UDP packets and applies them as JSBSim FCS inputs
    (`fcs/aileron-cmd-norm`, `fcs/elevator-cmd-norm`, `fcs/rudder-cmd-norm`,
    `fcs/throttle-cmd-norm`),
  - steps the JSBSim simulation, and
  - sends state back to Wingflight in the `fdm_packet` shape (or an extended version
    of it) at a sufficient rate.
- [ ] Decide packet-format question: extend the existing Gazebo-shaped `fdm_packet`/
  `servo_packet` (least invasive, keeps `target.c` mostly unchanged) vs. adopting a new
  JSBSim-native packet. **Recommendation: extend the existing structs** — add a servo
  channel array to the actuator-out packet and keep `fdm_packet` as-is (it already has
  everything JSBSim can trivially provide: angular velocity, linear acceleration,
  orientation quaternion, velocity, position).

### Phase 3 — FlightGear visualization
- [ ] Document the manual FlightGear install (link + version).
- [ ] Feed FlightGear from the same JSBSim instance driving the bridge, using either:
  - JSBSim's own `<output>` UDP directive in FlightGear-native-FDM format (JSBSim can
    emit multiple `<output>` protocols simultaneously — one to our bridge, one
    FlightGear-native to FlightGear directly), launching FlightGear with
    `--fdm=null --native-fdm=socket,in,<rate>,,<port>,udp` (visualization-only, no
    physics computed by FlightGear), or
  - Have the Python bridge re-broadcast position/orientation to FlightGear using its
    generic/native-fdm protocol.
  - **Note**: JSBSim's changelog shows the FlightGear network protocol version has
    changed and been reverted before (`v24`) — pin exact JSBSim/FlightGear versions
    together and verify compatibility before relying on this path.
- [ ] Provide a documented one-shot launch sequence: Wingflight SITL → JSBSim bridge →
  FlightGear, plus a PowerShell helper script analogous to `sitl-rc-check.ps1`.

### Phase 4 — Validation & docs
- [ ] Extend or add an RC-injection script (building on `sitl-rc-check.ps1`'s
  patterns) that drives RC through MSP and confirms aileron/elevator/rudder/throttle
  move as expected end-to-end through JSBSim.
- [ ] Update [src/main/target/SITL/README.md](../../src/main/target/SITL/README.md)
  with the new JSBSim/FlightGear workflow (keep the Gazebo section as a legacy
  alternative).
- [ ] Record the final, working versions of JSBSim/FlightGear/MinGW-w64 used, since
  none of these are pinned by a lockfile today.

## 6. Open Questions / Risks

- **§2.3 is a hypothesis, not yet confirmed** — must build first (Phase 0) to verify.
- FlightGear/JSBSim wire-protocol compatibility has shifted across versions upstream;
  needs pinning and a smoke test, not just documentation.
- No unit tests will be added for any of this (per standing project preference).
- This plan does not yet address realtime-pacing/timing behavior under JSBSim (JSBSim
  can run faster/slower than real time; Wingflight's SITL loop currently paces itself
  off packet timestamps from the Gazebo side — this logic needs review once JSBSim is
  the source).

## 7. Immediate Next Steps

1. ~~Implement Phase 0 (`mingw_sdk_install` target) and get a clean `make TARGET=SITL`
   build.~~ Done.
2. ~~Confirm/fix the `pwmOutConfig` servo gap (Phase 1).~~ Done.
3. Run [scripts/sitl-rc-check.ps1](../../scripts/sitl-rc-check.ps1) against the new
   build to confirm MSP/RC injection still works end-to-end with the new toolchain and
   servo-output changes.
4. Only then start the JSBSim bridge (Phase 2) — building it against a SITL binary
   that can't yet move a rudder would just hide the real problem.
