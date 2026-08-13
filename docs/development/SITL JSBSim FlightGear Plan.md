# SITL → JSBSim → FlightGear Integration Plan

Status: **Phase 0 through Phase 4 implemented and validated** (native MinGW-w64
toolchain vendored under `tools/mingw64`, `pwmOutConfig`/servo link gap fixed,
`make TARGET=SITL` builds and the resulting binary runs, `getServoCount()`/
RC-injection/servo output confirmed working end-to-end via `sitl-rc-check.ps1`, and
the JSBSim bridge (`scripts/jsbsim_bridge.py`) drives a live SITL instance end-to-end
via `MSP_ATTITUDE`). **Phase 3 (FlightGear) is implemented but not yet validated
against a real FlightGear install** (not present on this dev machine — JSBSim's
FlightGear-native UDP output was validated with a raw socket listener instead, see
§5 Phase 3). **Phase 4 (final docs/validation) is done**: `sitl-rc-check.ps1` gained
a `-Mode jsbsim` that proves the full RC → mixer → JSBSim → attitude loop end-to-end
(not just Wingflight's own servo output), the SITL README documents the new
JSBSim/FlightGear workflow alongside the legacy Gazebo path, and exact working
toolchain/simulator versions are recorded (see §9). FlightGear itself remains
unvalidated against a real install, as noted above.

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
- **Follow-up fix (confirmed working, 2026-08-11)**: linking and running was not
  sufficient on its own — `getServoCount()` was still always `0` at runtime because
  `servoConfig()`'s default `ioTags[]` (computed via `timerioTagGetByUsage()`) resolved
  to `IO_TAG_NONE` whenever `USABLE_TIMER_CHANNEL_COUNT == 0`, which SITL's `target.h`
  set by default (no real timer hardware). Fixed with a synthetic
  `const timerHardware_t timerHardware[4]` table in `target.c` (4 entries tagged
  `TIM_USE_SERVO`) plus `USABLE_TIMER_CHANNEL_COUNT 4` in `target.h`; SITL doesn't
  define `USE_TIMER_MGMT` (that's only set for real STM32F4/F7/H7/G4 targets in
  `common_pre.h`), so `drivers/timer_common.c` resolves `TIMER_HARDWARE`/
  `TIMER_CHANNEL_COUNT` straight to this array/count. Verified via
  `scripts/sitl-rc-check.ps1 -Mode smoke|sweep|stress`: `getServoCount()=4`,
  `controlResponsive: true`, sweep shows real roll/pitch/yaw servo deltas, stress ran
  259 cycles with 0 dropouts. (Gotcha for future re-verification: a stale `eeprom.bin`
  saved *before* this fix will keep reporting `servoCount=0` even after rebuilding,
  since the reset-fn that recomputes `ioTags` only runs on a genuine config reset —
  delete `eeprom.bin`/use `-FreshEeprom` to re-exercise it.)

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
- [x] Re-validate with [scripts/sitl-rc-check.ps1](../../scripts/sitl-rc-check.ps1)
  (smoke/sweep/stress) now that the new toolchain + fix are in place — **done**: the
  script connects, builds/runs SITL, and gets a valid `MSP_API_VERSION`/feature mask
  response. `rcInjectOk`/`controlResponsive` now come back `true` after fixing the
  underlying `getServoCount()==0` bug (see §2.3) and a stale-`eeprom.bin` false
  negative encountered during verification. Smoke/sweep/stress all pass with 0
  dropouts.
- [x] Fixed a `sitl-rc-check.ps1` bug found during this verification pass:
  `Build-Sitl`'s `cmd.exe /c "make ..."` call left its stdout unsuppressed, so it leaked
  into `Build-Sitl`'s (and transitively `Start-SitlIfNeeded`'s) return value —
  `$script:ResolvedPort` ended up as an array of build-log lines plus the port number
  instead of a plain int, which broke `TcpClient.ConnectAsync`'s overload resolution
  whenever `-BuildSitl` was combined with `-FreshEeprom`. Fixed by routing the build
  output through `Write-Host` instead of the success stream.

### Phase 2 — JSBSim as the FDM
- [x] Choose an existing JSBSim fixed-wing aircraft model (or adapt one) whose control
  surfaces map onto Wingflight's default mixer (S1 left aileron, S2 right aileron
  (opposite sign), S3 elevator, S4 rudder, M1 throttle — see
  [AGENTS.md](../../AGENTS.md)). **Done: `c172p`** (bundled with the `jsbsim` PyPI
  package), which exposes the standard `fcs/aileron-cmd-norm`, `fcs/elevator-cmd-norm`,
  `fcs/rudder-cmd-norm`, `fcs/throttle-cmd-norm` FCS inputs.
- [x] Build a small bridge (a Python script using the official `jsbsim` PyPI package,
  `pip install jsbsim`, run inside a local venv under `tools/`) that:
  - receives Wingflight's actuator UDP packets and applies them as JSBSim FCS inputs
    (`fcs/aileron-cmd-norm`, `fcs/elevator-cmd-norm`, `fcs/rudder-cmd-norm`,
    `fcs/throttle-cmd-norm`),
  - steps the JSBSim simulation, and
  - sends state back to Wingflight in the `fdm_packet` shape (or an extended version
    of it) at a sufficient rate.

  **Done: [scripts/jsbsim_bridge.py](../../scripts/jsbsim_bridge.py)**. Binds a UDP
  socket on `9002` to receive `servo_packet` (matching `target.c`'s `pwmLink`, which
  sends there) and sends `fdm_packet` to `9003` (matching `target.c`'s `stateLink`,
  which binds there). Runs JSBSim at a configurable fixed rate (default 120 Hz),
  paced to wall-clock time by default (`--no-realtime` to run flat-out). Maps
  S1/S2 (left/right aileron, averaged/opposite-signed) → `fcs/aileron-cmd-norm`,
  S3 → `fcs/elevator-cmd-norm`, S4 → `fcs/rudder-cmd-norm`, `motor_speed[0]` →
  `fcs/throttle-cmd-norm`. Computes the earth-to-body attitude quaternion from
  JSBSim's Euler angles (standard ZYX conversion) since `target.c`'s default path
  consumes a quaternion directly (`SET_IMU_FROM_EULER` not defined). Uses JSBSim's
  `accelerations/n-pilot-*-norm` (load factor, × 9.80665 → m/s²) for linear
  acceleration and `velocities/{p,q,r}-rad_sec` directly for angular velocity —
  both already in the standard aerospace body frame `target.c` expects (no extra
  sign-flipping needed on the bridge side; `target.c` applies its own sign
  convention when converting to fake accel/gyro readings).
  **Validated end-to-end**: launched a built `wingflight_SITL.elf` (existing
  `eeprom.bin`) alongside the bridge and polled `MSP_ATTITUDE` over the MSP TCP
  port — roll/pitch/yaw tracked JSBSim's evolving (untrimmed, idle-throttle glide)
  state smoothly over 10+ samples, confirming the full loop (JSBSim → bridge →
  UDP → `target.c` → IMU → MSP) works.
- [x] Decide packet-format question: extend the existing Gazebo-shaped `fdm_packet`/
  `servo_packet` (least invasive, keeps `target.c` mostly unchanged) vs. adopting a new
  JSBSim-native packet. **Recommendation: extend the existing structs** — add a servo
  channel array to the actuator-out packet and keep `fdm_packet` as-is (it already has
  everything JSBSim can trivially provide: angular velocity, linear acceleration,
  orientation quaternion, velocity, position). **Done** — this was already implemented
  in Phase 1 (`servo_packet.servo[8]`); no new packet format was needed for Phase 2.

### Phase 3 — FlightGear visualization
- [x] Document the manual FlightGear install (link + version). **Done** — see
  §8 below.
- [x] Feed FlightGear from the same JSBSim instance driving the bridge, using either:
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

  **Done, first option chosen**: [scripts/jsbsim_bridge.py](../../scripts/jsbsim_bridge.py)
  gained a `--flightgear`/`--fg-host`/`--fg-port`/`--fg-rate` option. When enabled, it
  writes a small JSBSim output-directives XML file (`<output type="FLIGHTGEAR"
  protocol="UDP" .../>`) and registers it via `FGFDMExec.set_output_directive()`
  **before** `load_model()` (required — JSBSim ignores it if called after). JSBSim
  itself then serializes its own native FDM UDP protocol every step, completely
  independent of the Wingflight `fdm_packet`/`servo_packet` link — no changes to the
  bridge's Wingflight-facing code were needed.
  **Validated without a FlightGear install**: bound a raw UDP listener on the
  configured port and ran the bridge with `--flightgear` — confirmed steady,
  correctly-rate-limited (30 Hz → 601 datagrams over ~20s) fixed-size (408-byte)
  binary frames, i.e. JSBSim is genuinely emitting FlightGear's native FDM struct.
  **Not yet validated against a real FlightGear process** (not installed on this
  dev machine — see §8 for install instructions and the exact JSBSim/FlightGear
  versions to pin together before trusting this end-to-end).
- [x] Provide a documented one-shot launch sequence: Wingflight SITL → JSBSim bridge →
  FlightGear, plus a PowerShell helper script analogous to `sitl-rc-check.ps1`.
  **Done**: [scripts/sitl-jsbsim-flightgear-launch.ps1](../../scripts/sitl-jsbsim-flightgear-launch.ps1)
  (see §8 for usage). Smoke-tested (without `-FgfsPath`, since FlightGear isn't
  installed here): starts SITL, starts the bridge with `--flightgear`, prints the
  matching `fgfs` command, and stops both cleanly with `-StopOnExit`.

### Phase 4 — Validation & docs
- [x] Extend or add an RC-injection script (building on `sitl-rc-check.ps1`'s
  patterns) that drives RC through MSP and confirms aileron/elevator/rudder/throttle
  move as expected end-to-end through JSBSim.
  **Done**: added a new `-Mode jsbsim` to
  [scripts/sitl-rc-check.ps1](../../scripts/sitl-rc-check.ps1), reusing all of its
  existing MSP helper infrastructure (connect/handshake, `Enable-RpyPassthroughOverride`,
  `Send-RcWithAck`, etc.) plus new pieces:
  - `Start-JsbsimBridge` launches `scripts/jsbsim_bridge.py` under
    `tools/jsbsim-venv` alongside SITL (and stops it in the `finally` cleanup).
  - `Get-Attitude` reads `MSP_ATTITUDE` (roll/pitch in 0.1°, yaw in whole °).
  - `Hold-RcAndReadAttitude` continuously refreshes an RC frame for a settle window
    (so RX signal timeout/failsafe never kicks in) and samples attitude at the end,
    once JSBSim's physics have had time to respond.
  - The mode drives roll then pitch to each extreme (1900/1100 µs) via the same
    `MIXER_OVERRIDE_PASSTHROUGH` mechanism the other modes already use while
    disarmed, and asserts the resulting `MSP_ATTITUDE` delta exceeds a threshold
    (default 3°) in both directions. Yaw is also measured and reported but does not
    gate pass/fail (slower/subtler response over a short hold on this airframe).
    Throttle/motor response through JSBSim is out of scope — disarmed motor output
    is forced to motor-stop regardless of RC, so it can't be exercised this way.
  - This closes the gap the other three modes have: `smoke`/`sweep`/`stress` only
    ever prove Wingflight's own mixer→servo pipeline reacts to RC. `jsbsim` proves
    the *entire* loop is alive: RC → mixer → `servo_packet` (UDP 9002) →
    `jsbsim_bridge.py` → JSBSim FCS → physics step → `fdm_packet` (UDP 9003) →
    Wingflight's fake IMU → `MSP_ATTITUDE`. Attitude can only change this way if
    JSBSim is genuinely running and reacting to our control inputs.
  - **Validated** against a live `wingflight_SITL.elf` + bridge (aircraft `c172p`):
    `.\scripts\sitl-rc-check.ps1 -Mode jsbsim -AutoStartSitl -StopSitlOnExit` passed
    with roll Δ68.1°, pitch Δ36°, yaw Δ28° (all far above the 3° threshold), exit
    code 0.
- [x] Update [src/main/target/SITL/README.md](../../src/main/target/SITL/README.md)
  with the new JSBSim/FlightGear workflow (keep the Gazebo section as a legacy
  alternative). **Done**.
- [x] Record the final, working versions of JSBSim/FlightGear/MinGW-w64 used, since
  none of these are pinned by a lockfile today. **Done** — see §9.

## 6. Open Questions / Risks

- **§2.3 is a hypothesis, not yet confirmed** — must build first (Phase 0) to verify.
- FlightGear/JSBSim wire-protocol compatibility has shifted across versions upstream;
  needs pinning and a smoke test, not just documentation.
- No unit tests will be added for any of this (per standing project preference).
- This plan does not yet address realtime-pacing/timing behavior under JSBSim (JSBSim
  can run faster/slower than real time; Wingflight's SITL loop currently paces itself
  off packet timestamps from the Gazebo side — this logic needs review once JSBSim is
  the source).
- **Possible root cause found for the `sitl-rc-check.ps1` sweep-mode Yaw quirk**: the
  script's `New-RcPayload` sends channels as `Roll, Pitch, Yaw, Collective(=1500),
  Throttle, Aux1-3`, but SITL's default `rcmap` (`"AETR1234"`, see
  `parseRcChannels()` in [src/main/rx/rx.c](../../src/main/rx/rx.c)) resolves
  payload index 2 to **Throttle** and index 3 to **Yaw** — i.e. the script's Yaw
  and Throttle values land on the wrong firmware channels, and the firmware's Yaw
  channel is fed the script's constant `Collective` placeholder (always 1500).
  Not yet fixed in `sitl-rc-check.ps1` (out of scope for the change that found
  this — see [SITL Joystick RC Input.md](SITL%20Joystick%20RC%20Input.md), which
  documents and uses the *correct* `Roll, Pitch, Throttle, Yaw, AUX...` order).
  Follow-up: re-check `sitl-rc-check.ps1`'s sweep-mode Yaw axis test against the
  corrected channel order before trusting it.

## 7. Immediate Next Steps

1. ~~Implement Phase 0 (`mingw_sdk_install` target) and get a clean `make TARGET=SITL`
   build.~~ Done.
2. ~~Confirm/fix the `pwmOutConfig` servo gap (Phase 1).~~ Done.
3. ~~Run [scripts/sitl-rc-check.ps1](../../scripts/sitl-rc-check.ps1) against the new
   build to confirm MSP/RC injection still works end-to-end with the new toolchain and
   servo-output changes.~~ Done — also found and fixed the real `getServoCount()==0`
   bug (§2.3) and a script bug (`Build-Sitl` output leaking into `ConnectAsync`'s
   argument). Channel-order fix from §6 was already applied in `New-RcPayload`.
4. ~~Only then start the JSBSim bridge (Phase 2)~~ Done — see
   [scripts/jsbsim_bridge.py](../../scripts/jsbsim_bridge.py), validated end-to-end
   against a live `wingflight_SITL.elf` via `MSP_ATTITUDE`.
5. ~~Add FlightGear visualization (Phase 3)~~ Done (pending real-FlightGear
   validation) — see [scripts/sitl-jsbsim-flightgear-launch.ps1](../../scripts/sitl-jsbsim-flightgear-launch.ps1)
   and §8.
6. ~~Manual/interactive RC input for SITL~~ Done — see
   [scripts/sitl-joystick-rc.py](../../scripts/sitl-joystick-rc.py) and
   [SITL Joystick RC Input.md](SITL%20Joystick%20RC%20Input.md) (USB joystick/gamepad
   bridge with a channel-mapping GUI, independent of the JSBSim/FlightGear work).
7. ~~Phase 4 (final docs/validation pass)~~ Done — `sitl-rc-check.ps1 -Mode jsbsim`
   validates the full RC→JSBSim→attitude loop end-to-end (see Phase 4 checklist
   above), [src/main/target/SITL/README.md](../../src/main/target/SITL/README.md) documents
   the JSBSim/FlightGear workflow, and §9 records the exact pinned versions used.
   Remaining open item: validating against a real FlightGear install (not present on
   this dev machine).

## 8. FlightGear install and launch sequence (Phase 3)

### Install

FlightGear is a large third-party GUI application and is **not** vendored under
`tools/` (unlike the MinGW-w64 toolchain and the JSBSim venv) — install it manually:

1. Download the Windows installer from the official site:
   [flightgear.org/download](https://www.flightgear.org/download/).
2. **Pin a specific version** and record it here once installed. This plan's bridge
   was built/tested against **JSBSim 1.3.1** (the version pinned in
   `tools/jsbsim-venv`, confirmed via `pip show jsbsim`) \u2014 pick whichever current
   stable FlightGear release you install and record the exact pair here. JSBSim's
   own changelog notes the FlightGear native-FDM wire format has changed (and been
   reverted) across versions before, so a mismatched pair may silently misrender or
   disconnect.
3. No project files depend on the install location; any `fgfs.exe` path works with
   `-FgfsPath` below.

### One-shot launch

```powershell
# Build SITL if needed, start SITL + the JSBSim bridge (FlightGear output enabled),
# and auto-launch FlightGear once you know its fgfs.exe path:
.\scripts\sitl-jsbsim-flightgear-launch.ps1 -BuildSitl -FlightGear `
    -FgfsPath "C:\Program Files\FlightGear <version>\bin\fgfs.exe" -StopOnExit
```

Without `-FgfsPath`, the script starts SITL + the bridge and prints the exact `fgfs`
command to run manually in another terminal:

```
fgfs --aircraft=c172p --fdm=null --native-fdm=socket,in,30,,5550,udp --disable-clouds3d
```

`--fdm=null` tells FlightGear not to run its own physics — it's purely a rendering
client of JSBSim's state, matching the plan's "visualization-only" approach.

### How it works

[scripts/jsbsim_bridge.py](../../scripts/jsbsim_bridge.py)'s `--flightgear` flag
registers a JSBSim output-directives file (`<output type="FLIGHTGEAR" protocol="UDP"
.../>`) via `FGFDMExec.set_output_directive()` **before** `load_model()` runs (required
— JSBSim silently ignores this call if made afterwards). JSBSim then serializes its
own native FDM struct to `--fg-host`:`--fg-port` every step at `--fg-rate` Hz,
completely independent of the `fdm_packet`/`servo_packet` link to Wingflight — the
same running JSBSim instance drives both Wingflight *and* FlightGear simultaneously.

### Validation status

Validated **without** a real FlightGear install: binding a raw UDP socket on the
configured port while the bridge ran with `--flightgear` showed steady, correctly
rate-limited (30 Hz → 601 datagrams over ~20s), fixed-size (408-byte) binary frames —
consistent with JSBSim genuinely emitting FlightGear's native FDM protocol. **Not yet
confirmed that a real FlightGear process renders this correctly** — do that once
FlightGear is installed, and record the exact version pair (JSBSim × FlightGear) that
works.

## 9. Pinned toolchain/simulator versions (Phase 4)

None of the following are pinned by a lockfile today (no `requirements.txt`/version
manifest exists for the vendored `tools/` directory) — this section is the source of
truth for "what actually worked" as of this validation pass. Update it whenever any of
these are intentionally upgraded.

| Component | Version | Notes |
|---|---|---|
| MinGW-w64 (native Windows toolchain) | WinLibs GCC **13.3.0**, MinGW-W64 x86_64-ucrt-posix-seh (built by Brecht Sanders, r1), mingw-w64 **11.0.1** UCRT | Vendored under `tools/mingw64/`. Confirmed via `tools/mingw64/bin/gcc.exe --version`. |
| JSBSim (Python package) | **1.3.1** | Installed in `tools/jsbsim-venv/` (gitignored). Confirmed via `pip show jsbsim`. |
| Python (JSBSim bridge venv) | **3.14.5** | `tools/jsbsim-venv/Scripts/python.exe --version`. |
| Aircraft model | **c172p** (Cessna 172P), bundled with JSBSim | Used for all Phase 2–4 validation. |
| FlightGear | **not installed / not pinned** | Manual install only (not vendored — see §8 Install). JSBSim's FlightGear-native UDP output was validated with a raw socket listener, not a real FlightGear process. Record the exact FlightGear version here once it's installed and validated against JSBSim 1.3.1. |

When upgrading JSBSim or MinGW-w64, re-run
`.\scripts\sitl-rc-check.ps1 -Mode jsbsim -BuildSitl -AutoStartSitl -StopSitlOnExit`
to confirm the end-to-end loop still works before trusting the new versions.
