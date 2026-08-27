# SITL → JSBSim → FlightGear Integration Plan

Status: **Phases 0–4 implemented; Phase 5 (gap review, 2026-08-27) applied.**
The native MinGW-w64 toolchain is vendored under `tools/mingw64`, the
`pwmOutConfig`/servo link gap is fixed, `make TARGET=SITL` builds and runs, RC
injection and servo output are confirmed end-to-end via `sitl-rc-check.ps1`, and
`scripts/jsbsim_bridge.py` drives a live SITL instance through JSBSim
(`sitl-rc-check.ps1 -Mode jsbsim` proves the whole RC → mixer → JSBSim →
`MSP_ATTITUDE` loop).

A gap review on 2026-08-27 (see §10) found and fixed several real defects that
the earlier "validated" runs did not catch. The two that mattered most: **from a
clean checkout the SITL target did not link at all**, and **throttle could never
reach JSBSim** (no motor channels existed in SITL's synthetic timer table, the
bridge read the wrong `motor_speed[]` index, and the JSBSim engine was never
started). A stale `wingflight_SITL.exe` in `obj/main` shadowing the built `.elf`
is the likely reason earlier phases recorded passes that do not reproduce.

All of §10's fixes are validated on this machine (§10.14): `make TARGET=SITL
DEBUG=GDB` links, and `sitl-rc-check.ps1` passes in all four modes -
`smoke`/`sweep`/`stress` plus `jsbsim` (roll 84.8°, pitch 40.9°, yaw 37.0°
against a 3° threshold). **FlightGear is still unvalidated against a real
install** (not present here), and §10.15 lists what else remains open. §11 is the
step-by-step "how to build, run and test this from a fresh clone".

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

- ~~**§2.3 is a hypothesis, not yet confirmed**~~ — confirmed and fixed (see §2.3).
- FlightGear/JSBSim wire-protocol compatibility has shifted across versions upstream;
  needs pinning and a smoke test, not just documentation. **Still open** — no
  FlightGear install exists on this dev machine, so only JSBSim's side of the wire
  has been observed (see §8 "Validation status").
- No unit tests will be added for any of this (per standing project preference).
- **Realtime pacing.** JSBSim can run faster or slower than real time, and
  `target.c`'s `updateState()` derives `simRate` from consecutive `fdm_packet`
  timestamps, which scales `micros64()` (and therefore the whole scheduler's notion
  of time). The bridge paces itself to wall-clock at `--rate` Hz by default, which
  keeps `simRate ≈ 1.0`; `--no-realtime` deliberately breaks that and makes
  Wingflight's clock run at whatever multiple of real time the host can sustain.
  Two related sharp edges, unchanged and worth knowing about:
  - `updateState()` only recomputes `simRate` when `0 < deltaSim < 0.02`, i.e. the
    FDM must send at **> 50 Hz** or the FC clock silently keeps its last scale
    factor. Don't run the bridge below `--rate 60`.
  - If the bridge stops, no more `fdm_packet`s arrive, so `updateState()` stops
    running, `simRate` freezes and (after the 500 ms timeout branch) the FC keeps
    running on its last time scale. Symptom: SITL appears alive over MSP but
    attitude is frozen. Check the bridge process first.
- ~~**`sitl-rc-check.ps1` sweep-mode Yaw quirk**~~ — fixed. `New-RcPayload` now
  sends `Roll, Pitch, Throttle, Yaw, AUX1-3, Collective`, matching what SITL's
  default `rcmap` (`"AETR1234"`, see `parseRcChannels()` in
  [src/main/rx/rx.c](../../src/main/rx/rx.c)) resolves those payload indices to, and
  parks the vestigial heli-only `Collective` placeholder on an unused AUX slot.
  Same order as [SITL Joystick RC Input.md](SITL%20Joystick%20RC%20Input.md).

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

The Python versions are pinned by
[scripts/requirements-jsbsim.txt](../../scripts/requirements-jsbsim.txt) and
[scripts/requirements-joystick.txt](../../scripts/requirements-joystick.txt) as of
the Phase 5 review (see §10.11); this table and those files must be kept in sync.
Re-validated 2026-08-27 on Python 3.13 (whatever `python -m venv` produces on the
machine) rather than the 3.14.5 recorded above - both work.

## 10. Phase 5 - Gap review (2026-08-27)

A fresh-clone review of everything Phases 0-4 claimed to have finished. Every item
below was reproduced on this machine before being fixed, and the fix is in the
tree. Ordered by how badly it broke the loop.

The headline: **from a clean checkout the SITL target did not even link, and once
it did, throttle could never reach JSBSim.** The earlier "validated" runs did not
catch either, for reasons worth understanding - see 10.1, 10.2 and 10.6.

### 10.1 SITL did not link from a clean tree (BLOCKER, fixed)

`make TARGET=SITL` failed at the link step on a clean `obj/`, with the vendored
`tools/mingw64` GCC 13.3.0 and with a system MinGW alike:

```
sensors/gyro_init.c:678: undefined reference to `mpuGyroReadRegister'
drivers/dma_common.c:     undefined reference to `dmaDescriptors'
rx/srxl2.c:291,313:       undefined reference to `microsISR'
```

All three come from feature macros that `common_pre.h` defines globally and
SITL's `target.h` never turned off, while `make/mcu/SITL.mk` excludes the drivers
that would define the symbols (`drivers/dma.c`, `drivers/accgyro/accgyro_mpu.c`,
`drivers/system.c`). An `-Ofast` build hid two of the three (LTO + `-gc-sections`
pruned the unreferenced code); a `DEBUG=GDB` build - the one every script under
`scripts/` actually uses - failed on all three.

Fixed in [target.h](../../src/main/target/SITL/target.h) by extending the existing
`#undef` block, exactly how every other unsupported feature is handled there:
`#undef USE_DMA`, `#undef USE_GYRO_REGISTER_DUMP`, `#undef USE_SERIALRX_SRXL2`
(the last was simply missed - every other `USE_SERIALRX_*` was already undef'd).

### 10.2 No motor channels existed, so throttle was structurally impossible (fixed)

The synthetic `timerHardware[]` table added in Phase 1 only declared four
`TIM_USE_SERVO` channels. `motorConfig()`'s reset function builds its `ioTags`
from `timerioTagGetByUsage(TIM_USE_MOTOR, ...)`, so every motor ioTag resolved to
`IO_TAG_NONE`, `motorInit()` counted `motorCount = 0`, and `motorsPwm[0]` (M1,
the wing's throttle) was never written by anything, ever.

Fixed by extending the table to four `TIM_USE_MOTOR` + four `TIM_USE_SERVO`
entries (`USABLE_TIMER_CHANNEL_COUNT` 8 to match). Four motors rather than one
keeps the legacy Gazebo quad path working; `motorPwmDevInit()` rejects more.

### 10.3 The bridge read the wrong motor channel (fixed)

`scripts/jsbsim_bridge.py` read `motor_speed[0]` as throttle. But
`pwmCompleteMotorUpdate()` in [target.c](../../src/main/target/SITL/target.c)
applies the legacy Gazebo ArduCopterPlugin motor remap:

```c
pwmPkt.motor_speed[3] = motorsPwm[0] / outScale;   // M1 - the wing's throttle
pwmPkt.motor_speed[0] = motorsPwm[1] / outScale;   // M2 - unused on a 1-motor wing
```

so even after 10.2 the bridge would have been reading a channel a fixed-wing
build never populates. The bridge now defaults to `motor_speed[3]`, with
`--throttle-motor-index` to override. `target.c` keeps the remap so the legacy
Gazebo path is unaffected.

### 10.4 The JSBSim engine was never started (fixed)

Even with 10.2 and 10.3 fixed, `fcs/throttle-cmd-norm` produced no thrust: JSBSim
piston models (`c172p` included) initialise with the engine cold and stopped. The
bridge now sets `propulsion/set-running = -1` after `run_ic()`
(`--no-engine-start` opts out). This also unblocked trimming, which previously
failed with "Sorry, udot doesn't appear to be trimmable" - there was no thrust to
trim against.

### 10.5 Nothing reached the simulator while the motor device was disabled (fixed)

`pwmCompleteMotorUpdate()` was the only place a `servo_packet` was ever sent, and
it is reached only through `motorWriteAll()`, which short-circuits on
`motorDevice->enabled`. While that flag is false the FDM receives nothing at all -
not even the control-surface channels, which do move while disarmed and which
every `sitl-rc-check.ps1` mode depends on. `updateState()` now refreshes and sends
the packet itself whenever `motorIsEnabled()` is false (motor channels zeroed,
matching the real disarmed output), through a shared `refreshPwmPacket()` helper.

### 10.6 A stale binary was shadowing every test run (fixed)

`sitl-rc-check.ps1` probed `obj/main/wingflight_SITL.exe` **before**
`obj/main/wingflight_SITL.elf`. `make TARGET=SITL` produces the `.elf`; a
month-old hand-built `.exe` sitting in `obj/main` therefore silently won every
run, so the script was testing month-old firmware while reporting on the current
tree. This is very likely why earlier phases recorded passes that do not
reproduce. The candidate list now puts `.elf` first and warns when an older
`.exe` is being ignored.

### 10.7 A leftover process on UDP 9002 silently swallowed the link (mitigated)

A stray `jsbsim_bridge.py` from an earlier session still bound to
`127.0.0.1:9002` made every later run look identical to "SITL sends nothing":
`sendto()` returned success, `packets_rx` stayed at 0. Both scripts now surface
this - the launcher warns when 9002/9003 are already held, the bridge exits with
an explicit message instead of a traceback when its bind fails, and its status
line calls out `packets_rx=0`. **If a run looks dead, check
`Get-NetUDPEndpoint -LocalPort 9002` first.**

### 10.8 No initial position - FlightGear would have rendered an empty ocean (fixed)

The bridge set only altitude and airspeed, leaving JSBSim's default lat/lon of
0/0 - a point in the Gulf of Guinea with no FlightGear scenery. The raw-socket
validation in section 8 could not have caught this (the bytes on the wire look
correct). The bridge now takes `--lat-deg`/`--lon-deg`/`--heading-deg`, defaulting
to KSFO (37.6136 / -122.3572) which the FlightGear base package ships scenery for,
and the launcher passes the same position to `fgfs` via `--lat/--lon/--altitude`
so FlightGear preloads scenery in the right place.

### 10.9 The aircraft started untrimmed (fixed)

Phase 2 validated attitude tracking during an "untrimmed, idle-throttle glide" -
the airframe was decelerating out of its envelope the whole time, which is a poor
baseline for judging control response (and a source of attitude change unrelated
to the control inputs). `--trim` (with `--trim-mode`, default 1 = full trim) now
trims at the initial conditions; a failed trim warns and continues rather than
aborting. `sitl-rc-check.ps1 -Mode jsbsim` passes `--trim`. Observed effect:
altitude holds ~3000 ft and IAS stays near the 90 kt IC instead of decaying.

### 10.10 A crash streamed NaNs into Wingflight's IMU (fixed)

Nothing stopped JSBSim integrating past ground impact or into non-finite state,
and those values went straight into `fdm_packet` and out through
`fakeAccSet()`/`fakeGyroSet()`/`imuSetAttitudeQuat()`. The bridge now checks the
state each step and re-runs the initial conditions on a crash or non-finite value
(`--min-agl-ft`, `--no-auto-reset`).

### 10.11 Fresh-machine setup was undocumented and unpinned (fixed)

`tools/` is gitignored in full, so a fresh clone has neither `tools/mingw64` nor
`tools/jsbsim-venv`, and the launcher failed with a "create it yourself" error.
There was also no requirements manifest anywhere - section 9's table was the only
record. Added [scripts/requirements-jsbsim.txt](../../scripts/requirements-jsbsim.txt)
(pins `jsbsim==1.3.1`) and
[scripts/requirements-joystick.txt](../../scripts/requirements-joystick.txt)
(`pygame`), plus `-SetupVenv` on the launcher, which creates the venv and installs
both. See section 11.

### 10.12 The barometer was never fed (fixed)

`target.h` enables `USE_FAKE_BARO`, but nothing ever called `fakeBaroSet()`, so
the firmware saw a constant 101325 Pa - 0 m MSL - no matter what the simulator
did. `updateState()` now derives ISA pressure and temperature from the FDM's
altitude. Note that altitude is *relative to the simulator's initial condition*,
because `fdm_packet.position_xyz` is NED metres from the sim origin; absolute MSL
altitude would need a new packet field.

The fake **compass** is deliberately *not* fed, and that is now commented in
`target.c`: SITL undefines `USE_IMU_CALC`, so attitude and heading come straight
from the simulator's quaternion and the mag is never consulted. Feed it only if
`USE_IMU_CALC` is ever turned on to test the AHRS itself.

### 10.13 Smaller launcher/bridge fixes

- The launcher started no RC source at all. SITL's RX is `FEATURE_RX_MSP`, so
  without one the mixer sits at failsafe and nothing flies. Added `-Joystick`
  (runs [scripts/sitl-joystick-rc.py](../../scripts/sitl-joystick-rc.py)) and an
  explicit note when no RC source is running.
- The launcher only watched the SITL process, so a bridge that died (bad model
  name, port in use) left it looping forever. It now watches bridge and
  FlightGear too.
- Bridge stdout was block-buffered, so the launcher's redirected log stayed empty
  until the process exited. Now line-buffered.
- The bridge's temporary FlightGear output-directive XML was never deleted, and
  `Stop-Process` (SIGTERM) skipped cleanup. Both handled.
- Servo pulse to surface scaling hardcoded 1000/1500/2000 us; added
  `--servo-min/--servo-mid/--servo-max` and `--invert-aileron/-elevator/-rudder`
  for airframes (or model conventions) where a control direction comes out
  reversed.
- `-Mode jsbsim` now launches the bridge with `--trim`.

### 10.14 Validation after the fixes

Run on this machine, 2026-08-27, vendored MinGW-w64 13.3.0 + JSBSim 1.3.1:

```
make TARGET=SITL DEBUG=GDB -j 8                  -> links, obj/main/wingflight_SITL.elf
sitl-rc-check.ps1 -Mode jsbsim ... -FreshEeprom  -> PASS
    getServoCount() = 4
    roll 84.8 deg, pitch 40.9 deg, yaw 37.0 deg (threshold 3 deg), exit 0
sitl-rc-check.ps1 -Mode smoke                    -> PASS  pitch_servo delta 938 us
sitl-rc-check.ps1 -Mode sweep                    -> PASS  roll/pitch/yaw 938 us each
sitl-rc-check.ps1 -Mode stress                   -> PASS  536 cycles, 0 dropouts
jsbsim_bridge.py + SITL, no RC                   -> packets_rx ~120/s, trim holds ~3000 ft
sitl-jsbsim-flightgear-launch.ps1 -Trim -FlightGear
                                                 -> SITL + bridge start and stop cleanly
```

### 10.15 Still open

- **FlightGear has never been run against this.** Everything in section 8 is
  still inferred from the bytes JSBSim puts on the wire. 10.8 removes the most
  likely cause of a "connects but shows nothing" result, but the version-pairing
  risk in section 6 stands.
- **`-Mode jsbsim` does not isolate control-induced motion from free dynamics.**
  It asserts that attitude changes by more than a threshold while a control is
  held at an extreme. A trimmed aircraft drifts far less than the untrimmed one
  Phase 2 measured, but the test still cannot distinguish "the elevator worked"
  from "the aircraft was diverging anyway". A stronger version would compare
  against a control-neutral run of the same duration. Treat the current check as
  a liveness test for the loop, not a correctness test for the mixer.
- **No automated throttle check.** A disarmed FC forces motor output to
  motor-stop, so the disarmed passthrough trick cannot exercise throttle. With
  10.2/10.3/10.4 fixed it should now work; verify it by hand per section 11.4.
- **Control-surface sign conventions are unverified.** Nothing has confirmed that
  a right-roll stick input produces a right roll in JSBSim rather than a left one;
  the check only looks at magnitude. Use `--invert-*` if a direction is wrong.
- **GPS/position is not fed to the firmware.** `fdm_packet` carries velocity and
  position but `updateState()` ignores both apart from the new baro derivation, so
  anything GPS-dependent is untestable in SITL today.

## 11. How to run it (fresh machine)

### 11.1 One-time setup

```powershell
# 1. Native C toolchain for TARGET=SITL: downloads a ~130 MB WinLibs zip into
#    downloads/ and unpacks it to tools/mingw64. No admin rights, no PATH changes.
#    Needs curl + unzip on PATH - Git Bash has both, plain PowerShell may not.
make mingw_sdk_install

# 2. Python venv for the JSBSim bridge (creates tools/jsbsim-venv, installs the
#    pinned requirements). Needs a system python3 on PATH.
.\scripts\sitl-jsbsim-flightgear-launch.ps1 -SetupVenv

# 3. (optional) FlightGear - manual installer, see section 8.
```

### 11.2 Build

```powershell
make TARGET=SITL DEBUG=GDB -j 8      # -> obj/main/wingflight_SITL.elf
```

`DEBUG=GDB` is what every script under `scripts/` uses; build it that way unless
you have a reason not to. If `obj/main/wingflight_SITL.exe` exists from an older
build, delete it (see 10.6).

### 11.3 Automated end-to-end check (start here)

```powershell
.\scripts\sitl-rc-check.ps1 -Mode jsbsim -AutoStartSitl -StopSitlOnExit -FreshEeprom
```

Drives roll and pitch to both extremes through the disarmed
`MIXER_OVERRIDE_PASSTHROUGH` path and asserts `MSP_ATTITUDE` moves, which
requires the whole loop to be alive: RC to mixer to `servo_packet` (UDP 9002) to
`jsbsim_bridge.py` to JSBSim FCS to physics to `fdm_packet` (UDP 9003) to the
fake IMU to MSP. Expect `getServoCount() = 4`, roll/pitch deltas far above the
3 deg threshold, exit code 0. Bridge output goes to
`obj/main/jsbsim_bridge_check_stdout.log`; `packets_rx=0` there means the UDP
link is dead (see 10.7).

Then the three modes that exercise Wingflight's own servo path without JSBSim:

```powershell
.\scripts\sitl-rc-check.ps1 -Mode smoke  -AutoStartSitl -StopSitlOnExit
.\scripts\sitl-rc-check.ps1 -Mode sweep  -AutoStartSitl -StopSitlOnExit
.\scripts\sitl-rc-check.ps1 -Mode stress -AutoStartSitl -StopSitlOnExit
```

### 11.4 Interactive flying, and verifying throttle

```powershell
.\scripts\sitl-jsbsim-flightgear-launch.ps1 -Trim -Joystick -StopOnExit
```

The bridge's status line shows what JSBSim actually receives:

```
[jsbsim-bridge] t=  12.02s alt= 2998.8ft ias= 88.2kts thr=0.00 ail=+0.00 ele=+0.00 rud=+0.00 packets_rx=1440
```

**This is how to verify the 10.2/10.3/10.4 throttle chain**: arm the aircraft,
raise the throttle stick, and confirm `thr=` follows it and IAS climbs. If a
control axis moves the wrong way, add `--invert-aileron` (or `-elevator` /
`-rudder`) rather than editing the mixer.

### 11.5 Adding FlightGear

```powershell
.\scripts\sitl-jsbsim-flightgear-launch.ps1 -Trim -Joystick -FlightGear `
    -FgfsPath "C:\Program Files\FlightGear <version>\bin\fgfs.exe" -StopOnExit
```

Without `-FgfsPath` the bridge prints the exact `fgfs` command to run by hand.
Once this works, record the FlightGear version in section 9 and update the
validation status in sections 8 and 10.15.

Troubleshooting:
- Blue void / nothing renders: the aircraft is not where FlightGear loaded
  scenery. Check `-LatDeg`/`-LonDeg` (default KSFO).
- FlightGear connects but the aircraft is frozen while the bridge status line
  keeps advancing: protocol-version mismatch between JSBSim and FlightGear
  (section 6).
- `packets_rx=0`: SITL is not running, or another process holds UDP 9002
  (`Get-NetUDPEndpoint -LocalPort 9002`).
