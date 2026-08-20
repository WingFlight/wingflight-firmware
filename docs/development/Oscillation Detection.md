# Oscillation Detection & Automatic Gain Reduction — Implementation Plan

## 1. Problem

Excess `master_gain` on a control-surface axis (alone or combined with `gain_curve`) can push the
fixed-wing rate loop into a self-sustained oscillation during certain maneuvers, typically in a
fairly narrow, low-frequency band (roughly 2-25 Hz) — well below the 60Hz+ band the existing
dynamic notch (`src/main/flight/dyn_notch_filter.c`) targets.

Goal: a per-axis monitor that reliably confirms a real gain-induced oscillation (not a hard
maneuver, gust, vibration, or single transient), reduces `master_gain` for the affected axis only,
and is provably side-effect free — never touches stored config, never increases authority, never
injects its own transient, and stays inert whenever its preconditions aren't met.

This scope covers: the detector core, automatic gain reduction, blackbox logging, and reporting
the state back to the radio — both CRSF/ELRS and FrSky (S.Port/FBUS) must be able to show the
pilot that oscillation was detected. **OSD warnings and beeper feedback are explicitly out of
scope for this pass** and are listed under Future Extensions only.

## 2. Detection signal

Use the tracking error, not raw gyro rate:

```
errorRate = setpoint - gyroRate     // already computed in pidApplyMode1()
```

A well-tuned axis simply following an aggressive pilot input (snap roll, rudder waggle,
knife-edge correction) keeps `errorRate` small even though `gyroRate` itself is large and
fast-changing. A gain-induced oscillation shows up specifically as a sustained, periodic residual
in the error, regardless of what the pilot is commanding. This is the main lever for avoiding
false positives.

Per-axis pipeline, each PID loop:

1. **Band-pass** `errorRate` with a single `biquadFilter_t` using the existing `BIQUAD_BPF` type
   (`src/main/common/filter.h`), centered in a configurable oscillation band (default ~4-20 Hz).
   No new filter primitive needed.
2. **Energy estimate**: leaky-integrator mean-square, `energy += (bp*bp - energy) * alpha` — no
   `sqrt`, cheap, bounded.
3. **Periodicity check**: count zero-crossings of the band-passed signal in a rolling window
   (e.g. 200 ms). Require a minimum crossing count (e.g. >= 6, i.e. >= 3 full cycles). Rejects a
   single spike/gust/step-response ring, which crosses zero once or twice, not periodically.
4. **Setpoint-activity gate**: freeze (don't reset, just hold) the accumulators while `|setpoint|`
   or its slew rate is very large/changing abruptly (first ~100 ms after a big stick step), so an
   intentional snap input's initial transient can't be scored at all.

A "candidate" sample = `energy > threshold AND crossings >= minCrossings AND not gated`.

## 3. Reliability: multi-criteria hysteresis

```
score += candidate ? +1 : -2      // asymmetric: slow to engage, faster to relax
score  = clamp(score, 0, ENGAGE_SCORE_MAX)
confirmed = (score >= ENGAGE_SCORE_MAX)   // reached after ~200-300ms sustained candidate=true
```

Four independent conditions must agree before anything happens: error-based signal (not raw
rate), band-pass energy, periodicity, and sustained persistence. Per-axis, independent state —
roll oscillating doesn't touch yaw's gain.

## 4. Gating conditions

The detector only runs (state stays frozen otherwise, never reset mid-oscillation) when:

- `isAirborne()` is true (`src/main/flight/setpoint.h`) — no detection on the ground/bench.
- `!gyroOverflowDetected()` — a saturated gyro must never look like "confirmed oscillation."
- Not in `FAILSAFE_MODE` / `GPS_RESCUE_MODE` — those paths already force safe behavior.
- `pid.pidMode == 1` (normal fixed-wing rate mode), not passthrough.

## 5. Mitigation: bounded, gradual, ratcheting

```
if confirmed: gainScale = max(oscLimiterFloor, gainScale - rampStep)  // back off one step further
// else: hold -- never eased back up mid-flight
```

Applied as one more multiplicative factor exactly where `gain_curve`/`fw_tpa_gain` are already
applied in `pidApplyMode1()`:

```c
const float masterGain = pid.masterGain[axis] * curveMult * pid.oscGainScale[axis];
```

Key properties:

- **Never persisted** — `pid.oscGainScale[axis]` is pure runtime state in `pid_t`
  (`src/main/flight/pid.h`), never written back into `pidProfile_t`. `diff`/`save`/profile
  switching are unaffected.
- **Only ever attenuates**: clamped to `[oscLimiterFloor, 1.0]` — can never exceed 1.0.
- **Gradual, not a snap cut**: gain eases down one small step per loop for as long as a sustained
  oscillation is confirmed, and stops the moment it's no longer detected — so a single mild event
  need not reach the floor.
- **Ratchets, never recovers mid-flight**: once the oscillation clears, `gainScale` holds at
  wherever it ended up — it is never eased back up during the same flight. If the same axis
  oscillates again later, it resumes backing off from that already-reduced level, cutting harder
  each time it recurs. A new arm or profile reload is the only way authority is restored.
- **Slew-limited** on the way down — an abrupt gain step could itself excite a transient.
- **Reported 'active' flag has its own 3s hold**, decoupled from `gainScale`: it stays asserted
  for a bit after the last engagement so a brief drop-out doesn't flicker the blackbox/telemetry
  indication, even though the underlying gain cut itself is permanent for the flight.

## 6. New accessor API (`src/main/flight/pid.h` / `pid.c`)

Read-only getters, mirroring the existing `pidGetAxisData()` / `pidGetRuntimeGains()` pattern —
all reporting consumers (CRSF, blackbox) go through these instead of touching detector internals:

```c
bool    pidOscLimiterActive(int axis);   // true while engaged, plus a 3s hold after (reporting only)
uint8_t pidOscLimiterScale(int axis);    // current gain scale, percent (100 = no cut)
```

## 7. New configuration (`pidProfile_t`, `src/main/pg/pid.h`)

Additive fields only, appended, no renumbering:

```c
uint8_t  osc_limiter;            // 0 = off (default until validated), 1 = on
uint8_t  osc_limiter_min_hz;     // band-pass low edge, default ~4
uint8_t  osc_limiter_max_hz;     // band-pass high edge, default ~20
uint8_t  osc_limiter_threshold;  // energy threshold
uint8_t  osc_limiter_floor;      // percent, min gain scale (default 50)
uint16_t osc_limiter_engage_ms;  // default ~250
```

Exposed via CLI (`cli/settings.c`) the same way `fw_tpa_gain`/`gain_curve` already are.

## 8. Blackbox logging

Two tiers, matching how flight-mode changes are already logged:

**a) Discrete event on state change (always captured, cheap)**

Add `FLIGHT_LOG_EVENT_OSC_LIMITER` to `FlightLogEvent` in
`src/main/blackbox/blackbox_fielddefs.h` (append-only numeric value):

```c
typedef struct flightLogEvent_oscLimiter_s {
    uint8_t axis;
    uint8_t active;     // 1 = engaged, 0 = released
    uint8_t gainScale;  // percent at time of transition
} flightLogEvent_oscLimiter_t;
```

Fired from `pid.c` only on the rising/falling edge of `confirmed`, via
`blackboxLogEvent(FLIGHT_LOG_EVENT_OSC_LIMITER, ...)` guarded by `#ifdef USE_BLACKBOX` — same
pattern as the existing `FLIGHT_LOG_EVENT_FLIGHTMODE` call in `src/main/blackbox/blackbox.c`.
Guarantees the event is in every log regardless of which `debug_mode` the pilot has selected.

**b) Continuous per-loop trace (opt-in, for tuning)**

Reuse the existing generic `debug[0..7]` blackbox fields instead of adding new always-present
columns. Add a `DEBUG_OSC_LIMITER` entry (append-only) to `src/main/build/debug.h`, and log
band-passed energy, zero-crossing score, and gain scale per axis via
`DEBUG_AXIS(OSC_LIMITER, axis, ...)` — same macro already used for `DEBUG_ITERM_RELAX` in
`pidApplyMode1()`/`applyItermRelax()` in `src/main/flight/pid.c`. Zero footprint unless the pilot
sets `set debug_mode = OSC_LIMITER`.

## 9. Radio reporting (this pass: CRSF + FrSky, both must work)

`src/main/telemetry/sensors.h` / `sensors.c` already define a protocol-agnostic sensor table
(`sensor_id_e`, `telemetrySensorValue()`) that both the CRSF backend (`src/main/telemetry/crsf.c`)
and the FrSky S.Port/FBUS backend (`src/main/telemetry/smartport.c`) read from — this is the same
mechanism `TELEM_FLIGHT_MODE`, `TELEM_ARMING_FLAGS`, and `TELEM_DEBUG_0..N` already use to reach
both radio families from one implementation. Reuse it instead of a CRSF-only patch, so FrSky
pilots aren't left out:

1. Add `TELEM_OSC_LIMITER` to `sensor_id_e` in `src/main/telemetry/sensors.h` (append-only).
2. Implement its value in `telemetrySensorValue()` in `src/main/telemetry/sensors.c`, returning a
   compact encoding built from `pidOscLimiterActive()`/`pidOscLimiterScale()` — e.g. a per-axis
   bitmask of which axes are currently latched, packed with the worst-case (lowest) gain scale.
3. Register the sensor in `smartportTelemetrySensors[]` in `src/main/telemetry/smartport.c` via
   the existing `TLM_SENSOR(...)` macro with an unused FrSky app ID, and in the equivalent CRSF
   custom-sensor table in `crsf.c` (`.sensor_id = TELEM_##NAME` pattern already used there) — one
   value definition, two protocol registrations.
4. Additionally extend `crsfFlightModeInfo()` in `src/main/telemetry/crsf.c` to append a suffix to
   the existing flight-mode text (e.g. `"ANGLE OSC!"`, capped to the existing 32-byte buffer) when
   any axis is latched. This is CRSF-specific but gives ELRS/CRSF pilots an unmissable indicator
   even if they haven't added the new sensor to a telemetry screen. Keep it a suffix, not a mode
   override — don't hide the real flight mode; `FAILSAFE`/`GPS-RESCUE` priority ordering stays
   as-is.

FrSky radios have no equivalent free-text flight-mode field in this codebase, so steps 1-3 (the
generic sensor) are the only reliable path to reach them and must not be skipped — the CRSF text
suffix in step 4 is a supplementary nice-to-have for CRSF only, not a substitute.

## 10. Explicitly out of scope for this pass

- **OSD warning** (`OSD_WARNING_OSCILLATION` in `src/main/osd/osd.h` /
  `src/main/osd/osd_warnings.c`) — not implemented now.
- **Beeper feedback** (`BEEPER_OSC_LIMITER` in `io/beeper.h`) — not implemented now.
- Other telemetry backends (HoTT, MAVLink, LTM) — not extended in this pass; CRSF and FrSky
  S.Port/FBUS are the required targets.

All of these read through the same `pidOscLimiterActive()`/`pidOscLimiterScale()` accessors, so
adding them later is additive and doesn't require revisiting the detector.

## 11. Safety / no-side-effect checklist

- Pure function of already-available, read-only signals (`setpoint`, `gyroRate`) — no new sensor
  dependency.
- No dynamic allocation, fixed-size state per axis, O(1) per loop.
- Doesn't alter `pidApplySetpoint()` or any flight-mode logic — only scales the final gain
  multiplier.
- Doesn't write to `pidProfile_t` — profile save/diff/copy semantics unchanged.
- Bounded output range guarantees the detector can only reduce, never increase or fully remove,
  authority.
- Disabled by default (`osc_limiter = 0`) and fully gated off airborne/gyro-health/failsafe state.
- Reporting reads (`pidOscLimiterActive/Scale`) are pure getters — cannot feed back into the
  detector or gain math.
- Event logging is edge-triggered, not per-loop, so a latched-and-stable condition can't spam the
  blackbox log.
- CRSF text stays within the existing 32-byte buffer; the new sensor's payload fits within the
  size already used by comparable sensors (e.g. `TELEM_FLIGHT_MODE`/`TELEM_ARMING_FLAGS`) in both
  the CRSF custom-telemetry frame and the FrSky S.Port/FBUS payload, so no protocol framing
  changes are needed.
- All new enum values (`FLIGHT_LOG_EVENT_OSC_LIMITER`, `DEBUG_OSC_LIMITER`, `TELEM_OSC_LIMITER`)
  are appended, never renumbering existing IDs, per this repo's stable-interface rule (see
  `AGENTS.md`).

## 12. Implementation order

1. Detector core (`pid.c`/`pid.h`) + `pidOscLimiterActive/Scale()` accessors.
2. New `pidProfile_t` fields + CLI plumbing, default off.
3. Blackbox event (`FLIGHT_LOG_EVENT_OSC_LIMITER`) — cheapest, gives immediate post-flight
   visibility for validating the detector itself.
4. Blackbox debug slot (`DEBUG_OSC_LIMITER`) — needed for tuning thresholds during flight-testing.
5. Generic `TELEM_OSC_LIMITER` sensor (`sensors.h`/`sensors.c`) registered in both
   `smartportTelemetrySensors[]` (FrSky) and the CRSF custom-sensor table.
6. CRSF flight-mode text suffix (supplementary, CRSF-only).
7. Document new params in `Changes.md` once implemented.

## 13. Validation plan

- Blackbox replay of existing flight logs (including hard aerobatics) to confirm zero false
  triggers before enabling by default.
- Confirm the blackbox event and debug fields decode correctly in a log viewer.
- Bench-test telemetry delivery on real hardware for both required radio families: a CRSF/ELRS
  radio (verify the new sensor value over CRSF custom telemetry, and the flight-mode text suffix)
  and a FrSky S.Port/FBUS radio (verify the new sensor value decodes correctly on-radio/on a
  companion app). Both must show the oscillation state — the feature isn't done until both pass.
