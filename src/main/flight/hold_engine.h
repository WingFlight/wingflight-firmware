/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "flight/imu.h"

// Shared quaternion track/freeze attitude-hold engine -- see hold_engine.c for the algorithm and
// its rationale. Each user (the main loop's ATT HOLD in atthold.c, the thrust-vector loop's hold
// in tv_hold.c) owns its own quatHold_t instance, so the two never share state and can be
// engaged, tuned, and reset independently, but they run the exact same algorithm -- a fix to the
// hold behavior lands in both at once instead of having to be ported by hand.

// Fraction of the normal I-term decay rate (and decay speed limit) applied to an axis that is
// actively holding a frozen target -- see the decay comment in pidApplyMode1. Slow enough that a
// real steady disturbance is still held, but not zero, so unneeded I eventually drains away and
// the surfaces re-center when nothing is actually happening. At the default iterm_decay_time this
// is a time constant of roughly 6 s.
#define QUATHOLD_HOLD_I_DECAY_SCALE 0.1f

typedef struct {
    bool        Active;
    bool        Tracking[3];   // per-axis: true while that axis's target is free-tracking (its own
                                // stick active, or still settling after the stick was released)
    float       SettleTime[3]; // per-axis: seconds since that axis's stick returned inside the deadband
    float       StallTime[3];  // per-axis: seconds a frozen axis has had a large error with no motion
    float       BleedTime[3];  // per-axis: seconds of full-rate I decay left after a stall re-capture
    float       ErrorDeg[3];   // per-axis: attitude error last loop, degrees (before the pre-airborne scaling)
    uint16_t    RecaptureCount; // stall re-captures since the mode engaged, all axes
    int8_t      LastRecaptureAxis; // axis of the most recent stall re-capture, -1 if none yet
    float       Gain;          // deg/s of correction rate per degree of attitude error
    float       Deadband;      // fraction (0..1) of stick deflection that keeps an axis tracking
    float       MaxRate;       // deg/s clamp on the commanded correction rate (safety limit)
    float       Rate[3];       // cached per-axis correction rate, refreshed once per PID iteration
    quaternion  qTarget;
} quatHold_t;

// gain is already scaled to deg/s per degree; deadband is a fraction (constrained to 0..1 here).
void quatHoldInit(quatHold_t *hold, float gain, float deadband, float maxRate);
void quatHoldSetGain(quatHold_t *hold, float gain);

// Call every PID loop with the desired engaged state; captures a fresh target on the rising edge.
void quatHoldSetState(quatHold_t *hold, bool state);

// True while this axis is actively holding a frozen target, as opposed to free-tracking under
// stick control or settling after a release.
bool quatHoldIsHolding(const quatHold_t *hold, int axis);

// Scale on the normal I-term decay rate/limit for this axis: QUATHOLD_HOLD_I_DECAY_SCALE while it
// is holding, 1.0 otherwise -- and also 1.0 for a short window after a stall re-capture, so the I
// that wound up while the aircraft was pinned drains in a couple of seconds instead of ~15 s.
float quatHoldIDecayScale(const quatHold_t *hold, int axis);

// Call once per axis per PID loop, roll first (the shared work is done on the first axis touched).
// Returns the pilot's setpoint unchanged while the axis is tracking, or the correction rate while
// it is holding.
float quatHoldApply(quatHold_t *hold, int axis, float pidSetpoint);

// Blackbox debug values, one axis's view of the engine's state. Callers put them in debug[1..6]
// (debug[0] is the setpoint/rate), in this order:
//   0 error, degrees x10        1 stall timer, ms (a re-capture fires at HOLD_STALL_TIME_S)
//   2 bleed timer, ms           3 stall re-capture count, all axes (wraps at 32768)
//   4 last re-captured axis, -1 none    5 tracking bitmask (bit 0 roll, 1 pitch, 2 yaw)
// The re-capture is otherwise silent to the pilot, so the count stepping is the tell-tale that a
// hold gave up on an axis.
#define QUATHOLD_DEBUG_COUNT 6
void quatHoldGetDebug(const quatHold_t *hold, int axis, int16_t out[QUATHOLD_DEBUG_COUNT]);
