/*
 * This file is part of Rotorflight.
 *
 * Rotorflight is free software. You can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Rotorflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software. If not, see <https://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "platform.h"

#ifdef USE_ACC

#include "build/build_config.h"
#include "build/debug.h"

#include "config/config.h"

#include "flight/hold_engine.h"

#include "tv_hold.h"

// Independent quaternion-based attitude/heading hold for the Thrust Vector loop
// (FEATURE_THRUST_VECTOR) -- engaged by its own switch (BOXTVHOLD, "THRUST VECTOR
// ATTITUDE HOLD"), completely decoupled from the main loop's ANGLE/AUTOHOVER/
// ATTHOLD chain in pid.c. This is what makes it possible to hold heading/attitude
// on the vectored nozzle while the aerodynamic control surfaces stay in plain
// rate/acro under the pilot's stick.
//
// A second, independent *instance* of the shared hold engine (hold_engine.c) that
// atthold.c also runs -- same "not sharing state" reasoning tv_pid.c documents for
// itself: this hold needs to run (or not) regardless of what the main loop's flight
// mode is doing, and must stay tunable/removable on its own, so it owns its own
// quatHold_t rather than touching atthold's. It shares only the algorithm, so a fix
// to the hold behavior (settle-then-capture, stall timeout, per-axis tracking,
// pre-airborne authority) reaches both loops at once instead of being ported by hand.

static FAST_DATA_ZERO_INIT quatHold_t tvHold;

INIT_CODE void tvHoldInit(const tvPidProfile_t *profile)
{
    quatHoldInit(&tvHold, profile->hold.gain / 10.0f, profile->hold.deadband / 100.0f,
                 profile->hold.max_rate);
}

int get_ADJUSTMENT_TV_HOLD_GAIN(void)
{
    return currentTvPidProfile->hold.gain;
}

void set_ADJUSTMENT_TV_HOLD_GAIN(int value)
{
    currentTvPidProfile->hold.gain = value;
    quatHoldSetGain(&tvHold, value / 10.0f);
}

// Called once on the rising edge of BOXTVHOLD so a stale target from a
// previous engagement can never linger -- mirrors attHoldSetState.
void tvHoldSetState(bool state)
{
    quatHoldSetState(&tvHold, state);
}

// True while this axis is actively holding a frozen target -- tv_pid.c uses this to
// decide how much I-term decay to apply, same as pid.c does for ATT HOLD.
bool tvHoldIsHolding(int axis)
{
    return quatHoldIsHolding(&tvHold, axis);
}

// Scale on the normal I-term decay for this axis -- see quatHoldIDecayScale.
float tvHoldIDecayScale(int axis)
{
    return quatHoldIDecayScale(&tvHold, axis);
}

float tvHoldApply(int axis, float pidSetpoint)
{
    if (!tvHold.Active) {
        return pidSetpoint;
    }

    const float setpoint = quatHoldApply(&tvHold, axis, pidSetpoint);

    DEBUG_AXIS(TVHOLD, axis, 0, setpoint);

    int16_t holdDebug[QUATHOLD_DEBUG_COUNT];
    quatHoldGetDebug(&tvHold, axis, holdDebug);
    for (int i = 0; i < QUATHOLD_DEBUG_COUNT; i++) {
        DEBUG_AXIS(TVHOLD, axis, 1 + i, holdDebug[i]);
    }

    return setpoint;
}

#endif
