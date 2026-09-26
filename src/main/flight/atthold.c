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
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "platform.h"

#ifdef USE_ACC

#include "build/build_config.h"
#include "build/debug.h"

#include "config/config.h"

#include "common/axis.h"

#include "flight/hold_engine.h"

#include "atthold.h"

// ATT HOLD: quaternion-based hold of whatever attitude the aircraft was in when the sticks were
// released -- any orientation, not just level or vertical. Each axis tracks or freezes
// independently based on its own stick, so e.g. pitch/yaw can stay held while aileron alone is
// worked. The algorithm itself (track/freeze, settle-then-capture, stall timeout, quaternion
// error math) lives in the shared hold_engine.c, which the thrust-vector loop's hold (tv_hold.c)
// also runs on its own independent instance -- this file is just the main loop's instance and its
// profile plumbing.

static FAST_DATA_ZERO_INIT quatHold_t attHold;

int get_ADJUSTMENT_ATTHOLD_GAIN(void)
{
    return currentPidProfile->atthold.gain;
}

void set_ADJUSTMENT_ATTHOLD_GAIN(int value)
{
    currentPidProfile->atthold.gain = value;
    quatHoldSetGain(&attHold, value / 10.0f);
}

INIT_CODE void attHoldInit(const pidProfile_t *pidProfile)
{
    quatHoldInit(&attHold, pidProfile->atthold.gain / 10.0f, pidProfile->atthold.deadband / 100.0f,
                 pidProfile->atthold.max_rate);
}

// Called once on the rising edge of ATTHOLD_MODE so a stale target from a previous engagement
// can never linger -- mirrors autoHoverSetState's rising-edge capture.
void attHoldSetState(bool state)
{
    quatHoldSetState(&attHold, state);
}

// True while this axis is actively holding a frozen target (as opposed to free-tracking under
// stick control or settling after a release). pid.c uses this to decide how much I-term decay to
// apply -- see pidApplyMode1.
bool attHoldIsHolding(int axis)
{
    return quatHoldIsHolding(&attHold, axis);
}

// Scale on the normal I-term decay for this axis -- see quatHoldIDecayScale.
float attHoldIDecayScale(int axis)
{
    return quatHoldIDecayScale(&attHold, axis);
}

float attHoldApply(int axis, float pidSetpoint)
{
    if (!attHold.Active) {
        return pidSetpoint;
    }

    const float setpoint = quatHoldApply(&attHold, axis, pidSetpoint);

    DEBUG_AXIS(ATTHOLD, axis, 0, setpoint);

    int16_t holdDebug[QUATHOLD_DEBUG_COUNT];
    quatHoldGetDebug(&attHold, axis, holdDebug);
    for (int i = 0; i < QUATHOLD_DEBUG_COUNT; i++) {
        DEBUG_AXIS(ATTHOLD, axis, 1 + i, holdDebug[i]);
    }

    return setpoint;
}

#endif
