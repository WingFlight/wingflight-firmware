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
#include <string.h>
#include <math.h>

#include "platform.h"

#include "common/axis.h"
#include "common/utils.h"
#include "common/maths.h"

#include "config/config.h"
#include "config/config_reset.h"

#include "pg/adjustments.h"
#include "pg/rates.h"

#include "fc/rc.h"
#include "fc/rc_rates.h"


FAST_DATA_ZERO_INIT controlRateConfig_t * currentControlRateProfile;


/*** Adjustment Functions ***/

int get_ADJUSTMENT_RATE_PROFILE(void)
{
    return getCurrentControlRateProfileIndex() + 1;
}

void set_ADJUSTMENT_RATE_PROFILE(int value)
{
    changeControlRateProfile(value - 1);
}

int get_ADJUSTMENT_PITCH_SRATE(void)
{
    return currentControlRateProfile->sRates[FD_PITCH];
}

void set_ADJUSTMENT_PITCH_SRATE(int value)
{
    currentControlRateProfile->sRates[FD_PITCH] =
        MIN(value, CONTROL_RATE_CONFIG_SUPER_RATE_MAX);
}

int get_ADJUSTMENT_ROLL_SRATE(void)
{
    return currentControlRateProfile->sRates[FD_ROLL];
}

void set_ADJUSTMENT_ROLL_SRATE(int value)
{
    currentControlRateProfile->sRates[FD_ROLL] =
        MIN(value, CONTROL_RATE_CONFIG_SUPER_RATE_MAX);
}

int get_ADJUSTMENT_YAW_SRATE(void)
{
    return currentControlRateProfile->sRates[FD_YAW];
}

void set_ADJUSTMENT_YAW_SRATE(int value)
{
    currentControlRateProfile->sRates[FD_YAW] =
        MIN(value, CONTROL_RATE_CONFIG_SUPER_RATE_MAX);
}

int get_ADJUSTMENT_PITCH_RC_RATE(void)
{
    return currentControlRateProfile->rcRates[FD_PITCH];
}

void set_ADJUSTMENT_PITCH_RC_RATE(int value)
{
    currentControlRateProfile->rcRates[FD_PITCH] =
        MIN(value, CONTROL_RATE_CONFIG_RC_RATES_MAX);
}

int get_ADJUSTMENT_ROLL_RC_RATE(void)
{
    return currentControlRateProfile->rcRates[FD_ROLL];
}

void set_ADJUSTMENT_ROLL_RC_RATE(int value)
{
    currentControlRateProfile->rcRates[FD_ROLL] =
        MIN(value, CONTROL_RATE_CONFIG_RC_RATES_MAX);
}

int get_ADJUSTMENT_YAW_RC_RATE(void)
{
    return currentControlRateProfile->rcRates[FD_YAW];
}

void set_ADJUSTMENT_YAW_RC_RATE(int value)
{
    currentControlRateProfile->rcRates[FD_YAW] =
        MIN(value, CONTROL_RATE_CONFIG_RC_RATES_MAX);
}

int get_ADJUSTMENT_PITCH_RC_EXPO(void)
{
    return currentControlRateProfile->rcExpo[FD_PITCH];
}

void set_ADJUSTMENT_PITCH_RC_EXPO(int value)
{
    currentControlRateProfile->rcExpo[FD_PITCH] =
        MIN(value, CONTROL_RATE_CONFIG_RC_EXPO_MAX);
}

int get_ADJUSTMENT_ROLL_RC_EXPO(void)
{
    return currentControlRateProfile->rcExpo[FD_ROLL];
}

void set_ADJUSTMENT_ROLL_RC_EXPO(int value)
{
    currentControlRateProfile->rcExpo[FD_ROLL] =
        MIN(value, CONTROL_RATE_CONFIG_RC_EXPO_MAX);
}

int get_ADJUSTMENT_YAW_RC_EXPO(void)
{
    return currentControlRateProfile->rcExpo[FD_YAW];
}

void set_ADJUSTMENT_YAW_RC_EXPO(int value)
{
    currentControlRateProfile->rcExpo[FD_YAW] =
        MIN(value, CONTROL_RATE_CONFIG_RC_EXPO_MAX);
}


/*** Rates Curve Function ***/

static float applyWingflightRates(const int axis, const float rcCommandAbs)
{
    float rcRate = currentControlRateProfile->rcRates[axis] * 5;
    float rcExpo = currentControlRateProfile->rcExpo[axis] / 100.0f;
    float shape = currentControlRateProfile->sRates[axis] / 16.0f + 2.0f;

    // Variable order Expo
    float expof = rcCommandAbs * (1.0f - rcExpo) + pow_approx(rcCommandAbs, shape) * rcExpo;

    // Final angle rate
    float angleRate = rcRate * expof;

    return angleRate;
}

float applyRatesCurve(const int axis, const float rcCommandf)
{
    const float rcCommandAbs = fabsf(rcCommandf);

    float rate = applyWingflightRates(axis, rcCommandAbs);

    rate = fminf(rate, SETPOINT_RATE_LIMIT);
    rate = copysignf(rate, rcCommandf);

    return rate;
}


INIT_CODE void loadControlRateProfile(void)
{
    const int profile = systemConfig()->activeRateProfile;

    currentControlRateProfile = controlRateProfilesMutable(profile);

    setpointInitProfile();
}

INIT_CODE void changeControlRateProfile(uint8_t controlRateProfileIndex)
{
    if (controlRateProfileIndex < CONTROL_RATE_PROFILE_COUNT) {
        systemConfigMutable()->activeRateProfile = controlRateProfileIndex;
    }

    loadControlRateProfile();
}

INIT_CODE void copyControlRateProfile(uint8_t dstControlRateProfileIndex, uint8_t srcControlRateProfileIndex)
{
    if (dstControlRateProfileIndex < CONTROL_RATE_PROFILE_COUNT &&
        srcControlRateProfileIndex < CONTROL_RATE_PROFILE_COUNT &&
        dstControlRateProfileIndex != srcControlRateProfileIndex)
    {
        memcpy(controlRateProfilesMutable(dstControlRateProfileIndex), controlRateProfiles(srcControlRateProfileIndex), sizeof(controlRateConfig_t));
    }
}
