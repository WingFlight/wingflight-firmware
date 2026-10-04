/*
 * This file is part of Wingflight.
 *
 * Wingflight is free software. You can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Wingflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software. If not, see <https://www.gnu.org/licenses/>.
 */

// GPS speed attenuation (SPA): scales P, D, F and B by GPS speed, alongside the
// throttle-based TPA. Dynamic pressure over the surfaces grows with the square
// of airspeed, so a tune that is right at cruise can oscillate in a fast dive.
// TPA cannot see that (the throttle is often closed in a dive); GPS speed can.
//
// Shape mirrors fw_tpa_gain/fw_tpa_curve: a baseline gain times a curve from
// the shared gain-curve pool, here evaluated at speed / speed_max instead of
// throttle. With no curve assigned SPA is off and the scale is exactly 1.0.
//
// GPS is ground-referenced, not airspeed, so wind shifts it. 3D speed is used
// where the receiver reports it so vertical dives count.
//
// Fix loss: the last scale is held for FW_SPA_HOLD_TIME_S, then eased back to
// 1.0 at FW_SPA_SLEW_RATE. Holding avoids a gain jump at high speed on a brief
// dropout; easing back avoids flying on a stale low gain indefinitely. When
// SPA engages (first fix, fix regained, profile change) the scale is eased onto
// the curve at the same rate instead of stepping to it.

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "platform.h"

#include "common/filter.h"
#include "common/maths.h"

#include "fc/runtime_config.h"

#include "flight/pid.h"

#include "io/gps.h"

#include "sensors/sensors.h"

#include "speed_atten.h"

typedef struct {
    float gain;           // Baseline scale (1.0 = unscaled)
    uint8_t curveIndex;   // 0 = SPA off
    float speedMax;       // km/h at curve x = 1000

    pt1Filter_t speedFilter;
    float speed;          // Filtered speed, km/h
    float scale;          // Output scale applied to P, D, F and B
    float lostTime;       // Seconds since the fix was lost
    bool haveSpeed;       // speedFilter holds a real speed
    bool tracking;        // scale follows the curve directly (not easing onto it)
} speedAtten_t;

static FAST_DATA_ZERO_INIT speedAtten_t spa;

void speedAttenReset(void)
{
    spa.scale = 1.0f;
    spa.speed = 0;
    spa.lostTime = 0;
    spa.haveSpeed = false;
    spa.tracking = false;
}

void speedAttenInit(const fwSpaConfig_t *config, float sampleRate)
{
    spa.gain = config->gain * 0.01f;
    spa.curveIndex = (config->curve <= GAIN_CURVE_COUNT) ? config->curve : 0;
    spa.speedMax = constrain(config->speed_max, FW_SPA_SPEED_MAX_MIN, FW_SPA_SPEED_MAX_MAX);

    pt1FilterInit(&spa.speedFilter, FW_SPA_SPEED_CUTOFF_HZ, sampleRate);
    spa.speedFilter.y1 = spa.speed;

    // A profile change can move the curve under us; ease onto the new one
    spa.tracking = false;
}

static float approach(float value, float target, float step)
{
    if (value < target)
        return fminf(value + step, target);
    return fmaxf(value - step, target);
}

void speedAttenUpdateFrom(bool haveFix, float speedKmh, float dT)
{
    if (!spa.curveIndex) {
        spa.scale = 1.0f;
        spa.tracking = false;
        return;
    }

    const float step = FW_SPA_SLEW_RATE * dT;

    if (haveFix) {
        // Start the filter at the first reading so it doesn't climb from zero
        if (!spa.haveSpeed) {
            spa.speedFilter.y1 = speedKmh;
            spa.haveSpeed = true;
        }
        spa.speed = pt1FilterApply(&spa.speedFilter, speedKmh);
        spa.lostTime = 0;

        const float target = spa.gain * pidGetGainCurveScaleAt(spa.curveIndex, spa.speed / spa.speedMax);

        if (spa.tracking) {
            spa.scale = target;
        } else {
            spa.scale = approach(spa.scale, target, step);
            spa.tracking = (spa.scale == target);
        }
    } else {
        spa.tracking = false;
        spa.haveSpeed = false;
        spa.lostTime = fminf(spa.lostTime + dT, FW_SPA_HOLD_TIME_S);

        if (spa.lostTime >= FW_SPA_HOLD_TIME_S)
            spa.scale = approach(spa.scale, 1.0f, step);
    }
}

void speedAttenUpdate(float dT)
{
#ifdef USE_GPS
    const bool haveFix = sensors(SENSOR_GPS) && STATE(GPS_FIX);
    // Only u-blox reports 3D speed; other providers leave it at 0, and 3D
    // speed is never below ground speed, so the larger one is the best estimate.
    const float speedCmS = MAX(gpsSol.speed3d, gpsSol.groundSpeed);

    speedAttenUpdateFrom(haveFix, speedCmS * 0.036f, dT);
#else
    speedAttenUpdateFrom(false, 0, dT);
#endif
}

bool speedAttenIsEnabled(void)
{
    return spa.curveIndex != 0;
}

float speedAttenGetScale(void)
{
    return spa.scale;
}

float speedAttenGetSpeed(void)
{
    return spa.speed;
}
