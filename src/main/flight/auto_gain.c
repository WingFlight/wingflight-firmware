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

// Auto gain: per-axis automatic gain control. When the rate loop oscillates (too much gain
// for the current airspeed or prop wash) the P and D feedback of that axis is scaled down
// until the oscillation stops, then eased back up. I, F and B are not scaled, so trim and
// stick authority stay as they are.
//
// Detection looks at the feedback itself, P + D in surface travel, high-passed. An
// oscillating loop drives the surface from one side to the other at a steady rhythm:
// AUTO_GAIN_SWINGS swings in a row, each past the threshold and each 20..250 ms after the
// last. Every further swing multiplies the scale by AUTO_GAIN_ATTACK, down to the configured
// minimum. Because the scaled feedback is what is measured, an oscillation that does not come
// from the loop (flutter, a commanded wag) stops being counted once the feedback has dropped
// below the threshold, instead of taking the gain to the floor.
//
// Swings while the pilot works the stick are commanded, not oscillation, so they are not
// counted. After AUTO_GAIN_HOLD_TIME_S without a cut the scale recovers at the configured
// rate. Where the cause is still there (a long dive) it finds the limit again: a few swings,
// a small cut, another hold.
//
// An axis runs auto gain when its auto_gain setting is on, or while the AUTO GAIN mode switch
// (BOXAUTOGAIN) is on, which turns it on for all three axes with the profile's settings.
//
// The scale is 1.0 before takeoff and in PASSTHROUGH/MANUAL, where the feedback does not
// reach the surfaces.

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "platform.h"

#include "common/filter.h"
#include "common/maths.h"

#include "auto_gain.h"

typedef struct {
    pt1Filter_t feedbackLpf;
    pt1Filter_t stickLpf;
    float feedback;         // High-passed feedback
    float scale;            // Output scale applied to P and D
    float sinceSwing;       // s since the feedback last changed side
    float sinceCut;         // s since the scale was last cut
    float sinceStick;       // s since the stick last moved
    int8_t side;            // Side of the threshold the feedback last swung to, 0 = none yet
    uint8_t swings;         // Swings in a row at an oscillation rhythm
} autoGainAxis_t;

typedef struct {
    bool enabled[PID_AXIS_COUNT];
    bool switchOn;          // AUTO GAIN mode switch
    float min;              // Lowest scale
    float threshold;        // Feedback swing, surface travel (1.0 = full)
    float recovery;         // Scale per second
    autoGainAxis_t axis[PID_AXIS_COUNT];
} autoGain_t;

static FAST_DATA_ZERO_INIT autoGain_t ag;

static void resetAxis(autoGainAxis_t *ax)
{
    ax->feedbackLpf.y1 = 0;
    ax->stickLpf.y1 = 0;
    ax->feedback = 0;
    ax->scale = 1.0f;
    ax->sinceSwing = 0;
    ax->sinceCut = 0;
    ax->sinceStick = 0;
    ax->side = 0;
    ax->swings = 0;
}

void autoGainReset(void)
{
    for (int axis = 0; axis < PID_AXIS_COUNT; axis++)
        resetAxis(&ag.axis[axis]);
}

void autoGainInit(const autoGainConfig_t *config, float sampleRate)
{
    ag.min = constrain(config->min, AUTO_GAIN_MIN_MIN, AUTO_GAIN_MIN_MAX) * 0.01f;
    ag.threshold = constrain(config->threshold, AUTO_GAIN_THRESHOLD_MIN, AUTO_GAIN_THRESHOLD_MAX) * 0.001f;
    ag.recovery = constrain(config->recovery, AUTO_GAIN_RECOVERY_MIN, AUTO_GAIN_RECOVERY_MAX) * 0.01f;

    for (int axis = 0; axis < PID_AXIS_COUNT; axis++) {
        autoGainAxis_t *ax = &ag.axis[axis];

        ag.enabled[axis] = config->enable[axis] != 0;

        // Gain only: the filter state and the scale carry over a profile change
        pt1FilterUpdate(&ax->feedbackLpf, AUTO_GAIN_HPF_CUTOFF_HZ, sampleRate);
        pt1FilterUpdate(&ax->stickLpf, AUTO_GAIN_HPF_CUTOFF_HZ, sampleRate);

        if (!autoGainIsEnabled(axis))
            resetAxis(ax);
        else
            ax->scale = fmaxf(ax->scale, ag.min);
    }
}

void autoGainSetSwitch(bool on)
{
    ag.switchOn = on;
}

void autoGainUpdate(int axis, float feedback, float stick, bool active, float dT)
{
    autoGainAxis_t *ax = &ag.axis[axis];

    if (!autoGainIsEnabled(axis) || !active) {
        resetAxis(ax);
        return;
    }

    ax->feedback = feedback - pt1FilterApply(&ax->feedbackLpf, feedback);

    const float stickMotion = stick - pt1FilterApply(&ax->stickLpf, stick);

    // Timers stop at a value past every limit they are compared with
    ax->sinceStick = (fabsf(stickMotion) > AUTO_GAIN_STICK_MOTION) ? 0 : fminf(ax->sinceStick + dT, AUTO_GAIN_STICK_HOLD_S);
    ax->sinceSwing = fminf(ax->sinceSwing + dT, 2 * AUTO_GAIN_HALF_PERIOD_MAX);
    ax->sinceCut = fminf(ax->sinceCut + dT, AUTO_GAIN_HOLD_TIME_S);

    const bool stickQuiet = ax->sinceStick >= AUTO_GAIN_STICK_HOLD_S;
    const int8_t side = (ax->feedback > ag.threshold) ? 1 : (ax->feedback < -ag.threshold) ? -1 : 0;

    if (side && side != ax->side) {
        const bool rhythm = ax->side &&
            ax->sinceSwing >= AUTO_GAIN_HALF_PERIOD_MIN && ax->sinceSwing <= AUTO_GAIN_HALF_PERIOD_MAX;

        ax->swings = (rhythm && stickQuiet) ? MIN(ax->swings + 1, AUTO_GAIN_SWINGS) : 0;
        ax->side = side;
        ax->sinceSwing = 0;

        if (ax->swings >= AUTO_GAIN_SWINGS) {
            ax->scale = fmaxf(ax->scale * AUTO_GAIN_ATTACK, ag.min);
            ax->sinceCut = 0;
        }
    }
    else if (ax->sinceSwing > AUTO_GAIN_HALF_PERIOD_MAX) {
        // Too slow for an oscillation: the next swing starts a new count
        ax->side = 0;
        ax->swings = 0;
    }

    if (ax->sinceCut >= AUTO_GAIN_HOLD_TIME_S)
        ax->scale = fminf(ax->scale + ag.recovery * dT, 1.0f);
}

bool autoGainIsEnabled(int axis)
{
    return ag.enabled[axis] || ag.switchOn;
}

float autoGainGetScale(int axis)
{
    return ag.axis[axis].scale;
}

float autoGainGetFeedback(int axis)
{
    return ag.axis[axis].feedback;
}
