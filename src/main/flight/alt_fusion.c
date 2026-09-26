/*
 * This file is part of Wingflight.
 *
 * Wingflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Wingflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 */

// Vertical inertial estimator: a third-order complementary filter fusing the earth-frame vertical
// acceleration with an altitude measurement (baro, or GPS altitude on boards without one). The
// accelerometer carries the short term, so vario responds with no lag; the measurement pulls the
// altitude, the velocity and an accelerometer bias estimate back in over the time constant tau,
// so neither accelerometer drift nor a constant error (IMU attitude error in a sustained turn,
// calibration) builds up. Same structure as INAV's navigation_pos_estimator.c vertical channel
// and ArduPilot's AP_InertialNav: gains 3/tau, 3/tau^2, 1/tau^3 place all three poles at -1/tau.

#include <stdbool.h>
#include <math.h>

#include "common/maths.h"

#include "flight/alt_fusion.h"

// Limits on what the accelerometer may feed in. A 3D airframe snapping, or an attitude estimate
// lost in a tumble, can briefly put several g of nonsense on the vertical axis; clip it so the
// estimate coasts on plausible values and the measurement pulls it back.
#define ALT_FUSION_MAX_ACC_MSS      30.0f   // about 3 g
#define ALT_FUSION_MAX_BIAS_MSS     2.0f

void altFusionInit(altFusion_t *fusion)
{
    fusion->valid = false;
    fusion->altitude = 0;
    fusion->vario = 0;
    fusion->accBias = 0;
    fusion->coastS = 0;
}

void altFusionUpdate(altFusion_t *fusion, float dt, bool haveAcc, float accUp, bool haveMeas, float measAltitude, float tau)
{
    if (haveMeas) {
        if (!fusion->valid) {
            // First measurement, or back after the estimate had expired: start from it rather
            // than converging from wherever the estimate was left. The bias is kept.
            fusion->altitude = measAltitude;
            fusion->vario = 0;
            fusion->valid = true;
        }
        fusion->coastS = 0;

        const float t = fmaxf(tau, 0.1f);
        const float err = measAltitude - fusion->altitude;

        fusion->altitude += (3.0f / t) * err * dt;
        fusion->vario += (3.0f / sq(t)) * err * dt;
        if (haveAcc) {
            // Measurement above the estimate means the accelerometer has been reading low: the
            // bias it is corrected by goes down.
            fusion->accBias = constrainf(fusion->accBias - (1.0f / (t * t * t)) * err * dt, -ALT_FUSION_MAX_BIAS_MSS, ALT_FUSION_MAX_BIAS_MSS);
        }
    } else {
        fusion->coastS += dt;
        if (!haveAcc || fusion->coastS > ALT_FUSION_COAST_S) {
            fusion->valid = false;
        }
    }

    if (!fusion->valid) {
        // Leave the altitude where it was (telemetry holds the last value rather than jumping to
        // 0) but claim no climb rate.
        fusion->vario = 0;
        return;
    }

    // Without an accelerometer this is a plain second-order filter on the measurement.
    const float acc = haveAcc ? constrainf(accUp, -ALT_FUSION_MAX_ACC_MSS, ALT_FUSION_MAX_ACC_MSS) - fusion->accBias : 0;
    fusion->altitude += fusion->vario * dt + 0.5f * acc * dt * dt;
    fusion->vario += acc * dt;
}
