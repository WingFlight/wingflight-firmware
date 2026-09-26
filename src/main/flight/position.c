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

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <limits.h>
#include <math.h>

#include "platform.h"

#include "build/debug.h"

#include "common/maths.h"
#include "common/filter.h"
#include "common/utils.h"

#include "fc/runtime_config.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/position.h"

#include "io/gps.h"

#include "sensors/sensors.h"
#include "sensors/barometer.h"
#include "sensors/acceleration.h"

#include "flight/alt_fusion.h"
#include "flight/imu.h"
#include "flight/pid.h"
#include "flight/position.h"

#define GRAVITY_MSS     9.80665f


typedef struct {

    uint8_t     source;

    float       altitude;
    float       variometer;
    bool        altitudeValid;

    bool        haveBaroAlt;
    bool        haveGpsAlt;

    float       baroAlt;
    float       baroAltOffset;
    bool        haveBaroAltOffset;

    float       gpsAlt;
    float       gpsAltOffset;
    bool        haveGpsAltOffset;

    float       accUp;          // earth-frame vertical acceleration, gravity removed, m/s^2
    altFusion_t fusion;

    difFilter_t varioFilter;

    filter_t    gpsFilter;
    filter_t    baroFilter;

    filter_t    gpsOffsetFilter;
    filter_t    baroOffsetFilter;

} altState_t;

static FAST_DATA altState_t alt;


float getAltitude(void)
{
    return alt.altitude;
}

float getVario(void)
{
    return alt.variometer;
}

int getEstimatedAltitudeCm(void)
{
    return lrintf(alt.altitude * 100);
}

int getEstimatedVarioCms(void)
{
    return lrintf(alt.variometer * 100);
}

// Whether getAltitude()/getEstimatedAltitudeCm() is a real estimate. Without a baro, or a GPS
// altitude that passes position_gps_min_sats with its ground offset recorded -- and, with
// accelerometer fusion, once the fusion has coasted past ALT_FUSION_COAST_S without either --
// there is nothing behind it, which an altitude controller would read as a real altitude.
bool hasEstimatedAltitude(void)
{
    return alt.altitudeValid;
}


static float calculateVario(float altitude)
{
    return difFilterApply(&alt.varioFilter, altitude);
}

// Earth-frame vertical acceleration with gravity removed, positive up, in m/s^2. rMat's bottom
// row is the earth Z axis in body coordinates (the same vector the Mahony update compares the
// accelerometer against), so its dot product with the body accelerometer is the earth-frame
// vertical specific force: +1 g at rest.
static bool calculateAccUp(float *accUp)
{
#ifdef USE_ACC
    if (sensors(SENSOR_ACC) && acc.isAccelUpdatedAtLeastOnce) {
        const float accZ = rMat[2][0] * acc.accADC[X] + rMat[2][1] * acc.accADC[Y] + rMat[2][2] * acc.accADC[Z];
        *accUp = (accZ * acc.dev.acc_1G_rec - 1.0f) * GRAVITY_MSS;
        return true;
    }
#endif
    UNUSED(accUp);
    return false;
}

void positionUpdate(void)
{
#ifdef USE_BARO
    if (alt.source == ALT_SOURCE_DEFAULT || alt.source == ALT_SOURCE_BARO_ONLY) {
        if (sensors(SENSOR_BARO) && baroIsReady()) {
            if (baro.baroAltitude < 15000 && baro.baroAltitude > -2500) {
                alt.baroAlt = filterApply(&alt.baroFilter, baro.baroAltitude / 100.0f);
                alt.haveBaroAlt = true;
            }
        }
        else {
            alt.haveBaroAlt = false;
        }
    }
#endif

#ifdef USE_GPS
    // ALT_SOURCE_DEFAULT is 0, so this used to be `alt.source & ALT_SOURCE_DEFAULT` -- always
    // false -- and a board without a baro never had an altitude on the default source.
    if (alt.source == ALT_SOURCE_DEFAULT || alt.source == ALT_SOURCE_GPS_ONLY) {
        if (sensors(SENSOR_GPS) && STATE(GPS_FIX) && gpsSol.numSat >= positionConfig()->gps_min_sats) {
            alt.gpsAlt = filterApply(&alt.gpsFilter, gpsSol.llh.altCm / 100.0f);
            alt.haveGpsAlt = true;
        }
        else {
            alt.haveGpsAlt = false;
        }
    }
#endif

    // Offsets are flagged explicitly: this used to test the offset itself against 0.0, and a baro
    // zeroed at calibration can average to exactly that on the ground.
    if (!ARMING_FLAG(ARMED)) {
        if (alt.haveBaroAlt) {
            alt.baroAltOffset = filterApply(&alt.baroOffsetFilter, alt.baroAlt);
            alt.haveBaroAltOffset = true;
        }
        if (alt.haveGpsAlt) {
            alt.gpsAltOffset = filterApply(&alt.gpsOffsetFilter, alt.gpsAlt);
            alt.haveGpsAltOffset = true;
        }
    }
    else {
        if (alt.haveBaroAlt && alt.haveBaroAltOffset && alt.haveGpsAlt && alt.haveGpsAltOffset) {
            alt.baroAltOffset = filterApply(&alt.baroOffsetFilter,
                alt.baroAlt - (alt.gpsAlt - alt.gpsAltOffset));
        }
    }

    bool haveMeas = false;
    bool measIsBaro = false;
    float measAltitude = 0;
    float measVario = 0;

    if (alt.haveBaroAlt && alt.haveBaroAltOffset) {
        haveMeas = true;
        measIsBaro = true;
        measAltitude = alt.baroAlt - alt.baroAltOffset;
        measVario = calculateVario(alt.baroAlt);
    }
    else if (alt.haveGpsAlt && alt.haveGpsAltOffset) {
        haveMeas = true;
        measAltitude = alt.gpsAlt - alt.gpsAltOffset;
        measVario = calculateVario(alt.gpsAlt);
    }

    const bool haveAcc = calculateAccUp(&alt.accUp);

    if (haveAcc) {
        const float tau = (measIsBaro ? positionConfig()->fusion_baro_tc : positionConfig()->fusion_gps_tc) / 10.0f;
        altFusionUpdate(&alt.fusion, pidGetDT(), true, alt.accUp, haveMeas, measAltitude, tau);
        alt.altitude = alt.fusion.altitude;
        alt.variometer = alt.fusion.vario;
        alt.altitudeValid = alt.fusion.valid;
    }
    else {
        alt.altitude = measAltitude;
        alt.variometer = measVario;
        alt.altitudeValid = haveMeas;
    }

    DEBUG(ALTITUDE, 0, alt.altitude * 100);
    DEBUG(ALTITUDE, 1, alt.variometer * 100);
    DEBUG(ALTITUDE, 2, measAltitude * 100);
    DEBUG(ALTITUDE, 3, measVario * 100);
    DEBUG(ALTITUDE, 4, alt.baroAlt * 100);
    DEBUG(ALTITUDE, 5, alt.gpsAlt * 100);
    DEBUG(ALTITUDE, 6, alt.accUp * 100);
    DEBUG(ALTITUDE, 7, alt.fusion.accBias * 100);
}

void INIT_CODE positionInit(void)
{
    alt.source = positionConfig()->alt_source;

    altFusionInit(&alt.fusion);

    difFilterInit(&alt.varioFilter, positionConfig()->vario_lpf / 100.0f, pidGetPidFrequency());

    lowpassFilterInit(&alt.gpsFilter, LPF_PT2, positionConfig()->gps_alt_lpf / 100.0f, pidGetPidFrequency(), LPF_EWMA);
    lowpassFilterInit(&alt.baroFilter, LPF_PT2, positionConfig()->baro_alt_lpf / 100.0f, pidGetPidFrequency(), LPF_EWMA);

    lowpassFilterInit(&alt.gpsOffsetFilter, LPF_PT2, positionConfig()->gps_offset_lpf / 1000.0f, pidGetPidFrequency(), LPF_EWMA);
    lowpassFilterInit(&alt.baroOffsetFilter, LPF_PT2, positionConfig()->baro_offset_lpf / 1000.0f, pidGetPidFrequency(), LPF_EWMA);
}
