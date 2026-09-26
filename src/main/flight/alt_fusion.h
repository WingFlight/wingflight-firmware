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

#pragma once

#include <stdbool.h>

// How long the estimate coasts on the accelerometer alone once the altitude measurement (baro or
// GPS) is gone, before it reports itself invalid.
#define ALT_FUSION_COAST_S          5.0f

typedef struct {
    bool valid;         // altitude/vario are a usable estimate
    float altitude;     // m
    float vario;        // m/s, positive up
    float accBias;      // m/s^2, the accelerometer's vertical error as the filter sees it
    float coastS;       // seconds since the last measurement
} altFusion_t;

void altFusionInit(altFusion_t *fusion);

// One filter step of dt seconds. accUp is the earth-frame vertical acceleration in m/s^2 with
// gravity removed (positive up); measAltitude is the baro or GPS altitude in m. tau is the
// filter's time constant in seconds for this measurement: how long the accelerometer is trusted
// over it.
void altFusionUpdate(altFusion_t *fusion, float dt, bool haveAcc, float accUp, bool haveMeas, float measAltitude, float tau);
