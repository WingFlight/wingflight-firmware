/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Betaflight. If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef USE_GPS_RESCUE

#include "flight/failsafe.h"

#include "gps_rescue.h"

// The multirotor rescue controller that used to live here is gone: it could never run on a wing.
// BOXRTH and the GPS rescue failsafe procedure both fly home with the fixed-wing controller in
// gps_nav.c (RTH_MODE). What remains is the failsafe-procedure check and the gps_rescue_* settings
// (pg/gps_rescue.c), which still supply the GPS minimum satellites and arming-without-fix rules.
bool gpsRescueIsConfigured(void)
{
    return failsafeConfig()->failsafe_procedure == FAILSAFE_PROCEDURE_GPS_RESCUE;
}

#endif
