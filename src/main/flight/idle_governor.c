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

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "common/maths.h"
#include "common/filter.h"

#include "pg/idle_governor.h"

#include "fc/runtime_config.h"
#include "fc/rc_modes.h"

#include "flight/motors.h"
#include "flight/pid.h"
#include "flight/idle_governor.h"

// Max rate of change of the idle governor's own throttle output, in throttle-fraction per second.
// Prevents a step in motor output when the pilot engages/disengages BOXIDLEUP or crosses the handover threshold.
#define IDLE_GOVERNOR_SLEW_RATE 1.0f

// Cutoff for the RPM error fed into the P term (RPM mode only). A prop under little to no
// aerodynamic load (fixed-wing idle, unlike a heli head) has very low inertia and tracks throttle
// changes almost instantly, so an unfiltered P term limit-cycles on ordinary RPM measurement
// noise/quantization long before a usefully corrective gain is reached. Filtering the error first
// trades a bit of response speed -- irrelevant at idle timescales -- for a much larger clean gain
// range.
#define IDLE_GOVERNOR_RPM_FILTER_HZ 2.0f

float idleGovernorApply(float throttle)
{
    static float governorOutput = 0.0f;
    static float integrator = 0.0f;
    static pt1Filter_t rpmErrorFilter;

    const idleGovernorConfig_t *cfg = idleGovernorConfig();

    const bool belowHandover = ARMING_FLAG(ARMED) &&
        IS_RC_MODE_ACTIVE(BOXIDLEUP) &&
        throttle < (cfg->idle_governor_handover / 100.0f);

    bool active = false;
    float target = throttle;

    switch (cfg->idle_governor_mode) {
    case IDLE_GOVERNOR_MODE_RPM:
        active = belowHandover && cfg->idle_governor_rpm > 0 && isMotorRpmSourceActive(0);
        if (active) {
            const float rpmError = cfg->idle_governor_rpm - getMotorRPMf(0);
            pt1FilterUpdate(&rpmErrorFilter, IDLE_GOVERNOR_RPM_FILTER_HZ, 1.0f / pidGetDT());
            const float filteredError = pt1FilterApply(&rpmErrorFilter, rpmError);

            const float ceilingFrac = cfg->idle_governor_ceiling / 100.0f;
            const float pTerm = filteredError * (cfg->idle_governor_gain * 0.000005f);

            // A pure P term always settles with some droop below the target RPM -- the throttle
            // needed to hold speed is rarely exactly "gain * error". The integrator slowly closes
            // that remaining gap to zero. Clamped (not conditional) anti-windup: simple and
            // sufficient for this slow, low-stakes loop.
            integrator += filteredError * (cfg->idle_governor_i_gain * 0.000005f) * pidGetDT();
            integrator = constrainf(integrator, 0.0f, ceilingFrac);

            target = constrainf(pTerm + integrator, 0.0f, ceilingFrac);
        }
        break;

    case IDLE_GOVERNOR_MODE_THROTTLE:
        // No RPM source required -- just holds a fixed throttle output for ESCs/motors without RPM telemetry.
        active = belowHandover;
        if (active) {
            target = constrainf(cfg->idle_governor_throttle / 100.0f, 0.0f, cfg->idle_governor_ceiling / 100.0f);
        }
        break;

    default:
        break;
    }

    if (!active) {
        governorOutput = throttle;
        rpmErrorFilter.y1 = 0.0f;
        integrator = 0.0f;
        return throttle;
    }

    governorOutput = slewLimit(governorOutput, target, IDLE_GOVERNOR_SLEW_RATE * pidGetDT());

    return governorOutput;
}
