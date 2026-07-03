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

#include "pg/idle_governor.h"

#include "fc/runtime_config.h"
#include "fc/rc_modes.h"

#include "flight/motors.h"
#include "flight/pid.h"
#include "flight/idle_governor.h"

// Max rate of change of the idle governor's own throttle output, in throttle-fraction per second.
// Prevents a step in motor output when the pilot engages/disengages BOXIDLEUP or crosses the handover threshold.
#define IDLE_GOVERNOR_SLEW_RATE 1.0f

float idleGovernorApply(float throttle)
{
    static float governorOutput = 0.0f;

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
            target = constrainf(rpmError * (cfg->idle_governor_gain * 0.0001f), 0.0f, cfg->idle_governor_ceiling / 100.0f);
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
        return throttle;
    }

    governorOutput = slewLimit(governorOutput, target, IDLE_GOVERNOR_SLEW_RATE * pidGetDT());

    return governorOutput;
}
