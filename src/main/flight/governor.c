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

#include "pg/governor.h"

#include "fc/runtime_config.h"
#include "fc/rc_modes.h"

#include "flight/motors.h"
#include "flight/pid.h"
#include "flight/governor.h"

// Max rate of change of the governor's own throttle output, in throttle-fraction per second, for
// the idle-hold modes below. Prevents a step in motor output when the pilot engages/disengages
// BOXGOVERNOR or crosses the handover threshold.
#define GOVERNOR_SLEW_RATE 1.0f

// Cutoff for the RPM error fed into the P term (RPM mode only). A prop under little to no
// aerodynamic load (fixed-wing idle, unlike a heli head) has very low inertia and tracks throttle
// changes almost instantly, so an unfiltered P term limit-cycles on ordinary RPM measurement
// noise/quantization long before a usefully corrective gain is reached. Filtering the error first
// trades a bit of response speed -- irrelevant at idle timescales -- for a much larger clean gain
// range.
#define GOVERNOR_RPM_FILTER_HZ 2.0f

// RPM_RANGE mode has full throttle authority across a whole flight (e.g. holding a dive-speed
// target), unlike the idle-only modes above, so it needs a loop that can sweep the whole range
// quickly enough to matter -- deliberately much faster than the idle-hold constants, not copied
// from them.
#define GOVERNOR_RPM_RANGE_SLEW_RATE 5.0f
#define GOVERNOR_RPM_RANGE_FILTER_HZ 5.0f

// RPM mode's optional max-RPM limiter is not a full-stick governor. It normally passes the pilot's
// throttle through unchanged and only pulls throttle down while measured RPM exceeds governor_rpm_max.
// Its correction is always bounded to [-throttle, 0], so unlike the idle-hold loop above it can never
// read as an unwanted ESC cut while the pilot holds real stick -- but its integrator/filter state must
// stay live for the whole time this limiter is engaged (armed, above handover, governor_rpm_max set,
// RPM source live), not just on frames where a correction is actually being applied. Wiping them on
// every frame RPM happens to be under the limit would force a cold P-only response each time RPM
// creeps back over it, instead of the smooth I-term unwind the loop already does on its own via
// positive error accumulation.
#define GOVERNOR_RPM_LIMIT_SLEW_RATE 5.0f
#define GOVERNOR_RPM_LIMIT_FILTER_HZ 5.0f

// RPM mode's idle-hold loop learns the throttle fraction that steady-state holds governor_rpm, and
// floors its commanded target there for a short window right after the governor engages. Without
// this, a rapid throttle chop crosses the handover while measured RPM is still near cruise speed,
// the resulting large negative error drives the P term straight to 0% throttle, and the ESC sees a
// hard cut followed by a sharp recovery once RPM decays back through governor_rpm -- exactly the
// kind of step that risks desync. Flooring at the learned value means a chop settles near the
// throttle that was already known to hold the target instead of passing through zero on the way
// there.
//
// The floor is deliberately time-limited (GOVERNOR_RPM_HOLD_FLOOR_DURATION) rather than applied for
// the whole time the governor is active: if it stayed on indefinitely, a stale-too-high estimate
// (from a battery/prop/ESC change, a lowered governor_rpm, etc.) would keep commanding more throttle
// than actually needed, which holds RPM above target forever -- and since the learn gate below only
// fires once RPM is close to target, that same stale value would then never be able to relearn
// downward. Limiting the floor to the initial transient means the unclamped P/I loop always gets the
// last word once the chop has settled, so it keeps converging on the true target regardless of what
// the floor guessed.
//
// GOVERNOR_RPM_HOLD_LEARN_BAND is a fraction of governor_rpm: the estimate only updates while
// measured RPM is close to target, so the transient itself can't corrupt it. Learning is done from
// the loop's own (pre-floor) P/I output, never from the floored value, so the floor can never teach
// itself back a value it originated.
#define GOVERNOR_RPM_HOLD_LEARN_TAU 5.0f
#define GOVERNOR_RPM_HOLD_LEARN_BAND 0.05f
#define GOVERNOR_RPM_HOLD_FLOOR_DURATION 1.5f

float governorApply(float throttle)
{
    static float governorOutput = 0.0f;
    static float integrator = 0.0f;
    static float limitIntegrator = 0.0f;
    static float rpmHoldThrottle = 0.0f;
    static float rpmHoldFloorTimer = 0.0f;
    static pt1Filter_t rpmErrorFilter;
    static pt1Filter_t rpmLimitErrorFilter;
    static bool rangeFaultLatched = false;

    const governorConfig_t *cfg = governorConfig();

    if (!IS_RC_MODE_ACTIVE(BOXGOVERNOR)) {
        rangeFaultLatched = false;
    }

    const bool belowHandover = ARMING_FLAG(ARMED) &&
        IS_RC_MODE_ACTIVE(BOXGOVERNOR) &&
        throttle < (cfg->governor_handover / 100.0f);

    bool active = false;
    bool fullRange = false;
    bool rpmLimit = false;
    bool rpmLimitEngaged = false;
    float target = throttle;

    switch (cfg->governor_mode) {
    case GOVERNOR_MODE_RPM:
        active = belowHandover && cfg->governor_rpm > 0 && isMotorRpmSourceActive(0);
        if (active) {
            limitIntegrator = 0.0f;
            rpmLimitErrorFilter.y1 = 0.0f;
            rpmHoldFloorTimer += pidGetDT();

            const float rpmError = cfg->governor_rpm - getMotorRPMf(0);
            pt1FilterUpdate(&rpmErrorFilter, GOVERNOR_RPM_FILTER_HZ, 1.0f / pidGetDT());
            const float filteredError = pt1FilterApply(&rpmErrorFilter, rpmError);

            const float ceilingFrac = cfg->governor_ceiling / 100.0f;
            const float pTerm = filteredError * (cfg->governor_gain * 0.000005f);

            // A pure P term always settles with some droop below the target RPM -- the throttle
            // needed to hold speed is rarely exactly "gain * error". The integrator slowly closes
            // that remaining gap to zero. Clamped (not conditional) anti-windup: simple and
            // sufficient for this slow, low-stakes loop.
            integrator += filteredError * (cfg->governor_i_gain * 0.000005f) * pidGetDT();
            integrator = constrainf(integrator, 0.0f, ceilingFrac);

            const float piTarget = constrainf(pTerm + integrator, 0.0f, ceilingFrac);

            // Floor at the learned hold-throttle for the initial transient only (see
            // GOVERNOR_RPM_HOLD_FLOOR_DURATION above) so a rapid chop settles near it instead of
            // diving to 0% while the pre-chop RPM is still high -- but let the P/I loop's own,
            // unfloored output win once that window has passed.
            target = (rpmHoldFloorTimer < GOVERNOR_RPM_HOLD_FLOOR_DURATION) ?
                constrainf(MAX(piTarget, rpmHoldThrottle), 0.0f, ceilingFrac) : piTarget;

            // Only learn while settled near governor_rpm, from the loop's own pre-floor output, so
            // neither a chop's transient nor the floor's own influence can feed back into the estimate.
            if (fabsf(filteredError) < cfg->governor_rpm * GOVERNOR_RPM_HOLD_LEARN_BAND) {
                rpmHoldThrottle += (piTarget - rpmHoldThrottle) *
                    MIN(pidGetDT() / GOVERNOR_RPM_HOLD_LEARN_TAU, 1.0f);
            }
        } else if (!belowHandover && ARMING_FLAG(ARMED) && IS_RC_MODE_ACTIVE(BOXGOVERNOR) &&
            cfg->governor_rpm_max > 0 && isMotorRpmSourceActive(0)) {
            rpmLimitEngaged = true;
            rpmErrorFilter.y1 = 0.0f;
            integrator = 0.0f;

            const float rpmError = cfg->governor_rpm_max - getMotorRPMf(0);
            pt1FilterUpdate(&rpmLimitErrorFilter, GOVERNOR_RPM_LIMIT_FILTER_HZ, 1.0f / pidGetDT());
            const float filteredError = pt1FilterApply(&rpmLimitErrorFilter, rpmError);

            const float pTerm = MIN(filteredError * (cfg->governor_gain * 0.000005f), 0.0f);

            // This integrator is intentionally negative-only: it may remove throttle to enforce
            // the configured max RPM, but it must never add throttle or govern to a target speed.
            limitIntegrator += filteredError * (cfg->governor_i_gain * 0.000005f) * pidGetDT();
            limitIntegrator = constrainf(limitIntegrator, -1.0f, 0.0f);

            const float correction = constrainf(pTerm + limitIntegrator, -throttle, 0.0f);
            if (correction < 0.0f) {
                active = true;
                rpmLimit = true;
                target = throttle + correction;
            }
        }
        break;

    case GOVERNOR_MODE_THROTTLE:
        // No RPM source required -- just holds a fixed throttle output for ESCs/motors without RPM telemetry.
        active = belowHandover;
        if (active) {
            target = constrainf(cfg->governor_throttle / 100.0f, 0.0f, cfg->governor_ceiling / 100.0f);
        }
        break;

    case GOVERNOR_MODE_RPM_RANGE:
        // Spans the entire stick (no handover split) -- this mode governs engine speed through a
        // whole flight, not just idle.
        fullRange = true;
        active = ARMING_FLAG(ARMED) && IS_RC_MODE_ACTIVE(BOXGOVERNOR) &&
            cfg->governor_rpm_max > 0 && !rangeFaultLatched;

        if (active && !isMotorRpmSourceActive(0)) {
            // RPM feedback lost while governing -- fail to raw-stick passthrough immediately
            // rather than freezing throttle output, and require the switch to be re-engaged
            // before resuming.
            rangeFaultLatched = true;
            active = false;
        }

        if (active) {
            const float targetRpm = scaleRangef(throttle, 0.0f, 1.0f,
                cfg->governor_rpm_min, cfg->governor_rpm_max);
            const float rpmError = targetRpm - getMotorRPMf(0);
            pt1FilterUpdate(&rpmErrorFilter, GOVERNOR_RPM_RANGE_FILTER_HZ, 1.0f / pidGetDT());
            const float filteredError = pt1FilterApply(&rpmErrorFilter, rpmError);

            const float ceilingFrac = cfg->governor_ceiling / 100.0f;
            const float pTerm = filteredError * (cfg->governor_gain * 0.000005f);

            integrator += filteredError * (cfg->governor_i_gain * 0.000005f) * pidGetDT();
            integrator = constrainf(integrator, 0.0f, ceilingFrac);

            target = constrainf(pTerm + integrator, 0.0f, ceilingFrac);
        }
        break;

    default:
        break;
    }

    if (!active) {
        governorOutput = throttle;
        rpmErrorFilter.y1 = 0.0f;
        integrator = 0.0f;
        rpmHoldFloorTimer = 0.0f;
        // Only reset the max-RPM limiter's own state when the limiter itself isn't engaged this
        // frame -- not merely because no correction was needed. Keeping it live lets the integrator
        // unwind smoothly (via its own positive-error accumulation) instead of hard-resetting every
        // frame RPM sits under governor_rpm_max, which would force a cold P-only response the next
        // time RPM creeps back over it.
        if (!rpmLimitEngaged) {
            rpmLimitErrorFilter.y1 = 0.0f;
            limitIntegrator = 0.0f;
        }
        return throttle;
    }

    const float slewRate = rpmLimit ? GOVERNOR_RPM_LIMIT_SLEW_RATE :
        (fullRange ? GOVERNOR_RPM_RANGE_SLEW_RATE : GOVERNOR_SLEW_RATE);
    governorOutput = slewLimit(governorOutput, target, slewRate * pidGetDT());

    return governorOutput;
}
