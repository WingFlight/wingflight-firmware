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

#include "flight/failsafe.h"
#include "flight/motors.h"
#include "flight/pid.h"
#include "flight/governor.h"

#include "rx/rx.h"

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
#define GOVERNOR_RPM_LIMIT_SLEW_RATE 5.0f
#define GOVERNOR_RPM_LIMIT_FILTER_HZ 5.0f

float governorApply(float throttle)
{
    static float governorOutput = 0.0f;
    static float integrator = 0.0f;
    static float limitIntegrator = 0.0f;
    static pt1Filter_t rpmErrorFilter;
    static pt1Filter_t rpmLimitErrorFilter;
    static bool rangeFaultLatched = false;

    const governorConfig_t *cfg = governorConfig();

    // RX-loss / switch-induced stage 2 failsafe must be able to cut power outright. The BOXGOVERNOR
    // aux channel defaults to holding its last value through signal loss (RX_FAILSAFE_MODE_HOLD), so
    // a governor switch left engaged before the link dropped would otherwise stay "on" throughout
    // failsafe. Without this check every governor mode would then keep forcing throttle back up to
    // its idle floor (or, for RPM_RANGE, toward governor_rpm_min) regardless of the failsafe-forced
    // low/off throttle input, silently overriding the failsafe throttle cut. Bypass the governor
    // entirely while failsafe is active so the (near-zero) failsafe throttle passes straight through,
    // same as every other flight mode.
    //
    // failsafeIsActive() alone is not enough to catch this on a real dropped link: it only reflects
    // failsafeUpdateState()'s phase machine, which is gated behind failsafeIsMonitoring() --
    // currently permanently false because failsafeStartMonitoring() is a stub (see failsafe.c).
    // rxIsReceivingSignal() is driven straight from rx.c's per-cycle channel validity check
    // (detectAndApplySignalLossBehaviour), independent of that stub, and goes false immediately for
    // both a real signal loss and a BOXFAILSAFE switch -- so it's checked first here. failsafeIsActive()
    // is kept as a second, belt-and-suspenders trigger so this still works once monitoring is restored.
    const bool failsafeActive = !rxIsReceivingSignal() || failsafeIsActive();

    if (!IS_RC_MODE_ACTIVE(BOXGOVERNOR) || failsafeActive) {
        rangeFaultLatched = false;
    }

    // Whenever the governor/idle-up feature is configured *and* a switch is actually assigned to
    // BOXGOVERNOR, that switch is a hard motor interlock, not just something that refines the
    // bottom of the throttle curve: the stick gets no authority at all until it's engaged. Without
    // this, a configured-but-disengaged governor left the stick free to drive the motor straight to
    // full power outside the (tiny, handover-bounded) idle-hold region below -- defeating the point
    // of a dedicated idle-up switch for safe ground arming/starting.
    //
    // The isModeActivationConditionPresent() check (same guard used for BOXTELEMETRY/BOXGPSRESCUE/
    // BOXPREARM elsewhere) is what keeps this from being a footgun: governor_mode is a global gain/
    // curve setup and can legitimately be left at RPM/THROTTLE/RPM_RANGE by someone who never wires
    // up a governor switch at all (e.g. relying on governor_handover alone with the box permanently
    // "on" some boards, or just hasn't gotten to it yet). IS_RC_MODE_ACTIVE() for a box with no
    // assigned range condition is always false, so without this guard those setups would have their
    // motor hard-cut to 0% forever instead of falling back to plain passthrough. Only once a switch
    // is actually assigned does "off" start meaning "interlocked".
    //
    // Failsafe is left alone here: it computes its own (already safe) throttle and must pass through
    // unaltered, same as the bypass below.
    if (!failsafeActive && cfg->governor_mode != GOVERNOR_MODE_OFF &&
        isModeActivationConditionPresent(BOXGOVERNOR) && !IS_RC_MODE_ACTIVE(BOXGOVERNOR)) {
        governorOutput = 0.0f;
        rpmErrorFilter.y1 = 0.0f;
        rpmLimitErrorFilter.y1 = 0.0f;
        integrator = 0.0f;
        limitIntegrator = 0.0f;
        return 0.0f;
    }

    const bool belowHandover = !failsafeActive && ARMING_FLAG(ARMED) &&
        IS_RC_MODE_ACTIVE(BOXGOVERNOR) &&
        throttle < (cfg->governor_handover / 100.0f);

    bool active = false;
    bool fullRange = false;
    bool rpmLimit = false;
    float target = throttle;

    switch (failsafeActive ? GOVERNOR_MODE_OFF : cfg->governor_mode) {
    case GOVERNOR_MODE_RPM:
        active = belowHandover && cfg->governor_rpm > 0 && isMotorRpmSourceActive(0);
        if (active) {
            limitIntegrator = 0.0f;
            rpmLimitErrorFilter.y1 = 0.0f;

            const float rpmError = cfg->governor_rpm - getMotorRPMf(0);
            pt1FilterUpdate(&rpmErrorFilter, GOVERNOR_RPM_FILTER_HZ, 1.0f / pidGetDT());
            const float filteredError = pt1FilterApply(&rpmErrorFilter, rpmError);

            const float ceilingFrac = cfg->governor_ceiling / 100.0f;
            const float idleFloorFrac = constrainf(cfg->governor_throttle / 100.0f, 0.0f, ceilingFrac);
            const float pTerm = filteredError * (cfg->governor_gain * 0.000005f);

            // A pure P term always settles with some droop below the target RPM -- the throttle
            // needed to hold speed is rarely exactly "gain * error". The integrator slowly closes
            // that remaining gap to zero. RPM idle hold starts from governor_throttle as a powered
            // idle floor, then only adds correction above that floor; this keeps the ESC alive
            // during a rapid throttle chop instead of allowing an RPM overshoot to command 0%.
            integrator += filteredError * (cfg->governor_i_gain * 0.000005f) * pidGetDT();
            integrator = constrainf(integrator, 0.0f, ceilingFrac - idleFloorFrac);

            target = constrainf(idleFloorFrac + pTerm + integrator, idleFloorFrac, ceilingFrac);
        } else if (!belowHandover && ARMING_FLAG(ARMED) && IS_RC_MODE_ACTIVE(BOXGOVERNOR) &&
            cfg->governor_rpm_max > 0 && isMotorRpmSourceActive(0)) {
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
        rpmLimitErrorFilter.y1 = 0.0f;
        integrator = 0.0f;
        limitIntegrator = 0.0f;
        return throttle;
    }

    const float slewRate = rpmLimit ? GOVERNOR_RPM_LIMIT_SLEW_RATE :
        (fullRange ? GOVERNOR_RPM_RANGE_SLEW_RATE : GOVERNOR_SLEW_RATE);
    governorOutput = slewLimit(governorOutput, target, slewRate * pidGetDT());

    return governorOutput;
}
