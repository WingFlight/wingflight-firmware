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

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef USE_SERVOS

#include "build/build_config.h"

#include "common/time.h"
#include "common/utils.h"

#include "config/config.h"

#include "drivers/time.h"

#include "fc/runtime_config.h"
#include "fc/rc_modes.h"

#include "flight/mixer.h"
#include "flight/servos.h"

#include "pg/servos.h"

#include "autotrim.h"

// Ported from iNav's BOXAUTOTRIM (servos.c): a switch-activated capture, not a continuous
// background feature. While the switch is on and armed, every servo whose output is fed by a
// stabilized axis (roll/pitch/yaw -- i.e. anything the gyro/PID loop can move, regardless of
// which flight mode is active) has its actual, already-mixed output averaged over a fixed
// window, and that average becomes the new center point. Unlike transmitter sub-trim, this
// never touches the RC/gyro "center" reference -- it edits servoParams()->mid, the same
// downstream-of-everything value CLI's `servo` command edits, so the result is identical
// regardless of flight mode.
//
// Turning the switch off before disarming aborts and restores the pre-trim center; only
// disarming while the capture has completed lets it stick (via the normal isConfigDirty()
// disarm-triggered EEPROM write, same as every other live-adjusted value).

#define AUTOTRIM_WINDOW_MS 2000

typedef enum {
    AUTOTRIM_IDLE,
    AUTOTRIM_COLLECTING,
    AUTOTRIM_SAVE_PENDING,
} autoTrimState_e;

typedef struct {
    autoTrimState_e state;
    timeMs_t        startedAt;
    uint16_t        backup[MAX_SUPPORTED_SERVOS];
    uint32_t        accum[MAX_SUPPORTED_SERVOS];
    uint32_t        accumCount[MAX_SUPPORTED_SERVOS];
} autoTrim_t;

static FAST_DATA_ZERO_INIT autoTrim_t autoTrim;

// A servo is only trimmed if some active rule feeds it from a stabilized axis -- rules driven
// by raw RC channels, overrides, or logic conditions alone are left untouched, since those
// aren't part of the gyro-corrected control path this feature is trimming out drift for.
static bool isTrimmableServo(int servo)
{
    for (int i = 0; i < MIXER_RULE_COUNT; i++) {
        const mixerRule_t *rule = mixerRules(i);

        if (rule->oper &&
            rule->output == MIXER_SERVO_OFFSET + servo &&
            rule->input >= MIXER_IN_STABILIZED_ROLL && rule->input <= MIXER_IN_STABILIZED_YAW) {
            return true;
        }
    }

    return false;
}

void autoTrimUpdate(void)
{
    const bool switchOn = IS_RC_MODE_ACTIVE(BOXAUTOTRIM);
    const uint8_t servoCount = getServoCount();

    switch (autoTrim.state) {
        case AUTOTRIM_IDLE:
            if (switchOn && ARMING_FLAG(ARMED)) {
                for (int s = 0; s < servoCount; s++) {
                    if (isTrimmableServo(s)) {
                        autoTrim.backup[s] = servoParams(s)->mid;
                        autoTrim.accum[s] = 0;
                        autoTrim.accumCount[s] = 0;
                    }
                }
                autoTrim.startedAt = millis();
                autoTrim.state = AUTOTRIM_COLLECTING;
            }
            break;

        case AUTOTRIM_COLLECTING:
            if (!switchOn || !ARMING_FLAG(ARMED)) {
                // Nothing has been written yet at this point -- just abandon the capture.
                autoTrim.state = AUTOTRIM_IDLE;
                break;
            }

            for (int s = 0; s < servoCount; s++) {
                if (isTrimmableServo(s)) {
                    autoTrim.accum[s] += getServoOutput(s);
                    autoTrim.accumCount[s]++;
                }
            }

            if (cmp32(millis(), autoTrim.startedAt) > AUTOTRIM_WINDOW_MS) {
                for (int s = 0; s < servoCount; s++) {
                    if (isTrimmableServo(s) && autoTrim.accumCount[s] > 0) {
                        servoParamsMutable(s)->mid = autoTrim.accum[s] / autoTrim.accumCount[s];
                    }
                }
                setConfigDirty();
                autoTrim.state = AUTOTRIM_SAVE_PENDING;
            }
            break;

        case AUTOTRIM_SAVE_PENDING:
            if (!switchOn) {
                // Pilot changed their mind before disarming -- revert to the pre-trim center.
                for (int s = 0; s < servoCount; s++) {
                    if (isTrimmableServo(s)) {
                        servoParamsMutable(s)->mid = autoTrim.backup[s];
                    }
                }
                setConfigDirty();
                autoTrim.state = AUTOTRIM_IDLE;
            }
            else if (!ARMING_FLAG(ARMED)) {
                // Landed with the new center committed -- the normal disarm-triggered EEPROM
                // write (isConfigDirty()) takes it from here.
                autoTrim.state = AUTOTRIM_IDLE;
            }
            break;
    }
}

#endif
