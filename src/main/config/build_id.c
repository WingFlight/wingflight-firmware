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

#include <stdbool.h>
#include <stdint.h>

#include "config/build_id.h"

const uint8_t wfBuildId[WF_BUILD_ID_LENGTH] = { WF_BUILD_ID_BYTES };

bool wfBuildIdIsValid(void)
{
    for (unsigned i = 0; i < WF_BUILD_ID_LENGTH; i++) {
        if (wfBuildId[i]) {
            return true;
        }
    }
    return false;
}
