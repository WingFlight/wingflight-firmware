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

/*
 * The build ID: a hash of the parameter manifest describing this build's
 * config layout. MSP2_WING_BUILD_ID reports it, so a client picks the manifest
 * that matches the board in front of it, and custom defaults carry it, so
 * records compiled against one layout are never applied to another.
 *
 * `make TARGET=<t> manifest` generates the manifest and drops wf_build_id.h
 * into the target's object directory; an ordinary build then picks it up. A
 * build made without that step has no ID (all zero bytes), and a client can
 * tell "not bound to a manifest" from "bound to one I should go and find"
 * rather than chasing a hash of eight zero bytes.
 *
 * Plain `make` therefore still works with no extra step, which matters: the
 * manifest build is a second, non-LTO compile of the whole tree and making
 * every developer pay for it on every build would be a poor trade.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#if defined(__has_include)
#  if __has_include("wf_build_id.h")
#    include "wf_build_id.h"
#  endif
#endif

#ifndef WF_BUILD_ID_BYTES
#define WF_BUILD_ID_BYTES 0, 0, 0, 0, 0, 0, 0, 0
#endif

#define WF_BUILD_ID_LENGTH 8

extern const uint8_t wfBuildId[WF_BUILD_ID_LENGTH];

// False for a build made without a manifest.
bool wfBuildIdIsValid(void);
