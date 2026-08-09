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

#include "types.h"
#include "platform.h"

#include "pg/pilot.h"

// v1: last version before displayName removal (OSD-only field -- shown as
// an alternate OSD craft-name element, no other consumer). Removing it
// shifts modelId and everything after into its old byte offset, so the PG
// version IS bumped (matching the mixerConfig_t v1->v2 precedent) to force
// pgLoad() to reset to defaults on upgrade instead of misreading the
// shifted bytes.
PG_REGISTER_WITH_RESET_TEMPLATE(pilotConfig_t, pilotConfig, PG_PILOT_CONFIG, 2);

PG_RESET_TEMPLATE(pilotConfig_t, pilotConfig,
    .name = INIT_ZERO,
    .modelId = 0,
);
