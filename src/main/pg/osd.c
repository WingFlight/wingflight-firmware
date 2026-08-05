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

#ifdef USE_OSD

#include "pg/pg_ids.h"
#include "pg/osd.h"

extern void pgResetFn_osdConfig(osdConfig_t *osdConfig);

// v9: last version before cms_background_type removal. v10: cms_background_type
// (CMS menu background transparency, only meaningful with the now-removed CMS
// menu) removed -- it sat mid-struct, so stat_show_cell_value shifts into its
// old byte offset. PG version IS bumped so pgLoad() resets this struct to
// defaults on upgrade instead of misreading the shifted byte (same rationale
// as the mixerConfig_t v1->v2 tail_rotor_mode removal).
PG_REGISTER_WITH_RESET_FN(osdConfig_t, osdConfig, PG_OSD_CONFIG, 10);

extern void pgResetFn_osdElementConfig(osdElementConfig_t *osdElementConfig);

PG_REGISTER_WITH_RESET_FN(osdElementConfig_t, osdElementConfig, PG_OSD_ELEMENT_CONFIG, 0);

#endif
