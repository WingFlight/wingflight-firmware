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

#include "platform.h"

#ifdef USE_FBUS_MUX_FPGA

#include "pg/pg_ids.h"
#include "pg/fbus_mux_fpga.h"

// The FPGA is disabled until the board config assigns its pins.
PG_REGISTER_WITH_RESET_TEMPLATE(fbusMuxFpgaConfig_t, fbusMuxFpgaConfig,
                                PG_FBUS_MUX_FPGA_CONFIG, 0);

PG_RESET_TEMPLATE(fbusMuxFpgaConfig_t, fbusMuxFpgaConfig,
    .csTag = IO_TAG_NONE,
    .cresetTag = IO_TAG_NONE,
    .cdoneTag = IO_TAG_NONE,
    .spiDevice = 0,
    .serialPort = 0,
    .mode = { FBUS_MUX_MODE_PWM, FBUS_MUX_MODE_PWM, FBUS_MUX_MODE_PWM, FBUS_MUX_MODE_PWM, FBUS_MUX_MODE_PWM },
);

#endif
