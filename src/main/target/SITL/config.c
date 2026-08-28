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

#include <stdint.h>
#include <stdbool.h>

#include "platform.h"

#ifdef USE_TARGET_CONFIG

#include "config/config.h"
#include "io/serial.h"
#include "pg/gps.h"

// SITL-specific config defaults. NOTE: like every config default, these apply
// on a config RESET only - an existing eeprom.bin keeps whatever it stored
// (delete it, or use scripts/sitl-rc-check.ps1 -FreshEeprom, to pick these up).
//
// - GPS provider MSP: there is no serial GPS in SITL; instead
//   scripts/jsbsim_bridge.py --msp-gps feeds JSBSim's position/velocity to the
//   firmware as MSP_SET_RAW_GPS frames, which gps.c only processes when the
//   provider is GPS_MSP.
// - Second MSP port on UART2 (TCP 127.0.0.1:5762): the GPS feed needs its own
//   MSP connection, because SITL's per-port TCP MSP server (dyad) accepts one
//   client at a time and UART1 (5761) is already taken by the RC/telemetry
//   client (sitl-joystick-rc.py or sitl-rc-check.ps1).
void targetConfiguration(void)
{
    gpsConfigMutable()->provider = GPS_MSP;

    serialPortConfig_t *uart2Config = serialFindPortConfigurationMutable(SERIAL_PORT_USART2);
    if (uart2Config) {
        uart2Config->functionMask = FUNCTION_MSP;
    }
}
#endif
