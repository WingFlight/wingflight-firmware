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
#include "pg/battery.h"
#include "pg/esc_sensor.h"
#include "pg/gps.h"

// SITL-specific config defaults. NOTE: like every config default, these apply
// on a config RESET only - an existing eeprom.bin keeps whatever it stored
// (delete it, or use wingflight-sitl-hitl tests/sitl-rc-check.ps1 -FreshEeprom, to pick these up).
//
// - GPS provider MSP: there is no serial GPS in SITL; instead
//   wingflight-sitl-hitl sitl/jsbsim_bridge.py --msp-gps feeds JSBSim's position/velocity to the
//   firmware as MSP_SET_RAW_GPS frames, which gps.c only processes when the
//   provider is GPS_MSP.
// - Second MSP port on UART2 (TCP 127.0.0.1:5762): the GPS feed needs its own
//   MSP connection, because SITL's per-port TCP MSP server (dyad) accepts one
//   client at a time and UART1 (5761) is already taken by the RC/telemetry
//   client (wingflight-sitl-hitl joystick_rc.py or sitl-rc-check.ps1).
// - Third MSP port on UART3 (TCP 127.0.0.1:5763), reserved for the Configurator,
//   so it can stay connected alongside the joystick RC and the GPS feed. Three
//   MSP ports is MAX_MSP_PORT_COUNT.
// - FBUS master on UART4 (TCP 127.0.0.1:5764), with ESC telemetry read over it:
//   wingflight-sitl-hitl sitl/jsbsim_bridge.py connects there and answers the
//   master's polls as a FrSky ESC, reporting the simulated pack voltage,
//   current, RPM, consumption and temperature. The battery voltage and current
//   meters use that ESC, since SITL has no ADC. Nothing on the port simply
//   means no telemetry, as with a real FC whose FBUS wire is unplugged.
void targetConfiguration(void)
{
    gpsConfigMutable()->provider = GPS_MSP;

#if defined(USE_FBUS_MASTER) && defined(USE_ESC_SENSOR)
    serialPortConfig_t *fbusPortConfig = serialFindPortConfigurationMutable(SERIAL_PORT_UART4);
    if (fbusPortConfig) {
        fbusPortConfig->functionMask = FUNCTION_FBUS_MASTER;
    }
    // validateAndFixConfig() turns FEATURE_ESC_SENSOR on for this combination.
    escSensorConfigMutable()->protocol = ESC_SENSOR_PROTO_FBUS;
    batteryConfigMutable()->voltageMeterSource = VOLTAGE_METER_ESC;
    batteryConfigMutable()->currentMeterSource = CURRENT_METER_ESC;
#endif

    for (serialPortIdentifier_e id = SERIAL_PORT_USART2; id <= SERIAL_PORT_USART3; id++) {
        serialPortConfig_t *portConfig = serialFindPortConfigurationMutable(id);
        if (portConfig) {
            portConfig->functionMask = FUNCTION_MSP;
        }
    }
}
#endif
