/*
 * This file is part of Cleanflight, Betaflight and Wingflight.
 *
 * Cleanflight, Betaflight and Wingflight are free software. You can
 * redistribute this software and/or modify this software under the terms
 * of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any
 * later version.
 *
 * Cleanflight, Betaflight and Wingflight are distributed in the hope that
 * they will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * The MSP config catalogue: every opcode that only reads or writes stored
 * configuration, moved here from msp.c unchanged (git log --follow msp.c for
 * their history). Step 5 of docs/parameter-addressing-design.md deletes this
 * file; the clients answer these opcodes themselves from
 * MSP2_WING_PARAM_READ / PARAM_WRITE, using codecs the manifest build
 * extracts from this very file (src/utils/wf_msp_codecs.py).
 *
 * What stays in msp.c: the frozen subset (§8.1), actions and live state,
 * and config opcodes whose bytes depend on runtime state
 * (docs/msp-opcode-classification.md). msp.c's dispatchers hand an opcode
 * they do not handle to the dispatcher here that matches them.
 *
 * Keep this file to config: an opcode that needs anything but its parameter
 * groups belongs in msp.c.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <limits.h>
#include <ctype.h>

#include "platform.h"

#include "blackbox/blackbox.h"
#include "blackbox/blackbox_io.h"

#include "build/build_config.h"
#include "build/debug.h"
#include "build/version.h"


#include "common/axis.h"
#include "common/bitarray.h"
#include "common/color.h"
#include "common/huffman.h"
#include "common/maths.h"
#include "common/printf.h"
#include "common/streambuf.h"
#include "common/utils.h"

#include "config/config.h"
#include "config/config_eeprom.h"
#include "config/feature.h"

#include "drivers/accgyro/accgyro.h"
#include "drivers/bus_i2c.h"
#include "drivers/bus_spi.h"
#include "drivers/compass/compass.h"
#include "drivers/display.h"
#include "drivers/dshot.h"
#include "drivers/dshot_command.h"
#include "drivers/fbus_master.h"
#include "drivers/rx_input_backup.h"
#include "drivers/fbus_sensor.h"
#include "drivers/fbus_xact.h"
#include "drivers/crsf_sensors.h"
#include "drivers/flash.h"
#include "drivers/io.h"
#include "drivers/motor.h"
#include "drivers/pwm_output.h"
#include "drivers/sdcard.h"
#include "drivers/serial.h"
#include "drivers/serial_escserial.h"
#include "drivers/system.h"
#include "drivers/usb_msc.h"
#include "drivers/freq.h"

#include "fc/board_info.h"
#include "fc/rc_rates.h"
#include "fc/core.h"
#include "fc/dispatch.h"
#include "fc/rc.h"
#include "fc/rc_adjustments.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "flight/failsafe.h"
#include "flight/gps_rescue.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/logic_condition.h"
#include "flight/pid.h"
#include "flight/tv_hold.h"
#include "flight/tv_pid.h"
#include "flight/position.h"
#include "flight/rpm_filter.h"
#include "flight/servos.h"

#include "io/asyncfatfs/asyncfatfs.h"
#include "io/beeper.h"
#include "io/flashfs.h"
#include "io/gps.h"
#include "io/ledstrip.h"
#include "io/serial.h"
#include "io/serial_4way.h"
#include "io/servos.h"
#include "io/usb_msc.h"

#include "msp/msp_box.h"
#include "msp/msp_param.h"
#include "msp/msp_runtime.h"
#include "msp/msp_protocol.h"
#include "msp/msp_protocol_v2_betaflight.h"
#include "msp/msp_protocol_v2_rotorflight.h"
#include "msp/msp_protocol_v2_common.h"
#include "msp/msp_serial.h"

#include "pg/beeper.h"
#include "pg/board.h"
#include "pg/dyn_notch.h"
#include "pg/gyrodev.h"
#include "pg/gps_nav.h"
#include "pg/governor.h"
#include "pg/motor.h"
#include "pg/rx.h"
#include "pg/rx_spi.h"
#include "pg/stats.h"
#include "pg/usb.h"
#include "pg/battery.h"
#include "pg/sbus_output.h"
#include "pg/fbus_master.h"
#include "pg/rx_input_backup.h"
#include "pg/bus_servo.h"
#include "pg/logic_condition.h"

#include "rx/rx.h"
#include "rx/rx_bind.h"
#include "rx/msp.h"

#include "scheduler/scheduler.h"

#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/battery.h"
#include "sensors/smartfuel.h"
#include "sensors/boardalignment.h"
#include "sensors/boardalignment_auto.h"
#include "sensors/boardmounttrim_auto.h"
#include "sensors/compass.h"
#include "sensors/esc_sensor.h"
#include "sensors/gyro.h"
#include "sensors/gyro_init.h"

#include "telemetry/msp_shared.h"
#include "telemetry/telemetry.h"

#ifdef USE_HARDWARE_REVISION_DETECTION
#include "hardware_revision.h"
#endif

#include "msp.h"
#include "msp/msp_catalogue.h"

// msp.c's mspCommonProcessOutCommand() / mspProcessOutCommand(): true if
// handled.
bool mspCatalogueProcessOutCommand(int16_t cmdMSP, sbuf_t *dst)
{
    switch (cmdMSP) {
    case MSP_FEATURE_CONFIG:
        sbufWriteU32(dst, featureConfig()->enabledFeatures);
        break;

#if defined(USE_BEEPER)
    case MSP_BEEPER_CONFIG:
        sbufWriteU32(dst, beeperConfig()->beeper_off_flags);
        sbufWriteU8(dst, beeperConfig()->dshotBeaconTone);
        sbufWriteU32(dst, beeperConfig()->dshotBeaconOffFlags);
        break;

#endif

    case MSP_VOLTAGE_METER_CONFIG:
        // Number of Voltage meters to follow
        sbufWriteU8(dst, MAX_VOLTAGE_SENSOR_ADC);
        // Voltage meters using ADC sensors
        for (int i = 0; i < MAX_VOLTAGE_SENSOR_ADC; i++) {
            sbufWriteU8(dst, 7);                                        // Frame length
            sbufWriteU8(dst, voltageSensorToMeterMap[i]);               // Meter id
            sbufWriteU8(dst, VOLTAGE_SENSOR_TYPE_ADC);                  // Meter type
            sbufWriteU16(dst, voltageSensorADCConfig(i)->scale);
            sbufWriteU16(dst, voltageSensorADCConfig(i)->divider);
            sbufWriteU8(dst, voltageSensorADCConfig(i)->divmul);
        }
        // Other voltage meter types go here
        break;

    case MSP_CURRENT_METER_CONFIG:
        // Number of Current meters to follow
        sbufWriteU8(dst, MAX_CURRENT_SENSOR_ADC);
        // Current meters using ADC sensors
        for (int i = 0; i < MAX_CURRENT_SENSOR_ADC; i++) {
            sbufWriteU8(dst, 6);                                            // Frame length
            sbufWriteU8(dst, currentSensorToMeterMap[i]);                   // Meter id
            sbufWriteU8(dst, CURRENT_SENSOR_TYPE_ADC);                      // Meter type
            sbufWriteU16(dst, currentSensorADCConfig(i)->scale);
            sbufWriteU16(dst, currentSensorADCConfig(i)->offset);
        }
        // Other current meter types go here
        break;

    case MSP_BATTERY_CONFIG:
        // Legacy fields: values of the active battery profile
        sbufWriteU16(dst, getBatteryCapacity());
        sbufWriteU8(dst, getBatteryProfileCellCount());
        sbufWriteU8(dst, batteryConfig()->voltageMeterSource);
        sbufWriteU8(dst, batteryConfig()->currentMeterSource);
        sbufWriteU16(dst, getBatteryMinCellVoltage());
        sbufWriteU16(dst, getBatteryMaxCellVoltage());
        sbufWriteU16(dst, getBatteryFullCellVoltage());
        sbufWriteU16(dst, getBatteryWarningCellVoltage());
        sbufWriteU8(dst, batteryConfig()->lvcPercentage);
        sbufWriteU8(dst, batteryConfig()->consumptionWarningPercentage);
        // All battery profiles
        for (int i = 0; i < BATTERY_PROFILE_COUNT; i++)
            sbufWriteU16(dst, batteryConfig()->batteryCapacity[i]);
        for (int i = 0; i < BATTERY_PROFILE_COUNT; i++)
            sbufWriteU8(dst, batteryConfig()->batteryCellCount[i]);
        for (int i = 0; i < BATTERY_PROFILE_COUNT; i++)
            sbufWriteU16(dst, batteryConfig()->vbatmincellvoltage[i]);
        for (int i = 0; i < BATTERY_PROFILE_COUNT; i++)
            sbufWriteU16(dst, batteryConfig()->vbatmaxcellvoltage[i]);
        for (int i = 0; i < BATTERY_PROFILE_COUNT; i++)
            sbufWriteU16(dst, batteryConfig()->vbatfullcellvoltage[i]);
        for (int i = 0; i < BATTERY_PROFILE_COUNT; i++)
            sbufWriteU16(dst, batteryConfig()->vbatwarningcellvoltage[i]);
        break;

    case MSP_BATTERY_PROFILE:
        sbufWriteU8(dst, batteryConfig()->batteryProfile); // The active battery profile
        break;

    case MSP_NAME:
        {
            const int nameLen = strlen(pilotConfig()->name);
            for (int i = 0; i < nameLen; i++) {
                sbufWriteU8(dst, pilotConfig()->name[i]);
            }
        }
        break;

    case MSP_PILOT_CONFIG:
        // Introduced in MSP API 12.7
        sbufWriteU8(dst, pilotConfig()->modelId);
        sbufWriteU8(dst, pilotConfig()->modelParam1Type);
        sbufWriteU16(dst, pilotConfig()->modelParam1Value);
        sbufWriteU8(dst, pilotConfig()->modelParam2Type);
        sbufWriteU16(dst, pilotConfig()->modelParam2Value);
        sbufWriteU8(dst, pilotConfig()->modelParam3Type);
        sbufWriteU16(dst, pilotConfig()->modelParam3Value);
        // Introduced in MSP API 12.9
        sbufWriteU32(dst, pilotConfig()->modelFlags);
        break;

    case MSP_FLIGHT_STATS:
        // Introduced in MSP API 12.9
        sbufWriteU32(dst, statsConfig()->stats_total_flights);
        sbufWriteU32(dst, statsConfig()->stats_total_time_s);
        sbufWriteU32(dst, statsConfig()->stats_total_dist_m);
        sbufWriteS8(dst, statsConfig()->stats_min_armed_time_s);
        break;

#if defined(USE_SMARTFUEL)
    case MSP2_GET_SMARTFUEL_CONFIG:
        sbufWriteU8(dst, batteryConfig()->smartfuel_mode);
        sbufWriteU8(dst, batteryConfig()->smartfuel_voltage_drop_rate);
        sbufWriteU8(dst, batteryConfig()->smartfuel_charge_drop_rate);
        sbufWriteU8(dst, batteryConfig()->smartfuel_sag_gain);
        break;

#endif

    case MSP2_WING_GOVERNOR_CONFIG:
        sbufWriteU8(dst, governorConfig()->governor_mode);
        sbufWriteU16(dst, governorConfig()->governor_rpm);
        sbufWriteU16(dst, governorConfig()->governor_gain);
        sbufWriteU16(dst, governorConfig()->governor_i_gain);
        sbufWriteU8(dst, governorConfig()->governor_throttle);
        sbufWriteU8(dst, governorConfig()->governor_handover);
        sbufWriteU8(dst, governorConfig()->governor_ceiling);
        sbufWriteU16(dst, governorConfig()->governor_rpm_min);
        sbufWriteU16(dst, governorConfig()->governor_rpm_max);
        break;

#if (defined(USE_FBUS_MASTER) || defined(USE_SPORT_MASTER))
    case MSP2_WING_FBUS_MASTER_CONFIG: {
        sbufWriteU8(dst, 1); // payload version -- only the forwarding slots so far
        for (int i = 0; i < FBUS_MASTER_MAX_FORWARDED_SENSORS; i++) {
            sbufWriteU8(dst, fbusMasterConfig()->forwardedSensors[i]);
        }
        break;
    }

#endif

#if defined(USE_RX_INPUT_BACKUP)
    case MSP2_WING_RX_INPUT_BACKUP_CONFIG: {
        // Read-only from the configurator's point of view except via the SET_
        // variant below - reboot is required for a changed provider/inverted/
        // halfDuplex/pinSwap to take effect (rxInputBackupInit() only
        // (re-)opens the port at boot), same as any other serial-port
        // function/config change.
        sbufWriteU8(dst, 1); // payload version
        sbufWriteU8(dst, rxInputBackupConfig()->provider);
        sbufWriteU8(dst, rxInputBackupConfig()->inverted);
        sbufWriteU8(dst, rxInputBackupConfig()->halfDuplex);
        sbufWriteU8(dst, rxInputBackupConfig()->pinSwap);
        break;
    }

#endif

    case MSP_BOARD_ALIGNMENT_CONFIG:
        // Signed: the range is -180..360, and these read back into an int32_t
        // on the other side. Same two bytes on the wire either way, so this
        // changes nothing for an existing client.
        sbufWriteS16(dst, boardAlignment()->rollDegrees);
        sbufWriteS16(dst, boardAlignment()->pitchDegrees);
        sbufWriteS16(dst, boardAlignment()->yawDegrees);
        break;

    case MSP2_WING_BOARD_MOUNT_TRIM:
        sbufWriteS16(dst, boardAlignment()->mountTrim.roll);
        sbufWriteS16(dst, boardAlignment()->mountTrim.pitch);
        sbufWriteS16(dst, boardAlignment()->mountTrim.yaw);
        break;

    case MSP2_WING_TV_PID_CONFIG:
        // Leading byte identifies which of the PID_PROFILE_COUNT TV profiles the
        // rest of this payload describes -- always "currently active", same as
        // MSP_PID_PROFILE for the main PID loop (the configurator/Lua suite select
        // the active profile via MSP2_WING_SELECT_TV_PROFILE, then re-read this).
        sbufWriteU8(dst, getCurrentTvProfileIndex());
        for (int i = 0; i < PID_ITEM_COUNT; i++) {
            sbufWriteU16(dst, currentTvPidProfile->pid[i].P);
            sbufWriteU16(dst, currentTvPidProfile->pid[i].I);
            sbufWriteU16(dst, currentTvPidProfile->pid[i].D);
            sbufWriteU16(dst, currentTvPidProfile->pid[i].F);
            sbufWriteU16(dst, currentTvPidProfile->pid[i].B);
        }
        sbufWriteU16(dst, currentTvPidProfile->master_gain[PID_ROLL]);
        sbufWriteU16(dst, currentTvPidProfile->master_gain[PID_PITCH]);
        sbufWriteU16(dst, currentTvPidProfile->master_gain[PID_YAW]);
        sbufWriteU8(dst, currentTvPidProfile->iterm_decay_time);
        sbufWriteU8(dst, currentTvPidProfile->iterm_decay_limit);
        sbufWriteU8(dst, currentTvPidProfile->iterm_relax_type);
        for (int i = 0; i < PID_AXIS_COUNT; i++) {
            sbufWriteU8(dst, currentTvPidProfile->iterm_relax_level[i]);
        }
        for (int i = 0; i < PID_AXIS_COUNT; i++) {
            sbufWriteU8(dst, currentTvPidProfile->iterm_relax_cutoff[i]);
        }
        for (int i = 0; i < PID_AXIS_COUNT; i++) {
            sbufWriteU8(dst, currentTvPidProfile->error_limit[i]);
        }
        for (int i = 0; i < PID_AXIS_COUNT; i++) {
            sbufWriteU8(dst, currentTvPidProfile->dterm_cutoff[i]);
        }
        for (int i = 0; i < PID_AXIS_COUNT; i++) {
            sbufWriteU8(dst, currentTvPidProfile->bterm_cutoff[i]);
        }
        for (int i = 0; i < PID_AXIS_COUNT; i++) {
            sbufWriteU8(dst, currentTvPidProfile->gyro_cutoff[i]);
        }
        sbufWriteU8(dst, currentTvPidProfile->hold.gain);
        sbufWriteU8(dst, currentTvPidProfile->hold.deadband);
        sbufWriteU16(dst, currentTvPidProfile->hold.max_rate);
        break;

    case MSP_DEBUG_CONFIG:
        sbufWriteU8(dst, DEBUG_COUNT);
        sbufWriteU8(dst, DEBUG_VALUE_COUNT);
        sbufWriteU8(dst, systemConfig()->debug_mode);
        sbufWriteU8(dst, systemConfig()->debug_axis);
        break;

    case MSP_ARMING_CONFIG:
        sbufWriteU8(dst, armingConfig()->auto_disarm_delay);
        sbufWriteU32(dst, armingConfig()->wiggle_flags);
        break;

    case MSP_RC_TUNING:
        for (int i = 0; i < 3; i++) {
            sbufWriteU8(dst, currentControlRateProfile->rcRates[i]);
            sbufWriteU8(dst, currentControlRateProfile->rcExpo[i]);
            sbufWriteU8(dst, currentControlRateProfile->sRates[i]);
            sbufWriteU8(dst, currentControlRateProfile->response_time[i]);
            sbufWriteU16(dst, currentControlRateProfile->accel_limit[i]);
        }
        for (int i = 0; i < 3; i++) {
            sbufWriteU8(dst, currentControlRateProfile->setpoint_boost_gain[i]);
            sbufWriteU8(dst, currentControlRateProfile->setpoint_boost_cutoff[i]);
        }
        sbufWriteU8(dst, currentControlRateProfile->yaw_dynamic_ceiling_gain);
        sbufWriteU8(dst, currentControlRateProfile->yaw_dynamic_deadband_gain);
        sbufWriteU8(dst, currentControlRateProfile->yaw_dynamic_deadband_filter);
        break;

    case MSP_PID_TUNING:
        for (int i = 0; i < PID_AXIS_COUNT; i++) {
            sbufWriteU16(dst, currentPidProfile->pid[i].P);
            sbufWriteU16(dst, currentPidProfile->pid[i].I);
            sbufWriteU16(dst, currentPidProfile->pid[i].D);
            sbufWriteU16(dst, currentPidProfile->pid[i].F);
        }
        for (int i = 0; i < PID_AXIS_COUNT; i++) {
            sbufWriteU16(dst, currentPidProfile->pid[i].B);
        }
        break;

    case MSP_ADJUSTMENT_RANGES:
        for (int i = 0; i < MAX_ADJUSTMENT_RANGE_COUNT; i++) {
            const adjustmentRange_t *adjRange = adjustmentRanges(i);
            sbufWriteU8(dst, adjRange->function);
            sbufWriteU8(dst, adjRange->enaChannel);
            sbufWriteU8(dst, adjRange->enaRange.startStep);
            sbufWriteU8(dst, adjRange->enaRange.endStep);
            sbufWriteU8(dst, adjRange->adjChannel);
            sbufWriteU8(dst, adjRange->adjRange1.startStep);
            sbufWriteU8(dst, adjRange->adjRange1.endStep);
            sbufWriteU8(dst, adjRange->adjRange2.startStep);
            sbufWriteU8(dst, adjRange->adjRange2.endStep);
            sbufWriteU16(dst, adjRange->adjMin);
            sbufWriteU16(dst, adjRange->adjMax);
            sbufWriteU8(dst, adjRange->adjStep);
        }
        break;

    case MSP_GET_ADJUSTMENT_FUNCTION_IDS:
        for (int i = 0; i < MAX_ADJUSTMENT_RANGE_COUNT; i++) {
            const adjustmentRange_t *adjRange = adjustmentRanges(i);
            sbufWriteU8(dst, adjRange->function);
        }
        break;

#if defined(USE_GPS)
    case MSP_GPS_CONFIG:
        sbufWriteU8(dst, gpsConfig()->provider);
        sbufWriteU8(dst, gpsConfig()->sbasMode);
        sbufWriteU8(dst, gpsConfig()->autoConfig);
        sbufWriteU8(dst, gpsConfig()->autoBaud);
        // Added in API version 1.43
        sbufWriteU8(dst, gpsConfig()->gps_set_home_point_once);
        sbufWriteU8(dst, gpsConfig()->gps_ublox_use_galileo);
        break;

#endif

#if defined(USE_GPS) && defined(USE_GPS_RESCUE)
    case MSP_GPS_RESCUE:
        sbufWriteU16(dst, gpsRescueConfig()->angle);
        sbufWriteU16(dst, gpsRescueConfig()->initialAltitudeM);
        sbufWriteU16(dst, gpsRescueConfig()->descentDistanceM);
        sbufWriteU16(dst, gpsRescueConfig()->rescueGroundspeed);
        sbufWriteU16(dst, gpsRescueConfig()->throttleMin);
        sbufWriteU16(dst, gpsRescueConfig()->throttleMax);
        sbufWriteU16(dst, gpsRescueConfig()->throttleHover);
        sbufWriteU8(dst,  gpsRescueConfig()->sanityChecks);
        sbufWriteU8(dst,  gpsRescueConfig()->minSats);
        // Added in API version 1.43
        sbufWriteU16(dst, gpsRescueConfig()->ascendRate);
        sbufWriteU16(dst, gpsRescueConfig()->descendRate);
        sbufWriteU8(dst, gpsRescueConfig()->allowArmingWithoutFix);
        sbufWriteU8(dst, gpsRescueConfig()->altitudeMode);
        // Added in API version 1.44
        sbufWriteU16(dst, gpsRescueConfig()->minRescueDth);
        break;

    case MSP_GPS_RESCUE_PIDS:
        sbufWriteU16(dst, gpsRescueConfig()->throttleP);
        sbufWriteU16(dst, gpsRescueConfig()->throttleI);
        sbufWriteU16(dst, gpsRescueConfig()->throttleD);
        sbufWriteU16(dst, gpsRescueConfig()->velP);
        sbufWriteU16(dst, gpsRescueConfig()->velI);
        sbufWriteU16(dst, gpsRescueConfig()->velD);
        sbufWriteU16(dst, gpsRescueConfig()->yawP);
        break;

#endif

#if (defined(USE_ACC))
    case MSP_ACC_TRIM:
        sbufWriteU16(dst, accelerometerConfig()->accelerometerTrims.values.pitch);
        sbufWriteU16(dst, accelerometerConfig()->accelerometerTrims.values.roll);

        break;

#endif

    case MSP_MIXER_INPUTS:
        for (int i = 0; i < MIXER_INPUT_COUNT; i++) {
          sbufWriteU16(dst, mixerInputs(i)->rate);
          sbufWriteU16(dst, mixerInputs(i)->min);
          sbufWriteU16(dst, mixerInputs(i)->max);
        }
        break;

    case MSP_MIXER_RULES:
        for (int i = 0; i < MIXER_RULE_COUNT; i++) {
          sbufWriteU8(dst, mixerRules(i)->oper);
          sbufWriteU8(dst, mixerRules(i)->input);
          sbufWriteU8(dst, mixerRules(i)->output);
          sbufWriteU16(dst, mixerRules(i)->offset);
          sbufWriteU16(dst, mixerRules(i)->weight);
          sbufWriteU16(dst, mixerRules(i)->weightNeg);
          sbufWriteU16(dst, mixerRules(i)->speed);
          sbufWriteU8(dst, mixerRules(i)->curve);
          sbufWriteU8(dst, mixerRules(i)->condition);
          sbufWriteU8(dst, mixerRules(i)->role);
        }
        break;

    case MSP_MIXER_CURVES:
        for (int i = 0; i < MIXER_CURVE_COUNT; i++) {
            sbufWriteU8(dst, mixerCurves(i)->count);
            for (int p = 0; p < MIXER_CURVE_POINTS; p++) {
                sbufWriteU16(dst, mixerCurves(i)->points[p].x);
                sbufWriteU16(dst, mixerCurves(i)->points[p].y);
            }
        }
        break;

    case MSP_GAIN_CURVES:
        for (int i = 0; i < GAIN_CURVE_COUNT; i++) {
            sbufWriteU8(dst, gainCurves(i)->count);
            for (int p = 0; p < GAIN_CURVE_POINTS; p++) {
                sbufWriteU16(dst, gainCurves(i)->points[p].x);
                sbufWriteU16(dst, gainCurves(i)->points[p].y);
            }
        }
        break;

    case MSP_LOGIC_CONDITIONS:
        for (int i = 0; i < LOGIC_CONDITION_COUNT; i++) {
            sbufWriteU8(dst, logicConditions(i)->enabled);
            sbufWriteU8(dst, logicConditions(i)->operation);
            sbufWriteU8(dst, logicConditions(i)->operandAType);
            sbufWriteU16(dst, logicConditions(i)->operandAValue);
            sbufWriteU8(dst, logicConditions(i)->operandBType);
            sbufWriteU16(dst, logicConditions(i)->operandBValue);
        }
        break;

    case MSP_RX_CONFIG:
        sbufWriteU8(dst, rxConfig()->serialrx_provider);
        sbufWriteU8(dst, rxConfig()->serialrx_inverted);
        sbufWriteU8(dst, rxConfig()->halfDuplex);
        sbufWriteU16(dst, rxConfig()->rx_pulse_min);
        sbufWriteU16(dst, rxConfig()->rx_pulse_max);
#ifdef USE_RX_SPI
        sbufWriteU8(dst, rxSpiConfig()->rx_spi_protocol);
        sbufWriteU32(dst, rxSpiConfig()->rx_spi_id);
        sbufWriteU8(dst, rxSpiConfig()->rx_spi_rf_channel_count);
#else
        sbufWriteU8(dst, 0);
        sbufWriteU32(dst, 0);
        sbufWriteU8(dst, 0);
#endif
        sbufWriteU8(dst, rxConfig()->pinSwap);
        break;

    case MSP_FAILSAFE_CONFIG:
        sbufWriteU8(dst, failsafeConfig()->failsafe_delay);
        sbufWriteU8(dst, failsafeConfig()->failsafe_off_delay);
        sbufWriteU16(dst, failsafeConfig()->failsafe_throttle);
        sbufWriteU8(dst, failsafeConfig()->failsafe_switch_mode);
        sbufWriteU16(dst, failsafeConfig()->failsafe_throttle_low_delay);
        sbufWriteU8(dst, failsafeConfig()->failsafe_procedure);
        // Appended field -- older clients that only read the six bytes above are unaffected.
        sbufWriteU16(dst, failsafeConfig()->failsafe_recovery_delay);
        break;

    case MSP2_WING_GPS_NAV_CONFIG:
        // gpsNavConfig_t (PG_GPS_NAV) -- the fixed-wing BOXRTH/BOXLOITER/GPS-rescue
        // (FAILSAFE_PROCEDURE_GPS_RESCUE) nav controller's tuning, see gps_nav.c.
        // No MSP command existed for this at all before -- CLI-only (nav_* settings).
        sbufWriteU16(dst, gpsNavConfig()->loiterRadiusM);
        sbufWriteU8(dst, gpsNavConfig()->loiterDirection);
        sbufWriteU16(dst, gpsNavConfig()->rthAltitudeM);
        sbufWriteU8(dst, gpsNavConfig()->minSats);
        sbufWriteU8(dst, gpsNavConfig()->maxBankAngleDeg);
        sbufWriteU8(dst, gpsNavConfig()->maxPitchAngleDeg);
        sbufWriteU16(dst, gpsNavConfig()->bearingKp);
        sbufWriteU16(dst, gpsNavConfig()->altitudeKp);
        // Appended fields -- older clients that only read the bytes above are unaffected.
        sbufWriteU16(dst, gpsNavConfig()->altitudeKd);
        sbufWriteU8(dst, gpsNavConfig()->throttle);
        sbufWriteU8(dst, gpsNavConfig()->turnCoordination);
        break;

    case MSP_RSSI_CONFIG:
        sbufWriteU8(dst, rxConfig()->rssi_channel);
        sbufWriteU8(dst, rxConfig()->rssi_scale);
        sbufWriteU8(dst, rxConfig()->rssi_invert);
        sbufWriteU8(dst, rxConfig()->rssi_offset);
        break;

    case MSP_RX_MAP:
        sbufWriteData(dst, rxConfig()->rcmap, RX_MAPPABLE_CHANNEL_COUNT);
        break;

    case MSP_RC_CONFIG:
        sbufWriteU16(dst, rcControlsConfig()->rc_center);
        sbufWriteU16(dst, rcControlsConfig()->rc_deflection);
        sbufWriteU16(dst, rcControlsConfig()->rc_min_throttle);
        sbufWriteU16(dst, rcControlsConfig()->rc_max_throttle);
        sbufWriteU8(dst, rcControlsConfig()->rc_roll_deadband);
        sbufWriteU8(dst, rcControlsConfig()->rc_pitch_deadband);
        sbufWriteU8(dst, rcControlsConfig()->rc_yaw_deadband);
        break;

    case MSP_TELEMETRY_CONFIG:
        sbufWriteU8(dst, telemetryConfig()->telemetry_inverted);
        sbufWriteU8(dst, telemetryConfig()->halfDuplex);
        sbufWriteU8(dst, telemetryConfig()->pinSwap);
        sbufWriteU8(dst, telemetryConfig()->crsf_telemetry_mode);
        sbufWriteU16(dst, telemetryConfig()->crsf_telemetry_link_rate);
        sbufWriteU16(dst, telemetryConfig()->crsf_telemetry_link_ratio);
        for (int i = 0; i < TELEM_SENSOR_SLOT_COUNT; i++) {
            sbufWriteU8(dst, telemetryConfig()->telemetry_sensors[i]);
        }
        break;

#if defined(USE_LED_STRIP_STATUS_MODE)
    case MSP_LED_COLORS:
        for (int i = 0; i < LED_CONFIGURABLE_COLOR_COUNT; i++) {
            const hsvColor_t *color = &ledStripStatusModeConfig()->colors[i];
            sbufWriteU16(dst, color->h);
            sbufWriteU8(dst, color->s);
            sbufWriteU8(dst, color->v);
        }
        break;

#endif

#if defined(USE_LED_STRIP)
    case MSP_LED_STRIP_CONFIG:
        for (int i = 0; i < LED_MAX_STRIP_LENGTH; i++) {
#ifdef USE_LED_STRIP_STATUS_MODE
            const ledConfig_t *ledConfig = &ledStripStatusModeConfig()->ledConfigs[i];
            sbufWriteU64(dst, *ledConfig);
#else
            sbufWriteU64(dst, 0);
#endif
        }

        // API 1.41 - add indicator for advanced profile support and the current profile selection
        // 0 = basic ledstrip available
        // 1 = advanced ledstrip available
#ifdef USE_LED_STRIP_STATUS_MODE
        sbufWriteU8(dst, 1);   // advanced ledstrip available
#else
        sbufWriteU8(dst, 0);   // only simple ledstrip available
#endif
        sbufWriteU8(dst, ledStripConfig()->ledstrip_profile);
        break;

#endif

#if defined(USE_LED_STRIP_STATUS_MODE)
    case MSP_LED_STRIP_MODECOLOR:
        for (int i = 0; i < LED_MODE_COUNT; i++) {
            for (int j = 0; j < LED_DIRECTION_COUNT; j++) {
                sbufWriteU8(dst, i);
                sbufWriteU8(dst, j);
                sbufWriteU8(dst, ledStripStatusModeConfig()->modeColors[i].color[j]);
            }
        }

        for (int j = 0; j < LED_SPECIAL_COLOR_COUNT; j++) {
            sbufWriteU8(dst, LED_MODE_COUNT);
            sbufWriteU8(dst, j);
            sbufWriteU8(dst, ledStripStatusModeConfig()->specialColors.color[j]);
        }

        sbufWriteU8(dst, LED_AUX_CHANNEL);
        sbufWriteU8(dst, 0);
        sbufWriteU8(dst, ledStripStatusModeConfig()->ledstrip_aux_channel);
        break;

#endif

#if defined(USE_LED_STRIP)
    case MSP_LED_STRIP_SETTINGS:
        sbufWriteU8(dst, ledStripConfigMutable()->ledstrip_beacon_armed_only);
        sbufWriteU8(dst, ledStripConfigMutable()->ledstrip_beacon_color);
        sbufWriteU8(dst, ledStripConfigMutable()->ledstrip_beacon_percent);
        sbufWriteU16(dst, ledStripConfigMutable()->ledstrip_beacon_period_ms);
        sbufWriteU16(dst, ledStripConfigMutable()->ledstrip_blink_period_ms);
        sbufWriteU8(dst, ledStripConfigMutable()->ledstrip_brightness);
        sbufWriteU8(dst, ledStripConfigMutable()->ledstrip_fade_rate);
        sbufWriteU8(dst, ledStripConfigMutable()->ledstrip_flicker_rate);
        sbufWriteU8(dst, ledStripConfigMutable()->ledstrip_grb_rgb);
        sbufWriteU8(dst, ledStripConfigMutable()->ledstrip_profile);
        sbufWriteU8(dst, ledStripConfigMutable()->ledstrip_race_color);
        sbufWriteU8(dst, ledStripConfigMutable()->ledstrip_visual_beeper);
        sbufWriteU8(dst, ledStripConfigMutable()->ledstrip_visual_beeper_color);
        break;

#endif

    case MSP_BLACKBOX_CONFIG:
#ifdef USE_BLACKBOX
        sbufWriteU8(dst, 1); // Blackbox supported
        sbufWriteU8(dst, blackboxConfig()->device);
        sbufWriteU8(dst, blackboxConfig()->mode);
        sbufWriteU16(dst, blackboxConfig()->denom);
        sbufWriteU32(dst, blackboxConfig()->fields);
        sbufWriteU16(dst, blackboxConfig()->initialEraseFreeSpaceKiB);
        sbufWriteU8(dst, blackboxConfig()->rollingErase);
        sbufWriteU8(dst, blackboxConfig()->gracePeriod);
#else
        sbufWriteU8(dst, 0); // Blackbox not supported
        sbufWriteU8(dst, 0);
        sbufWriteU8(dst, 0);
        sbufWriteU16(dst, 0);
        sbufWriteU32(dst, 0);
        sbufWriteU16(dst, 0);
        sbufWriteU8(dst, 0);
        sbufWriteU8(dst, 0);
#endif
        break;

    case MSP_SENSOR_ALIGNMENT:
#ifdef USE_MULTI_GYRO
        sbufWriteU8(dst, gyroDeviceConfig(0)->alignment);
        sbufWriteU8(dst, gyroDeviceConfig(1)->alignment);
#else
        sbufWriteU8(dst, gyroDeviceConfig(0)->alignment);
        sbufWriteU8(dst, ALIGN_DEFAULT);
#endif
#if defined(USE_MAG)
        sbufWriteU8(dst, compassConfig()->mag_alignment);
#else
        sbufWriteU8(dst, 0);
#endif
        break;

    case MSP_ADVANCED_CONFIG:
        sbufWriteU8(dst, 1); // compat: gyro denom
        sbufWriteU8(dst, pidConfig()->pid_process_denom);
        sbufWriteU8(dst, motorConfig()->dev.useUnsyncedPwm);
        sbufWriteU8(dst, motorConfig()->dev.motorPwmProtocol);
        sbufWriteU16(dst, motorConfig()->dev.motorPwmRate);
        break;

    case MSP_FILTER_CONFIG:
        sbufWriteU8(dst, gyroConfig()->gyro_hardware_lpf);
        sbufWriteU8(dst, gyroConfig()->gyro_lpf1_type);
        sbufWriteU16(dst, gyroConfig()->gyro_lpf1_static_hz);
        sbufWriteU8(dst, gyroConfig()->gyro_lpf2_type);
        sbufWriteU16(dst, gyroConfig()->gyro_lpf2_static_hz);
        sbufWriteU16(dst, gyroConfig()->gyro_soft_notch_hz_1);
        sbufWriteU16(dst, gyroConfig()->gyro_soft_notch_cutoff_1);
        sbufWriteU16(dst, gyroConfig()->gyro_soft_notch_hz_2);
        sbufWriteU16(dst, gyroConfig()->gyro_soft_notch_cutoff_2);
#if defined(USE_DYN_LPF)
        sbufWriteU16(dst, gyroConfig()->gyro_lpf1_dyn_min_hz);
        sbufWriteU16(dst, gyroConfig()->gyro_lpf1_dyn_max_hz);
#else
        sbufWriteU16(dst, 0);
        sbufWriteU16(dst, 0);
#endif
#if defined(USE_DYN_NOTCH_FILTER)
        sbufWriteU8(dst, dynNotchConfig()->dyn_notch_count);
        sbufWriteU8(dst, dynNotchConfig()->dyn_notch_q);
        sbufWriteU16(dst, dynNotchConfig()->dyn_notch_min_hz);
        sbufWriteU16(dst, dynNotchConfig()->dyn_notch_max_hz);
#else
        sbufWriteU8(dst, 0);
        sbufWriteU8(dst, 0);
        sbufWriteU16(dst, 0);
        sbufWriteU16(dst, 0);
#endif
#if defined(USE_RPM_FILTER)
        sbufWriteU8(dst, rpmFilterConfig()->preset);
        sbufWriteU8(dst, rpmFilterConfig()->min_hz);
#else
        sbufWriteU8(dst, 0);
        sbufWriteU8(dst, 0);
#endif
        break;

    case MSP_PID_PROFILE:
        sbufWriteU8(dst, currentPidProfile->pid_mode);
        sbufWriteU8(dst, currentPidProfile->iterm_decay_time);
        sbufWriteU8(dst, currentPidProfile->iterm_decay_limit);
        sbufWriteU8(dst, currentPidProfile->error_limit[0]);
        sbufWriteU8(dst, currentPidProfile->error_limit[1]);
        sbufWriteU8(dst, currentPidProfile->error_limit[2]);
        sbufWriteU8(dst, currentPidProfile->gyro_cutoff[0]);
        sbufWriteU8(dst, currentPidProfile->gyro_cutoff[1]);
        sbufWriteU8(dst, currentPidProfile->gyro_cutoff[2]);
        sbufWriteU8(dst, currentPidProfile->dterm_cutoff[0]);
        sbufWriteU8(dst, currentPidProfile->dterm_cutoff[1]);
        sbufWriteU8(dst, currentPidProfile->dterm_cutoff[2]);
        sbufWriteU8(dst, currentPidProfile->iterm_relax_type);
        sbufWriteU8(dst, currentPidProfile->iterm_relax_cutoff[0]);
        sbufWriteU8(dst, currentPidProfile->iterm_relax_cutoff[1]);
        sbufWriteU8(dst, currentPidProfile->iterm_relax_cutoff[2]);
        /* Angle mode */
        sbufWriteU8(dst, currentPidProfile->angle.level_strength);
        sbufWriteU8(dst, currentPidProfile->angle.level_limit);
        /* Horizon mode */
        sbufWriteU8(dst, currentPidProfile->horizon.level_strength);
        /* Acro trainer */
        sbufWriteU8(dst, currentPidProfile->trainer.gain);
        sbufWriteU8(dst, currentPidProfile->trainer.angle_limit);
        /* Att Hold */
        sbufWriteU8(dst, currentPidProfile->atthold.gain);
        sbufWriteU8(dst, currentPidProfile->atthold.deadband);
        /* B-term cutoffs */
        sbufWriteU8(dst, currentPidProfile->bterm_cutoff[0]);
        sbufWriteU8(dst, currentPidProfile->bterm_cutoff[1]);
        sbufWriteU8(dst, currentPidProfile->bterm_cutoff[2]);
        /* Fixed-wing throttle-based gain attenuation (gain + curve index) */
        sbufWriteU8(dst, currentPidProfile->fw_tpa_gain);
        sbufWriteU8(dst, currentPidProfile->fw_tpa_curve);
        /* Master gain (per axis) */
        sbufWriteU16(dst, currentPidProfile->master_gain[PID_ROLL]);
        sbufWriteU16(dst, currentPidProfile->master_gain[PID_PITCH]);
        sbufWriteU16(dst, currentPidProfile->master_gain[PID_YAW]);
        /* Auto Hover */
        sbufWriteU8(dst, currentPidProfile->autohover.gain);
        sbufWriteU8(dst, currentPidProfile->autohover.max_angle);
        sbufWriteU16(dst, currentPidProfile->autohover.max_rate);
        /* Cross-axis relax */
        sbufWriteU8(dst, currentPidProfile->cross_axis_relax_strength);
        sbufWriteU8(dst, currentPidProfile->cross_axis_relax_level);
        sbufWriteU8(dst, currentPidProfile->cross_axis_relax_cutoff);
        sbufWriteU8(dst, currentPidProfile->cross_axis_relax_pitch_strength);
        /* Gain curve assignment (per axis) */
        sbufWriteU8(dst, currentPidProfile->gain_curve[PID_ROLL]);
        sbufWriteU8(dst, currentPidProfile->gain_curve[PID_PITCH]);
        sbufWriteU8(dst, currentPidProfile->gain_curve[PID_YAW]);
        /* Att Hold max rate */
        sbufWriteU16(dst, currentPidProfile->atthold.max_rate);
        /* Auto Hover roll deadband */
        sbufWriteU8(dst, currentPidProfile->autohover.roll_deadband);
        /* Auto Hover throttle assist */
        sbufWriteU8(dst, currentPidProfile->autohover.throttle_assist_gain);
        sbufWriteU8(dst, currentPidProfile->autohover.throttle_assist_max);
        sbufWriteU16(dst, currentPidProfile->autohover.throttle_assist_trigger_ms);
        /* API 22.4: optional per-axis attitude limits. Zero inherits the shared limit. */
        sbufWriteU8(dst, attitudeLimits(getCurrentPidProfileIndex())->angle_roll);
        sbufWriteU8(dst, attitudeLimits(getCurrentPidProfileIndex())->angle_pitch);
        sbufWriteU8(dst, attitudeLimits(getCurrentPidProfileIndex())->trainer_roll);
        sbufWriteU8(dst, attitudeLimits(getCurrentPidProfileIndex())->trainer_pitch);
        break;

    case MSP_SENSOR_CONFIG:
#if defined(USE_ACC)
        sbufWriteU8(dst, accelerometerConfig()->acc_hardware);
#else
        sbufWriteU8(dst, 0);
#endif
#ifdef USE_BARO
        sbufWriteU8(dst, barometerConfig()->baro_hardware);
#else
        sbufWriteU8(dst, BARO_NONE);
#endif
#ifdef USE_MAG
        sbufWriteU8(dst, compassConfig()->mag_hardware);
#else
        sbufWriteU8(dst, MAG_NONE);
#endif
        sbufWriteU8(dst, gyroConfig()->gyro_to_use);
        sbufWriteU8(dst, gyroConfig()->gyro_high_fsr);
        sbufWriteU8(dst, gyroConfig()->gyroMovementCalibrationThreshold);
        sbufWriteU16(dst, gyroConfig()->gyroCalibrationDuration);
        sbufWriteU8(dst, gyroConfig()->checkOverflow);
        break;

    default:
        return false;
    }
    return true;
}

// msp.c's mspFcProcessOutCommandWithArg(): replies that read their request.
mspResult_e mspCatalogueProcessOutCommandWithArg(mspDescriptor_t srcDesc, int16_t cmdMSP, sbuf_t *src, sbuf_t *dst)
{
    UNUSED(srcDesc);
    UNUSED(src);
    UNUSED(dst);

    switch (cmdMSP) {
    case MSP_GET_MIXER_INPUT:
        {
            const int rem = sbufBytesRemaining(src);
            if (rem != 1) {
                return MSP_RESULT_ERROR;
            }

            const uint8_t i = sbufReadU8(src);
            if (i >= MIXER_INPUT_COUNT) {
                return MSP_RESULT_ERROR;
            }

            sbufWriteU16(dst, mixerInputs(i)->rate);
            sbufWriteU16(dst, mixerInputs(i)->min);
            sbufWriteU16(dst, mixerInputs(i)->max);
        }
        break;

#if defined(USE_SERVOS)
    case MSP_GET_SERVO_CONFIG:
        {
            const int rem = sbufBytesRemaining(src);
            if (rem != 1) {
                return MSP_RESULT_ERROR;
            }

            const uint8_t i = sbufReadU8(src);
            if (i >= MAX_SUPPORTED_SERVOS) {
                return MSP_RESULT_ERROR;
            }

            sbufWriteU16(dst, servoParams(i)->mid);
            sbufWriteU16(dst, servoParams(i)->min);
            sbufWriteU16(dst, servoParams(i)->max);
            sbufWriteU16(dst, servoParams(i)->rneg);
            sbufWriteU16(dst, servoParams(i)->rpos);
            sbufWriteU16(dst, servoParams(i)->rate);
            sbufWriteU16(dst, servoParams(i)->speed);
            sbufWriteU16(dst, servoParams(i)->flags);
        }
        break;

#endif

    case MSP_GET_ADJUSTMENT_RANGE:
        {
            const int rem = sbufBytesRemaining(src);
            if (rem != 1) {
                return MSP_RESULT_ERROR;
            }

            const uint8_t i = sbufReadU8(src);

            if (i >= MAX_ADJUSTMENT_RANGE_COUNT) {
                return MSP_RESULT_ERROR;
            }

            // Serialize EXACTLY one entry using the same field order/encoding
            // as the legacy MSP_ADJUSTMENT_RANGES response.
            const adjustmentRange_t *adjRange = adjustmentRanges(i);

            sbufWriteU8(dst, adjRange->function);
            sbufWriteU8(dst, adjRange->enaChannel);
            sbufWriteU8(dst, adjRange->enaRange.startStep);
            sbufWriteU8(dst, adjRange->enaRange.endStep);
            sbufWriteU8(dst, adjRange->adjChannel);
            sbufWriteU8(dst, adjRange->adjRange1.startStep);
            sbufWriteU8(dst, adjRange->adjRange1.endStep);
            sbufWriteU8(dst, adjRange->adjRange2.startStep);
            sbufWriteU8(dst, adjRange->adjRange2.endStep);
            sbufWriteU16(dst, adjRange->adjMin);
            sbufWriteU16(dst, adjRange->adjMax);
            sbufWriteU8(dst, adjRange->adjStep);
        }
        break;        

    default:
        return MSP_RESULT_CMD_UNKNOWN;
    }
    return MSP_RESULT_ACK;
}

// msp.c's mspCommonProcessInCommand() / mspProcessInCommand(): setters. The
// last handler in the chain, so an opcode nobody handles is an error here.
mspResult_e mspCatalogueProcessInCommand(mspDescriptor_t srcDesc, int16_t cmdMSP, sbuf_t *src)
{
    UNUSED(srcDesc);
    uint32_t i;
    uint8_t value;
    const unsigned int dataSize = sbufBytesRemaining(src);
    UNUSED(i);
    UNUSED(value);
    UNUSED(dataSize);

    switch (cmdMSP) {
#if (defined(USE_ACC))
    case MSP_SET_ACC_TRIM:
        accelerometerConfigMutable()->accelerometerTrims.values.pitch = sbufReadU16(src);
        accelerometerConfigMutable()->accelerometerTrims.values.roll  = sbufReadU16(src);

        break;

#endif

    case MSP_SET_DEBUG_CONFIG:
        systemConfigMutable()->debug_mode = sbufReadU8(src);
        systemConfigMutable()->debug_axis = sbufReadU8(src);
        break;

    case MSP_SET_ARMING_CONFIG:
        armingConfigMutable()->auto_disarm_delay = sbufReadU8(src);
        armingConfigMutable()->wiggle_flags = sbufReadU32(src);
        break;

    case MSP_SET_PID_TUNING:
        for (int i = 0; i < PID_AXIS_COUNT; i++) {
            currentPidProfile->pid[i].P = sbufReadU16(src);
            currentPidProfile->pid[i].I = sbufReadU16(src);
            currentPidProfile->pid[i].D = sbufReadU16(src);
            currentPidProfile->pid[i].F = sbufReadU16(src);
        }
        if (sbufBytesRemaining(src) >= 6) {
            for (int i = 0; i < PID_AXIS_COUNT; i++) {
                currentPidProfile->pid[i].B = sbufReadU16(src);
            }
        }
        pidLoadProfile(currentPidProfile);
        break;

    case MSP_SET_ADJUSTMENT_RANGE:
        i = sbufReadU8(src);
        if (i < MAX_ADJUSTMENT_RANGE_COUNT) {
            adjustmentRange_t *adjRange = adjustmentRangesMutable(i);
            adjRange->function = sbufReadU8(src);
            adjRange->enaChannel = sbufReadU8(src);
            adjRange->enaRange.startStep = sbufReadU8(src);
            adjRange->enaRange.endStep = sbufReadU8(src);
            adjRange->adjChannel = sbufReadU8(src);
            adjRange->adjRange1.startStep = sbufReadU8(src);
            adjRange->adjRange1.endStep = sbufReadU8(src);
            adjRange->adjRange2.startStep = sbufReadU8(src);
            adjRange->adjRange2.endStep = sbufReadU8(src);
            adjRange->adjMin = sbufReadU16(src);
            adjRange->adjMax = sbufReadU16(src);
            adjRange->adjStep = sbufReadU8(src);
            adjustmentRangeReset(i);
        } else {
            return MSP_RESULT_ERROR;
        }
        break;

    case MSP_SET_RC_TUNING:
        for (int i = 0; i < 3; i++) {
            currentControlRateProfile->rcRates[i] = sbufReadU8(src);
            currentControlRateProfile->rcExpo[i] = sbufReadU8(src);
            currentControlRateProfile->sRates[i] = sbufReadU8(src);
            currentControlRateProfile->response_time[i] = sbufReadU8(src);
            currentControlRateProfile->accel_limit[i] = sbufReadU16(src);
        }
        if (sbufBytesRemaining(src) >= 6) {
            for (int i = 0; i < 3; i++) {
                currentControlRateProfile->setpoint_boost_gain[i] =
                    sbufReadU8(src);
                currentControlRateProfile->setpoint_boost_cutoff[i] =
                    sbufReadU8(src);
            }
        }
        if (sbufBytesRemaining(src) >= 3) {
            currentControlRateProfile->yaw_dynamic_ceiling_gain = sbufReadU8(src);
            currentControlRateProfile->yaw_dynamic_deadband_gain = sbufReadU8(src);
            currentControlRateProfile->yaw_dynamic_deadband_filter= sbufReadU8(src);
        }
        loadControlRateProfile();
        break;

    case MSP_SET_MOTOR_CONFIG:
        motorConfigMutable()->minthrottle = sbufReadU16(src);
        motorConfigMutable()->maxthrottle = sbufReadU16(src);
        motorConfigMutable()->mincommand = sbufReadU16(src);

        // sbufReadU8(src); MSP_MOTOR_CONFIG has motorCount here
        sbufReadU8(src); // compat: motorPoleCount

#if defined(USE_DSHOT_TELEMETRY)
        motorConfigMutable()->dev.useDshotTelemetry = sbufReadU8(src);
#else
        sbufReadU8(src);
#endif
        motorConfigMutable()->dev.motorPwmProtocol = sbufReadU8(src);
        motorConfigMutable()->dev.motorPwmRate = sbufReadU16(src);
        motorConfigMutable()->dev.useUnsyncedPwm = sbufReadU8(src);

        for (int i = 0; i < 4; i++)
            motorConfigMutable()->motorPoleCount[i] = sbufReadU8(src);
        for (int i = 0; i < 4; i++)
            motorConfigMutable()->motorRpmLpf[i] = sbufReadU8(src);

        motorConfigMutable()->motor1GearRatio[0] = sbufReadU16(src);
        motorConfigMutable()->motor1GearRatio[1] = sbufReadU16(src);
        motorConfigMutable()->motor2GearRatio[0] = sbufReadU16(src);
        motorConfigMutable()->motor2GearRatio[1] = sbufReadU16(src);
        break;

#if defined(USE_GPS)
    case MSP_SET_GPS_CONFIG:
        gpsConfigMutable()->provider = sbufReadU8(src);
        gpsConfigMutable()->sbasMode = sbufReadU8(src);
        gpsConfigMutable()->autoConfig = sbufReadU8(src);
        gpsConfigMutable()->autoBaud = sbufReadU8(src);
        if (sbufBytesRemaining(src) >= 2) {
            // Added in API version 1.43
            gpsConfigMutable()->gps_set_home_point_once = sbufReadU8(src);
            gpsConfigMutable()->gps_ublox_use_galileo = sbufReadU8(src);
        }
        break;

#endif

#if defined(USE_GPS) && defined(USE_GPS_RESCUE)
        case MSP_SET_GPS_RESCUE:
        gpsRescueConfigMutable()->angle = sbufReadU16(src);
        gpsRescueConfigMutable()->initialAltitudeM = sbufReadU16(src);
        gpsRescueConfigMutable()->descentDistanceM = sbufReadU16(src);
        gpsRescueConfigMutable()->rescueGroundspeed = sbufReadU16(src);
        gpsRescueConfigMutable()->throttleMin = sbufReadU16(src);
        gpsRescueConfigMutable()->throttleMax = sbufReadU16(src);
        gpsRescueConfigMutable()->throttleHover = sbufReadU16(src);
        gpsRescueConfigMutable()->sanityChecks = sbufReadU8(src);
        gpsRescueConfigMutable()->minSats = sbufReadU8(src);
        if (sbufBytesRemaining(src) >= 6) {
            // Added in API version 1.43
            gpsRescueConfigMutable()->ascendRate = sbufReadU16(src);
            gpsRescueConfigMutable()->descendRate = sbufReadU16(src);
            gpsRescueConfigMutable()->allowArmingWithoutFix = sbufReadU8(src);
            gpsRescueConfigMutable()->altitudeMode = sbufReadU8(src);
        }
        if (sbufBytesRemaining(src) >= 2) {
            // Added in API version 1.44
            gpsRescueConfigMutable()->minRescueDth = sbufReadU16(src);
        }
        break;

    case MSP_SET_GPS_RESCUE_PIDS:
        gpsRescueConfigMutable()->throttleP = sbufReadU16(src);
        gpsRescueConfigMutable()->throttleI = sbufReadU16(src);
        gpsRescueConfigMutable()->throttleD = sbufReadU16(src);
        gpsRescueConfigMutable()->velP = sbufReadU16(src);
        gpsRescueConfigMutable()->velI = sbufReadU16(src);
        gpsRescueConfigMutable()->velD = sbufReadU16(src);
        gpsRescueConfigMutable()->yawP = sbufReadU16(src);
        break;

#endif

    case MSP_SET_SENSOR_ALIGNMENT:
        gyroDeviceConfigMutable(0)->alignment = sbufReadU8(src);
#ifdef USE_MULTI_GYRO
        gyroDeviceConfigMutable(1)->alignment = sbufReadU8(src);
#else
        // One gyro device: gyroDeviceConfig has a single element, and the
        // accessor does not bounds-check, so element 1 is past the array.
        sbufReadU8(src);
#endif
#if defined(USE_MAG)
        compassConfigMutable()->mag_alignment = sbufReadU8(src);
#else
        sbufReadU8(src);
#endif
        break;

    case MSP_SET_ADVANCED_CONFIG:
        sbufReadU8(src);  // compat: gyro denom
        pidConfigMutable()->pid_process_denom = sbufReadU8(src);
        break;

    case MSP_SET_FILTER_CONFIG:
        gyroConfigMutable()->gyro_hardware_lpf = sbufReadU8(src);
        gyroConfigMutable()->gyro_lpf1_type = sbufReadU8(src);
        gyroConfigMutable()->gyro_lpf1_static_hz = sbufReadU16(src);
        gyroConfigMutable()->gyro_lpf2_type = sbufReadU8(src);
        gyroConfigMutable()->gyro_lpf2_static_hz = sbufReadU16(src);
        gyroConfigMutable()->gyro_soft_notch_hz_1 = sbufReadU16(src);
        gyroConfigMutable()->gyro_soft_notch_cutoff_1 = sbufReadU16(src);
        gyroConfigMutable()->gyro_soft_notch_hz_2 = sbufReadU16(src);
        gyroConfigMutable()->gyro_soft_notch_cutoff_2 = sbufReadU16(src);
#if defined(USE_DYN_LPF)
        gyroConfigMutable()->gyro_lpf1_dyn_min_hz = sbufReadU16(src);
        gyroConfigMutable()->gyro_lpf1_dyn_max_hz = sbufReadU16(src);
#else
        sbufReadU16(src);
        sbufReadU16(src);
#endif
#if defined(USE_DYN_NOTCH_FILTER)
        dynNotchConfigMutable()->dyn_notch_count = sbufReadU8(src);
        dynNotchConfigMutable()->dyn_notch_q = sbufReadU8(src);
        dynNotchConfigMutable()->dyn_notch_min_hz = sbufReadU16(src);
        dynNotchConfigMutable()->dyn_notch_max_hz = sbufReadU16(src);
#else
        sbufReadU8(src);
        sbufReadU8(src);
        sbufReadU16(src);
        sbufReadU16(src);
#endif
#if defined(USE_RPM_FILTER)
        if (sbufBytesRemaining(src) >= 2) {
            rpmFilterConfigMutable()->preset = sbufReadU8(src);
            rpmFilterConfigMutable()->min_hz = sbufReadU8(src);
        }
#endif
        // reinitialize the gyro filters with the new values
        validateAndFixGyroConfig();
        gyroInitFilters();
#if defined(USE_RPM_FILTER)
        validateAndFixRPMFilterConfig();
        rpmFilterInit();
#endif
        break;

    case MSP_SET_PID_PROFILE:
        currentPidProfile->pid_mode = sbufReadU8(src);
        currentPidProfile->iterm_decay_time = sbufReadU8(src);
        currentPidProfile->iterm_decay_limit = sbufReadU8(src);
        currentPidProfile->error_limit[0] = sbufReadU8(src);
        currentPidProfile->error_limit[1] = sbufReadU8(src);
        currentPidProfile->error_limit[2] = sbufReadU8(src);
        currentPidProfile->gyro_cutoff[0] = sbufReadU8(src);
        currentPidProfile->gyro_cutoff[1] = sbufReadU8(src);
        currentPidProfile->gyro_cutoff[2] = sbufReadU8(src);
        currentPidProfile->dterm_cutoff[0] = sbufReadU8(src);
        currentPidProfile->dterm_cutoff[1] = sbufReadU8(src);
        currentPidProfile->dterm_cutoff[2] = sbufReadU8(src);
        currentPidProfile->iterm_relax_type = sbufReadU8(src);
        currentPidProfile->iterm_relax_cutoff[0] = sbufReadU8(src);
        currentPidProfile->iterm_relax_cutoff[1] = sbufReadU8(src);
        currentPidProfile->iterm_relax_cutoff[2] = sbufReadU8(src);
        /* Angle mode */
        currentPidProfile->angle.level_strength = sbufReadU8(src);
        currentPidProfile->angle.level_limit = sbufReadU8(src);
        /* Horizon mode */
        currentPidProfile->horizon.level_strength = sbufReadU8(src);
        /* Acro trainer */
        currentPidProfile->trainer.gain = sbufReadU8(src);
        currentPidProfile->trainer.angle_limit = sbufReadU8(src);
        /* Att Hold */
        if (sbufBytesRemaining(src) >= 2) {
            currentPidProfile->atthold.gain = sbufReadU8(src);
            currentPidProfile->atthold.deadband = sbufReadU8(src);
        }
        /* B-term cutoffs */
        if (sbufBytesRemaining(src) >= 3) {
            currentPidProfile->bterm_cutoff[0] = sbufReadU8(src);
            currentPidProfile->bterm_cutoff[1] = sbufReadU8(src);
            currentPidProfile->bterm_cutoff[2] = sbufReadU8(src);
        }
        /* Fixed-wing throttle-based gain attenuation (gain + curve index) */
        if (sbufBytesRemaining(src) >= 2) {
            currentPidProfile->fw_tpa_gain = sbufReadU8(src);
            currentPidProfile->fw_tpa_curve = sbufReadU8(src);
        }
        /* Master gain (per axis) */
        if (sbufBytesRemaining(src) >= 6) {
            currentPidProfile->master_gain[PID_ROLL] = sbufReadU16(src);
            currentPidProfile->master_gain[PID_PITCH] = sbufReadU16(src);
            currentPidProfile->master_gain[PID_YAW] = sbufReadU16(src);
        }
        /* Auto Hover */
        if (sbufBytesRemaining(src) >= 4) {
            currentPidProfile->autohover.gain = sbufReadU8(src);
            currentPidProfile->autohover.max_angle = sbufReadU8(src);
            currentPidProfile->autohover.max_rate = sbufReadU16(src);
        }
        /* Cross-axis relax */
        if (sbufBytesRemaining(src) >= 3) {
            currentPidProfile->cross_axis_relax_strength = sbufReadU8(src);
            currentPidProfile->cross_axis_relax_level = sbufReadU8(src);
            currentPidProfile->cross_axis_relax_cutoff = sbufReadU8(src);
        }
        if (sbufBytesRemaining(src) >= 1) {
            currentPidProfile->cross_axis_relax_pitch_strength = sbufReadU8(src);
        }
        /* Gain curve assignment (per axis) */
        if (sbufBytesRemaining(src) >= 3) {
            currentPidProfile->gain_curve[PID_ROLL] = sbufReadU8(src);
            currentPidProfile->gain_curve[PID_PITCH] = sbufReadU8(src);
            currentPidProfile->gain_curve[PID_YAW] = sbufReadU8(src);
        }
        /* Att Hold max rate */
        if (sbufBytesRemaining(src) >= 2) {
            currentPidProfile->atthold.max_rate = sbufReadU16(src);
        }
        /* Auto Hover roll deadband */
        if (sbufBytesRemaining(src) >= 1) {
            currentPidProfile->autohover.roll_deadband = sbufReadU8(src);
        }
        /* Auto Hover throttle assist */
        if (sbufBytesRemaining(src) >= 4) {
            currentPidProfile->autohover.throttle_assist_gain = sbufReadU8(src);
            currentPidProfile->autohover.throttle_assist_max = sbufReadU8(src);
            currentPidProfile->autohover.throttle_assist_trigger_ms = sbufReadU16(src);
        }
        /* Older clients omit this extension and must not erase the axis limits. */
        if (sbufBytesRemaining(src) >= 4) {
            attitudeLimits_t *limits = attitudeLimitsMutable(getCurrentPidProfileIndex());
            limits->angle_roll = sbufReadU8(src);
            limits->angle_pitch = sbufReadU8(src);
            limits->trainer_roll = sbufReadU8(src);
            limits->trainer_pitch = sbufReadU8(src);
        }
        /* Load new values */
        pidLoadProfile(currentPidProfile);
        break;

    case MSP_SET_SENSOR_CONFIG:
#if defined(USE_ACC)
        accelerometerConfigMutable()->acc_hardware = sbufReadU8(src);
#else
        sbufReadU8(src);
#endif
#if defined(USE_BARO)
        barometerConfigMutable()->baro_hardware = sbufReadU8(src);
#else
        sbufReadU8(src);
#endif
#if defined(USE_MAG)
        compassConfigMutable()->mag_hardware = sbufReadU8(src);
#else
        sbufReadU8(src);
#endif
        gyroConfigMutable()->gyro_to_use = sbufReadU8(src);
        gyroConfigMutable()->gyro_high_fsr = sbufReadU8(src);
        gyroConfigMutable()->gyroMovementCalibrationThreshold = sbufReadU8(src);
        gyroConfigMutable()->gyroCalibrationDuration = sbufReadU16(src);
        gyroConfigMutable()->checkOverflow = sbufReadU8(src);
        validateAndFixGyroConfig();
        break;

#if defined(USE_BEEPER)
    case MSP_SET_BEEPER_CONFIG:
        beeperConfigMutable()->beeper_off_flags = sbufReadU32(src);
        if (sbufBytesRemaining(src) >= 1) {
            beeperConfigMutable()->dshotBeaconTone = sbufReadU8(src);
        }
        if (sbufBytesRemaining(src) >= 4) {
            beeperConfigMutable()->dshotBeaconOffFlags = sbufReadU32(src);
        }
        break;

#endif

    case MSP_SET_BOARD_ALIGNMENT_CONFIG:
        // sbufReadU16() into an int32_t cannot produce a negative value: -10
        // arrives as 0xFFF6 and widens to 65526, which degreesToRadians()
        // then quietly turns into 6 degrees. Read it signed, as the mount
        // trim below already does.
        boardAlignmentMutable()->rollDegrees = sbufReadS16(src);
        boardAlignmentMutable()->pitchDegrees = sbufReadS16(src);
        boardAlignmentMutable()->yawDegrees = sbufReadS16(src);
        break;

    case MSP2_WING_SET_BOARD_MOUNT_TRIM:
        boardAlignmentMutable()->mountTrim.roll = sbufReadS16(src);
        boardAlignmentMutable()->mountTrim.pitch = sbufReadS16(src);
        boardAlignmentMutable()->mountTrim.yaw = sbufReadS16(src);
        break;

    case MSP2_WING_SET_TV_PID_CONFIG:
        if (dataSize != PID_ITEM_COUNT * 5 * sizeof(uint16_t) + 3 * sizeof(uint16_t) + 3 + PID_AXIS_COUNT * 6
                         + 2 * sizeof(uint8_t) + sizeof(uint16_t)) {
            return MSP_RESULT_ERROR;
        }
        for (int i = 0; i < PID_ITEM_COUNT; i++) {
            currentTvPidProfile->pid[i].P = sbufReadU16(src);
            currentTvPidProfile->pid[i].I = sbufReadU16(src);
            currentTvPidProfile->pid[i].D = sbufReadU16(src);
            currentTvPidProfile->pid[i].F = sbufReadU16(src);
            currentTvPidProfile->pid[i].B = sbufReadU16(src);
        }
        currentTvPidProfile->master_gain[PID_ROLL] = sbufReadU16(src);
        currentTvPidProfile->master_gain[PID_PITCH] = sbufReadU16(src);
        currentTvPidProfile->master_gain[PID_YAW] = sbufReadU16(src);
        currentTvPidProfile->iterm_decay_time = sbufReadU8(src);
        currentTvPidProfile->iterm_decay_limit = sbufReadU8(src);
        currentTvPidProfile->iterm_relax_type = sbufReadU8(src);
        for (int i = 0; i < PID_AXIS_COUNT; i++) {
            currentTvPidProfile->iterm_relax_level[i] = sbufReadU8(src);
        }
        for (int i = 0; i < PID_AXIS_COUNT; i++) {
            currentTvPidProfile->iterm_relax_cutoff[i] = sbufReadU8(src);
        }
        for (int i = 0; i < PID_AXIS_COUNT; i++) {
            currentTvPidProfile->error_limit[i] = sbufReadU8(src);
        }
        for (int i = 0; i < PID_AXIS_COUNT; i++) {
            currentTvPidProfile->dterm_cutoff[i] = sbufReadU8(src);
        }
        for (int i = 0; i < PID_AXIS_COUNT; i++) {
            currentTvPidProfile->bterm_cutoff[i] = sbufReadU8(src);
        }
        for (int i = 0; i < PID_AXIS_COUNT; i++) {
            currentTvPidProfile->gyro_cutoff[i] = sbufReadU8(src);
        }
        currentTvPidProfile->hold.gain = sbufReadU8(src);
        currentTvPidProfile->hold.deadband = sbufReadU8(src);
        currentTvPidProfile->hold.max_rate = sbufReadU16(src);
        tvPidLoadProfile(currentTvPidProfile);
        tvHoldInit(currentTvPidProfile);
        break;

    case MSP_SET_MIXER_INPUT:
        i = sbufReadU8(src);
        if (i >= MIXER_INPUT_COUNT) {
            return MSP_RESULT_ERROR;
        }
        mixerInputsMutable(i)->rate = sbufReadU16(src);
        mixerInputsMutable(i)->min = sbufReadU16(src);
        mixerInputsMutable(i)->max = sbufReadU16(src);
        break;

    case MSP_SET_MIXER_RULE:
        i = sbufReadU8(src);
        if (i >= MIXER_RULE_COUNT) {
            return MSP_RESULT_ERROR;
        }
        mixerRulesMutable(i)->oper = sbufReadU8(src);
        mixerRulesMutable(i)->input = sbufReadU8(src);
        mixerRulesMutable(i)->output = sbufReadU8(src);
        mixerRulesMutable(i)->offset = sbufReadU16(src);
        mixerRulesMutable(i)->weight = sbufReadU16(src);
        mixerRulesMutable(i)->weightNeg = sbufReadU16(src);
        mixerRulesMutable(i)->speed = sbufReadU16(src);
        mixerRulesMutable(i)->curve = sbufReadU8(src);
        mixerRulesMutable(i)->condition = sbufReadU8(src);
        mixerRulesMutable(i)->role = sbufReadU8(src);
        mixerCaptureRuleSign(i);
        break;

    case MSP_SET_MIXER_CURVE:
        i = sbufReadU8(src);
        if (i >= MIXER_CURVE_COUNT) {
            return MSP_RESULT_ERROR;
        }
        {
            // count is later used unchecked as an array bound/loop limit by
            // mixerEvaluateCurve() (points[count-1], points[i+1]) -- reject
            // anything outside the wire format's actual valid range here,
            // same as the CLI's own "mixer_curve <i> count <n>" validation
            // (src/main/cli/cli.c), rather than trusting a raw byte that
            // could otherwise drive an out-of-bounds read at evaluation time.
            uint8_t pointCount = sbufReadU8(src);
            if (pointCount < 2 || pointCount > MIXER_CURVE_POINTS) {
                return MSP_RESULT_ERROR;
            }
            mixerCurvesMutable(i)->count = pointCount;
        }
        for (int p = 0; p < MIXER_CURVE_POINTS; p++) {
            mixerCurvesMutable(i)->points[p].x = sbufReadU16(src);
            mixerCurvesMutable(i)->points[p].y = sbufReadU16(src);
        }
        break;

    case MSP_SET_GAIN_CURVE:
        i = sbufReadU8(src);
        if (i >= GAIN_CURVE_COUNT) {
            return MSP_RESULT_ERROR;
        }
        {
            // Same out-of-bounds-read concern as MSP_SET_MIXER_CURVE above,
            // for pidEvaluateGainCurve()'s points[count-1]/points[i+1].
            uint8_t pointCount = sbufReadU8(src);
            if (pointCount < 2 || pointCount > GAIN_CURVE_POINTS) {
                return MSP_RESULT_ERROR;
            }
            gainCurvesMutable(i)->count = pointCount;
        }
        for (int p = 0; p < GAIN_CURVE_POINTS; p++) {
            gainCurvesMutable(i)->points[p].x = sbufReadU16(src);
            gainCurvesMutable(i)->points[p].y = sbufReadU16(src);
        }
        break;

    case MSP_SET_LOGIC_CONDITION:
        i = sbufReadU8(src);
        if (i >= LOGIC_CONDITION_COUNT) {
            return MSP_RESULT_ERROR;
        }
        logicConditionsMutable(i)->enabled = sbufReadU8(src);
        logicConditionsMutable(i)->operation = sbufReadU8(src);
        logicConditionsMutable(i)->operandAType = sbufReadU8(src);
        logicConditionsMutable(i)->operandAValue = sbufReadU16(src);
        logicConditionsMutable(i)->operandBType = sbufReadU8(src);
        logicConditionsMutable(i)->operandBValue = sbufReadU16(src);
        break;

    case MSP_SET_RX_CONFIG:
        // Make sure ELRS commands don't confuse us
        if (sbufBytesRemaining(src) >= 7) {
            rxConfigMutable()->serialrx_provider = sbufReadU8(src);
            rxConfigMutable()->serialrx_inverted = sbufReadU8(src);
            rxConfigMutable()->halfDuplex = sbufReadU8(src);
            rxConfigMutable()->rx_pulse_min = sbufReadU16(src);
            rxConfigMutable()->rx_pulse_max = sbufReadU16(src);
            if (sbufBytesRemaining(src) >= 6) {
                sbufReadU8(src);
                sbufReadU32(src);
                sbufReadU8(src);
            }
            if (sbufBytesRemaining(src) >= 1) {
                rxConfigMutable()->pinSwap = sbufReadU8(src);
            }
        }
    #ifdef USE_RX_SPI
        if (sbufBytesRemaining(src) >= 6) {
            rxSpiConfigMutable()->rx_spi_protocol = sbufReadU8(src);
            rxSpiConfigMutable()->rx_spi_id = sbufReadU32(src);
            rxSpiConfigMutable()->rx_spi_rf_channel_count = sbufReadU8(src);
        }
    #endif
        break;

    case MSP_SET_FAILSAFE_CONFIG:
        failsafeConfigMutable()->failsafe_delay = sbufReadU8(src);
        failsafeConfigMutable()->failsafe_off_delay = sbufReadU8(src);
        failsafeConfigMutable()->failsafe_throttle = sbufReadU16(src);
        failsafeConfigMutable()->failsafe_switch_mode = sbufReadU8(src);
        failsafeConfigMutable()->failsafe_throttle_low_delay = sbufReadU16(src);
        failsafeConfigMutable()->failsafe_procedure = sbufReadU8(src);
        // Appended field -- older clients that only send the six bytes above leave this
        // untouched, same pattern used elsewhere in this function (e.g. MSP_SET_TELEMETRY_CONFIG).
        if (sbufBytesRemaining(src) >= 2) {
            failsafeConfigMutable()->failsafe_recovery_delay = sbufReadU16(src);
        }
        break;

    case MSP2_WING_SET_GPS_NAV_CONFIG:
        gpsNavConfigMutable()->loiterRadiusM = sbufReadU16(src);
        gpsNavConfigMutable()->loiterDirection = sbufReadU8(src);
        gpsNavConfigMutable()->rthAltitudeM = sbufReadU16(src);
        gpsNavConfigMutable()->minSats = sbufReadU8(src);
        gpsNavConfigMutable()->maxBankAngleDeg = sbufReadU8(src);
        gpsNavConfigMutable()->maxPitchAngleDeg = sbufReadU8(src);
        gpsNavConfigMutable()->bearingKp = sbufReadU16(src);
        gpsNavConfigMutable()->altitudeKp = sbufReadU16(src);
        // Appended fields -- older clients that only send the bytes above leave these untouched.
        if (sbufBytesRemaining(src) >= 4) {
            gpsNavConfigMutable()->altitudeKd = sbufReadU16(src);
            gpsNavConfigMutable()->throttle = sbufReadU8(src);
            gpsNavConfigMutable()->turnCoordination = sbufReadU8(src);
        }
        break;

    case MSP_SET_RSSI_CONFIG:
        rxConfigMutable()->rssi_channel = sbufReadU8(src);
        rxConfigMutable()->rssi_scale = sbufReadU8(src);
        rxConfigMutable()->rssi_invert = sbufReadU8(src);
        rxConfigMutable()->rssi_offset = sbufReadU8(src);
        break;

    case MSP_SET_RX_MAP:
        for (int i = 0; i < RX_MAPPABLE_CHANNEL_COUNT; i++) {
            rxConfigMutable()->rcmap[i] = sbufReadU8(src);
        }
        break;

    case MSP_SET_RC_CONFIG:
        rcControlsConfigMutable()->rc_center = sbufReadU16(src);
        rcControlsConfigMutable()->rc_deflection = sbufReadU16(src);
        rcControlsConfigMutable()->rc_min_throttle = sbufReadU16(src);
        rcControlsConfigMutable()->rc_max_throttle = sbufReadU16(src);
        rcControlsConfigMutable()->rc_roll_deadband = sbufReadU8(src);
        rcControlsConfigMutable()->rc_pitch_deadband = sbufReadU8(src);
        rcControlsConfigMutable()->rc_yaw_deadband = sbufReadU8(src);
        break;

    case MSP_SET_TELEMETRY_CONFIG:
        telemetryConfigMutable()->telemetry_inverted = sbufReadU8(src);
        telemetryConfigMutable()->halfDuplex = sbufReadU8(src);
        if (sbufBytesRemaining(src) >= 1) {
            telemetryConfigMutable()->pinSwap = sbufReadU8(src);
        }
        if (sbufBytesRemaining(src) >= 5 + TELEM_SENSOR_SLOT_COUNT) {
            telemetryConfigMutable()->crsf_telemetry_mode = sbufReadU8(src);
            telemetryConfigMutable()->crsf_telemetry_link_rate = sbufReadU16(src);
            telemetryConfigMutable()->crsf_telemetry_link_ratio = sbufReadU16(src);
            for (int i = 0; i < TELEM_SENSOR_SLOT_COUNT; i++) {
                telemetryConfigMutable()->telemetry_sensors[i] = sbufReadU8(src);
            }
        }
        break;

#if defined(USE_LED_STRIP_STATUS_MODE)
    case MSP_SET_LED_COLORS:
        for (int i = 0; i < LED_CONFIGURABLE_COLOR_COUNT; i++) {
            hsvColor_t *color = &ledStripStatusModeConfigMutable()->colors[i];
            color->h = sbufReadU16(src);
            color->s = sbufReadU8(src);
            color->v = sbufReadU8(src);
        }
        break;

#endif

#if defined(USE_LED_STRIP)
    case MSP_SET_LED_STRIP_CONFIG:
        {
            i = sbufReadU8(src);
            if (i >= LED_MAX_STRIP_LENGTH || dataSize != (1 + 8)) {
                return MSP_RESULT_ERROR;
            }
#ifdef USE_LED_STRIP_STATUS_MODE
            ledConfig_t *ledConfig = &ledStripStatusModeConfigMutable()->ledConfigs[i];
            *ledConfig = sbufReadU64(src);
            reevaluateLedConfig();
#else
            sbufReadU64(src);
#endif
            // API 1.41 - selected ledstrip_profile
            if (sbufBytesRemaining(src) >= 1) {
                ledStripConfigMutable()->ledstrip_profile = sbufReadU8(src);
            }
        }
        break;

    case MSP_SET_LED_STRIP_SETTINGS:
        ledStripConfigMutable()->ledstrip_beacon_armed_only = sbufReadU8(src);
        ledStripConfigMutable()->ledstrip_beacon_color = sbufReadU8(src);
        ledStripConfigMutable()->ledstrip_beacon_percent = sbufReadU8(src);
        ledStripConfigMutable()->ledstrip_beacon_period_ms = sbufReadU16(src);
        ledStripConfigMutable()->ledstrip_blink_period_ms = sbufReadU16(src);
        ledStripConfigMutable()->ledstrip_brightness = sbufReadU8(src);
        ledStripConfigMutable()->ledstrip_fade_rate = sbufReadU8(src);
        ledStripConfigMutable()->ledstrip_flicker_rate = sbufReadU8(src);
        ledStripConfigMutable()->ledstrip_grb_rgb = sbufReadU8(src);
        ledStripConfigMutable()->ledstrip_profile = sbufReadU8(src);
        ledStripConfigMutable()->ledstrip_race_color = sbufReadU8(src);
        ledStripConfigMutable()->ledstrip_visual_beeper = sbufReadU8(src);
        ledStripConfigMutable()->ledstrip_visual_beeper_color = sbufReadU8(src);
        break;

#endif

    case MSP_SET_NAME:
        memset(pilotConfigMutable()->name, 0, ARRAYLEN(pilotConfig()->name));
        for (unsigned int i = 0; i < MIN(MAX_NAME_LENGTH, dataSize); i++) {
            pilotConfigMutable()->name[i] = sbufReadU8(src);
        }
        break;

    case MSP_SET_PILOT_CONFIG:
        // Introduced in MSP API 12.7
        pilotConfigMutable()->modelId = sbufReadU8(src);
        pilotConfigMutable()->modelParam1Type = sbufReadU8(src);
        pilotConfigMutable()->modelParam1Value = sbufReadU16(src);
        pilotConfigMutable()->modelParam2Type = sbufReadU8(src);
        pilotConfigMutable()->modelParam2Value = sbufReadU16(src);
        pilotConfigMutable()->modelParam3Type = sbufReadU8(src);
        pilotConfigMutable()->modelParam3Value = sbufReadU16(src);
        if (sbufBytesRemaining(src) >= 4) {
            // Introduced in MSP API 12.9
            pilotConfigMutable()->modelFlags = sbufReadU32(src);
        }
        break;

    case MSP_SET_FLIGHT_STATS:
        // Introduced in MSP API 12.9
        statsConfigMutable()->stats_total_flights = sbufReadU32(src);
        statsConfigMutable()->stats_total_time_s = sbufReadU32(src);
        statsConfigMutable()->stats_total_dist_m = sbufReadU32(src);
        statsConfigMutable()->stats_min_armed_time_s = sbufReadS8(src);
        break;

#if (defined(USE_BOARD_INFO))
    case MSP_SET_BOARD_INFO:
        if (!boardInformationIsSet()) {
            uint8_t length = sbufReadU8(src);
            char boardName[MAX_BOARD_NAME_LENGTH + 1];
            sbufReadData(src, boardName, MIN(length, MAX_BOARD_NAME_LENGTH));
            if (length > MAX_BOARD_NAME_LENGTH) {
                sbufAdvance(src, length - MAX_BOARD_NAME_LENGTH);
                length = MAX_BOARD_NAME_LENGTH;
            }
            boardName[length] = '\0';
            length = sbufReadU8(src);
            char boardDesign[MAX_BOARD_DESIGN_LENGTH + 1];
            sbufReadData(src, boardDesign, MIN(length, MAX_BOARD_DESIGN_LENGTH));
            if (length > MAX_BOARD_DESIGN_LENGTH) {
                sbufAdvance(src, length - MAX_BOARD_DESIGN_LENGTH);
                length = MAX_BOARD_DESIGN_LENGTH;
            }
            boardDesign[length] = '\0';
            length = sbufReadU8(src);
            char manufacturerId[MAX_MANUFACTURER_ID_LENGTH + 1];
            sbufReadData(src, manufacturerId, MIN(length, MAX_MANUFACTURER_ID_LENGTH));
            if (length > MAX_MANUFACTURER_ID_LENGTH) {
                sbufAdvance(src, length - MAX_MANUFACTURER_ID_LENGTH);
                length = MAX_MANUFACTURER_ID_LENGTH;
            }
            manufacturerId[length] = '\0';

            setBoardName(boardName);
            setBoardDesign(boardDesign);
            setManufacturerId(manufacturerId);
            persistBoardInformation();
        } else {
            return MSP_RESULT_ERROR;
        }

        break;

#endif

#if (defined(USE_BOARD_INFO)) && (defined(USE_SIGNATURE))
    case MSP_SET_SIGNATURE:
        if (!signatureIsSet()) {
            uint8_t signature[SIGNATURE_LENGTH];
            sbufReadData(src, signature, SIGNATURE_LENGTH);
            setSignature(signature);
            persistSignature();
        } else {
            return MSP_RESULT_ERROR;
        }

        break;

#endif

    case MSP_SET_VOLTAGE_METER_CONFIG: {
        uint8_t id = sbufReadU8(src);
        int index;
        for (index = 0; index < MAX_VOLTAGE_SENSOR_ADC; index++) {
            if (id == voltageSensorToMeterMap[index])
                break;
        }
        if (index < MAX_VOLTAGE_SENSOR_ADC) {
            voltageSensorADCConfigMutable(index)->scale = sbufReadU16(src);
            voltageSensorADCConfigMutable(index)->divider = sbufReadU16(src);
            voltageSensorADCConfigMutable(index)->divmul = sbufReadU8(src);
        } else {
            sbufReadU16(src);
            sbufReadU16(src);
            sbufReadU8(src);
        }
        break;
    }

    case MSP_SET_CURRENT_METER_CONFIG: {
        uint8_t id = sbufReadU8(src);
        int index;
        for (index = 0; index < MAX_CURRENT_SENSOR_ADC; index++) {
            if (id == currentSensorToMeterMap[index])
                break;
        }
        if (index < MAX_CURRENT_SENSOR_ADC) {
            currentSensorADCConfigMutable(index)->scale = sbufReadU16(src);
            currentSensorADCConfigMutable(index)->offset = sbufReadU16(src);
        } else {
            sbufReadU16(src);
            sbufReadU16(src);
        }
        break;
    }

    case MSP_SET_BATTERY_CONFIG: {
        // Legacy fields: values of the active battery profile
        const uint8_t profile = batteryConfig()->batteryProfile;
        batteryConfigMutable()->batteryCapacity[profile] = sbufReadU16(src);
        batteryConfigMutable()->batteryCellCount[profile] = sbufReadU8(src);
        batteryConfigMutable()->voltageMeterSource = sbufReadU8(src);
        batteryConfigMutable()->currentMeterSource = sbufReadU8(src);
        batteryConfigMutable()->vbatmincellvoltage[profile] = sbufReadU16(src);
        batteryConfigMutable()->vbatmaxcellvoltage[profile] = sbufReadU16(src);
        batteryConfigMutable()->vbatfullcellvoltage[profile] = sbufReadU16(src);
        batteryConfigMutable()->vbatwarningcellvoltage[profile] = sbufReadU16(src);
        batteryConfigMutable()->lvcPercentage = sbufReadU8(src);
        batteryConfigMutable()->consumptionWarningPercentage = sbufReadU8(src);
        // All battery profiles
        if (sbufBytesRemaining(src) >= 2 * BATTERY_PROFILE_COUNT) {
            for (int i = 0; i < BATTERY_PROFILE_COUNT; i++)
                batteryConfigMutable()->batteryCapacity[i] = sbufReadU16(src);
        }
        if (sbufBytesRemaining(src) >= 9 * BATTERY_PROFILE_COUNT) {
            for (int i = 0; i < BATTERY_PROFILE_COUNT; i++)
                batteryConfigMutable()->batteryCellCount[i] = sbufReadU8(src);
            for (int i = 0; i < BATTERY_PROFILE_COUNT; i++)
                batteryConfigMutable()->vbatmincellvoltage[i] = sbufReadU16(src);
            for (int i = 0; i < BATTERY_PROFILE_COUNT; i++)
                batteryConfigMutable()->vbatmaxcellvoltage[i] = sbufReadU16(src);
            for (int i = 0; i < BATTERY_PROFILE_COUNT; i++)
                batteryConfigMutable()->vbatfullcellvoltage[i] = sbufReadU16(src);
            for (int i = 0; i < BATTERY_PROFILE_COUNT; i++)
                batteryConfigMutable()->vbatwarningcellvoltage[i] = sbufReadU16(src);
        }
        break;
    }

#if defined(USE_SMARTFUEL)
    case MSP2_SET_SMARTFUEL_CONFIG:
        batteryConfigMutable()->smartfuel_mode = sbufReadU8(src);
        batteryConfigMutable()->smartfuel_voltage_drop_rate = sbufReadU8(src);
        batteryConfigMutable()->smartfuel_charge_drop_rate = sbufReadU8(src);
        batteryConfigMutable()->smartfuel_sag_gain = sbufReadU8(src);
        smartFuelInit();
        break;

#endif

    case MSP2_WING_SET_GOVERNOR_CONFIG:
        governorConfigMutable()->governor_mode = sbufReadU8(src);
        governorConfigMutable()->governor_rpm = sbufReadU16(src);
        governorConfigMutable()->governor_gain = sbufReadU16(src);
        governorConfigMutable()->governor_i_gain = sbufReadU16(src);
        governorConfigMutable()->governor_throttle = sbufReadU8(src);
        governorConfigMutable()->governor_handover = sbufReadU8(src);
        governorConfigMutable()->governor_ceiling = sbufReadU8(src);
        governorConfigMutable()->governor_rpm_min = sbufReadU16(src);
        governorConfigMutable()->governor_rpm_max = sbufReadU16(src);
        break;

#if (defined(USE_FBUS_MASTER) || defined(USE_SPORT_MASTER))
    case MSP2_WING_SET_FBUS_MASTER_CONFIG:
        for (int i = 0; i < FBUS_MASTER_MAX_FORWARDED_SENSORS; i++) {
            fbusMasterConfigMutable()->forwardedSensors[i] = sbufReadU8(src);
        }
        // Forwarding buffers are only loaded from config at boot -- reload
        // them now so the change is live immediately, without a reboot.
        fbusSensorInitForwarding();
        break;

#endif

#if defined(USE_RX_INPUT_BACKUP)
    case MSP2_WING_SET_RX_INPUT_BACKUP_CONFIG:
        sbufReadU8(src); // payload version, unused for now
        rxInputBackupConfigMutable()->provider = sbufReadU8(src);
        rxInputBackupConfigMutable()->inverted = sbufReadU8(src);
        rxInputBackupConfigMutable()->halfDuplex = sbufReadU8(src);
        rxInputBackupConfigMutable()->pinSwap = sbufReadU8(src);
        // Unlike MSP2_WING_SET_FBUS_MASTER_CONFIG above, there's no live-reload
        // here - rxInputBackupInit() only (re-)opens the port at boot, so this
        // needs the same save-and-reboot flow every other serial-port function/
        // config change already goes through.
        break;

#endif

    default:
        // we do not know how to handle the (valid) message, indicate error MSP $M!
        return MSP_RESULT_ERROR;
    }
    return MSP_RESULT_ACK;
}
