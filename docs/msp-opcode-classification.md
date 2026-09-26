# MSP opcode classification

Every opcode `msp/msp.c` handles, sorted by what step 5 of
[parameter-addressing-design.md](parameter-addressing-design.md) does to it, and which of
Wingflight's own clients send it today. Generated from the source on 2026-09-26: firmware
dispatch functions, `MSPCodes` references in the configurator (by opcode number; reply
handlers excluded), and the `*_COMMAND` constants in the Lua suite. The addressed
`MSP2_WING_*` opcodes in `msp_param.c` / `msp_runtime.c` are not listed; they stay.

**C** = sent by wingflight-configurator, **L** = sent by wingflight-lua-ethos-suite.
**codec** = the STM32F7X2 manifest has an extracted codec for it (`msp_codecs`), so the
clients' virtual MSP layers can answer it from `PARAM_READ` / `PARAM_WRITE`; **manual** =
the extractor left it to be hand-written; **not built** = not compiled into that target.

| class | opcodes | C | L | neither | codec |
| --- | --- | --- | --- | --- | --- |
| frozen | 16 | 14 | 8 | 2 | 4 |
| keep | 35 | 29 | 14 | 4 | 1 |
| runtime | 26 | 23 | 19 | 1 | 0 |
| live | 28 | 24 | 4 | 4 | 1 |
| config | 104 | 87 | 56 | 13 | 89 |

## What this means for step 5

- **Step 5 deletes *config* only.** The clients' pages and tabs keep sending these opcodes;
  the virtual MSP layers (configurator `src/js/param/virtual_msp.js`, suite
  `tasks/msp/virtual.lua`) answer them from the manifest's codecs instead of the firmware.
  On STM32F7X2, 89 of the 104 have a codec, 8 are left manual and
  7 are not built. Step 5 cannot land until every *config* opcode marked C or L
  has a codec verified on each target, with setters routed (stage B). Still manual and
  used by a client: `MSP_CURRENT_METER_CONFIG`, `MSP_SET_CURRENT_METER_CONFIG`, `MSP_VOLTAGE_METER_CONFIG`, `MSP_SET_VOLTAGE_METER_CONFIG`, `MSP_SET_LED_STRIP_MODECOLOR`, `MSP2_WING_SET_TV_PID_CONFIG`.
- *Config* opcodes marked neither are used by no Wingflight client; deleting them breaks
  only upstream tools, which §13 accepts.
- *Runtime* opcodes stay in the firmware. Each reads or writes stored config, but what goes on
  the wire also depends on state only the running firmware has: box IDs resolved against
  the compiled-in box table, ports and servos that exist on this board, channel counts, a
  refusal while logging, RPM filter and XACT state. A codec for them would be a second copy
  of that logic. The profile actions (`MSP_COPY_PROFILE`, `MSP_SET_BATTERY_PROFILE`,
  `MSP2_WING_COPY_TV_PID_PROFILE`) and `MSP_EXPERIMENTAL` are commands, not a byte layout.
- *Keep* and *runtime* are what `msp/msp_compat.c` must hold alongside *frozen*. *Keep* is
  longer than §8.2 listed: the configurator's connect flow alone needs
  `MSP_SET_ARMING_DISABLED` and `MSP_SET_RTC`, and calibration, overrides and the
  trial/auto-align actions are not config.

## Frozen (§8.1) — third-party ESC tools

| # | opcode | C | L | codec |
| --- | --- | --- | --- | --- |
| 1 | `MSP_API_VERSION` | C | L | codec |
| 2 | `MSP_FC_VARIANT` | C |  |  |
| 3 | `MSP_FC_VERSION` | C | L | codec |
| 4 | `MSP_BOARD_INFO` | C |  |  |
| 5 | `MSP_BUILD_INFO` | C |  |  |
| 68 | `MSP_REBOOT` | C | L |  |
| 101 | `MSP_STATUS` | C | L |  |
| 104 | `MSP_MOTOR` | C |  |  |
| 123 | `MSP_ESC_SENSOR_CONFIG` | C | L | codec |
| 131 | `MSP_MOTOR_CONFIG` | C | L |  |
| 160 | `MSP_UID` | C | L |  |
| 214 | `MSP_SET_MOTOR` |  |  |  |
| 216 | `MSP_SET_ESC_SENSOR_CONFIG` | C | L | codec |
| 245 | `MSP_SET_PASSTHROUGH` | C |  |  |
| `0x3003` | `MSP2_SEND_DSHOT_COMMAND` |  |  |  |
| `0x5F14` | `MSP2_WING_ESC_SENSOR_TRIAL` | C |  |  |

## Keep — actions and non-config state

| # | opcode | C | L | codec |
| --- | --- | --- | --- | --- |
| 71 | `MSP_DATAFLASH_READ` | C |  |  |
| 72 | `MSP_DATAFLASH_ERASE` | C | L |  |
| 98 | `MSP_CAMERA_CONTROL` |  |  |  |
| 99 | `MSP_SET_ARMING_DISABLED` | C |  |  |
| 161 | `MSP_SET_XACT_SCAN` | C |  |  |
| 186 | `MSP_SET_TX_INFO` |  |  |  |
| 190 | `MSP_MIXER_OVERRIDE` | C |  |  |
| 191 | `MSP_SET_MIXER_OVERRIDE` | C | L |  |
| 192 | `MSP_SERVO_OVERRIDE` | C |  |  |
| 193 | `MSP_SET_SERVO_OVERRIDE` | C | L |  |
| 194 | `MSP_MOTOR_OVERRIDE` | C |  |  |
| 195 | `MSP_SET_MOTOR_OVERRIDE` | C |  |  |
| 196 | `MSP_SET_SERVO_OVERRIDE_ALL` |  | L |  |
| 200 | `MSP_SET_RAW_RC` | C |  |  |
| 201 | `MSP_SET_RAW_GPS` |  |  |  |
| 205 | `MSP_ACC_CALIBRATION` | C | L |  |
| 206 | `MSP_MAG_CALIBRATION` | C |  |  |
| 208 | `MSP_RESET_CONF` | C |  |  |
| 210 | `MSP_SELECT_SETTING` | C | L |  |
| 211 | `MSP_SET_HEADING` |  |  |  |
| 213 | `MSP_SET_SERVO_CENTER` |  | L | codec |
| 217 | `MSP_ESC_PARAMETERS` | C | L |  |
| 218 | `MSP_SET_ESC_PARAMETERS` | C | L |  |
| 219 | `MSP_SET_RESET_CURR_PID` | C |  |  |
| 230 | `MSP_MULTIPLE_MSP` | C |  |  |
| 244 | `MSP_SET_4WIF_ESC_FWD_PROG` | C | L |  |
| 246 | `MSP_SET_RTC` | C | L |  |
| 250 | `MSP_EEPROM_WRITE` | C | L |  |
| `0x3000` | `MSP2_BETAFLIGHT_BIND` | C |  |  |
| `0x5F00` | `MSP2_WING_BOARD_AUTO_ALIGN` | C |  |  |
| `0x5F05` | `MSP2_WING_BOARD_MOUNT_TRIM_AUTO` | C | L |  |
| `0x5F08` | `MSP2_WING_CLEAR_FBUS_SENSORS` | C |  |  |
| `0x5F10` | `MSP2_WING_SELECT_TV_PROFILE` | C | L |  |
| `0x5F12` | `MSP2_WING_RX_SERIAL_TRIAL` | C |  |  |
| `0x5F13` | `MSP2_WING_RX_INPUT_BACKUP_TRIAL` | C |  |  |

## Runtime — stored config, but runtime state decides the bytes; profile actions

| # | opcode | C | L | codec |
| --- | --- | --- | --- | --- |
| 34 | `MSP_MODE_RANGES` | C | L |  |
| 35 | `MSP_SET_MODE_RANGE` | C | L |  |
| 37 | `MSP_SET_FEATURE_CONFIG` | C | L |  |
| 54 | `MSP_SERIAL_CONFIG` | C | L |  |
| 55 | `MSP_SET_SERIAL_CONFIG` | C | L |  |
| 77 | `MSP_RXFAIL_CONFIG` | C | L |  |
| 78 | `MSP_SET_RXFAIL_CONFIG` | C | L |  |
| 81 | `MSP_SET_BLACKBOX_CONFIG` | C | L |  |
| 116 | `MSP_BOXNAMES` | C | L |  |
| 119 | `MSP_BOXIDS` | C | L |  |
| 120 | `MSP_SERVO_CONFIGURATIONS` | C |  |  |
| 124 | `MSP_SET_SERVO_CONFIG` |  |  |  |
| 154 | `MSP_RPM_FILTER_V2` | C |  |  |
| 155 | `MSP_SET_RPM_FILTER_V2` | C |  |  |
| 158 | `MSP_EXPERIMENTAL` |  | L |  |
| 159 | `MSP_SET_EXPERIMENTAL` |  | L |  |
| 162 | `MSP_XACT_PARAMS` | C |  |  |
| 163 | `MSP_SET_XACT_PARAMS` | C |  |  |
| 176 | `MSP_SET_BATTERY_PROFILE` | C | L |  |
| 183 | `MSP_COPY_PROFILE` | C | L |  |
| 212 | `MSP_SET_SERVO_CONFIGURATION` | C | L |  |
| 231 | `MSP_SERVO_CURVES` | C | L |  |
| 232 | `MSP_SET_SERVO_CURVE` | C | L |  |
| 233 | `MSP_SERVO_TRIM` | C |  |  |
| 238 | `MSP_MODE_RANGES_EXTRA` | C | L |  |
| `0x5F11` | `MSP2_WING_COPY_TV_PID_PROFILE` | C | L |  |

## Live — reports state, not settings (§11.1)

| # | opcode | C | L | codec |
| --- | --- | --- | --- | --- |
| 58 | `MSP_SONAR_ALTITUDE` | C |  | codec |
| 70 | `MSP_DATAFLASH_SUMMARY` | C | L |  |
| 79 | `MSP_SDCARD_SUMMARY` | C | L |  |
| 102 | `MSP_RAW_IMU` | C |  |  |
| 103 | `MSP_SERVO` | C |  |  |
| 105 | `MSP_RC` | C |  |  |
| 106 | `MSP_RAW_GPS` | C |  |  |
| 107 | `MSP_COMP_GPS` | C |  |  |
| 108 | `MSP_ATTITUDE` | C | L |  |
| 109 | `MSP_ALTITUDE` | C |  |  |
| 110 | `MSP_ANALOG` | C |  |  |
| 113 | `MSP_RC_COMMAND` | C |  |  |
| 114 | `MSP_RX_CHANNELS` | C |  |  |
| 115 | `MSP_SETPOINT` |  |  |  |
| 128 | `MSP_VOLTAGE_METERS` | C |  |  |
| 129 | `MSP_CURRENT_METERS` | C |  |  |
| 130 | `MSP_BATTERY_STATE` | C |  |  |
| 139 | `MSP_MOTOR_TELEMETRY` | C |  |  |
| 164 | `MSP_GPSSVINFO` | C |  |  |
| 165 | `MSP_XACT_SERVO_LIST` | C |  |  |
| 187 | `MSP_TX_INFO` |  |  |  |
| 199 | `MSP_LOGIC_CONDITIONS_STATUS` | C |  |  |
| 247 | `MSP_RTC` |  |  |  |
| 254 | `MSP_DEBUG` | C |  |  |
| `0x3004` | `MSP2_GET_VTX_DEVICE_STATUS` |  |  |  |
| `0x5F06` | `MSP2_WING_EFFECTIVE_PID_GAINS` | C |  |  |
| `0x5F07` | `MSP2_WING_FBUS_SENSORS` | C |  |  |
| `0x5F0D` | `MSP2_WING_RX_INPUT_BACKUP_STATUS` | C | L |  |

## Config — the catalogue step 5 deletes

| # | opcode | C | L | codec |
| --- | --- | --- | --- | --- |
| 10 | `MSP_NAME` | C | L | codec |
| 11 | `MSP_SET_NAME` | C | L | codec |
| 12 | `MSP_PILOT_CONFIG` | C |  | codec |
| 13 | `MSP_SET_PILOT_CONFIG` | C |  | codec |
| 14 | `MSP_FLIGHT_STATS` | C | L | codec |
| 15 | `MSP_SET_FLIGHT_STATS` | C | L | codec |
| 32 | `MSP_BATTERY_CONFIG` | C | L | codec |
| 33 | `MSP_SET_BATTERY_CONFIG` | C | L | codec |
| 36 | `MSP_FEATURE_CONFIG` | C | L | codec |
| 38 | `MSP_BOARD_ALIGNMENT_CONFIG` | C | L | codec |
| 39 | `MSP_SET_BOARD_ALIGNMENT_CONFIG` | C | L | codec |
| 40 | `MSP_CURRENT_METER_CONFIG` | C |  | manual |
| 41 | `MSP_SET_CURRENT_METER_CONFIG` | C |  | manual |
| 42 | `MSP_MIXER_CONFIG` | C |  | codec |
| 43 | `MSP_SET_MIXER_CONFIG` | C |  | codec |
| 44 | `MSP_RX_CONFIG` | C | L | codec |
| 45 | `MSP_SET_RX_CONFIG` | C |  | codec |
| 46 | `MSP_LED_COLORS` | C |  | codec |
| 47 | `MSP_SET_LED_COLORS` | C |  | codec |
| 48 | `MSP_LED_STRIP_CONFIG` | C |  | codec |
| 49 | `MSP_SET_LED_STRIP_CONFIG` | C |  | codec |
| 50 | `MSP_RSSI_CONFIG` | C |  | codec |
| 51 | `MSP_SET_RSSI_CONFIG` | C |  | codec |
| 52 | `MSP_ADJUSTMENT_RANGES` | C | L | codec |
| 53 | `MSP_SET_ADJUSTMENT_RANGE` | C | L | codec |
| 56 | `MSP_VOLTAGE_METER_CONFIG` | C |  | manual |
| 57 | `MSP_SET_VOLTAGE_METER_CONFIG` | C |  | manual |
| 59 | `MSP_DEBUG_CONFIG` | C |  | codec |
| 60 | `MSP_SET_DEBUG_CONFIG` | C |  | codec |
| 61 | `MSP_ARMING_CONFIG` | C | L | codec |
| 62 | `MSP_SET_ARMING_CONFIG` | C | L | codec |
| 64 | `MSP_RX_MAP` | C | L | codec |
| 65 | `MSP_SET_RX_MAP` | C |  | codec |
| 66 | `MSP_RC_CONFIG` | C | L | codec |
| 67 | `MSP_SET_RC_CONFIG` | C | L | codec |
| 73 | `MSP_TELEMETRY_CONFIG` | C | L | codec |
| 74 | `MSP_SET_TELEMETRY_CONFIG` | C | L | codec |
| 75 | `MSP_FAILSAFE_CONFIG` | C |  | codec |
| 76 | `MSP_SET_FAILSAFE_CONFIG` | C |  | codec |
| 80 | `MSP_BLACKBOX_CONFIG` | C | L | codec |
| 88 | `MSP_VTX_CONFIG` |  |  | not built |
| 89 | `MSP_SET_VTX_CONFIG` |  |  | not built |
| 90 | `MSP_ADVANCED_CONFIG` | C | L | codec |
| 91 | `MSP_SET_ADVANCED_CONFIG` | C | L | codec |
| 92 | `MSP_FILTER_CONFIG` | C | L | codec |
| 93 | `MSP_SET_FILTER_CONFIG` | C | L | codec |
| 94 | `MSP_PID_PROFILE` | C | L | codec |
| 95 | `MSP_SET_PID_PROFILE` | C | L | codec |
| 96 | `MSP_SENSOR_CONFIG` | C |  | codec |
| 97 | `MSP_SET_SENSOR_CONFIG` | C |  | codec |
| 111 | `MSP_RC_TUNING` | C | L | codec |
| 112 | `MSP_PID_TUNING` | C | L | codec |
| 125 | `MSP_GET_SERVO_CONFIG` |  | L | codec |
| 126 | `MSP_SENSOR_ALIGNMENT` | C | L | codec |
| 127 | `MSP_LED_STRIP_MODECOLOR` | C |  | codec |
| 132 | `MSP_GPS_CONFIG` | C |  | codec |
| 135 | `MSP_GPS_RESCUE` |  |  | codec |
| 136 | `MSP_GPS_RESCUE_PIDS` |  |  | codec |
| 137 | `MSP_VTXTABLE_BAND` |  |  | not built |
| 138 | `MSP_VTXTABLE_POWERLEVEL` |  |  | not built |
| 150 | `MSP_LED_STRIP_SETTINGS` | C |  | codec |
| 151 | `MSP_SET_LED_STRIP_SETTINGS` | C |  | codec |
| 156 | `MSP_GET_ADJUSTMENT_RANGE` |  | L | codec |
| 167 | `MSP_GET_ADJUSTMENT_FUNCTION_IDS` |  |  | codec |
| 170 | `MSP_MIXER_INPUTS` | C |  | codec |
| 171 | `MSP_SET_MIXER_INPUT` | C | L | codec |
| 172 | `MSP_MIXER_RULES` | C | L | codec |
| 173 | `MSP_SET_MIXER_RULE` | C | L | codec |
| 174 | `MSP_GET_MIXER_INPUT` |  | L | codec |
| 175 | `MSP_BATTERY_PROFILE` |  | L | codec |
| 177 | `MSP_MIXER_CURVES` | C | L | codec |
| 178 | `MSP_SET_MIXER_CURVE` | C | L | codec |
| 184 | `MSP_BEEPER_CONFIG` | C | L | codec |
| 185 | `MSP_SET_BEEPER_CONFIG` | C | L | codec |
| 188 | `MSP_GAIN_CURVES` | C | L | codec |
| 189 | `MSP_SET_GAIN_CURVE` | C | L | codec |
| 197 | `MSP_LOGIC_CONDITIONS` | C |  | codec |
| 198 | `MSP_SET_LOGIC_CONDITION` | C |  | codec |
| 202 | `MSP_SET_PID_TUNING` | C | L | codec |
| 204 | `MSP_SET_RC_TUNING` | C | L | codec |
| 220 | `MSP_SET_SENSOR_ALIGNMENT` | C | L | codec |
| 221 | `MSP_SET_LED_STRIP_MODECOLOR` | C |  | manual |
| 222 | `MSP_SET_MOTOR_CONFIG` | C | L | codec |
| 223 | `MSP_SET_GPS_CONFIG` | C |  | codec |
| 225 | `MSP_SET_GPS_RESCUE` |  |  | not built |
| 226 | `MSP_SET_GPS_RESCUE_PIDS` |  |  | codec |
| 227 | `MSP_SET_VTXTABLE_BAND` |  |  | not built |
| 228 | `MSP_SET_VTXTABLE_POWERLEVEL` |  |  | not built |
| 239 | `MSP_SET_ACC_TRIM` | C | L | codec |
| 240 | `MSP_ACC_TRIM` | C | L | codec |
| 248 | `MSP_SET_BOARD_INFO` |  |  | manual |
| 249 | `MSP_SET_SIGNATURE` |  |  | manual |
| `0x4000` | `MSP2_GET_SMARTFUEL_CONFIG` | C | L | codec |
| `0x4001` | `MSP2_SET_SMARTFUEL_CONFIG` | C | L | codec |
| `0x5F01` | `MSP2_WING_GOVERNOR_CONFIG` | C | L | codec |
| `0x5F02` | `MSP2_WING_SET_GOVERNOR_CONFIG` | C | L | codec |
| `0x5F03` | `MSP2_WING_BOARD_MOUNT_TRIM` | C | L | codec |
| `0x5F04` | `MSP2_WING_SET_BOARD_MOUNT_TRIM` | C | L | codec |
| `0x5F09` | `MSP2_WING_FBUS_MASTER_CONFIG` | C |  | codec |
| `0x5F0A` | `MSP2_WING_SET_FBUS_MASTER_CONFIG` | C |  | codec |
| `0x5F0B` | `MSP2_WING_TV_PID_CONFIG` | C | L | codec |
| `0x5F0C` | `MSP2_WING_SET_TV_PID_CONFIG` | C | L | manual |
| `0x5F0E` | `MSP2_WING_RX_INPUT_BACKUP_CONFIG` | C |  | codec |
| `0x5F0F` | `MSP2_WING_SET_RX_INPUT_BACKUP_CONFIG` | C |  | codec |
