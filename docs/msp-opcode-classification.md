# MSP opcode classification

Every opcode `msp/msp.c` handles, sorted by what step 5 of
[parameter-addressing-design.md](parameter-addressing-design.md) does to it, and which of
Wingflight's own clients send it today. Generated from the source on 2026-09-26: firmware
dispatch functions, `MSPCodes` references in the configurator (by opcode number; reply
handlers excluded), and the `*_COMMAND` constants in the Lua suite. The addressed
`MSP2_WING_*` opcodes in `msp_param.c` / `msp_runtime.c` are not listed; they stay.

**C** = sent by wingflight-configurator, **L** = sent by wingflight-lua-ethos-suite.

| class | opcodes | C | L | neither |
| --- | --- | --- | --- | --- |
| frozen | 16 | 14 | 8 | 2 |
| keep | 35 | 29 | 14 | 4 |
| live | 28 | 24 | 4 | 4 |
| config | 130 | 110 | 75 | 14 |

## What this means for step 5

- **The configurator's tabs are the bulk of it.** Step 5 cannot land until every *config*
  opcode marked C has been replaced by addressed access in the tab that sends it (step 3).
- **The Lua suite is a second client with the same problem, and the plan did not list it.**
  §13 assumed a Wingflight Lua script would have to be *written* against `PARAM_READ` /
  `PARAM_WRITE`; one already exists and sends the *config* opcodes marked L. It has to be
  migrated too, and it cannot fetch a manifest the way the configurator does -- see §14.
- *Config* opcodes marked neither are used by no Wingflight client; deleting them breaks
  only upstream tools, which §13 accepts.
- *Keep* is what `msp/msp_compat.c` must hold alongside *frozen*. It is longer than §8.2
  listed: the configurator's connect flow alone needs `MSP_SET_ARMING_DISABLED` and
  `MSP_SET_RTC`, and calibration, overrides and the trial/auto-align actions are not config.

## Frozen (§8.1) — third-party ESC tools

| # | opcode | C | L |
| --- | --- | --- | --- |
| 1 | `MSP_API_VERSION` | C | L |
| 2 | `MSP_FC_VARIANT` | C |  |
| 3 | `MSP_FC_VERSION` | C | L |
| 4 | `MSP_BOARD_INFO` | C |  |
| 5 | `MSP_BUILD_INFO` | C |  |
| 68 | `MSP_REBOOT` | C | L |
| 101 | `MSP_STATUS` | C | L |
| 104 | `MSP_MOTOR` | C |  |
| 123 | `MSP_ESC_SENSOR_CONFIG` | C | L |
| 131 | `MSP_MOTOR_CONFIG` | C | L |
| 160 | `MSP_UID` | C | L |
| 214 | `MSP_SET_MOTOR` |  |  |
| 216 | `MSP_SET_ESC_SENSOR_CONFIG` | C | L |
| 245 | `MSP_SET_PASSTHROUGH` | C |  |
| `0x3003` | `MSP2_SEND_DSHOT_COMMAND` |  |  |
| `0x5F14` | `MSP2_WING_ESC_SENSOR_TRIAL` | C |  |

## Keep — actions and non-config state

| # | opcode | C | L |
| --- | --- | --- | --- |
| 71 | `MSP_DATAFLASH_READ` | C |  |
| 72 | `MSP_DATAFLASH_ERASE` | C | L |
| 98 | `MSP_CAMERA_CONTROL` |  |  |
| 99 | `MSP_SET_ARMING_DISABLED` | C |  |
| 161 | `MSP_SET_XACT_SCAN` | C |  |
| 186 | `MSP_SET_TX_INFO` |  |  |
| 190 | `MSP_MIXER_OVERRIDE` | C |  |
| 191 | `MSP_SET_MIXER_OVERRIDE` | C | L |
| 192 | `MSP_SERVO_OVERRIDE` | C |  |
| 193 | `MSP_SET_SERVO_OVERRIDE` | C | L |
| 194 | `MSP_MOTOR_OVERRIDE` | C |  |
| 195 | `MSP_SET_MOTOR_OVERRIDE` | C |  |
| 196 | `MSP_SET_SERVO_OVERRIDE_ALL` |  | L |
| 200 | `MSP_SET_RAW_RC` | C |  |
| 201 | `MSP_SET_RAW_GPS` |  |  |
| 205 | `MSP_ACC_CALIBRATION` | C | L |
| 206 | `MSP_MAG_CALIBRATION` | C |  |
| 208 | `MSP_RESET_CONF` | C |  |
| 210 | `MSP_SELECT_SETTING` | C | L |
| 211 | `MSP_SET_HEADING` |  |  |
| 213 | `MSP_SET_SERVO_CENTER` |  | L |
| 217 | `MSP_ESC_PARAMETERS` | C | L |
| 218 | `MSP_SET_ESC_PARAMETERS` | C | L |
| 219 | `MSP_SET_RESET_CURR_PID` | C |  |
| 230 | `MSP_MULTIPLE_MSP` | C |  |
| 244 | `MSP_SET_4WIF_ESC_FWD_PROG` | C | L |
| 246 | `MSP_SET_RTC` | C | L |
| 250 | `MSP_EEPROM_WRITE` | C | L |
| `0x3000` | `MSP2_BETAFLIGHT_BIND` | C |  |
| `0x5F00` | `MSP2_WING_BOARD_AUTO_ALIGN` | C |  |
| `0x5F05` | `MSP2_WING_BOARD_MOUNT_TRIM_AUTO` | C | L |
| `0x5F08` | `MSP2_WING_CLEAR_FBUS_SENSORS` | C |  |
| `0x5F10` | `MSP2_WING_SELECT_TV_PROFILE` | C | L |
| `0x5F12` | `MSP2_WING_RX_SERIAL_TRIAL` | C |  |
| `0x5F13` | `MSP2_WING_RX_INPUT_BACKUP_TRIAL` | C |  |

## Live — reports state, not settings (§11.1)

| # | opcode | C | L |
| --- | --- | --- | --- |
| 58 | `MSP_SONAR_ALTITUDE` | C |  |
| 70 | `MSP_DATAFLASH_SUMMARY` | C | L |
| 79 | `MSP_SDCARD_SUMMARY` | C | L |
| 102 | `MSP_RAW_IMU` | C |  |
| 103 | `MSP_SERVO` | C |  |
| 105 | `MSP_RC` | C |  |
| 106 | `MSP_RAW_GPS` | C |  |
| 107 | `MSP_COMP_GPS` | C |  |
| 108 | `MSP_ATTITUDE` | C | L |
| 109 | `MSP_ALTITUDE` | C |  |
| 110 | `MSP_ANALOG` | C |  |
| 113 | `MSP_RC_COMMAND` | C |  |
| 114 | `MSP_RX_CHANNELS` | C |  |
| 115 | `MSP_SETPOINT` |  |  |
| 128 | `MSP_VOLTAGE_METERS` | C |  |
| 129 | `MSP_CURRENT_METERS` | C |  |
| 130 | `MSP_BATTERY_STATE` | C |  |
| 139 | `MSP_MOTOR_TELEMETRY` | C |  |
| 164 | `MSP_GPSSVINFO` | C |  |
| 165 | `MSP_XACT_SERVO_LIST` | C |  |
| 187 | `MSP_TX_INFO` |  |  |
| 199 | `MSP_LOGIC_CONDITIONS_STATUS` | C |  |
| 247 | `MSP_RTC` |  |  |
| 254 | `MSP_DEBUG` | C |  |
| `0x3004` | `MSP2_GET_VTX_DEVICE_STATUS` |  |  |
| `0x5F06` | `MSP2_WING_EFFECTIVE_PID_GAINS` | C |  |
| `0x5F07` | `MSP2_WING_FBUS_SENSORS` | C |  |
| `0x5F0D` | `MSP2_WING_RX_INPUT_BACKUP_STATUS` | C | L |

## Config — the catalogue step 5 deletes

| # | opcode | C | L |
| --- | --- | --- | --- |
| 10 | `MSP_NAME` | C | L |
| 11 | `MSP_SET_NAME` | C | L |
| 12 | `MSP_PILOT_CONFIG` | C |  |
| 13 | `MSP_SET_PILOT_CONFIG` | C |  |
| 14 | `MSP_FLIGHT_STATS` | C | L |
| 15 | `MSP_SET_FLIGHT_STATS` | C | L |
| 32 | `MSP_BATTERY_CONFIG` | C | L |
| 33 | `MSP_SET_BATTERY_CONFIG` | C | L |
| 34 | `MSP_MODE_RANGES` | C | L |
| 35 | `MSP_SET_MODE_RANGE` | C | L |
| 36 | `MSP_FEATURE_CONFIG` | C | L |
| 37 | `MSP_SET_FEATURE_CONFIG` | C | L |
| 38 | `MSP_BOARD_ALIGNMENT_CONFIG` | C | L |
| 39 | `MSP_SET_BOARD_ALIGNMENT_CONFIG` | C | L |
| 40 | `MSP_CURRENT_METER_CONFIG` | C |  |
| 41 | `MSP_SET_CURRENT_METER_CONFIG` | C |  |
| 42 | `MSP_MIXER_CONFIG` | C |  |
| 43 | `MSP_SET_MIXER_CONFIG` | C |  |
| 44 | `MSP_RX_CONFIG` | C | L |
| 45 | `MSP_SET_RX_CONFIG` | C |  |
| 46 | `MSP_LED_COLORS` | C |  |
| 47 | `MSP_SET_LED_COLORS` | C |  |
| 48 | `MSP_LED_STRIP_CONFIG` | C |  |
| 49 | `MSP_SET_LED_STRIP_CONFIG` | C |  |
| 50 | `MSP_RSSI_CONFIG` | C |  |
| 51 | `MSP_SET_RSSI_CONFIG` | C |  |
| 52 | `MSP_ADJUSTMENT_RANGES` | C | L |
| 53 | `MSP_SET_ADJUSTMENT_RANGE` | C | L |
| 54 | `MSP_SERIAL_CONFIG` | C | L |
| 55 | `MSP_SET_SERIAL_CONFIG` | C | L |
| 56 | `MSP_VOLTAGE_METER_CONFIG` | C |  |
| 57 | `MSP_SET_VOLTAGE_METER_CONFIG` | C |  |
| 59 | `MSP_DEBUG_CONFIG` | C |  |
| 60 | `MSP_SET_DEBUG_CONFIG` | C |  |
| 61 | `MSP_ARMING_CONFIG` | C | L |
| 62 | `MSP_SET_ARMING_CONFIG` | C | L |
| 64 | `MSP_RX_MAP` | C | L |
| 65 | `MSP_SET_RX_MAP` | C |  |
| 66 | `MSP_RC_CONFIG` | C | L |
| 67 | `MSP_SET_RC_CONFIG` | C | L |
| 73 | `MSP_TELEMETRY_CONFIG` | C | L |
| 74 | `MSP_SET_TELEMETRY_CONFIG` | C | L |
| 75 | `MSP_FAILSAFE_CONFIG` | C |  |
| 76 | `MSP_SET_FAILSAFE_CONFIG` | C |  |
| 77 | `MSP_RXFAIL_CONFIG` | C | L |
| 78 | `MSP_SET_RXFAIL_CONFIG` | C | L |
| 80 | `MSP_BLACKBOX_CONFIG` | C | L |
| 81 | `MSP_SET_BLACKBOX_CONFIG` | C | L |
| 88 | `MSP_VTX_CONFIG` |  |  |
| 89 | `MSP_SET_VTX_CONFIG` |  |  |
| 90 | `MSP_ADVANCED_CONFIG` | C | L |
| 91 | `MSP_SET_ADVANCED_CONFIG` | C | L |
| 92 | `MSP_FILTER_CONFIG` | C | L |
| 93 | `MSP_SET_FILTER_CONFIG` | C | L |
| 94 | `MSP_PID_PROFILE` | C | L |
| 95 | `MSP_SET_PID_PROFILE` | C | L |
| 96 | `MSP_SENSOR_CONFIG` | C |  |
| 97 | `MSP_SET_SENSOR_CONFIG` | C |  |
| 111 | `MSP_RC_TUNING` | C | L |
| 112 | `MSP_PID_TUNING` | C | L |
| 116 | `MSP_BOXNAMES` | C | L |
| 119 | `MSP_BOXIDS` | C | L |
| 120 | `MSP_SERVO_CONFIGURATIONS` | C |  |
| 124 | `MSP_SET_SERVO_CONFIG` |  |  |
| 125 | `MSP_GET_SERVO_CONFIG` |  | L |
| 126 | `MSP_SENSOR_ALIGNMENT` | C | L |
| 127 | `MSP_LED_STRIP_MODECOLOR` | C |  |
| 132 | `MSP_GPS_CONFIG` | C |  |
| 135 | `MSP_GPS_RESCUE` |  |  |
| 136 | `MSP_GPS_RESCUE_PIDS` |  |  |
| 137 | `MSP_VTXTABLE_BAND` |  |  |
| 138 | `MSP_VTXTABLE_POWERLEVEL` |  |  |
| 150 | `MSP_LED_STRIP_SETTINGS` | C |  |
| 151 | `MSP_SET_LED_STRIP_SETTINGS` | C |  |
| 154 | `MSP_RPM_FILTER_V2` | C |  |
| 155 | `MSP_SET_RPM_FILTER_V2` | C |  |
| 156 | `MSP_GET_ADJUSTMENT_RANGE` |  | L |
| 158 | `MSP_EXPERIMENTAL` |  | L |
| 159 | `MSP_SET_EXPERIMENTAL` |  | L |
| 162 | `MSP_XACT_PARAMS` | C |  |
| 163 | `MSP_SET_XACT_PARAMS` | C |  |
| 167 | `MSP_GET_ADJUSTMENT_FUNCTION_IDS` |  |  |
| 170 | `MSP_MIXER_INPUTS` | C |  |
| 171 | `MSP_SET_MIXER_INPUT` | C | L |
| 172 | `MSP_MIXER_RULES` | C | L |
| 173 | `MSP_SET_MIXER_RULE` | C | L |
| 174 | `MSP_GET_MIXER_INPUT` |  | L |
| 175 | `MSP_BATTERY_PROFILE` |  | L |
| 176 | `MSP_SET_BATTERY_PROFILE` | C | L |
| 177 | `MSP_MIXER_CURVES` | C | L |
| 178 | `MSP_SET_MIXER_CURVE` | C | L |
| 183 | `MSP_COPY_PROFILE` | C | L |
| 184 | `MSP_BEEPER_CONFIG` | C | L |
| 185 | `MSP_SET_BEEPER_CONFIG` | C | L |
| 188 | `MSP_GAIN_CURVES` | C | L |
| 189 | `MSP_SET_GAIN_CURVE` | C | L |
| 197 | `MSP_LOGIC_CONDITIONS` | C |  |
| 198 | `MSP_SET_LOGIC_CONDITION` | C |  |
| 202 | `MSP_SET_PID_TUNING` | C | L |
| 204 | `MSP_SET_RC_TUNING` | C | L |
| 212 | `MSP_SET_SERVO_CONFIGURATION` | C | L |
| 220 | `MSP_SET_SENSOR_ALIGNMENT` | C | L |
| 221 | `MSP_SET_LED_STRIP_MODECOLOR` | C |  |
| 222 | `MSP_SET_MOTOR_CONFIG` | C | L |
| 223 | `MSP_SET_GPS_CONFIG` | C |  |
| 225 | `MSP_SET_GPS_RESCUE` |  |  |
| 226 | `MSP_SET_GPS_RESCUE_PIDS` |  |  |
| 227 | `MSP_SET_VTXTABLE_BAND` |  |  |
| 228 | `MSP_SET_VTXTABLE_POWERLEVEL` |  |  |
| 231 | `MSP_SERVO_CURVES` | C | L |
| 232 | `MSP_SET_SERVO_CURVE` | C | L |
| 233 | `MSP_SERVO_TRIM` | C |  |
| 238 | `MSP_MODE_RANGES_EXTRA` | C | L |
| 239 | `MSP_SET_ACC_TRIM` | C | L |
| 240 | `MSP_ACC_TRIM` | C | L |
| 248 | `MSP_SET_BOARD_INFO` |  |  |
| 249 | `MSP_SET_SIGNATURE` |  |  |
| `0x4000` | `MSP2_GET_SMARTFUEL_CONFIG` | C | L |
| `0x4001` | `MSP2_SET_SMARTFUEL_CONFIG` | C | L |
| `0x5F01` | `MSP2_WING_GOVERNOR_CONFIG` | C | L |
| `0x5F02` | `MSP2_WING_SET_GOVERNOR_CONFIG` | C | L |
| `0x5F03` | `MSP2_WING_BOARD_MOUNT_TRIM` | C | L |
| `0x5F04` | `MSP2_WING_SET_BOARD_MOUNT_TRIM` | C | L |
| `0x5F09` | `MSP2_WING_FBUS_MASTER_CONFIG` | C |  |
| `0x5F0A` | `MSP2_WING_SET_FBUS_MASTER_CONFIG` | C |  |
| `0x5F0B` | `MSP2_WING_TV_PID_CONFIG` | C | L |
| `0x5F0C` | `MSP2_WING_SET_TV_PID_CONFIG` | C | L |
| `0x5F0E` | `MSP2_WING_RX_INPUT_BACKUP_CONFIG` | C |  |
| `0x5F0F` | `MSP2_WING_SET_RX_INPUT_BACKUP_CONFIG` | C |  |
| `0x5F11` | `MSP2_WING_COPY_TV_PID_PROFILE` | C | L |
