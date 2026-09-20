## SITL

`TARGET=SITL` builds Wingflight as a native executable that talks to an external
flight dynamics model over UDP. The simulator tooling (JSBSim bridge, launcher,
aircraft models, joystick RC, automated checks) lives in the
[wingflight-sitl-hitl](https://github.com/WingFlight/wingflight-sitl-hitl) repo,
whose `setup-sitl.ps1` checks out, builds and launches everything.

### Build

```sh
make mingw_sdk_install      # Windows only, once: native MinGW-w64 GCC into tools/mingw64
make TARGET=SITL            # -> obj/main/wingflight_SITL.elf (a native executable)
```

On Linux/macOS the system `gcc` is used. Delete any stale
`obj/main/wingflight_SITL.exe`: `make` only produces the `.elf`.

### Ports

| Direction | Address | Payload |
|---|---|---|
| Wingflight -> simulator | `udp://127.0.0.1:9002` | `servo_packet`: `motor_speed[4]` (M1-M4) + `servo[8]` (S1-S8, us) |
| Simulator -> Wingflight | `udp://127.0.0.1:9003` | `fdm_packet`: IMU, attitude quaternion, velocity, NED position |
| UARTx <-> MSP clients | `tcp://127.0.0.1:576x` | one client per port; config defaults: 5761 RC, 5762 GPS feed (`GPS_MSP`), 5763 Configurator |

Both structs are defined in [target.h](target.h). `motor_speed[i]` carries M(i+1),
so the wing throttle M1 is `motor_speed[0]`. SITL's RX is `FEATURE_RX_MSP`, so RC
arrives as `MSP_SET_RAW_RC`; SITL-specific config defaults are in [config.c](config.c).

### eeprom.bin

`eeprom.bin` in the working directory holds the saved config. Its size is
`EEPROM_SIZE` (32768 bytes) in [target.h](target.h). On a missing `eeprom.bin`,
the first launch writes the default config and exits; start it again. A
wrong-sized file still loads (with a warning) and is rewritten at `EEPROM_SIZE`
on save. A stale `eeprom.bin` from an older build can mask config-default
changes; delete it to re-apply the defaults.

"Save and reboot" writes `eeprom.bin` and then **exits** SITL (there is no
in-process restart). Features SITL doesn't compile in (LED_STRIP, OSD, SOFTSERIAL,
DYN_NOTCH, RPM_FILTER, ...) are cleared when you save, and SITL prints
`[config] features 0x... not supported by this build` on stderr.
