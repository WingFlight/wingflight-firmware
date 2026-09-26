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
| Simulator -> Wingflight | `udp://127.0.0.1:9003` | `fdm_packet`: IMU, attitude quaternion, velocity, NED position; `baro_packet`: absolute pressure and temperature |
| UARTx <-> MSP clients | `tcp://127.0.0.1:576x` | one client per port; config defaults: 5761 RC, 5762 GPS feed (`GPS_MSP`), 5763 Configurator |
| UART4 <-> FBUS sensors | `tcp://127.0.0.1:5764` | config default: FBUS master; the simulator answers as a FrSky ESC and echoes each master frame, as the single wire would |

Both structs are defined in [target.h](target.h). `motor_speed[i]` carries M(i+1),
so the wing throttle M1 is `motor_speed[0]`. SITL's RX is `FEATURE_RX_MSP`, so RC
arrives as `MSP_SET_RAW_RC`; SITL-specific config defaults are in [config.c](config.c):
GPS provider MSP, MSP on UART2/UART3, FBUS master on UART4 with
`esc_sensor_protocol = FBUS`, and battery voltage/current from the ESC (there is no ADC).

`udpThread()` tells `fdm_packet` and `baro_packet` apart by size, so a
simulator that sends no `baro_packet` still works: the fake baro then gets an
ISA pressure derived from the `fdm_packet` altitude above the start point. The
fake baro reports nothing until it has a first sample, so it never calibrates
on a made-up value.

TCP serial ports deliver received bytes to a port's `rxCallback`, as a UART
interrupt would, on the port's own thread. Anything such a callback touches
runs concurrently with the main loop; `micros()`/`millis()` are locked for
that reason (see target.c).

### Blackbox

SITL has onboard dataflash: `blackbox_flash.bin` in the working directory,
next to `eeprom.bin`, driven by [drivers/flash_file.c](../../drivers/flash_file.c)
as a 128 MiB NOR chip (`USE_FLASH_FILE`). flashfs, the blackbox FLASH device
(the default here), rolling erase, the CLI's `flash_*` commands and MSP's
dataflash commands all run on it unchanged. The file only grows as far as
data has been written, and every write is flushed, so logs survive SITL being
killed. SITL also sets its RTC from the host clock at boot, so logs carry
real start times.

The file flash deliberately registers no `flashConfig` PG: a newly registered
PG missing from `eeprom.bin` makes the firmware reset the whole config.

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
