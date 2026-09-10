# Flight controller hardware

Wingflight ships firmware for four flight controllers, all Matek boards with an STM32 MCU.
Each one has a target directory under `src/main/target/<TARGET>/`, and that directory is
the authoritative description of the board: `target.h` names the pins, sensors and serial
ports, `target.c` lists the timer outputs, and `target.mk` selects the MCU family and the
drivers that are compiled in.

The Wingflight Configurator identifies a connected board by the values below. The
**Target** column is what the Firmware Flasher and `FC.CONFIG.targetName` report, the
**Board identifier** is `TARGET_BOARD_IDENTIFIER` from `target.h`, and the **USB product
string** is what the operating system shows when the board is plugged in.

| Target      | MCU        | Board identifier | USB product string | Gyro / Acc            | Barometer                 | Compass                        | Logging storage        | UARTs            | Timer outputs (`target.c`) | Default receiver UART |
| ----------- | ---------- | ---------------- | ------------------ | --------------------- | ------------------------- | ------------------------------ | ---------------------- | ---------------- | -------------------------- | --------------------- |
| `MATEKF405` | STM32F405  | `MKF4`           | `MatekF4`          | MPU6000 / MPU6500     | BMP280, MS5611, BMP085    | HMC5883, QMC5883, LIS3MDL      | On-board flash, SD card (SPI) | 1, 2, 3, 4, 5    | S1–S7                      | UART2                 |
| `MATEKF411` | STM32F411  | `MK41`           | `MatekF411`        | MPU6000 / MPU6500     | BMP280, MS5611, BMP085    | none                           | SD card (SPI)          | 1, 2             | S1–S6                      | UART1                 |
| `MATEKF722` | STM32F722  | `MKF7`           | `MatekF7`          | MPU6500 / ICM20689    | BMP280, MS5611, BMP085    | HMC5883, QMC5883, LIS3MDL      | SD card (SPI)          | 1, 2, 3, 4, 5    | S1–S8                      | UART2                 |
| `MATEKH743` | STM32H743  | `M743`           | `MATEK-H743`       | MPU6000 / MPU6500     | MS5611, BMP280, DPS310    | HMC5883, QMC5883, LIS3MDL      | SD card (SDIO)         | 1, 2, 3, 4, 6, 7, 8 | S1–S12                  | UART6                 |

Notes on the table:

* "Timer outputs" are the pads that `target.c` assigns a timer to for motor or servo use,
  named as on the Matek silkscreen. Any of them can drive a servo or an ESC; which one does
  what is decided by the mixer and the `resource` assignments, not by the board. See
  [Getting Started](Getting%20Started.md), stage 3 (Wiring), and the `resource`, `timer`
  and `dma` commands in the [CLI](Cli.md) chapter.
* "Default receiver UART" is `SERIALRX_UART` in `target.h`, the port a serial receiver is
  expected on before you change anything. Any UART can be reassigned in
  **Configuration → Ports**.
* Sensor lists are the drivers compiled into the target. Which of them is actually fitted
  depends on the board revision you bought; check the vendor page for your exact unit.
* STM32F411 has less flash and RAM than the other three MCUs. The project
  [README](../README.md) marks F411 support as end-of-life, so prefer an F405, F722 or
  H743 board for a new build.

The current in-tree targets are the four above only. Boards that were supported by
Cleanflight or Betaflight are **not** automatically supported by Wingflight; the
`docs/boards/` directory was pruned to match, see [boards/README.md](boards/README.md).

## Choosing a board

Before buying, count what you will connect: one timer output per servo and per ESC, one
UART for the receiver, and further UARTs for GPS, telemetry, a backup receiver input, ESC
telemetry and so on. A four-servo, one-motor trainer with a serial receiver fits on any of
the four boards; a twin-motor model with flaps, retracts, GPS and a telemetry radio wants the
extra outputs and UARTs of the F722 or the H743.

## Development targets

`src/main/target/` also contains `NUCLEOF722`, `NUCLEOH743` and `SITL`. These are for
firmware developers: the two Nucleo targets run on ST evaluation boards without flight
sensors, and SITL (software in the loop) runs the firmware on a PC. None of them is a
flight controller you can fly.

## Adding a board

Wiring a board that is not in this list is possible through the CLI `resource` commands,
see [Custom Board Configuration](Custom%20Board%20Configuration.md), but it is unsupported
and every pin has to be verified by hand. Adding a proper target means a pull request with a
new `src/main/target/<TARGET>/` directory. For the configurator side — the board drawing
and silkscreen labels shown in the Setup journey — see the configurator's
`docs/adding-a-board-profile.md`.
