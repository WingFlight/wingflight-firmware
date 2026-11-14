<p align="center">
  <picture>
    <source media="(prefers-color-scheme: dark)" srcset="https://raw.githubusercontent.com/WingFlight/wingflight-landing-page/master/logos/wf_logo_dark_white.svg">
    <img src="https://raw.githubusercontent.com/WingFlight/wingflight-landing-page/master/logos/wf_logo_dark.svg" alt="Wingflight" width="420">
  </picture>
</p>

# Wingflight Firmware

Wingflight is open-source flight controller firmware for fixed-wing aircraft: 3D and aerobatic
planes, jets, gliders and sport models. It is a gyro stabilizer for line-of-sight flying, with the
setup, tuning and logging tools to go with it.

This repository is the firmware. It is for fixed-wing aircraft only; it does not fly helicopters
or multirotors.

**[Documentation](https://doc.wingflight.org)** ·
**[Getting started](https://doc.wingflight.org/getting-started/)** ·
**[Discord](https://discord.gg/aEyyAJTXRw)** ·
**[Releases](https://github.com/WingFlight/wingflight-firmware/releases)**

> Wingflight is under active development. Firmware is published as numbered snapshots (`0.0.x`),
> not stable releases, and settings can change between them.


## Features

**Flight control**

* Rate stabilization built for control surfaces, with throttle-based gain attenuation
* [Attitude Hold](https://doc.wingflight.org/flight-modes/atthold/): holds the attitude the stick
  was released at, on each axis
* Angle (self-levelling) and [Trainer](https://doc.wingflight.org/flight-modes/trainer/) modes
* [Thrust vectoring](https://doc.wingflight.org/configurator/tabs/thrust-vector/), with its own
  attitude hold for hovering and post-stall flight
* 3D helpers: Snap Relax, Prop-Hang Relax and Cross-Axis Relax
* [Auto Trim](https://doc.wingflight.org/flight-modes/auto-trim/), and trims from the transmitter
* Passthrough and Manual modes for setup and for flying without the gyro
* [Throttle governor](https://doc.wingflight.org/flight-modes/governor/) for idle hold or RPM control
* Experimental [GPS return-to-home and loiter](https://doc.wingflight.org/flight-modes/gps-rth/)

**Outputs**

* Up to 24 servos, each on a PWM pin or a bus channel (S.BUS, FBUS)
* Rule-based mixer: ailerons, elevons, V-tail, flaps, differential thrust and custom layouts
* Motor protocols: PWM, OneShot, Multishot, DShot, Castle Link and SRXL2

**Receivers and telemetry**

* Receivers: CRSF/ELRS, S.BUS, F.Port, FBUS, SRXL2, Spektrum DSM, IBUS, SUMD, XBUS, Jeti EX Bus, GHOST
* A [backup receiver](https://doc.wingflight.org/flight-modes/backup-rx-input/) that takes over
  when the main link is lost
* Telemetry: CRSF, S.Port, FBUS, SRXL2, HoTT, IBUS and Jeti EX Bus
* ESC telemetry: BLHeli32, Hobbywing, Scorpion, Kontronik, OMP Hobby, ZTW, APD, YGE, FLYROTOR,
  Graupner, XDFly, FBUS and SRXL2 ESCs

**Tuning and tools**

* PID and rate profiles, switchable in flight
* Adjustments from transmitter knobs and switches, and logic conditions
* Configuration from the transmitter with Lua scripts for Ethos and EdgeTX
* Gyro filtering with dynamic notch and RPM filters
* High-rate Blackbox logging to onboard flash or SD card
* Battery voltage, current and BEC sensors, barometer, GPS and RGB LEDs


## The Wingflight project

| | |
| --- | --- |
| **Firmware** | This repository |
| [Configurator](https://github.com/WingFlight/wingflight-configurator) | Flashes and configures the flight controller, on the desktop or in a browser |
| [Blackbox Explorer](https://github.com/WingFlight/wingflight-blackbox) | Analyses flight logs |
| [Lua suite for Ethos](https://github.com/WingFlight/wingflight-lua-ethos-suite) | Configuration and dashboard on FrSky Ethos radios |
| [Lua scripts for EdgeTX](https://github.com/WingFlight/wingflight-lua-edgetx) | Configuration on EdgeTX radios |
| [Targets](https://github.com/WingFlight/wingflight-targets) | Board configurations |
| [Documentation](https://github.com/WingFlight/wingflight-docs) | Source of [doc.wingflight.org](https://doc.wingflight.org) |


## Hardware

Wingflight builds one firmware per MCU family: STM32F405, STM32F411, STM32F7X2, STM32F745, STM32G47X
and STM32H743. The board itself (pins and defaults) is a configuration applied on top, kept in
[wingflight-targets](https://github.com/WingFlight/wingflight-targets). A board needs enough
suitable outputs for the model's servos and motors.

A flight controller running Rotorflight can be converted with
[Rotorflight to Wingflight](https://github.com/WingFlight/rotorflight-to-wingflight).


## Installation

Flash the firmware with the
[Wingflight Configurator](https://github.com/WingFlight/wingflight-configurator/releases). It picks
the right firmware and board configuration for you. See
[Flashing the Firmware](https://doc.wingflight.org/getting-started/flashing-the-firmware/).

Flashing with other tools is not recommended.


## Building

```
make arm_sdk_install      # once: installs the ARM toolchain under tools/
make TARGET=STM32F7X2     # one target; `make unified` builds all
make test                 # unit tests
```

See [Building the Firmware](https://doc.wingflight.org/contributing/building-the-firmware/) and
[AGENTS.md](AGENTS.md) for the layout of the code and the rules for changing it.


## Contributing

Wingflight is a community project. You can help by:

* testing snapshots and reporting what you find, with a Blackbox log where it helps
* [reporting bugs](https://github.com/WingFlight/wingflight-firmware/issues) and suggesting improvements
* helping other pilots on [Discord](https://discord.gg/aEyyAJTXRw)
* fixing bugs and adding features (see [Contributing](https://doc.wingflight.org/contributing/))
* improving the [documentation](https://github.com/WingFlight/wingflight-docs)


## Origins

Wingflight is open source and free of charge, with no warranty.

It is a fork of [Rotorflight](https://github.com/rotorflight), which is a fork of
[Betaflight](https://github.com/betaflight) 4.3, which is a fork of
[Cleanflight](https://github.com/cleanflight). Thank you to everyone who contributed along the way.
