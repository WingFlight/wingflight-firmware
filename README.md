# Wingflight

[Wingflight](https://github.com/WingFlight) is a Flight Control software suite designed for
fixed-wing aircraft. It consists of:

- Wingflight Flight Controller Firmware (this repository)
- Wingflight Configurator, for flashing and configuring the flight controller
- Wingflight Blackbox Explorer, for analyzing blackbox flight logs
- Wingflight Lua Scripts, for configuring the flight controller using a transmitter

Wingflight is a fixed-wing fork of [Rotorflight](https://github.com/rotorflight), which is itself
built on Betaflight 4.3. It's important to note that Wingflight is exclusively designed for
fixed-wing aircraft; it does _not_ target multi-rotor or helicopter use, unlike its parent project.


## Information

Tutorials, documentation, and flight videos can be found on the [Wingflight GitHub organization](https://github.com/WingFlight).


## Features

Wingflight has many features inherited from Rotorflight and Betaflight:

* Many receiver protocols: CRSF, S.BUS, FBUS, F.Port, SRXL2, IBUS, XBUS, EXBUS, GHOST, CPPM
* Support for various telemetry protocols: CSRF, S.Port, FBUS, HoTT, etc.
* ESC telemetry protocols: Hobbywing, Scorpion, Kontronik, Castle, OMP, ZTW, APD, YGE, XDFly, FLYROTOR
* Remote configuration and tuning with the transmitter
  - With knobs / switches assigned to functions
  - With LUA scripts on EdgeTX, OpenTX and Ethos
* Extra servo/motor outputs for AUX functions
* Fully customisable servo/motor mixer
* Sensors for battery voltage, current, BEC, etc.
* Advanced gyro filtering
  - Dynamic RPM based notch filters
  - Dynamic notch filters based on FFT
  - Dynamic LPF
* High-speed Blackbox logging

Plus lots of features inherited from Betaflight:

* Configuration profiles for changing various tuning parameters
* Rates profiles for changing the stick feel and agility
* Multiple ESC protocols: PWM, DSHOT, Multishot, etc.
* Multi-color RGB LEDs
* GPS support (telemetry & logging only)

And much more...

> Note: this feature list is inherited from Rotorflight and hasn't yet been audited for what
> applies to fixed-wing aircraft specifically (e.g. heli-only features like rotor speed governor,
> swash/PID tuning, and Tail Torque Assist have been dropped from the list above, but the
> remaining items still need a fixed-wing accuracy pass).


## Hardware support

Wingflight inherits its supported flight controller boards from Rotorflight (see
[WingFlight/wingflight-targets](https://github.com/WingFlight/wingflight-targets)), which in turn
supports all flight controller boards supported by Betaflight 4.3, assuming the board has enough
suitable I/O pins for connecting all the servos and motors required.

Wingflight supports STM32F405, STM32F722, STM32F745 and STM32H743 MCUs from ST.

The support for lesser MCUs like STM32G474 and STM32F411 is EOL and will be removed soon.


## Installation

Download and flash the Wingflight firmware with the
[Wingflight Configurator](https://github.com/WingFlight/wingflight-configurator/releases).

Flashing the Wingflight firmware with any other flashing tools is not recommended.


## Contributing

Wingflight is an open-source community project. Anybody can join in and help to make it better by:

* helping other users in [GitHub Discussions](https://github.com/WingFlight) or other online forums
* [reporting](https://github.com/WingFlight) bugs and issues, and suggesting improvements
* testing new software versions, new features and fixes; and providing feedback
* participating in discussions on new features
* contributing to the software development - fixing bugs, implementing new features and improvements
* translating Wingflight Configurator into a new language, or helping to maintain an existing translation


## Origins

Wingflight is software that is **open source** and is available free of charge without warranty.

Wingflight is forked from [Rotorflight](https://github.com/rotorflight), which in turn is forked from
[Betaflight](https://github.com/betaflight), which in turn is forked from [Cleanflight](https://github.com/cleanflight).

Big thanks to everyone who has contributed along the journey!


## Contact

Team Wingflight can be contacted via [GitHub Issues and Discussions](https://github.com/WingFlight).
