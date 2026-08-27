## SITL with JSBSim + FlightGear (Wingflight, recommended)

This is the current, actively-maintained way to fly Wingflight's `SITL` target with
real flight dynamics: [JSBSim](https://jsbsim.sourceforge.net/) computes the physics
(fixed-wing aircraft models, e.g. the bundled `c172p`), and
[FlightGear](https://www.flightgear.org/) optionally renders it. The legacy Gazebo 8
workflow below still exists as an alternative, but is unmaintained/untested against
current Wingflight.

For full details (toolchain install, protocol design, gotchas, and version pinning),
see [docs/development/SITL JSBSim FlightGear Plan.md](../../../docs/development/SITL%20JSBSim%20FlightGear%20Plan.md).

### Quick start

```powershell
# One-time, on a fresh clone (tools/ is gitignored, so nothing is there yet):
make mingw_sdk_install                                        # native C toolchain
.\scripts\sitl-jsbsim-flightgear-launch.ps1 -SetupVenv         # JSBSim/pygame venv

# Build SITL, start it, start the JSBSim bridge (c172p by default), and validate the
# full RC -> mixer -> JSBSim -> attitude loop end-to-end:
.\scripts\sitl-rc-check.ps1 -Mode jsbsim -BuildSitl -AutoStartSitl -StopSitlOnExit

# For interactive flying/visualization instead of a one-shot validation run, use the
# JSBSim + FlightGear launcher (-Joystick needs a USB joystick; without an RC source
# the mixer just sits at failsafe. FlightGear is optional and not vendored - install
# it manually, see the plan doc's install instructions):
.\scripts\sitl-jsbsim-flightgear-launch.ps1 -BuildSitl -Trim -Joystick -FlightGear `
    -FgfsPath "C:\Program Files\FlightGear <version>\bin\fgfs.exe" -StopOnExit
```

If a run looks dead (`packets_rx=0` in the bridge output), check that no leftover
process still holds the simulator's UDP ports: `Get-NetUDPEndpoint -LocalPort 9002`.
And delete any old `obj/main/wingflight_SITL.exe` - `make` builds
`wingflight_SITL.elf`, and a stale `.exe` beside it is a well-worn trap.

Key pieces:
- [scripts/jsbsim_bridge.py](../../../scripts/jsbsim_bridge.py) — Python bridge:
  receives Wingflight's `servo_packet` (control surfaces + motor speed) over UDP,
  drives the corresponding JSBSim FCS properties, steps the simulation, and sends
  JSBSim's resulting state back as an `fdm_packet` (same wire format the legacy
  Gazebo path used, see the note below). Optional `--flightgear` flag additionally
  makes JSBSim emit its own native FlightGear UDP FDM stream for visualization.
- [scripts/sitl-jsbsim-flightgear-launch.ps1](../../../scripts/sitl-jsbsim-flightgear-launch.ps1) —
  one-shot launcher for SITL + the bridge + (optionally) FlightGear.
- [scripts/sitl-rc-check.ps1](../../../scripts/sitl-rc-check.ps1) `-Mode jsbsim` —
  drives roll/pitch RC to each extreme through the disarmed mixer-passthrough
  override and asserts JSBSim's resulting `MSP_ATTITUDE` actually changes, proving
  the entire simulation loop (RC → mixer → bridge → JSBSim physics → fake IMU → MSP)
  is alive, not just Wingflight's own servo output.
- For manual/interactive control (USB joystick/gamepad instead of the automated RC
  checks above), see the joystick section further down — it works the same way
  regardless of which physics backend (Gazebo or JSBSim) is driving the simulation.

Pinned versions that are known to work together (JSBSim, MinGW-w64, Python venv) are
recorded in the plan doc's §9 — update them there if you upgrade any component.

## Legacy: SITL in gazebo 8 with ArduCopterPlugin
SITL (software in the loop) simulator allows you to run betaflight/cleanflight without any hardware.
Currently only tested on Ubuntu 16.04, x86_64, gcc (Ubuntu 5.4.0-6ubuntu1~16.04.4) 5.4.0 20160609.

### install gazebo 8
see here: [Installation](http://gazebosim.org/tutorials?cat=install)

### copy & modify world
for Ubunutu 16.04:
`cp /usr/share/gazebo-8/worlds/iris_arducopter_demo.world .`

change `real_time_update_rate` in `iris_arducopter_demo.world`:
`<real_time_update_rate>0</real_time_update_rate>`
to
`<real_time_update_rate>100</real_time_update_rate>`
***this suggest set to non-zero***

`100` mean what speed your computer should run in (Hz).
Faster computer can set to a higher rate.
see [here](http://gazebosim.org/tutorials?tut=modifying_world&cat=build_world#PhysicsProperties) for detail.
`max_step_size` should NOT higher than `0.0025` as I tested.
smaller mean more accurate, but need higher speed CPU to run as realtime.

### build betaflight
run `make TARGET=SITL`

### settings
to avoid simulation speed slow down, suggest to set some settings belows:

In `configuration` page:

1. `ESC/Motor`: `PWM`, disable `Motor PWM speed Sparted from PID speed`
2. `PID loop frequency` as high as it can.

### start and run
1. start betaflight: `./obj/main/betaflight_SITL.elf`
2. start gazebo: `gazebo --verbose ./iris_arducopter_demo.world`
4. connect your transmitter and fly/test, I used a app to send `MSP_SET_RAW_RC`, code available [here](https://github.com/cs8425/msp-controller).

### flying with a USB joystick/gamepad (Wingflight)
For a documented, ready-to-use tool that reads a USB joystick/gamepad and feeds it
into SITL as RC input (with a GUI for binding axes/buttons to RC channels), see
[docs/development/SITL Joystick RC Input.md](../../../docs/development/SITL%20Joystick%20RC%20Input.md)
and [scripts/sitl-joystick-rc.py](../../../scripts/sitl-joystick-rc.py).

### note
Wingflight	->	sim (gazebo or the JSBSim bridge)	`udp://127.0.0.1:9002`
sim (gazebo or the JSBSim bridge)	->	Wingflight	`udp://127.0.0.1:9003`

The wire format on these two ports (`servo_packet` out / `fdm_packet` in) is shared
by both physics backends — [scripts/jsbsim_bridge.py](../../../scripts/jsbsim_bridge.py)
speaks the exact same protocol Gazebo's `ArduCopterPlugin` did, so nothing in
`target.c`/`udplink.c` needed to change to add JSBSim support.

UARTx will bind on `tcp://127.0.0.1:576x` when port been open.

`eeprom.bin`, size 8192 Byte, is for config saving.
size can be changed in `src/main/target/SITL/pg.ld` >> `__FLASH_CONFIG_Size`
