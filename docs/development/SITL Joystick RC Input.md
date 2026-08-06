# Flying SITL with a USB Joystick/Gamepad

This documents [scripts/sitl-joystick-rc.py](../../scripts/sitl-joystick-rc.py), a
tool that reads a USB joystick/gamepad and feeds it into Wingflight `TARGET=SITL`
as live RC input, with a small GUI for binding axes/buttons/hats to RC channels.

It complements [scripts/sitl-rc-check.ps1](../../scripts/sitl-rc-check.ps1) (which
drives scripted RC test sequences for validation) — this tool is for interactive,
manual "hands on sticks" flying of SITL.

## How it works

SITL's `DEFAULT_RX_FEATURE` is `FEATURE_RX_MSP` (see
[src/main/target/SITL/target.h](../../src/main/target/SITL/target.h)), so RC input
is expected over MSP (`MSP_SET_RAW_RC`) on the same TCP MSP port used by
`sitl-rc-check.ps1` (auto-detected, typically `5760` or `5761`). The tool opens a
persistent TCP connection to that port and streams the joystick state as an
`MSP_SET_RAW_RC` frame continuously (50 Hz by default) while its window is open.

## Requirements

- Python 3.
- [pygame](https://www.pygame.org/) for joystick input (SDL2) and the mapping GUI:

  ```powershell
  python -m venv tools\pyenv
  tools\pyenv\Scripts\pip install pygame
  ```

  (`tools/` is gitignored, matching how the MinGW SDK and any future JSBSim venv
  are vendored — see [SITL JSBSim FlightGear Plan.md](SITL%20JSBSim%20FlightGear%20Plan.md).)
- A SITL build (`make TARGET=SITL`) and a USB joystick/gamepad plugged in.

## Usage

1. Start SITL (`.\obj\main\wingflight_SITL.elf`, or let the tool connect once you
   start it manually — it retries the connection every second).
2. Run the bridge:

   ```powershell
   tools\pyenv\Scripts\python scripts\sitl-joystick-rc.py
   ```

3. In the GUI, click **bind** next to a channel row, then move the axis / press the
   button / tap the hat you want assigned to that channel — it binds automatically
   and exits bind mode. `Esc` cancels an in-progress bind.
4. Use the **I** button on a row to invert an assigned axis, **clear** to unassign
   it, and **Save**/**Load** to persist/restore the mapping.
5. Each channel row shows its live output in microseconds (1000-2000) as you move
   the joystick.

Useful flags:

| Flag | Meaning |
|---|---|
| `--list-joysticks` | List detected joysticks/gamepads (index, axes/buttons/hats) and exit |
| `--joystick N` | Use joystick index `N` (default `0`) |
| `--channels N` | Number of RC channels to send/show, 4-18 (default `8`) |
| `--mapping PATH` | Mapping JSON file to load/save (default `scripts/sitl-joystick-mapping.json`, gitignored — it's per-user hardware config) |
| `--host` / `--port` / `--port-candidates` | Override MSP connection target (same auto-detect ports as `sitl-rc-check.ps1`) |
| `--rate` | RC frame send rate in Hz (default `50`) |

On exit, the tool sends a few neutral RC frames (throttle low, everything else
centered) before closing the connection.

## Channel order (important)

RC channels are sent in the exact order the firmware's default `rcmap`
(`"AETR1234"`, see `parseRcChannels()` in
[src/main/rx/rx.c](../../src/main/rx/rx.c)) expects them: **Roll, Pitch, Throttle,
Yaw, AUX1, AUX2, ...**. This is *not* the same order as the `Roll/Pitch/Yaw/.../
Throttle` argument order used internally by `sitl-rc-check.ps1`'s helper
functions — do not copy that script's channel ordering when writing new tools
against `MSP_SET_RAW_RC`; use the payload order documented above.

## Mapping an AUX channel to arm

The joystick tool only maps physical controls to RC channel values — arming still
works exactly like it does with a real transmitter: assign a button/switch to an
AUX channel here, then configure that AUX channel as the arm switch with the usual
CLI `aux` command (see [Modes.md](../Modes.md)), e.g.:

```
aux 0 0 4 1800 2100
```

(range on AUX1 arms `BOXARM`). Bind a joystick button to `AUX1` in the GUI so it
toggles into that range.

## Limitations

- Analog axes and buttons/hats are supported; there is no rumble/force-feedback
  and no calibration curve editor beyond invert + deadband.
- Only one joystick is read at a time (`--joystick` selects which one).
- The mapping file is plain JSON and per-user; it's gitignored rather than
  committed, since it's tied to a specific piece of hardware.
