# Wingflight HIL Bridge (skeleton)

Host-side bridge for real-board HIL testing (see `../../hitl_plan.txt` and
`PROTOCOL.md` in this folder). This is the Phase B "HIL Bridge (host tool)"
piece — it does not depend on the firmware repo build at all, only on the
wire protocol.

## Status

Skeleton only. `--source synthetic` works today with no Gazebo dependency
(useful for testing the serial link + firmware injection independently).
`--source gazebo` is a starting point only — see the TODOs in
`hitl_bridge/sensor_source.py`; it needs a running Phase A world and its
real topic/message-type names confirmed via `gz topic -l` before it will
actually work. Track remaining work in `../../hitl_todo.md`.

## Layout

- `hitl_bridge/protocol.py` — wire format encode/decode, mirrors
  `src/main/io/hil_sensor.c`.
- `hitl_bridge/sensor_source.py` — `SensorSource` abstraction:
  `SyntheticSensorSource` (no Gazebo needed) and `GazeboSensorSource`
  (gz-transport, not yet verified against a real world).
- `hitl_bridge/serial_link.py` — pyserial TX wrapper (`SerialLink`) plus a
  `NullLink` for `--dry-run`.
- `hitl_bridge/bridge.py` — fixed-rate read/send loop.
- `hitl_bridge/__main__.py` — CLI entry point.
- `tests/test_protocol.py` — protocol round-trip/CRC unit tests (no
  hardware/Gazebo required).

## Usage

```powershell
# Install dependencies (only pyserial needed for --source synthetic)
pip install -r requirements.txt

# Dry run: exercise the synthetic source + packet encoding, no serial port
python -m hitl_bridge --source synthetic --dry-run -v

# Send synthetic test data to a real board over a real UART
python -m hitl_bridge --source synthetic --port COM5

# Run the unit tests
python -m unittest discover tests
```

On the firmware side, assign a UART to the HIL sensor function first:

```
serial <n> 2097152 115200 115200 0 115200
```

(`2097152` = `FUNCTION_HIL_SENSOR` = `1<<21`; baud fields are ignored by
this function today since `hil_sensor.c` forces 921600 — see
`PROTOCOL.md`.) Build with
`make TARGET=STM32F7X2 EXTRA_FLAGS=-DUSE_HIL_SENSOR_OVERRIDE` to include the
required firmware support.

## Next steps

See `../../hitl_todo.md` for the current open/done task list.
