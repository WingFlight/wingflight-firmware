# Wingflight HITL

Real-board Hardware-In-The-Loop testing with a PX4-style Gazebo environment.
See:

- [`../hitl_plan.txt`](../hitl_plan.txt) — full design/plan doc.
- [`../hitl_todo.md`](../hitl_todo.md) — current open/done task tracker.
- [`gazebo/`](gazebo/) — Phase A: local, project-scoped Gazebo Harmonic
  install scripts + world/model assets.
- [`bridge/`](bridge/) — Phase B: Python host bridge (wire protocol,
  serial link, sensor sources).

Firmware-side support (Phase C, `USE_HIL_SENSOR_OVERRIDE`) lives in the main
`src/main/` tree, not here - see `bridge/PROTOCOL.md` for the wire format
shared between firmware and bridge.

Everything under this `hitl/` directory is deliberately outside `/tools/`
(which the top-level `.gitignore` reserves for downloaded build-toolchain
artifacts, e.g. the ARM GCC SDK) so it stays version-controlled.
