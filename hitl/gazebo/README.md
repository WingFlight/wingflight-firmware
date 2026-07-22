# Wingflight HITL Gazebo Environment (Phase A)

Self-contained, project-local Gazebo Harmonic setup for the real-board HITL
plan (`../../hitl_plan.txt`, `../../hitl_todo.md`). Nothing here installs
anything outside this directory / a locally-tagged Docker image - see
`install.sh`.

## Status

Scaffolding only - install/fetch/verify scripts exist and are believed
correct (commands verified against official docs, see comments in each
file), but have **not been executed** in this pass (no Gazebo/Docker
available in the environment this was written in). Track remaining
verification work in `../../hitl_todo.md`.

## Layout

- `Dockerfile` — installs Gazebo Harmonic (`gz-harmonic`) + Python
  transport bindings inside a container, following the official binary
  install steps (gazebosim.org/docs/harmonic/install_ubuntu). Nothing
  touches the host.
- `docker-compose.yml` — builds/runs that container, bind-mounting this
  directory and `../bridge` in at `/workspace`.
- `install.sh` — entry point. Default: builds the Docker image. `--native`:
  installs Gazebo directly on the host instead (opt-in, not recommended,
  for users who explicitly don't want Docker isolation).
- `scripts/fetch_px4_models.sh` — sparse/shallow git clone of
  [PX4/PX4-gazebo-models](https://github.com/PX4/PX4-gazebo-models)
  (`models/` + `worlds/` only) into `.cache/px4-gazebo-models/` (repo-local,
  gitignored - NOT `~/.simulation-gazebo` like that repo's own helper script
  defaults to).
- `scripts/verify_topics.sh` — starts a headless `gz sim` server for a
  given world and runs `gz topic -l`, per Phase A step 3 in the plan.
- `worlds/wingflight_plane.sdf` — draft world (standard gz-sim
  physics/scene/sensor system plugins + ground/sun) with a **commented-out,
  placeholder** `<include>` for the PX4 fixed-wing model. Has a TODO to fill
  in the real model folder name once `fetch_px4_models.sh` has run - the
  exact PX4 fixed-wing model name is not hardcoded here since it hasn't
  been confirmed against a real checkout yet (candidates per
  docs.px4.io/main/en/sim_gazebo_gz: `rc_cessna`, `advanced_plane`).

## Usage

```bash
cd hitl/gazebo
bash install.sh                                    # build local Docker image
docker compose run --rm gazebo bash scripts/fetch_px4_models.sh
docker compose run --rm gazebo bash scripts/verify_topics.sh
```

Or without Docker (`bash install.sh --native`, then run the scripts
directly on the host - only on Ubuntu 22.04/24.04).

## Next steps

See `../../hitl_todo.md` (Phase A section) for the current open/done list -
in short: run the above for real, confirm the model folder name, fill in
`wingflight_plane.sdf`'s `<include>`, and confirm IMU/air_pressure topic
names so `hitl/bridge/hitl_bridge/sensor_source.py`'s `GazeboSensorSource`
can be wired up and tested (Phase B).
