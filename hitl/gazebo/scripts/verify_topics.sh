#!/usr/bin/env bash
# Starts a headless Gazebo server for a given world and lists the resulting
# gz-transport topics, per Phase A step 3 / Verification step 2 in
# ../../hitl_plan.txt ("Verify actuator/sensor topics available for the
# plane model via `gz topic -l`").
#
# Usage: ./scripts/verify_topics.sh [world_sdf_path]
#   Defaults to hitl/gazebo/worlds/wingflight_plane.sdf (see the TODO in
#   that file - it needs the plane model's <include> filled in first via
#   fetch_px4_models.sh before it will show sensor topics for a vehicle;
#   until then this just confirms the Gazebo install + gz-transport work).

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
GAZEBO_DIR="$(dirname "$SCRIPT_DIR")"
WORLD="${1:-$GAZEBO_DIR/worlds/wingflight_plane.sdf}"

if ! command -v gz >/dev/null 2>&1; then
    echo "error: 'gz' CLI not found. Run install.sh first (or run this" \
         "inside the wingflight-gazebo container)." >&2
    exit 1
fi

echo "Starting headless gz-sim server with: $WORLD"
gz sim -s -r -v4 "$WORLD" &
GZ_PID=$!
trap 'kill "$GZ_PID" 2>/dev/null || true' EXIT

# Give the server a moment to come up before querying topics.
sleep 5

echo
echo "--- gz topic -l ---"
gz topic -l

echo
echo "Look for IMU / air_pressure / navsat topics for the plane model above."
echo "If the list only shows /clock, /stats, /world/.../pose/info etc. and"
echo "nothing sensor-specific, the plane model include in wingflight_plane.sdf"
echo "still needs to be filled in (see the TODO in that file) - this script"
echo "only confirms the Gazebo install itself is working so far."
