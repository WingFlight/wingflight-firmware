#!/usr/bin/env bash
# Fetches the models/ and worlds/ directories from PX4/PX4-gazebo-models
# (https://github.com/PX4/PX4-gazebo-models) into a repo-local cache -
# NOT into the user's home directory (that repo's own `simulation-gazebo`
# helper script defaults to ~/.simulation-gazebo; we deliberately avoid
# that here per the "keep tools local to the repo" requirement).
#
# Uses a shallow, blobless, sparse clone so only the needed directories are
# downloaded (not the full history/other content of that repo).
#
# Usage: ./scripts/fetch_px4_models.sh [ref]
#   ref defaults to "main".

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
GAZEBO_DIR="$(dirname "$SCRIPT_DIR")"
CACHE_DIR="$GAZEBO_DIR/.cache/px4-gazebo-models"
REPO_URL="https://github.com/PX4/PX4-gazebo-models.git"
REF="${1:-main}"

mkdir -p "$(dirname "$CACHE_DIR")"

if [[ -d "$CACHE_DIR/.git" ]]; then
    echo "Updating existing checkout at $CACHE_DIR ..."
    git -C "$CACHE_DIR" fetch --depth 1 origin "$REF"
    git -C "$CACHE_DIR" checkout "$REF"
    git -C "$CACHE_DIR" reset --hard "origin/$REF"
else
    echo "Cloning $REPO_URL (models/, worlds/ only) into $CACHE_DIR ..."
    git clone --depth 1 --filter=blob:none --sparse --branch "$REF" "$REPO_URL" "$CACHE_DIR"
    git -C "$CACHE_DIR" sparse-checkout set models worlds
fi

echo
echo "Done. Fetched into: $CACHE_DIR"
echo
echo "Known PX4 fixed-wing candidates (per docs.px4.io/main/en/sim_gazebo_gz,"
echo "confirmed there - NOT verified against this checkout's exact folder"
echo "names yet):"
echo "  - rc_cessna       (PX4 make target: gz_rc_cessna)"
echo "  - advanced_plane  (PX4 make target: gz_advanced_plane)"
echo
echo "Confirm the actual model folder name for your target airframe with:"
echo "  ls \"$CACHE_DIR/models\" | grep -i -E 'cessna|plane'"
echo
echo "Then update hitl/gazebo/worlds/wingflight_plane.sdf's <include><uri>"
echo "to match (see the TODO comment in that file), and run"
echo "./scripts/verify_topics.sh to confirm the IMU/air_pressure topic names"
echo "via 'gz topic -l' (Phase A step 3 in ../../hitl_plan.txt)."
