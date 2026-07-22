#!/usr/bin/env bash
# Phase A install script: sets up a local, project-scoped Gazebo Harmonic
# environment for Wingflight HITL. Nothing is installed system-wide -
# everything lives under this directory (hitl/gazebo) and inside a Docker
# image tagged only for this project (wingflight-gazebo:local).
#
# Usage:
#   ./install.sh            # build the Docker image (default, recommended)
#   ./install.sh --native    # install Gazebo Harmonic directly on this host
#                             instead (only if you explicitly do NOT want
#                             Docker isolation - installs system packages,
#                             see gazebosim.org/docs/harmonic/install_ubuntu)
#
# Run this from any directory; paths are resolved relative to this script.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

MODE="docker"
if [[ "${1:-}" == "--native" ]]; then
    MODE="native"
fi

if [[ "$MODE" == "docker" ]]; then
    if ! command -v docker >/dev/null 2>&1; then
        echo "error: docker not found. Install Docker, or re-run with --native" \
             "to install Gazebo Harmonic directly on this host instead." >&2
        exit 1
    fi

    echo "Building local Gazebo Harmonic image (wingflight-gazebo:local)..."
    docker compose -f docker-compose.yml build

    echo
    echo "Done. Nothing was installed on this host; everything lives in the"
    echo "'wingflight-gazebo:local' image and under hitl/gazebo/.cache."
    echo
    echo "Next steps:"
    echo "  1. Fetch PX4 models/worlds locally:"
    echo "       docker compose -f docker-compose.yml run --rm gazebo ./scripts/fetch_px4_models.sh"
    echo "  2. Verify sensor/actuator topics for the fetched plane model:"
    echo "       docker compose -f docker-compose.yml run --rm gazebo ./scripts/verify_topics.sh"
    exit 0
fi

# --native: only for hosts where Docker isolation is explicitly not wanted.
# This runs the same commands as the Dockerfile, but directly on the host -
# see https://gazebosim.org/docs/harmonic/install_ubuntu (verified there,
# not guessed). Requires Ubuntu 22.04 (Jammy) or 24.04 (Noble).
echo "WARNING: --native installs Gazebo Harmonic system-wide on this host" >&2
echo "(not isolated in Docker, and not contained to this repo directory)." >&2
read -r -p "Continue? [y/N] " reply
if [[ ! "$reply" =~ ^[Yy]$ ]]; then
    echo "Aborted."
    exit 1
fi

sudo apt-get update
sudo apt-get install -y curl lsb-release gnupg
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
    | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install -y gz-harmonic python3-gz-transport13 python3-gz-msgs10

echo
echo "Done. Next steps:"
echo "  1. ./scripts/fetch_px4_models.sh"
echo "  2. ./scripts/verify_topics.sh"
