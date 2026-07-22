# syntax=docker/dockerfile:1
# Gazebo Harmonic environment for Wingflight HITL (Phase A).
#
# Everything Gazebo-related is installed INSIDE this container - nothing is
# installed on the host. Only this Dockerfile (an install script, in effect)
# and the world/model overlay files under hitl/gazebo/ are version
# controlled; the image itself and any models fetched at runtime stay local
# to this project (image tag is not pushed anywhere).
#
# Commands below match the official binary install steps documented at
# https://gazebosim.org/docs/harmonic/install_ubuntu (verified against that
# page, not guessed).

FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    lsb-release \
    gnupg \
    ca-certificates \
    git \
    python3 \
    python3-pip \
    python3-venv \
 && rm -rf /var/lib/apt/lists/*

RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
 && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/gazebo-stable.list \
 && apt-get update \
 && apt-get install -y --no-install-recommends \
    gz-harmonic \
    python3-gz-transport13 \
    python3-gz-msgs10 \
 && rm -rf /var/lib/apt/lists/*

# pyserial for the bridge (hitl/bridge) when run from inside this container.
RUN pip3 install --no-cache-dir --break-system-packages pyserial

# Bridge + Gazebo assets are bind-mounted at runtime (see docker-compose.yml)
# rather than baked into the image, so editing them on the host doesn't
# require an image rebuild.
WORKDIR /workspace

CMD ["bash"]
