# syntax=docker/dockerfile:1.7
#
# Lang2Sim simulation runtime image (Part 3).
#   ROS 2 Jazzy + Gazebo Harmonic + Nav2 + TurtleBot3 + foxglove_bridge,
#   plus a Python venv for the in-container control API (FastAPI, port 9090).
#
# Targets (always pass --target; the last stage is not the one you usually want):
#   sim      X11 / WSLg / headless+Foxglove. Default runtime image.
#   sim-vnc  sim + Xvfb + x11vnc + noVNC on :6080 (macOS GUI, CI, any-OS fallback).
#
# Build (from repo root):
#   docker buildx build -f docker/sim/Dockerfile --target sim     -t lang2sim-sim:dev     --load .
#   docker buildx build -f docker/sim/Dockerfile --target sim-vnc -t lang2sim-sim:dev-vnc --load .
#
# Multi-arch: the base is the official `ros` library image (amd64 + arm64).
# Do NOT switch to osrf/ros:jazzy-desktop; that tag is amd64-only.
# Every dependency is baked in here. Nothing apt-installs at runtime.

ARG ROS_DISTRO=jazzy

###############################################################################
FROM ros:${ROS_DISTRO}-ros-base AS sim
###############################################################################
ARG ROS_DISTRO
ARG USERNAME=lang2sim
ARG USER_UID=1000
ARG USER_GID=1000

LABEL org.opencontainers.image.title="lang2sim-sim" \
      org.opencontainers.image.description="Lang2Sim runtime: ROS 2 Jazzy, Gazebo Harmonic, Nav2, TurtleBot3" \
      org.opencontainers.image.source="https://github.com/DuttaNeel07/Lang2Sim" \
      org.opencontainers.image.licenses="MIT"

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# ROS / Gazebo / Nav2 / TurtleBot3 (all Jazzy binaries, Harmonic via gz_*_vendor)
# plus GL, X11 test tools and process helpers.
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    rm -f /etc/apt/apt.conf.d/docker-clean \
 && apt-get update \
 && apt-get install -y --no-install-recommends \
      ros-${ROS_DISTRO}-ros-gz \
      ros-${ROS_DISTRO}-ros-gz-bridge \
      ros-${ROS_DISTRO}-ros-gz-sim \
      ros-${ROS_DISTRO}-ros-gz-image \
      ros-${ROS_DISTRO}-gz-sim-vendor \
      ros-${ROS_DISTRO}-gz-tools-vendor \
      ros-${ROS_DISTRO}-navigation2 \
      ros-${ROS_DISTRO}-nav2-bringup \
      ros-${ROS_DISTRO}-nav2-simple-commander \
      ros-${ROS_DISTRO}-nav2-rviz-plugins \
      ros-${ROS_DISTRO}-robot-state-publisher \
      ros-${ROS_DISTRO}-joint-state-publisher \
      ros-${ROS_DISTRO}-xacro \
      ros-${ROS_DISTRO}-tf2-ros \
      ros-${ROS_DISTRO}-tf2-tools \
      ros-${ROS_DISTRO}-rviz2 \
      ros-${ROS_DISTRO}-turtlebot3-msgs \
      ros-${ROS_DISTRO}-turtlebot3-description \
      ros-${ROS_DISTRO}-turtlebot3-navigation2 \
      ros-${ROS_DISTRO}-turtlebot3-gazebo \
      ros-${ROS_DISTRO}-foxglove-bridge \
      ros-${ROS_DISTRO}-rosbag2-storage-mcap \
      ros-${ROS_DISTRO}-teleop-twist-keyboard \
      mesa-utils \
      libgl1-mesa-dri \
      libglx-mesa0 \
      libegl1 \
      x11-apps \
      x11-xserver-utils \
      python3-venv \
      python3-pip \
      gosu \
      tini \
      curl \
      ca-certificates \
      iproute2 \
      procps \
 && rm -rf /var/lib/apt/lists/*

# Control API venv. --system-site-packages keeps rclpy / nav2_simple_commander
# importable; pydantic v2 comes from here (Noble's apt pydantic is v1).
ENV L2S_HOME=/opt/lang2sim \
    L2S_VENV=/opt/lang2sim/venv
RUN python3 -m venv --system-site-packages "${L2S_VENV}" \
 && "${L2S_VENV}/bin/pip" install --no-cache-dir \
      "fastapi>=0.110,<1" \
      "uvicorn[standard]>=0.29,<1" \
      "pydantic>=2.6,<3" \
      "httpx>=0.27,<1" \
      "websockets>=12"

# Non-root runtime user. Noble images ship an `ubuntu` user on 1000; drop it so
# lang2sim owns 1000. The entrypoint remaps to the host UID/GID at start.
RUN if id ubuntu &>/dev/null; then userdel -r ubuntu; fi \
 && groupadd --gid "${USER_GID}" "${USERNAME}" \
 && useradd --uid "${USER_UID}" --gid "${USER_GID}" --create-home --shell /bin/bash "${USERNAME}" \
 && usermod -aG video "${USERNAME}" \
 && mkdir -p /lang2sim/projects /lang2sim/catalog "${L2S_HOME}/src" "${L2S_HOME}/bin" \
 && chown -R "${USERNAME}:${USERNAME}" /lang2sim "${L2S_HOME}/src" \
 && echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> "/home/${USERNAME}/.bashrc"

# Pre-cache the Gazebo Fuel models the TurtleBot3 worlds <include> by URL
# (Ground Plane, Sun). Without this, gz downloads them at sim start and the world
# fails to load offline / behind a proxy, which then kills the robot spawn.
# Unpacked straight into gz's cache layout (<host>/<owner>/models/<name>/<ver>/,
# lowercased) with curl; `gz fuel download` is flakier behind proxies.
ENV GZ_FUEL_CACHE_PATH=/opt/lang2sim/fuel
RUN for m in "Ground Plane" "Sun"; do \
      d="${GZ_FUEL_CACHE_PATH}/fuel.gazebosim.org/openrobotics/models/${m,,}/1"; \
      mkdir -p "$d" \
   && curl -fsSL --retry 3 -o /tmp/model.zip \
        "https://fuel.gazebosim.org/1.0/OpenRobotics/models/${m// /%20}.zip" \
   && python3 -m zipfile -e /tmp/model.zip "$d" \
   && rm -f /tmp/model.zip \
   && test -f "$d/model.sdf" || exit 1; \
    done \
 && chmod -R a+rwX "${GZ_FUEL_CACHE_PATH}"

COPY --chmod=0755 docker/sim/entrypoint.sh docker/sim/smoke_test.sh ${L2S_HOME}/bin/

# Paths and ports below are the contract with sim2lang/core/config.py (Step 2).
ENV L2S_USER=${USERNAME} \
    L2S_SRC=/opt/lang2sim/src \
    L2S_PROJECTS_DIR=/lang2sim/projects \
    L2S_CATALOG_DIR=/lang2sim/catalog \
    L2S_API_PORT=9090 \
    L2S_FOXGLOVE_PORT=8765 \
    L2S_NOVNC_PORT=6080 \
    L2S_API_APP=lang2sim.runtime.control_api.server:app \
    L2S_DISPLAY_MODE=auto \
    L2S_GPU=auto \
    L2S_FOXGLOVE=auto \
    PATH=/opt/lang2sim/venv/bin:/opt/lang2sim/bin:${PATH} \
    ROS_DISTRO=${ROS_DISTRO} \
    GZ_VERSION=harmonic \
    TURTLEBOT3_MODEL=waffle \
    ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
    QT_X11_NO_MITSHM=1 \
    NVIDIA_VISIBLE_DEVICES=all \
    NVIDIA_DRIVER_CAPABILITIES=all

# The lang2sim package (control API now; launch/nav/self-healing later). It is
# imported via PYTHONPATH, not pip-installed, so the host's heavy deps (textual,
# openai, shapely...) stay out of the image. Dev runs bind-mount the repo over
# this path (docker_runner src_dir) to skip rebuilds.
COPY --chown=root:root pyproject.toml ${L2S_SRC}/pyproject.toml
COPY --chown=root:root lang2sim ${L2S_SRC}/lang2sim

EXPOSE 9090 8765
WORKDIR /lang2sim

# Healthy if the API answers, or if we're deliberately idling (no API yet).
HEALTHCHECK --interval=15s --timeout=3s --start-period=30s --retries=3 \
  CMD curl -fsS "http://127.0.0.1:${L2S_API_PORT}/v1/health" >/dev/null || test -f /tmp/l2s_idle

# Entrypoint starts as root only to remap UID/GID and device groups, then
# drops to ${L2S_USER} with gosu. tini reaps children and forwards signals.
ENTRYPOINT ["/usr/bin/tini", "--", "/opt/lang2sim/bin/entrypoint.sh"]
CMD ["serve"]

###############################################################################
FROM sim AS sim-vnc
###############################################################################
# Virtual X server + VNC + noVNC so the real Gazebo/RViz GUI is reachable at
# http://localhost:6080 on any OS. Software rendering only (llvmpipe).
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update \
 && apt-get install -y --no-install-recommends \
      xvfb \
      x11vnc \
      novnc \
      python3-websockify \
      fluxbox \
      xterm \
 && rm -rf /var/lib/apt/lists/* \
 && printf '%s\n' \
      '<!doctype html><meta http-equiv="refresh" content="0; url=vnc.html?autoconnect=1&resize=scale">' \
      > /usr/share/novnc/index.html

ENV L2S_DISPLAY_MODE=vnc \
    L2S_VNC_GEOMETRY=1600x900

EXPOSE 6080
