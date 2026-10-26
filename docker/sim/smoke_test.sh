#!/usr/bin/env bash
# Lang2Sim sim image smoke test.
#
# Host side (from repo root):
#   docker/sim/smoke_test.sh <mode> [--image IMG] [--interactive]
#     gpu       NVIDIA (needs nvidia-container-toolkit) + host X11    [Ubuntu]
#     dri       Intel/AMD via /dev/dri + host X11                      [Ubuntu]
#     software  llvmpipe + host X11                                    [Ubuntu, any GPU]
#     wslg      WSLg display + /dev/dxg                                [Windows 11, inside WSL2]
#     vnc       Xvfb + noVNC on http://localhost:6080 (sim-vnc image)  [macOS, CI, anywhere]
#     headless  no display, server-only gz sim                         [anywhere]
#   --interactive keeps Gazebo open after the checks so you can look at it.
#
# Inside the container (what the host side runs):
#   smoke_test.sh --inside [--interactive]
#
# Step 1 Definition of Done: `docker/sim/smoke_test.sh dri --interactive`
# (or gpu/software) shows a Gazebo window on an Ubuntu desktop.
set -Eeuo pipefail

###############################################################################
# Inside the container
###############################################################################
inside() {
  local interactive=0
  [[ "${1:-}" == "--interactive" ]] && interactive=1

  local pass=0 fail=0
  ok()   { printf '  \033[32mPASS\033[0m %s\n' "$*"; pass=$((pass + 1)); }
  bad()  { printf '  \033[31mFAIL\033[0m %s\n' "$*"; fail=$((fail + 1)); }
  info() { printf '  ---- %s\n' "$*"; }

  echo "== lang2sim smoke test (display=${L2S_EFFECTIVE_DISPLAY_MODE:-?} gpu=${L2S_EFFECTIVE_GPU:-?} arch=$(uname -m))"

  # 1. ROS packages the later steps depend on.
  local pkg
  for pkg in ros_gz_sim ros_gz_bridge nav2_bringup nav2_simple_commander \
             turtlebot3_gazebo turtlebot3_description turtlebot3_navigation2 \
             robot_state_publisher rviz2 foxglove_bridge; do
    if ros2 pkg prefix "$pkg" >/dev/null 2>&1; then ok "ros pkg $pkg"; else bad "ros pkg $pkg missing"; fi
  done

  # 2. Python deps for the control API (venv + rclpy side by side).
  if python3 - <<'PY' >/dev/null 2>&1
import fastapi, uvicorn, httpx, pydantic, rclpy, nav2_simple_commander
assert pydantic.VERSION.startswith("2"), pydantic.VERSION
PY
  then ok "python: fastapi/uvicorn/httpx/pydantic v2 + rclpy + nav2_simple_commander"
  else bad "python deps import failed"; fi

  # 3. Gazebo Harmonic CLI.
  local gzv
  if gzv="$(gz sim --versions 2>/dev/null | head -n1)" && [[ "$gzv" == 8.* ]]; then
    ok "gz sim $gzv (Harmonic)"
  else
    bad "gz sim not Harmonic (got '${gzv:-nothing}')"
  fi

  # 4. GL + X11 (only when there is a display).
  if [[ -n "${DISPLAY:-}" ]]; then
    local glx renderer
    if glx="$(glxinfo -B 2>&1)"; then
      renderer="$(sed -n 's/^OpenGL renderer string: //p' <<<"$glx")"
      ok "glxinfo: ${renderer:-unknown renderer}"
      if [[ "${L2S_EFFECTIVE_GPU:-}" == "nvidia" || "${L2S_EFFECTIVE_GPU:-}" == "dri" ]] \
         && grep -qi llvmpipe <<<"$renderer"; then
        bad "expected hardware GL for gpu=${L2S_EFFECTIVE_GPU}, got llvmpipe"
      fi
    else
      bad "glxinfo failed: $(tail -n1 <<<"$glx")"
    fi

    local rc=0
    timeout 4 xclock >/dev/null 2>&1 || rc=$?
    if [[ $rc -eq 124 ]]; then ok "xclock opened (killed after 4s)"; else bad "xclock exited early (rc=$rc)"; fi
  else
    info "no DISPLAY: skipping glxinfo/xclock"
  fi

  # 5. gz sim: world loads and the clock advances. GUI too when there's a display.
  local gz_args=(-r -v 2 shapes.sdf) mode_label="gui+server"
  if [[ -z "${DISPLAY:-}" ]]; then gz_args=(-s "${gz_args[@]}"); mode_label="server-only"; fi
  export GZ_PARTITION="l2s_smoke_$$"
  local log=/tmp/l2s-logs/smoke_gz.log
  mkdir -p /tmp/l2s-logs
  setsid gz sim "${gz_args[@]}" >"$log" 2>&1 &
  local gz_pid=$!
  # shellcheck disable=SC2064
  trap "kill -- -$gz_pid 2>/dev/null || true" EXIT

  local i up=0
  for i in $(seq 1 60); do
    if gz topic -l 2>/dev/null | grep -q '/clock'; then up=1; break; fi
    kill -0 "$gz_pid" 2>/dev/null || break
    sleep 1
  done
  if [[ $up -eq 1 ]]; then
    ok "gz sim ($mode_label) up after ${i}s"
    if timeout 20 gz topic -e -t /clock -n 1 >/dev/null 2>&1; then
      ok "gz /clock publishing (sim stepping)"
    else
      bad "gz /clock never published"
    fi
    if [[ -n "${DISPLAY:-}" ]]; then
      # The GUI's 3D scene publishes its camera pose once it has rendered.
      local gui=0
      for i in $(seq 1 30); do
        if gz topic -l 2>/dev/null | grep -q '/gui/camera/pose'; then gui=1; break; fi
        sleep 1
      done
      if [[ $gui -eq 1 ]]; then ok "gz GUI rendering (/gui/camera/pose)"; else bad "gz GUI never came up (see $log)"; fi
    fi
  else
    bad "gz sim did not come up in 60s; last log lines:"
    tail -n 20 "$log" | sed 's/^/        /'
  fi

  echo "== $pass passed, $fail failed"

  if [[ $interactive -eq 1 && $up -eq 1 ]]; then
    if [[ "${L2S_EFFECTIVE_DISPLAY_MODE:-}" == "vnc" ]]; then
      echo "== Gazebo is running. Open http://localhost:${L2S_NOVNC_PORT:-6080}  (Ctrl-C to stop)"
    else
      echo "== Gazebo is running on your display. Ctrl-C to stop."
    fi
    wait "$gz_pid" || true
  fi

  [[ $fail -eq 0 ]]
}

###############################################################################
# Host side
###############################################################################
usage() { sed -n '2,20p' "$0" | sed 's/^# \{0,1\}//'; exit "${1:-0}"; }

host() {
  local mode="${1:-}"; [[ -n "$mode" ]] || usage 1; shift
  local image="" interactive=()
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --image) image="$2"; shift 2 ;;
      --interactive|-i) interactive=(--interactive); shift ;;
      -h|--help) usage 0 ;;
      *) echo "unknown arg: $1" >&2; usage 1 ;;
    esac
  done

  local args=(--rm --shm-size=1g
              -e "L2S_HOST_UID=$(id -u)" -e "L2S_HOST_GID=$(id -g)"
              -e L2S_FOXGLOVE=0)
  [[ ${#interactive[@]} -gt 0 && -t 0 ]] && args+=(-it)

  local need_x11=0
  case "$mode" in
    gpu)      need_x11=1; args+=(--gpus all -e L2S_DISPLAY_MODE=x11 -e L2S_GPU=nvidia) ;;
    dri)      need_x11=1; args+=(--device /dev/dri -e L2S_DISPLAY_MODE=x11 -e L2S_GPU=dri) ;;
    software) need_x11=1; args+=(-e L2S_DISPLAY_MODE=x11 -e L2S_GPU=software) ;;
    wslg)
      [[ -d /mnt/wslg ]] || { echo "wslg mode must run inside WSL2 (no /mnt/wslg)" >&2; exit 1; }
      args+=(-v /tmp/.X11-unix:/tmp/.X11-unix -v /mnt/wslg:/mnt/wslg
             -e DISPLAY="${DISPLAY:-:0}" -e WAYLAND_DISPLAY="${WAYLAND_DISPLAY:-wayland-0}"
             -e L2S_DISPLAY_MODE=wslg -e L2S_GPU=auto)
      if [[ -e /dev/dxg ]]; then args+=(--device /dev/dxg -v /usr/lib/wsl:/usr/lib/wsl:ro); fi
      ;;
    vnc)
      : "${image:=${L2S_IMAGE_VNC:-lang2sim-sim:dev-vnc}}"
      args+=(-p "127.0.0.1:${L2S_NOVNC_PORT:-6080}:6080" -e L2S_DISPLAY_MODE=vnc -e L2S_GPU=software)
      ;;
    headless) args+=(-e L2S_DISPLAY_MODE=headless -e L2S_GPU=software) ;;
    *) echo "unknown mode: $mode" >&2; usage 1 ;;
  esac
  : "${image:=${L2S_IMAGE:-lang2sim-sim:dev}}"

  if [[ $need_x11 -eq 1 ]]; then
    [[ -n "${DISPLAY:-}" ]] || { echo "DISPLAY is not set; run from a desktop session" >&2; exit 1; }
    if command -v xhost >/dev/null 2>&1; then
      xhost "+SI:localuser:$(id -un)" >/dev/null || true
    fi
    args+=(-v /tmp/.X11-unix:/tmp/.X11-unix:rw -e "DISPLAY=$DISPLAY")
    if [[ -n "${XAUTHORITY:-}" && -f "$XAUTHORITY" ]]; then
      args+=(-v "$XAUTHORITY:/tmp/.Xauthority:ro" -e XAUTHORITY=/tmp/.Xauthority)
    elif [[ -f "$HOME/.Xauthority" ]]; then
      args+=(-v "$HOME/.Xauthority:/tmp/.Xauthority:ro" -e XAUTHORITY=/tmp/.Xauthority)
    fi
  fi

  # ${arr[@]+...} keeps empty arrays safe under `set -u` on macOS bash 3.2.
  echo "+ docker run ${args[*]} $image smoke ${interactive[*]+${interactive[*]}}" >&2
  exec docker run "${args[@]}" "$image" smoke ${interactive[@]+"${interactive[@]}"}
}

if [[ "${1:-}" == "--inside" ]]; then
  shift
  inside "$@"
else
  host "$@"
fi
