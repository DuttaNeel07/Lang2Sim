#!/usr/bin/env bash
# Lang2Sim sim container entrypoint.
#
# Phase 1 (root): remap the runtime user to the host UID/GID, join the groups
#                 that own GPU device nodes, then re-exec as that user (gosu).
# Phase 2 (user): source ROS, configure display + GL, start optional sidecars
#                 (noVNC, foxglove_bridge), then run the command.
#
# Commands:
#   serve            Control API on :$L2S_API_PORT if importable, else idle.  (default)
#   idle             Keep the container alive (sidecars still run).
#   smoke [args]     Run the in-container smoke test (smoke_test.sh --inside).
#   shell | bash     Interactive shell with ROS sourced.
#   <anything else>  exec'd as-is, e.g. `gz sim shapes.sdf` or `ros2 launch ...`.
#
# Environment (set by the host-side docker_runner; all optional):
#   L2S_HOST_UID / L2S_HOST_GID   host user to remap to (default: leave as 1000)
#   L2S_DISPLAY_MODE  auto | x11 | wslg | vnc | headless
#   L2S_GPU           auto | nvidia | dri | software
#   L2S_FOXGLOVE      auto | 1 | 0     (auto = on when headless)
#   L2S_VNC_GEOMETRY  e.g. 1600x900    (vnc mode only)
set -Eeuo pipefail

L2S_USER="${L2S_USER:-lang2sim}"
L2S_HOME="${L2S_HOME:-/opt/lang2sim}"
L2S_LOG_DIR="${L2S_LOG_DIR:-/tmp/l2s-logs}"
RUNTIME_ENV_FILE=/tmp/l2s_runtime.env

log()  { printf '[entrypoint] %s\n' "$*" >&2; }
warn() { printf '[entrypoint] WARN: %s\n' "$*" >&2; }
die()  { printf '[entrypoint] ERROR: %s\n' "$*" >&2; exit 1; }

###############################################################################
# Phase 1: root
###############################################################################
remap_user() {
  local want_uid="${L2S_HOST_UID:-}" want_gid="${L2S_HOST_GID:-}"
  local cur_uid cur_gid
  cur_uid="$(id -u "$L2S_USER")"
  cur_gid="$(id -g "$L2S_USER")"

  if [[ -n "$want_gid" && "$want_gid" != "0" && "$want_gid" != "$cur_gid" ]]; then
    log "remapping group $L2S_USER: $cur_gid -> $want_gid"
    groupmod -o -g "$want_gid" "$L2S_USER"
  fi
  if [[ -n "$want_uid" && "$want_uid" != "0" && "$want_uid" != "$cur_uid" ]]; then
    log "remapping user $L2S_USER: $cur_uid -> $want_uid"
    # usermod re-owns the home directory; image-owned dirs are fixed below.
    usermod -o -u "$want_uid" "$L2S_USER"
  fi
  if [[ "$want_uid" == "0" ]]; then
    warn "L2S_HOST_UID=0 (rootful host user?) - keeping container user at $(id -u "$L2S_USER")"
  fi

  # Image-owned dirs (non-recursive on bind mounts: only chown if empty/ours).
  local d
  for d in /lang2sim /lang2sim/projects /lang2sim/catalog "$L2S_HOME/src"; do
    [[ -d "$d" ]] || continue
    if [[ -z "$(find "$d" -mindepth 1 -maxdepth 1 -print -quit 2>/dev/null)" ]]; then
      chown "$L2S_USER:$L2S_USER" "$d" 2>/dev/null || true
    fi
  done
}

join_device_groups() {
  # GPU nodes are owned by host GIDs (video/render) that don't exist in the
  # image. Create a matching group if needed and add the user to it.
  local dev gid grp
  shopt -s nullglob
  for dev in /dev/dri/card* /dev/dri/renderD* /dev/dxg /dev/nvidia*; do
    [[ -e "$dev" ]] || continue
    gid="$(stat -c '%g' "$dev")"
    [[ "$gid" == "0" ]] && continue
    grp="$(getent group "$gid" | cut -d: -f1 || true)"
    if [[ -z "$grp" ]]; then
      grp="hostdev$gid"
      groupadd -o -g "$gid" "$grp"
    fi
    usermod -aG "$grp" "$L2S_USER"
  done
  shopt -u nullglob
}

if [[ "$(id -u)" == "0" && "${L2S_SKIP_PRIVDROP:-0}" != "1" ]]; then
  remap_user
  join_device_groups
  mkdir -p "$L2S_LOG_DIR" && chown "$L2S_USER:$L2S_USER" "$L2S_LOG_DIR"
  export HOME="/home/$L2S_USER"
  exec gosu "$L2S_USER" "$0" "$@"
fi

###############################################################################
# Phase 2: runtime user
###############################################################################
mkdir -p "$L2S_LOG_DIR"
export HOME="${HOME:-/home/$L2S_USER}"

# ROS setup scripts reference unset variables; relax -u while sourcing.
set +u
# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
if [[ -f "$L2S_HOME/ws/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "$L2S_HOME/ws/install/setup.bash"
fi
set -u

# Make the bind-mounted / copied sim2lang package importable.
if [[ -d "${L2S_SRC:-}" ]]; then
  export PYTHONPATH="${L2S_SRC}${PYTHONPATH:+:$PYTHONPATH}"
fi

#------------------------------------------------------------------------------
# Display
#------------------------------------------------------------------------------
x11_socket_for() {  # ":0" / ":0.0" / "host:0" -> /tmp/.X11-unix/X0
  local d="${1#*:}"
  printf '/tmp/.X11-unix/X%s' "${d%%.*}"
}

resolve_display_mode() {
  local mode="${L2S_DISPLAY_MODE:-auto}"
  if [[ "$mode" == "auto" ]]; then
    if [[ -d /mnt/wslg ]]; then
      mode=wslg
    elif [[ -n "${DISPLAY:-}" && -S "$(x11_socket_for "$DISPLAY")" ]]; then
      mode=x11
    elif command -v Xvfb >/dev/null 2>&1; then
      mode=vnc
    else
      mode=headless
    fi
  fi
  printf '%s' "$mode"
}

start_vnc() {
  command -v Xvfb >/dev/null 2>&1 \
    || die "L2S_DISPLAY_MODE=vnc needs the sim-vnc image target"
  local disp=":99" geom="${L2S_VNC_GEOMETRY:-1600x900}"
  local sock; sock="$(x11_socket_for "$disp")"

  log "vnc: Xvfb $disp ${geom}x24, noVNC on :${L2S_NOVNC_PORT:-6080}"
  Xvfb "$disp" -screen 0 "${geom}x24" -nolisten tcp +extension GLX +render -noreset \
    >"$L2S_LOG_DIR/xvfb.log" 2>&1 &

  for _ in $(seq 1 50); do [[ -S "$sock" ]] && break; sleep 0.1; done
  [[ -S "$sock" ]] || die "Xvfb did not come up (see $L2S_LOG_DIR/xvfb.log)"
  export DISPLAY="$disp"

  fluxbox >"$L2S_LOG_DIR/fluxbox.log" 2>&1 &
  # No VNC password: the host must publish 6080 on 127.0.0.1 only.
  x11vnc -display "$disp" -forever -shared -nopw -localhost -rfbport 5900 -quiet \
    >"$L2S_LOG_DIR/x11vnc.log" 2>&1 &
  websockify --web /usr/share/novnc "0.0.0.0:${L2S_NOVNC_PORT:-6080}" localhost:5900 \
    >"$L2S_LOG_DIR/novnc.log" 2>&1 &
}

setup_display() {
  local mode; mode="$(resolve_display_mode)"
  case "$mode" in
    x11)
      [[ -n "${DISPLAY:-}" ]] || die "L2S_DISPLAY_MODE=x11 but DISPLAY is unset"
      [[ -S "$(x11_socket_for "$DISPLAY")" ]] \
        || warn "X socket $(x11_socket_for "$DISPLAY") not found; is /tmp/.X11-unix mounted?"
      ;;
    wslg)
      export DISPLAY="${DISPLAY:-:0}"
      export WAYLAND_DISPLAY="${WAYLAND_DISPLAY:-wayland-0}"
      export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/mnt/wslg/runtime-dir}"
      export PULSE_SERVER="${PULSE_SERVER:-/mnt/wslg/PulseServer}"
      ;;
    vnc)
      start_vnc
      ;;
    headless)
      unset DISPLAY WAYLAND_DISPLAY
      ;;
    *)
      die "unknown L2S_DISPLAY_MODE='$mode' (auto|x11|wslg|vnc|headless)"
      ;;
  esac
  L2S_EFFECTIVE_DISPLAY_MODE="$mode"
  export L2S_EFFECTIVE_DISPLAY_MODE
}

#------------------------------------------------------------------------------
# GL / GPU
#------------------------------------------------------------------------------
setup_gpu() {
  local gpu="${L2S_GPU:-auto}"
  if [[ "$gpu" == "auto" ]]; then
    if [[ "$L2S_EFFECTIVE_DISPLAY_MODE" == "vnc" ]]; then
      gpu=software                                   # Xvfb has no GPU
    elif command -v nvidia-smi >/dev/null 2>&1 && nvidia-smi -L >/dev/null 2>&1; then
      gpu=nvidia
    elif [[ -e /dev/dxg ]]; then
      gpu=dri                                        # WSLg d3d12 via mesa
    elif compgen -G '/dev/dri/renderD*' >/dev/null; then
      gpu=dri
    else
      gpu=software
    fi
  fi

  case "$gpu" in
    nvidia)
      export __NV_PRIME_RENDER_OFFLOAD=1
      export __GLX_VENDOR_LIBRARY_NAME=nvidia
      ;;
    dri)
      if [[ -e /dev/dxg && -d /usr/lib/wsl/lib ]]; then
        export LD_LIBRARY_PATH="/usr/lib/wsl/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
        export GALLIUM_DRIVER="${GALLIUM_DRIVER:-d3d12}"
      fi
      ;;
    software)
      # Decision 4: low VRAM / no GPU => llvmpipe with the GUI still on.
      export LIBGL_ALWAYS_SOFTWARE=1
      export GALLIUM_DRIVER=llvmpipe
      ;;
    *)
      die "unknown L2S_GPU='$gpu' (auto|nvidia|dri|software)"
      ;;
  esac
  L2S_EFFECTIVE_GPU="$gpu"
  export L2S_EFFECTIVE_GPU
}

#------------------------------------------------------------------------------
# Sidecars
#------------------------------------------------------------------------------
start_foxglove() {
  local want="${L2S_FOXGLOVE:-auto}"
  if [[ "$want" == "auto" ]]; then
    [[ "$L2S_EFFECTIVE_DISPLAY_MODE" == "headless" ]] && want=1 || want=0
  fi
  [[ "$want" == "1" ]] || return 0
  log "foxglove_bridge on :${L2S_FOXGLOVE_PORT:-8765}"
  ros2 launch foxglove_bridge foxglove_bridge_launch.xml \
    port:="${L2S_FOXGLOVE_PORT:-8765}" address:=0.0.0.0 \
    >"$L2S_LOG_DIR/foxglove_bridge.log" 2>&1 &
}

write_runtime_env() {
  # Read by the control API (Step 2) to report what the container resolved to.
  {
    printf 'L2S_EFFECTIVE_DISPLAY_MODE=%s\n' "$L2S_EFFECTIVE_DISPLAY_MODE"
    printf 'L2S_EFFECTIVE_GPU=%s\n' "$L2S_EFFECTIVE_GPU"
    printf 'DISPLAY=%s\n' "${DISPLAY:-}"
    printf 'ROS_DISTRO=%s\n' "${ROS_DISTRO:-}"
    printf 'GZ_VERSION=%s\n' "${GZ_VERSION:-}"
  } >"$RUNTIME_ENV_FILE"
}

###############################################################################
# Main
###############################################################################
cmd="${1:-serve}"
[[ $# -gt 0 ]] && shift

case "$cmd" in
  shell|bash)
    # Interactive debugging: no sidecars unless asked for via env.
    setup_display; setup_gpu; write_runtime_env
    exec bash -l
    ;;
esac

setup_display
setup_gpu
write_runtime_env
log "display=$L2S_EFFECTIVE_DISPLAY_MODE gpu=$L2S_EFFECTIVE_GPU uid=$(id -u) ros=${ROS_DISTRO:-?} gz=${GZ_VERSION:-?}"

case "$cmd" in
  serve)
    start_foxglove
    app_module="${L2S_API_APP%%:*}"
    if python3 -c "import importlib, sys; importlib.import_module(sys.argv[1])" "$app_module" 2>/dev/null; then
      rm -f /tmp/l2s_idle
      log "control API $L2S_API_APP on :${L2S_API_PORT:-9090}"
      exec uvicorn "$L2S_API_APP" --host 0.0.0.0 --port "${L2S_API_PORT:-9090}"
    fi
    warn "control API '$app_module' not importable (expected before Step 2); idling"
    touch /tmp/l2s_idle
    exec sleep infinity
    ;;
  idle)
    start_foxglove
    touch /tmp/l2s_idle
    exec sleep infinity
    ;;
  smoke)
    exec "$L2S_HOME/bin/smoke_test.sh" --inside "$@"
    ;;
  *)
    start_foxglove
    exec "$cmd" "$@"
    ;;
esac
