#!/usr/bin/env bash
# MVP demo, container side: bring up world + robot + Nav2 (+ GUIs), wait until
# the robot is publishing, run the waypoint mission, then optionally hold.
# Started by scripts/demo_mvp.py via the image entrypoint (`bash -c ...`), so
# display/GPU env is already configured.
#
# Env: L2S_DEMO_GUI / L2S_DEMO_RVIZ  true|false (default: true if DISPLAY set)
#      L2S_DEMO_HOLD                 1 = keep sim running after the mission
#      L2S_DEMO_RUN_DIR              where to write logs (default /tmp/l2s-logs)
# Args are passed to mvp_mission.py (e.g. --loop 0).
set -Eeo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"

has_display=false; [[ -n "${DISPLAY:-}" ]] && has_display=true
GUI="${L2S_DEMO_GUI:-$has_display}"
RVIZ="${L2S_DEMO_RVIZ:-$has_display}"
RUN_DIR="${L2S_DEMO_RUN_DIR:-/tmp/l2s-logs}"
mkdir -p "$RUN_DIR"
LOG="$RUN_DIR/demo_launch.log"

say() { printf '[demo] %s\n' "$*"; }

say "display=${L2S_EFFECTIVE_DISPLAY_MODE:-?} gpu=${L2S_EFFECTIVE_GPU:-?} gazebo_gui=$GUI rviz=$RVIZ"
say "launching TurtleBot3 world + Nav2 (log: $LOG)"
ros2 launch "$HERE/mvp_launch.py" gui:="$GUI" rviz:="$RVIZ" >"$LOG" 2>&1 &
LAUNCH_PID=$!

# shellcheck disable=SC2329  # invoked via trap
cleanup() {
  if kill -0 "$LAUNCH_PID" 2>/dev/null; then
    say "stopping simulation"
    kill -INT "$LAUNCH_PID" 2>/dev/null || true
    for _ in $(seq 1 20); do kill -0 "$LAUNCH_PID" 2>/dev/null || break; sleep 0.5; done
    kill -KILL "$LAUNCH_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT
trap 'exit 130' INT TERM

say "waiting for the robot to publish /odom ..."
deadline=$((SECONDS + 240))
until timeout 10 ros2 topic echo --once /odom --field header.frame_id >/dev/null 2>&1; do
  if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
    say "launch exited early; last log lines:"; tail -n 40 "$LOG"; exit 1
  fi
  if (( SECONDS > deadline )); then
    say "robot not up after 240s; last log lines:"; tail -n 40 "$LOG"; exit 1
  fi
done
say "robot is up: /odom, /scan, /tf flowing"

rc=0
python3 -u "$HERE/mvp_mission.py" "$@" || rc=$?
if (( rc == 0 )); then say "MISSION COMPLETE"; else say "MISSION FAILED (rc=$rc); see $LOG"; fi

if [[ "${L2S_DEMO_HOLD:-0}" == "1" ]]; then
  say "holding the simulation open (Ctrl-C in the host terminal to stop)"
  # Use RViz's "Nav2 Goal" button to send more goals by hand.
  wait "$LAUNCH_PID" || true
fi
exit "$rc"
