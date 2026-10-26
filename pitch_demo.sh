#!/usr/bin/env bash
# Lang2Sim pitch demo (Ubuntu): prompt -> LLM/parser -> world + map -> Gazebo + Nav2 mission -> NQI score.
#   bash pitch_demo.sh                                   # default warehouse prompt
#   bash pitch_demo.sh "an empty room with a slippery floor"
# Ctrl-C stops the simulation cleanly.
set -e
cd "$(dirname "$0")"
PROMPT="${1:-a warehouse with 3 racks and 2 crates}"   # no "dark": dim light hides the robot in Gazebo

if [ "$1" = "--status" ]; then   # is the robot alive and driving?
  curl -s localhost:9090/v1/sim/status; echo; curl -s localhost:9090/v1/nav/status; echo
  curl -s localhost:9090/v1/telemetry; echo; exit 0
fi
say() { printf '\033[1;36m==> %s\033[0m\n' "$*"; }

# 1. Python env
if [ -z "$VIRTUAL_ENV" ]; then
  if [ ! -d .venv ]; then say "creating .venv (one time)"; python3 -m venv .venv; fi
  source .venv/bin/activate
fi
python -c "import lang2sim.pipeline, PIL" 2>/dev/null || { say "installing lang2sim (one time)"; pip install -q -e ".[sim,dev]"; }

# 2. Docker
docker info >/dev/null 2>&1 || { echo "Docker not reachable. Run: sudo usermod -aG docker \$USER  (then log out/in), or: sudo systemctl start docker"; exit 1; }

# 3. Sim image (build once if missing)
IMAGE="${L2S_IMAGE:-lang2sim-sim:pitch}"
if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
  if docker image inspect lang2sim-sim:dev >/dev/null 2>&1; then IMAGE=lang2sim-sim:dev
  else say "building $IMAGE (10-20 min, one time)"; docker build -f docker/sim/Dockerfile --target sim -t "$IMAGE" .; fi
fi
export L2S_IMAGE="$IMAGE"

# 4. Flags: GPU, LLM, display
FLAGS=(--no-camera)   # camera sensor not needed for the demo; keeps RTF up on CPU rendering
docker info --format '{{json .Runtimes}}' 2>/dev/null | grep -q nvidia || FLAGS+=(--software)
[ -n "$NVIDIA_API_KEY$OPENAI_API_KEY" ] || FLAGS+=(--no-llm)
command -v xhost >/dev/null && xhost +SI:localuser:"$(id -un)" >/dev/null 2>&1 || true
docker rm -f lang2sim-sim >/dev/null 2>&1 || true   # leftover from an earlier run

say "image $IMAGE  flags: ${FLAGS[*]}"
if [ "$1" = "--fixture" ]; then   # fallback: known-good warehouse + mission
  SW=(--no-camera); [[ " ${FLAGS[*]} " == *" --software "* ]] && SW+=(--software)
  exec python scripts/run_sim.py --fixture small_warehouse --mission "${SW[@]}"
fi
say "Gazebo + RViz windows open in ~20 s. The robot (TurtleBot3) starts at the bottom-left corner."
say "RViz shows it best: robot model, red laser dots, map, green planned path. Status: bash pitch_demo.sh --status"
exec python scripts/prompt_to_sim.py "$PROMPT" "${FLAGS[@]}"
