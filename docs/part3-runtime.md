# Part 3: simulation runtime

Part 3 takes a **run folder** (a Gazebo Harmonic `world.sdf`, a matching
`map.yaml`/`map.pgm`, `spec.json` and `manifest.json`), simulates it with a
TurtleBot3 Waffle and Nav2 inside a Docker container, and lets you watch it and
drive it. Nobody runs Docker or ROS commands by hand: everything goes through
`scripts/run_sim.py` (later `lang2sim run`) and the control API.

## Quickstart

```bash
git clone https://github.com/DuttaNeel07/Lang2Sim.git && cd Lang2Sim
python3 -m venv .venv && source .venv/bin/activate
pip install -e ".[sim,dev]"
python -m sim2lang.runtime.doctor                       # is this machine ready?
python scripts/run_sim.py --fixture small_warehouse --mission
```

`run_sim.py` checks the machine, picks how you will see the simulation, pulls
and starts the sim image, opens the run, waits until every readiness check
passes (Gazebo clock, odometry, laser scan, Nav2 active, robot localized) and,
with `--mission`, drives the run's waypoints and goal, records telemetry and
writes `benchmark_report.json` (NQI). `--benchmark` runs the mission nominal and
under stress and prints the comparison table ([telemetry.md](telemetry.md)).
Ctrl-C stops everything.

To open your own run folder instead of a fixture:

```bash
python scripts/run_sim.py ~/.lang2sim/projects/<project>/runs/<timestamp>
```

The folder must be under `~/.lang2sim/projects/` (it is mounted into the
container). Logs land in its `logs/`: `sim.log`, `world_check.txt`,
`heal_log.jsonl`.

## Per operating system

| Platform | You see the sim as | Rendering | Notes |
| --- | --- | --- | --- |
| Ubuntu 22.04 / 24.04 / 26.04 | Gazebo + RViz windows | Intel/AMD GPU via `/dev/dri`; NVIDIA via nvidia-container-toolkit; else CPU | Run from a desktop terminal, not over SSH. Install Docker Engine, add yourself to the `docker` group, log out and in. |
| Windows 11 + WSL2 | Gazebo + RViz windows (WSLg) | GPU via `/dev/dxg`, else CPU | Docker Desktop with WSL integration on; run everything from an Ubuntu terminal inside WSL2. |
| Windows 11 (PowerShell) | Browser tab, http://localhost:6080 | CPU | Works, but use WSL2 for real windows. |
| macOS (Apple Silicon or Intel) | Foxglove by default; `--display vnc` for the real Gazebo GUI in a browser | CPU | Docker cannot pass a Mac GPU through, so it runs slower than real time. Give Docker Desktop 8 GB and 4+ CPUs (Settings > Resources). |
| Headless server / CI | Foxglove, or nothing | CPU | `--display headless` |

Useful flags: `--display x11|wslg|vnc|headless`, `--software` (force CPU
rendering), `--no-gui`, `--no-rviz`, `--no-camera` (skip the camera bridge on
GPU-less machines), `--image <tag>`, `--dev` (mount this checkout's code into the
container, for development).

### Watching in Foxglove (macOS default, headless)

1. Install Foxglove (desktop app or https://app.foxglove.dev).
2. Open connection > Foxglove WebSocket > `ws://localhost:8765`.
3. Add a 3D panel and enable `/map`, `/scan`, `/tf`, `/plan` and
   `/global_costmap/costmap`; set the display frame to `map`.

## Run folder contract

| File | Written by | Meaning |
| --- | --- | --- |
| `world.sdf` | Part 1 | Gazebo Harmonic world, SDF 1.9+, **no robot inside** |
| `map.yaml`, `map.pgm` | Part 1 | Nav2 map; map frame equals the world frame |
| `spec.json` | Part 1/2 | The `SimSpec`, including the mission |
| `manifest.json` | Part 1/2 | `RunManifest`: file names, world name, robot, initial pose |
| `sim_launch.py`, `world.l2s.sdf`, `nav2_params.l2s.yaml`, `logs/` | Part 3 | Generated per run; do not edit |
| `robot.l2s.sdf`, `bridge.l2s.yaml`, `logs/telemetry_setup.txt` | Part 3 | Robot with chassis bumper + ground-truth pose, and its bridge topics |
| `logs/trajectory_<label>.json` | Part 3 | Recorded run (sim time, world frame), `Trajectory.to_dict()` format |
| `benchmark_report.json` | Telemetry | NQI scores of the recorded runs (nominal, adversarial) |

Before every launch the world goes through a pre-flight check
(`sim2lang/runtime/world_check.py`). It rejects worlds Gazebo or Nav2 cannot use
(invalid XML, malformed poses, a robot already in the world, unresolvable
`model://` URIs) with a readable report, and adds missing Gazebo systems the
robot needs (Sensors with ogre2, Imu, ...) to a normalized copy.

Three complete examples live in `catalog/fixtures/` (see its README).

## Control API (`http://127.0.0.1:9090/v1`)

Use `sim2lang.runtime.control_api.client.ControlClient` (async) or
`sim2lang.runtime.session.SimSession` rather than raw HTTP.

| Method | Path | Purpose |
| --- | --- | --- |
| GET | `/health`, `/info` | API alive; what the container resolved (display, GPU, ROS, Gazebo) |
| POST | `/sim/start` | `SimStartRequest`: pre-flight, generate `sim_launch.py`, launch |
| GET | `/sim/status` | `SimStatusResponse`: status, readiness checks, RTF, sim time, world report |
| GET | `/sim/health` | healthy / degraded / crashed and why |
| POST | `/sim/stop` | Stop the run |
| GET | `/sim/logs?tail=N` | Last N lines of the launch output |
| POST | `/nav/initial_pose`, `/nav/goal`, `/nav/waypoints`, `/nav/cancel` | Navigation (409 until the sim is ready) |
| GET | `/nav/status` | `NavStatus`: task, waypoint, distance remaining, ETA, recoveries |
| WS | `/nav/feedback?hz=5` | Stream of `{"nav", "telemetry", "live", "sim"}` |
| GET | `/telemetry` | `SimStatus`: map pose, velocity, RTF, render mode, health |
| GET | `/heal/log` | Self-healing actions this run |
| GET | `/dashboard` | Live benchmark dashboard (data: `/run`, `/run/map`, `/run/trajectories`, `/run/report`) |
| GET | `/telemetry/live` | `TelemetryLive`: world pose + source, speed, lidar clearance, distance to goal, collisions |
| GET | `/telemetry/setup` | Was this run's robot instrumented (bumper + ground truth), and why not |
| POST | `/telemetry/record/start`, `/telemetry/record/stop` | Record a run; stop returns `TelemetryRecordResult` and writes `logs/trajectory_<label>.json` |
| POST/GET | `/world/spawn`, `/world/light`, `/world/reset`, `/world/entities` | Live mutations (Part 4); spawns are logged with their footprint |

## Self-healing

While a run is up the API server checks every 2 s and recovers four faults,
logging each action to `logs/heal_log.jsonl`:

| Fault | Recovery |
| --- | --- |
| Missing package | Reported only; nothing is installed at runtime (rebuild the image) |
| Missing TF (robot_state_publisher died, TF lookups failing) | Restart robot_state_publisher |
| Unlinked scan (bridge died, no `/scan` while the clock runs) | Restart the ROS-Gazebo bridge |
| Goal in collision (planner cannot reach it) | Move the goal to the nearest free map cell and resend, keeping the rest of the mission |

Each recovery is rate-limited (15 s cooldown, 3 attempts per 5 minutes, then
it gives up and says so). Set `L2S_SELF_HEAL=0` to turn it off.

## Tests

| Command | Checks |
| --- | --- |
| `pytest -q` | Unit tests; no Docker needed (runs in CI) |
| `docker/sim/smoke_test.sh <dri\|gpu\|software\|wslg\|vnc\|headless> [--interactive]` | Image, OpenGL, Gazebo GUI |
| `L2S_INTEGRATION=1 L2S_TEST_IMAGE=lang2sim-sim:dev pytest -q tests/integration` | Container modes, every fixture reaches ready, self-healing with killed nodes |

Build a local image first:
`docker buildx build -f docker/sim/Dockerfile --target sim -t lang2sim-sim:dev --load .`
(`--target sim-vnc -t lang2sim-sim:dev-vnc` for the browser view).

## Troubleshooting

| Symptom | Fix |
| --- | --- |
| `doctor` says Docker is not reachable | Start Docker Desktop / `sudo systemctl start docker`; on Linux, be in the `docker` group |
| No window appears on Ubuntu | Run from the desktop session; `echo $DISPLAY` must print something like `:0` |
| `glxinfo` shows `llvmpipe` with `dri` | The container is not getting the GPU; `--software` works, just slower |
| `422 world rejected` from `/sim/start` | Read the issues list (also in `logs/world_check.txt`) and fix the world |
| Sim runs far below real time | Give Docker more CPUs; use `--no-camera` and `--no-gui` on CPU rendering |
| A goal is never reached | `GET /v1/heal/log` and `logs/sim.log`; check the goal is inside free space on the map |
