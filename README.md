# Lang2Sim

> Natural Language to Executable Robot Simulation with Dynamic Stress Testing.

## Overview
Sim2Lang provides an interactive, full-screen Terminal User Interface (TUI) copilot (`lang2sim`) powered by NVIDIA Nemotron 3 Ultra 550B A55B. It autonomously compiles natural language into procedurally generated Gazebo `.sdf` worlds, zero-SLAM 2D occupancy navigation grids, containerized native desktop Gazebo/RViz2 GUI launches with X11 pass-through, and closed-loop telemetry benchmark scoring ($NQI$).

## Installation
```bash
pip install -e .
```

## Launch
```bash
lang2sim
```

## Run a simulation (Part 3)
```bash
pip install -e ".[sim,dev]"
python scripts/run_sim.py --fixture small_warehouse --mission
```
Per-OS setup, the run-folder contract, the control API and troubleshooting:
[docs/part3-runtime.md](docs/part3-runtime.md).


## Benchmark and demo
- One sentence to a scored run: in the TUI `/sim build a dark warehouse with 3 racks and 2 crates`
  (or `bash pitch_demo.sh "..."` / `python scripts/prompt_to_sim.py "..."`)
- While it runs, just say it: "drop a crate in front of the robot", "put a rack across the aisle",
  "dim the lights to 10%", "detour and reach rack 2", "stop the robot"; `/stop` ends the sim
- Live dashboard: `http://localhost:9090/dashboard` while a sim runs
- In the TUI: `/mission`, `/stress`, `/benchmark`, `/report`, `/dashboard`, `/replay demo`, `/help`
- Score a run folder (also recovers runs cut short by a crash): `python -m lang2sim.telemetry.benchmark <run_dir>`
- Replay fallback, no Docker needed: `python -m lang2sim.telemetry.replay catalog/demo_runs/empty_room_benchmark --loop`
- Pitch runbook: [docs/demo.md](docs/demo.md) · telemetry internals: [docs/telemetry.md](docs/telemetry.md)
