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

