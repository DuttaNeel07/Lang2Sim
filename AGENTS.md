# AGENTS.md

Mandatory rules and architectural guidelines for AI coding agents. Violations will result in rejected PRs.

**Read [CONTRIBUTING.md](CONTRIBUTING.md) first.** It covers the general developer workflow (forking, environment setup, `pre-commit`, `pytest`, conventional commits, and targeting `staging`). Everything there applies. This document specifies AI-agent-specific requirements on top.

---

## Codebase Architecture & Reference

Always inspect the existing codebase before inventing new patterns or duplicating modules. Key subsystems:

- **`lang2sim/agent/`**: Agent brain, ReAct loop (`react_loop.py`), Nemotron LLM client (`nemotron_client.py`), grounding (`grounding.py`), and tools registry (`tools.py`).
- **`lang2sim/compiler/`**: SDF generation from spatial specs (`sdf_compiler.py`), Jinja2 world harmonic templates (`templates/world_harmonic.sdf.jinja`), SIR generation and validation (`sir_generator.py`, `sir_validator.py`).
- **`lang2sim/world_gen/`**: Spatial layout solver using Shapely (`shapely_solver.py`), occupancy map slicing (`map_slicer.py`), and fixture catalog loader (`catalog_loader.py`).
- **`lang2sim/runtime/`**: Execution orchestrator: Docker runner (`docker_runner.py`), ROS 2 bridge (`ros_bridge.py`), Control API FastAPI server & client (`control_api/`), display setup (`display.py`), health monitor (`health_monitor.py`), self-healing (`self_healing.py`), and run folder management (`run_folder.py`).
- **`lang2sim/telemetry/`**: Real-time subscribers (`ros_subscribers.py`, `live.py`), camera/vision capture (`vision.py`), trajectory evaluation (`trajectory.py`), NQI scoring ($NQI$) (`nqi_scorer.py`), and headless replay (`replay.py`).
- **`lang2sim/cli/`**: Textual TUI interface (`app.py`), screens (`screens/`), and widgets (`widgets/`).
- **`catalog/`**: Scene fixtures (`empty_room`, `small_warehouse`, `tb3_world`), map files (`.yaml`, `.pgm`), and demo run logs.
- **`docker/sim/`**: Container definitions for ROS 2 Humble + Gazebo Harmonic + Nav2 stack.

---

## AI-Specific Entry-Point Commands

Agents working on Lang2Sim should be familiar with these core CLI entry points:

```bash
# Launch the full-screen Textual TUI
lang2sim

# Run simulation workflow against a catalog fixture (requires Docker)
python scripts/run_sim.py --fixture small_warehouse --mission

# Replay an existing benchmark run without Docker
python -m lang2sim.telemetry.replay catalog/demo_runs/empty_room_benchmark --loop

# Run benchmark scoring on a run directory
python -m lang2sim.telemetry.benchmark catalog/demo_runs/empty_room_benchmark

# Lint and format checks
ruff check .
ruff format --check .

# Run test suite
pytest
```

---

## AI-Specific Rules

### 1. Search Before Implementing
Search the repository before introducing new utilities, schemas, or helper functions. Check existing models in `lang2sim/core/models.py`, config in `lang2sim/core/config.py`, and existing tool bindings in `lang2sim/agent/tools.py`. Do not create competing implementations.

### 2. Adhere to Target Branch Conventions
- The primary integration branch is `staging` (not `main`).
- Any new branch must branch off `staging` and open PRs targeting `staging`.

### 3. Use Conventional Commits
All commits must strictly follow the Conventional Commits format:
`<type>(<optional scope>): <description in imperative mood>` (e.g., `feat(runtime): add support for custom lidar topic`, `fix(telemetry): handle missing trajectory frames`).

### 4. Communication Guidelines
- **Terse and precise:** State clearly what changed and why. Avoid unsolicited diff recaps or meta-commentary.
- **No filler phrases:** Avoid openers like "Sure, I'd be happy to help", "As stated previously", or "Great question".
- **Focus on rationale:** Detail the non-obvious design choices or trade-offs rather than narrating self-evident code.

### 5. Validation Before Submission
Never consider a task complete without running:
```bash
ruff check .
ruff format --check .
pytest
```
Ensure no regressions or broken imports are introduced.
