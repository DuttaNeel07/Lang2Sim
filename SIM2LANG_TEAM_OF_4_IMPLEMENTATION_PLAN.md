# Sim2Lang: 4-Person Parallel Implementation Blueprint & GitHub Issues Specification

> **Document Version:** 2.0.0  
> **Source Architecture Reference:** `Sim2Lang Robot Simulation Architecture (1).md`  
> **Target Execution Strategy:** 4-Person Agile Sprint / Parallel Feature Branches / Zero-Blocker PR Integration  

---

## 1. Executive Overview & Team Work Distribution

The **Sim2Lang** platform automates the entire robotics simulation lifecycle: transforming conversational natural language into procedurally generated Gazebo `.sdf` worlds, zero-SLAM 2D occupancy navigation grids (`map.yaml`, `map.pgm`), containerized native desktop Gazebo/RViz2 GUI launches with X11 pass-through, programmatic Nav2 Simple Commander missions, runtime adversarial perturbations via FastMCP, dual-stream visual perception (robot camera + X11 GUI screen grabbing), and closed-loop telemetry benchmark scoring ($NQI$).

To enable a **team of 4 engineers** to work fully in parallel without stepping on each other's code or getting blocked by dependencies, the codebase is partitioned into **4 decoupled, independently testable domains**:

```
                                  ┌────────────────────────────────────────────────────────┐
                                  │      DAY 0 SEED: sim2lang/core/ (Shared Schemas)        │
                                  └──────────────────────────┬─────────────────────────────┘
                                                             │
                 ┌───────────────────────────┬───────────────┴───────────────┬───────────────────────────┐
                 ▼                           ▼                               ▼                           ▼
          ┌──────────────┐            ┌──────────────┐                ┌──────────────┐            ┌──────────────┐
          │    PART 1    │            │    PART 2    │                │    PART 3    │            │    PART 4    │
          │ World Gen,   │            │ Agent Brain  │                │ Runtime,     │            │ FastMCP,     │
          │ Compiler &   │            │ & Textual    │                │ Launch &     │            │ Telemetry &  │
          │ Asset Catalog│            │ TUI Frontend │                │ Self-Healing │            │ Vision Grab  │
          └──────┬───────┘            └──────┬───────┘                └──────┬───────┘            └──────┬───────┘
                 │                           │                               │                           │
                 ▼                           ▼                               ▼                           ▼
          [PR 1: Geometry]            [PR 2: Mocked Tools]            [PR 3: Standalone Sim]      [PR 4: Bridge & Math]
                 └───────────────────────────┴───────────────┬───────────────┴───────────────────────────┘
                                                             │
                                                             ▼
                                  ┌────────────────────────────────────────────────────────┐
                                  │   INTEGRATION PHASE: Wire Real Tools & Run Archetypes   │
                                  │   VALIDATION: 4 Benchmark Archetypes (Pitch Ready)     │
                                  └────────────────────────────────────────────────────────┘
```

### High-Level Responsibility Matrix

| Subsystem / Role | Assigned Developer | Core Responsibilities | Key Deliverables |
| :--- | :--- | :--- | :--- |
| **Part 1: Procedural World Builder, Compiler & Catalog** | **Developer 1** | Spatial geometry, Shapely 2.0 OBB packing, zero-SLAM map slicing, multi-target SDF Jinja2 compiler, scene catalog, SIR schema validation. | `sim2lang/world_gen/`, `sim2lang/compiler/` (SDF/SIR), `catalog/`, `sim2lang/core/` |
| **Part 2: Autonomous Agent Brain & Textual TUI** | **Developer 2** | NVIDIA Nemotron 3 Ultra 550B client, ReAct reasoning loop, typed tool registry, 80/20 split OpenCode-style Textual TUI (`lang2sim`). | `sim2lang/cli/`, `sim2lang/agent/`, `pyproject.toml` |
| **Part 3: Container Runtime, Nav2 Launch & Self-Healing** | **Developer 3** | Docker X11 display pass-through, hardware acceleration matrix, parametric `sim_launch.py`, Nav2 Simple Commander bringup, self-healing log monitor. | `sim2lang/runtime/`, `sim2lang/compiler/launch_builder.py` |
| **Part 4: FastMCP Perturbations, Vision & Telemetry Engine** | **Developer 4** | In-process FastMCP tool server, Gazebo dynamic mutation bridge, dual-stream visual observer, ROS 2 subscribers, NQI benchmark scorer. | `sim2lang/perturbation/`, `sim2lang/telemetry/` |

---

## 2. Team Operational Protocol & Git Workflow

### 2.1 The "Day 0" Seed Commit (Immediate Action)
Before any developer creates a feature branch, **Developer 1** (or team lead) commits the root project skeleton to `main`:
1. `pyproject.toml` (package metadata, dependencies, and `[project.scripts]` entrypoint `lang2sim = "sim2lang.cli.app:main"`).
2. `sim2lang/core/models.py` (canonical Pydantic v2 schemas: `Pose2D`, `Vector3D`, `SkyWeatherSpec`, `EnvironmentSpec`, `ObjectSpec`, `SensorNoiseSpec`, `MissionSpec`, `SimSpec`, `SimulationIR`).
3. `sim2lang/core/config.py` (filesystem paths, default project directories `~/.lang2sim/projects/`, environment variables).

> **Why this prevents blockers:** Every teammate imports the exact same typed data structures from Day 1. There is zero ambiguity about coordinates, units, or JSON keys.

### 2.2 Branching Convention
```bash
git checkout -b feature/part-1-world-compiler-catalog
git checkout -b feature/part-2-agent-brain-textual-tui
git checkout -b feature/part-3-runtime-docker-nav2
git checkout -b feature/part-4-fastmcp-vision-telemetry
```

### 2.3 PR Review & Merge Order
To ensure a clean, conflict-free integration:
1. **Merge PR #1 (World Gen & Catalog):** Establishes the asset depot, verified maps, and procedural world compilation.
2. **Merge PR #3 (Runtime & Launch):** Ingests compiled `world.sdf` and `map.yaml` from PR #1, launching Gazebo and Nav2.
3. **Merge PR #4 (FastMCP & Telemetry):** Connects dynamic perturbations, vision screen grabbing, and NQI telemetry to the running Gazebo/Nav2 stack from PR #3.
4. **Merge PR #2 (Agent & TUI):** Replaces mock tool stubs in `sim2lang/agent/tools.py` with the real implementations provided by PRs #1, #3, and #4, exposing the full interactive CLI copilot.

---

## 3. GitHub Issue Specifications (Ready to Post)

---

### GitHub Issue #1: Procedural World Builder, Multi-Target SDF Compiler & Asset Catalog

**Title:** `[Part 1] Procedural World Builder, Multi-Target SDF Compiler & Curated Asset Catalog`  
**Assignee:** Developer 1  
**Labels:** `geometry`, `compiler`, `sdf`, `catalog`, `core`  
**Milestone:** Phase 1 (Hour 0–24)  

#### 1. Objective
Implement the procedural spatial arrangement engine, zero-SLAM 2D map slicer, multi-target SDF Jinja2 template generator, Simulation Intermediate Representation (SIR) compiler pipeline, and curated asset catalog.

#### 2. Detailed Deliverables & File Mapping
* [sim2lang/core/models.py](file:///home/gigikoneti/l2s/sim2lang/core/models.py): Canonical Pydantic v2 schemas for `Pose2D`, `Vector3D`, `SkyWeatherSpec`, `EnvironmentSpec`, `ObjectSpec`, `CameraPerturbation`, `LidarPerturbation`, `SensorNoiseSpec`, `DynamicObstacleTrigger`, `MissionSpec`, `SimSpec`, and `SimulationIR`.
* [sim2lang/core/config.py](file:///home/gigikoneti/l2s/sim2lang/core/config.py): Project workspace path manager (`~/.lang2sim/projects/<project_name>/`), scene catalog paths, template roots.
* `catalog/`:
  * `catalog/scene_catalog.json`: Index pre-built environments (AWS Small Warehouse, robotics lab, obstacle arena) with semantic anchors (e.g. `rack_1: [2.5, 3.0, 0.0]`, `dock: [0.0, 0.0, 0.0]`), verified Nav2 costmaps (`map.yaml`/`map.pgm`), surface friction ($\mu = 0.8$), and inertial bounds.
  * `catalog/maps/aws_small_warehouse.yaml` and `catalog/maps/aws_small_warehouse.pgm`.
  * `catalog/models/`: PBR textured SDF models (pallet racks, benches, barrels, wooden crates).
* [sim2lang/world_gen/shapely_solver.py](file:///home/gigikoneti/l2s/sim2lang/world_gen/shapely_solver.py):
  * Shapely 2.0 computational geometry solver for Oriented Bounding Box (OBB) packing.
  * Replaces naive rejection sampling with polygon buffering:
    $$W_{\text{corridor}} \ge 2 \cdot r_{\text{robot}} + \delta_{\text{safety}} \quad (\ge 2.2\text{m})$$
    $$\text{Wall standoff} \ge 0.5\text{m}$$
  * Semantic grouping rules (e.g. chairs anchored facing desks, pallets aligned along rack faces).
* [sim2lang/world_gen/map_slicer.py](file:///home/gigikoneti/l2s/sim2lang/world_gen/map_slicer.py):
  * Direct 3D geometric slicing between LiDAR scanning plane $z \in [0.1\text{m}, 0.5\text{m}]$.
  * Shapely rasterizer fallback: converts 2D polygon layouts directly into 2D NumPy occupancy arrays ($0.05\text{m}$ resolution) and writes standard P5 grayscale `map.pgm` and `map.yaml` in $<3.5\text{s}$ without Cartographer/slam_toolbox.
* [sim2lang/compiler/sdf_compiler.py](file:///home/gigikoneti/l2s/sim2lang/compiler/sdf_compiler.py):
  * Jinja2 template engine synthesizing multi-target worlds:
    * **Gazebo Classic (SDF 1.6):** Plugins `libgazebo_ros_init.so`, `libgazebo_ros_ray_sensor.so`, `libgazebo_ros_bumper.so`.
    * **Gazebo Harmonic (SDF 1.9/1.10):** System plugins `gz::sim::systems::Physics`, `gz::sim::systems::UserCommands`, `gz::sim::systems::SceneBroadcaster`, `gz::sim::systems::Contact`.
  * Multi-environment tags: `<sky>`, `<clouds>`, volumetric `<fog>` (density $0.0 - 0.5$), `<surface><friction>` ($\mu = 0.8$ concrete down to $0.15$ slippery ice), and `<heightmap>` terrain.
* [sim2lang/compiler/sir_generator.py](file:///home/gigikoneti/l2s/sim2lang/compiler/sir_generator.py) & [sim2lang/compiler/sir_validator.py](file:///home/gigikoneti/l2s/sim2lang/compiler/sir_validator.py):
  * PipeGen 5-stage compiler: converts `MissionState.json` into canonical `simulation_ir.json`.
  * Verifies geometric bounds, non-intersection of static entities, and waypoint reachability.

#### 3. Isolated Unit Testing Strategy (No Gazebo Needed)
- Test `shapely_solver.py` with 10 random warehouse layout requests: verify 0 polygon overlaps and minimum corridor clearance $> 2.2\text{m}$.
- Test `map_slicer.py`: verify output `map.yaml` parses correctly and `map.pgm` can be loaded via `PIL.Image`.
- Test `sdf_compiler.py`: validate generated SDF XML using Python `sdformat` bindings (`sdformat13`/`sdformat14`) or `xml.etree.ElementTree`.

#### 4. Definition of Done
- [ ] `sim2lang/core/models.py` matches Section 4.3 Pydantic schemas.
- [ ] Catalog indexes `aws_small_warehouse` with semantic coordinates.
- [ ] Shapely solver packs objects without collision.
- [ ] `world.sdf`, `map.yaml`, and `map.pgm` are emitted to `~/.lang2sim/projects/<name>/` within 4 seconds.
- [ ] 100% passing pytest suite in `tests/test_world_gen.py`.

---

### GitHub Issue #2: Autonomous Supervisor Agent Brain & OpenCode-Style Textual TUI

**Title:** `[Part 2] Autonomous Supervisor Agent (Nemotron 3 ReAct) & OpenCode-Style Textual TUI (lang2sim)`  
**Assignee:** Developer 2  
**Labels:** `agent`, `tui`, `textual`, `nemotron`, `cli`  
**Milestone:** Phase 1 & 2 (Hour 0–36)  

#### 1. Objective
Build the interactive, full-screen Terminal User Interface (TUI) in Python `Textual` (`lang2sim`), integrate the NVIDIA Nemotron 3 Ultra 550B ReAct execution loop, maintain multi-turn conversational context, and implement the typed tool registry.

#### 2. Detailed Deliverables & File Mapping
* `pyproject.toml`:
  * Package setup with `[project.scripts]` defining `lang2sim = "sim2lang.cli.app:main"`.
  * Dependencies: `textual>=0.70.0`, `rich>=13.7.0`, `openai>=1.14.0`, `pydantic>=2.6.0`, etc.
* [sim2lang/cli/app.py](file:///home/gigikoneti/l2s/sim2lang/cli/app.py):
  * Main Textual `App` with reactive state management, asynchronous event loop, and global hotkeys (`Ctrl+P` command palette, `Tab` agent switch, `Ctrl+C` exit).
* [sim2lang/cli/screens/](file:///home/gigikoneti/l2s/sim2lang/cli/screens/):
  * `landing_screen.py`: Screen 1 featuring centered ASCII/block logo `lang2sim`, floating input card (`#161616` background, `border-left: wide #00d2ff`), badge pills (`Build · Nemotron 3 Ultra 550B A55B Nvidia · medium`), history navigation (`PgUp`/`PgDn`).
  * `workspace_screen.py`: Screen 2 featuring dynamic 80/20 split-screen workspace:
    * **Left Pane (80%):** Prompt display with vertical cyan accent bar, collapsible thought block (`+ Thought: 1.0s` in orange), streaming markdown responses, tool execution pills (`🗄️ Build · Nemotron · 19.2s`), sticky input card.
    * **Right Sidebar Pane (20%):** Session token context, Gazebo simulation status (GUI active, Sim Time, RTF), Robot telemetry ($x, y$, linear speed, obstacle clearance), Bumper collision counter, Vision perception status (/camera FPS).
* [sim2lang/cli/widgets/](file:///home/gigikoneti/l2s/sim2lang/cli/widgets/): `thought_box.py`, `telemetry_sidebar.py`, `prompt_card.py`, `action_pill.py`.
* [sim2lang/cli/styles/app.tcss](file:///home/gigikoneti/l2s/sim2lang/cli/styles/app.tcss): Textual CSS definitions (`#0c0c0c` pitch black background, `#00d2ff` cyan borders, `#161616` cards).
* [sim2lang/agent/nemotron_client.py](file:///home/gigikoneti/l2s/sim2lang/agent/nemotron_client.py):
  * OpenAI-compatible API client pointing to NVIDIA NIM endpoint for Nemotron 3 Ultra 550B A55B.
  * Supports streaming response tokens, tool calling function schema generation, and token counter.
  * **Offline Heuristic Fallback Parser:** Rule-based regex parser ensuring basic pipeline execution even if network fails during live demonstrations.
* [sim2lang/agent/react_loop.py](file:///home/gigikoneti/l2s/sim2lang/agent/react_loop.py):
  * Asynchronous Perception-Reasoning-Action supervisor loop: *Think $\to$ Tool Call $\to$ Observe $\to$ Re-plan*.
  * Manages multi-turn conversation memory and context files (`.sim2lang.json` and human-readable `SIM2LANG.md`).
* [sim2lang/agent/tools.py](file:///home/gigikoneti/l2s/sim2lang/agent/tools.py):
  * Typed tool registry mapping 14 functions across:
    * *Procedural/World:* `solve_spatial_layout`, `compile_world_sdf`, `generate_zero_slam_map`.
    * *Runtime/Launch:* `launch_simulation_stack`, `inspect_topic_health`.
    * *Perturbation:* `spawn_dynamic_entity`, `set_ambient_lighting`, `reset_simulation`.
    * *Telemetry/Vision:* `capture_gazebo_window`, `capture_rviz_window`, `read_robot_camera`, `query_robot_telemetry`, `auto_repair_tf`.

#### 3. Isolated Unit Testing Strategy (No Simulator Needed)
- Developer 2 implements mock returns in `tools.py` (e.g. `launch_simulation_stack()` returns `"Mock Gazebo running on :0 (PID 9999)"`).
- Test `lang2sim` CLI launch: verify landing screen switches to workspace screen upon entering a prompt.
- Test thought collapsible box expanding and collapsing on click/keypress.
- Test streaming token rendering and fallback parser with simulated network disconnect.

#### 4. Definition of Done
- [ ] Executable via `lang2sim` from any terminal directory.
- [ ] Textual TUI renders Screen 1 and Screen 2 cleanly at 60 FPS without UI freezes.
- [ ] Nemotron 3 API client streams tokens and extracts tool calls.
- [ ] Collapsible thought blocks and tool status pills render correctly.
- [ ] Offline heuristic fallback works without an API key.

---

### GitHub Issue #3: Containerized Simulation Runtime, Nav2 Launch & Self-Healing Engine

**Title:** `[Part 3] Dockerized Runtime (X11 Pass-Through), ROS 2 Nav2 Launch & Self-Healing Supervisor`  
**Assignee:** Developer 3  
**Labels:** `docker`, `x11`, `ros2`, `nav2`, `self-healing`  
**Milestone:** Phase 2 (Hour 12–36)  

#### 1. Objective
Implement the Dockerized simulation runtime with X11 display pass-through, hardware GPU/DRI acceleration detection, parametric ROS 2 Python launch scripts, Nav2 Simple Commander mission dispatching, and automated self-healing error supervisor.

#### 2. Detailed Deliverables & File Mapping
* [sim2lang/runtime/docker_runner.py](file:///home/gigikoneti/l2s/sim2lang/runtime/docker_runner.py):
  * **Host Display Server Authorization:** Automatically executes `xhost +local:root > /dev/null 2>&1`.
  * **Hardware Launch Matrix:**
    * *NVIDIA GPU:* `--gpus all --runtime=nvidia -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all`
    * *Intel/AMD Integrated:* `--device /dev/dri:/dev/dri -e LIBGL_ALWAYS_INDIRECT=0`
    * *Headless / Fallback:* `-e LIBGL_ALWAYS_SOFTWARE=1 -e QT_X11_NO_MITSHM=1`
  * Standard Docker execution template mounting X11 sockets (`/tmp/.X11-unix`), `.Xauthority`, `~/.lang2sim/projects`, and `catalog/` into container (`sih26050_simulation:latest` or `docker-project_zero_sim:latest`).
* [sim2lang/runtime/hardware_probe.py](file:///home/gigikoneti/l2s/sim2lang/runtime/hardware_probe.py):
  * Hardware health check: queries `/proc/meminfo` and NVML. If available VRAM $< 2.0\text{GB}$, forces headless physics (`-s`) to preserve graphics resources for X11/web rendering.
* [sim2lang/compiler/launch_builder.py](file:///home/gigikoneti/l2s/sim2lang/compiler/launch_builder.py):
  * Parametric ROS 2 launch generator producing `sim_launch.py`.
  * Integrates Gazebo server/client, `robot_state_publisher`, `nav2_bringup` (with map server loading `map.yaml`), static transforms, and RViz2.
  * Standardizes on pre-tuned TurtleBot3 Waffle / TurtleBot4 configurations.
* [sim2lang/runtime/nav2_dispatcher.py](file:///home/gigikoneti/l2s/sim2lang/runtime/nav2_dispatcher.py):
  * Python interface to `nav2_simple_commander.BasicNavigator`:
    * `setInitialPose(initial_pose)`
    * `goToPose(goal_pose)`
    * `followWaypoints(waypoint_list)`
    * `cancelTask()`
    * `getFeedback()` and `isTaskComplete()`
* [sim2lang/runtime/self_healing.py](file:///home/gigikoneti/l2s/sim2lang/runtime/self_healing.py):
  * Subprocess stdout/stderr log monitor with deterministic regex recovery table:
    * *Missing package:* detects `Package 'xyz' not found` $\to$ executes `apt-get install ros-$ROS_DISTRO-xyz` or `rosdep`.
    * *Transform frame drop:* detects `frame 'base_scan' does not exist` $\to$ dynamically spawns ephemeral `tf2_ros static_transform_publisher`.
    * *Unlinked sensor topic:* scans topics; if laser data is on `/lidar/points` instead of `/scan`, spawns `topic_tools relay`.
    * *Unreachable goal in collision:* detects Nav2 planner collision abort $\to$ evaluates distance transform of `map.pgm` and shifts goal pose outward along normal by $0.3\text{m}$.
* [sim2lang/runtime/mode_b_fallback.py](file:///home/gigikoneti/l2s/sim2lang/runtime/mode_b_fallback.py):
  * Dual-mode supervisor: monitors `/clock` and Real-Time Factor (RTF). If RTF $< 0.3$ for $> 3$ consecutive seconds or an OpenGL crash occurs, triggers hot-switch to Mode B synthetic MCAP replay.

#### 3. Isolated Unit Testing Strategy (No Agent/Compiler Needed)
- Test `docker_runner.py` using a minimal test container and verify native `xclock` or `glxgears` appears on the host display.
- Test `launch_builder.py` by generating a launch script for an empty world and executing `ros2 launch`.
- Test `self_healing.py` by passing simulated error log snippets and asserting the correct recovery command is formulated.

#### 4. Definition of Done
- [ ] Docker container launches with X11 forwarding; Gazebo GUI and RViz2 desktop windows open visibly on host desktop.
- [ ] Nav2 `BasicNavigator` initializes localization and commands robot navigation.
- [ ] Pre-flight VRAM probe correctly selects headless vs GUI mode.
- [ ] Self-healing monitor intercepts simulated TF disconnects and emits recovery command.

---

### GitHub Issue #4: In-Process FastMCP Dynamic Perturbations, Dual-Stream Vision & Telemetry NQI Scorer

**Title:** `[Part 4] FastMCP Dynamic Stress Controls, Dual-Stream Visual Perception & Telemetry NQI Evaluator`  
**Assignee:** Developer 4  
**Labels:** `fastmcp`, `telemetry`, `vision`, `nqi`, `perturbation`  
**Milestone:** Phase 3 (Hour 24–48)  

#### 1. Objective
Implement the in-process FastMCP server and Gazebo bridge for live simulation mutations (model spawning, dynamic lighting, sub-second world resets), the dual-stream visual observer (X11 GUI screen grabbing + ROS camera), contact collision monitoring, and the composite Navigation Quality Index ($NQI$) benchmark evaluator.

#### 2. Detailed Deliverables & File Mapping
* [sim2lang/perturbation/mcp_server.py](file:///home/gigikoneti/l2s/sim2lang/perturbation/mcp_server.py):
  * In-process FastMCP 3.4 server running in the application's memory space ($0\text{ms}$ IPC overhead, preventing socket timeouts during demos).
  * Exposes `@mcp.tool()` decorators for:
    * `spawn_dynamic_entity(model_name, pose)`
    * `set_ambient_lighting(luminance)`
    * `reset_simulation(mode)`
    * `apply_chassis_perturbation(velocity_bias)`
* [sim2lang/perturbation/gazebo_bridge.py](file:///home/gigikoneti/l2s/sim2lang/perturbation/gazebo_bridge.py):
  * Simulator-agnostic dispatcher:
    * *Gazebo Harmonic:* calls `gz service -s /world/<name>/create` (EntityFactory) and `gz service -s /world/<name>/set_physics`.
    * *Gazebo Classic:* calls `/spawn_entity` (`gazebo_msgs/srv/SpawnEntity`) and `/set_light_properties`.
    * *Sub-second reset:* calls `/world/<name>/control` (`reset: {all: true}`) or `/reset_world` snapping robot back in $<0.3\text{s}$ without restarting GUI.
* [sim2lang/telemetry/vision.py](file:///home/gigikoneti/l2s/sim2lang/telemetry/vision.py):
  * **Dual-Stream Visual Observer:**
    * *Stream 1 (Robot Perspective):* Subscribes to `/camera/image_raw` using `cv_bridge` or pure NumPy.
    * *Stream 2 (Desktop GUI Window):* Captures native desktop GUI windows via `mss` / `xdotool` targeting `"Gazebo"` and `"RViz"`. Visually detects Play/Pause state, RTF, scene tree, Displays sidebar, costmap coloring, and path line.
  * Encodes frames as base64 JPEG strings for Nemotron multimodal visual reasoning.
* [sim2lang/telemetry/ros_subscribers.py](file:///home/gigikoneti/l2s/sim2lang/telemetry/ros_subscribers.py):
  * Background asynchronous subscriber thread logging `/odom`, `/tf`, and `/sim2lang/contact_states`.
  * Monitors robot chassis contact bumper plugin: increments collision counter $C_{\text{force}}$ for any contact force vector exceeding $5.0\text{N}$.
* [sim2lang/telemetry/nqi_scorer.py](file:///home/gigikoneti/l2s/sim2lang/telemetry/nqi_scorer.py):
  * Mathematical evaluation engine computing the composite Navigation Quality Index:
    1. **Path Traversal Efficiency ($E_p$):**
       $$E_p = \frac{L_{\text{optimal}}}{L_{\text{actual}}} = \frac{\int_{p_0}^{p_T} \|dp_{A^*}\|}{\int_0^T \|\mathbf{v}(t)\| dt}$$
    2. **Trajectory Smoothness & Actuator Stress ($S_\tau$):**
       $$S_\tau = \int_0^T \left( \|\dot{\mathbf{a}}(t)\|^2 + \dot{\omega}(t)^2 \right) dt$$
    3. **Minimum Obstacle Clearance ($C_{\text{min}}$):**
       $$C_{\text{min}} = \min_{t \in [0, T]} \left( \min_{o \in \mathcal{O}} \|\mathbf{p}_{\text{robot}}(t) - \mathbf{p}_o\|_2 - r_{\text{robot}} \right)$$
    4. **Mission Success Metric ($S_{\text{mission}}$):**
       $$S_{\text{mission}} = \mathbb{I}(\|\mathbf{p}(T) - \mathbf{p}_{\text{goal}}\|_2 \le \epsilon_{\text{pos}}) \cdot \mathbb{I}(C_{\text{force}} == 0)$$
    5. **Composite NQI Score:**
       $$NQI = 100 \cdot \left[ w_1 E_p + w_2 \frac{1}{1 + \alpha S_\tau} + w_3 \min\left(1, \frac{C_{\text{min}}}{C_{\text{safe}}}\right) \right] - P_{\text{collision}} \cdot C_{\text{force}}$$
       *(Normalized weights: $w_1 = 0.35, w_2 = 0.25, w_3 = 0.40, \alpha = 0.05, C_{\text{safe}} = 0.5\text{m}, P_{\text{collision}} = 50$)*
  * Emits structured `benchmark_report.json` and renders the Rich-styled comparative analytics table (Nominal vs. Adversarial Stress Run).

#### 3. Isolated Unit Testing Strategy (No Full Stack Needed)
- Test `vision.py`: grab desktop screen on active Linux workstation; verify JPEG base64 string is produced and uncorrupted.
- Test `nqi_scorer.py`: feed two synthetic trajectory arrays (nominal straight line vs. erratic oscillating detour) and assert $E_p$, $S_\tau$, and $NQI$ match mathematical expectations.
- Test `mcp_server.py`: execute tool calls directly in Python `asyncio` loop without socket errors.

#### 4. Definition of Done
- [ ] FastMCP tools execute in-process with $0\text{ms}$ IPC overhead.
- [ ] Gazebo bridge spawns dynamic model and dims lights in running simulator.
- [ ] Sub-second world reset restores robot pose without reloading Gazebo.
- [ ] Vision capture grabs both `/camera/image_raw` and desktop X11 windows.
- [ ] Bumper sensor registers physical contact force $> 5.0\text{N}$.
- [ ] NQI scorer generates `benchmark_report.json` and renders Rich comparison table.

---

## 4. Complete Data Contracts & Shared Specifications

All four parts strictly adhere to the canonical schema defined in `sim2lang/core/models.py`.

```python
"""
Canonical Simulation Specification Schema (Pydantic v2)
File: sim2lang/core/models.py
"""

from pydantic import BaseModel, Field
from typing import List, Optional, Literal

class Pose2D(BaseModel):
    x: float = Field(..., description="X coordinate in meters relative to world origin")
    y: float = Field(..., description="Y coordinate in meters relative to world origin")
    yaw: float = Field(0.0, ge=-3.14159, le=3.14159, description="Yaw orientation in radians")

class Vector3D(BaseModel):
    x: float = Field(..., description="Size X in meters")
    y: float = Field(..., description="Size Y in meters")
    z: float = Field(..., description="Size Z in meters")

class SkyWeatherSpec(BaseModel):
    sky_enabled: bool = True
    clouds_speed: float = Field(10.0, ge=0.0, le=50.0, description="Procedural cloud animation speed")
    fog_enabled: bool = False
    fog_density: float = Field(0.0, ge=0.0, le=0.5, description="Volumetric fog density (0.05 = light, 0.2 = heavy mist)")
    sun_elevation: float = Field(0.8, ge=0.0, le=1.57, description="Solar elevation angle in radians (0.0 = sunset/dusk)")

class EnvironmentSpec(BaseModel):
    environment_type: Literal["indoor_warehouse", "indoor_office", "outdoor_field", "outdoor_terrain"] = "indoor_warehouse"
    room_dimensions: Vector3D = Field(..., description="Outer room/field boundaries in meters")
    wall_height: float = Field(2.5, ge=0.0, le=10.0, description="Wall boundary height (0.0 for open outdoor fields)")
    ambient_light_luminance: float = Field(
        1.0, ge=0.0, le=1.0, description="Ambient light factor (1.0 = nominal, 0.15 = degraded night)"
    )
    ground_terrain: Literal["concrete", "grass_soil", "dirt_mud", "asphalt_road", "uneven_heightmap", "tiles"] = "concrete"
    surface_friction: float = Field(0.8, ge=0.05, le=1.5, description="Surface friction coefficient (0.8 = standard, 0.15 = slippery ice/mud)")
    sky_weather: SkyWeatherSpec = Field(default_factory=SkyWeatherSpec)

class ObjectSpec(BaseModel):
    id: str = Field(..., description="Unique alphanumeric identifier")
    model_name: str = Field(..., description="Asset identifier (e.g., 'pallet_box', 'storage_rack')")
    dimensions: Vector3D = Field(..., description="Bounding dimensions of the object")
    pose: Pose2D = Field(..., description="2D ground plane pose")
    is_static: bool = Field(True, description="Static obstacle vs dynamically mutable entity")
    semantic_parent_id: Optional[str] = Field(None, description="Anchor ID for grouped objects")

class CameraPerturbation(BaseModel):
    enabled: bool = False
    motion_blur_kernel: int = Field(0, ge=0, le=31, description="Blur kernel size (odd integer)")
    brightness_attenuation: float = Field(1.0, ge=0.0, le=1.0, description="Illumination multiplier")
    gaussian_noise_stddev: float = Field(0.0, ge=0.0, le=0.5, description="Additive noise standard deviation")

class LidarPerturbation(BaseModel):
    enabled: bool = False
    dropout_probability: float = Field(0.0, ge=0.0, le=0.9, description="Random beam dropout rate")
    range_noise_stddev: float = Field(0.0, ge=0.0, le=0.2, description="Gaussian noise standard deviation in meters")

class SensorNoiseSpec(BaseModel):
    camera: CameraPerturbation = Field(default_factory=CameraPerturbation)
    lidar: LidarPerturbation = Field(default_factory=LidarPerturbation)

class DynamicObstacleTrigger(BaseModel):
    trigger_type: Literal["time", "waypoint_distance", "immediate"]
    trigger_value: float = Field(..., description="Seconds from start or distance in meters to target")
    model_name: str = Field("wooden_crate", description="Asset identifier to spawn dynamically")
    spawn_pose: Pose2D = Field(..., description="Spawn coordinate in world reference frame")
    velocity: Optional[Pose2D] = Field(None, description="Linear and angular velocity vector")

class MissionSpec(BaseModel):
    initial_pose: Pose2D = Field(..., description="Robot starting coordinate and heading")
    waypoints: List[Pose2D] = Field(default_factory=list, description="Intermediate navigation waypoints")
    goal_pose: Pose2D = Field(..., description="Final navigation goal pose")
    timeout_seconds: float = Field(120.0, ge=10.0, le=600.0, description="Maximum operational allowance")

class SimSpec(BaseModel):
    spec_version: str = "2.0.0"
    environment: EnvironmentSpec
    objects: List[ObjectSpec] = Field(default_factory=list)
    robot_model: Literal["turtlebot3_waffle", "turtlebot4", "ridgeback"] = "turtlebot3_waffle"
    sensor_noise: SensorNoiseSpec = Field(default_factory=SensorNoiseSpec)
    adversarial_triggers: List[DynamicObstacleTrigger] = Field(default_factory=list)
    mission: MissionSpec
```

---

## 5. End-to-End Testing & Benchmark Validation Suite

Once all 4 PRs are merged, validate the complete system against the **4 Standardized Benchmark Archetypes** defined in Section 8.5 of the architecture:

```
┌─────────────────────────────────────────────────────────────────────────────────────────────────┐
│                             STANDARDIZED BENCHMARK ARCHETYPE TEST SUITE                         │
├─────────────┬──────────────────────────────────────┬────────────────────────┬───────────────────┤
│ Archetype   │ Natural Language Reference Prompt    │ Primary Execution Path │ Verification Goal │
├─────────────┼──────────────────────────────────────┼────────────────────────┼───────────────────┤
│ Archetype 1 │ "Spawn a 14x10 logistics warehouse   │ Path A: Pre-built      │ Instant launch    │
│ (Logistics  │ with 8 pallet racks, dim ambient     │ Catalog Retrieval &    │ (<3s), verified   │
│ Warehouse)  │ lights to 20%, and dispatch AMR to   │ Semantic Anchor        │ anchor pose, zero │
│             │ rack 5."                             │ Dispatch               │ SLAM latency.     │
├─────────────┼──────────────────────────────────────┼────────────────────────┼───────────────────┤
│ Archetype 2 │ "Synthesize a 18x12m robotics lab    │ Path B: Shapely 2.0    │ Corridor bounds   │
│ (Custom     │ with 6 workbenches along the walls,  │ OBB Solver & Zero-SLAM │ W_aisle >= 2.2m,  │
│ Bespoke     │ 2 central carts, and navigate from   │ Ray Slicer (map.yaml,  │ 0 polygon overlap,│
│ Lab)        │ dock to workbench 4."                │ map.pgm)               │ clean Nav2 costmap│
├─────────────┼──────────────────────────────────────┼────────────────────────┼───────────────────┤
│ Archetype 3 │ "Create an open agricultural field   │ Multi-Environment SDF  │ Optical contrast  │
│ (Outdoor    │ of 30x20m with dirt terrain, surface │ Generator: <sky>,      │ attenuation,      │
│ Field with  │ friction 0.5, sun at dusk, and 0.15  │ volumetric <fog>, and  │ waypoint sequence │
│ Fog/Sunset) │ fog density; patrol 4 waypoints."    │ low surface traction   │ traversal.        │
├─────────────┼──────────────────────────────────────┼────────────────────────┼───────────────────┤
│ Archetype 4 │ "While AMR is navigating, drop light │ In-Process FastMCP:    │ Costmap dynamic   │
│ (Adversarial│ to 10%, inject camera blur, and drop │ Live entity spawn &    │ re-inflation,     │
│ Runtime     │ a dynamic wooden crate across the    │ dynamic illumination   │ evasive detour,   │
│ Stress)     │ robot's planned path."               │ without window reload  │ 0 contact forces. │
└─────────────┴──────────────────────────────────────┴────────────────────────┴───────────────────┘
```

### 5.1 Verification Checklist for Archetype Runs
1. **Archetype 1:** Run `lang2sim`. Type prompt. Verify Gazebo GUI opens on host desktop in $<3\text{s}$. TurtleBot3 navigates to `rack_5` $[7.5, 3.0, 0.0]$. Zero SLAM run.
2. **Archetype 2:** Verify custom room bounds are generated. Verify `map.yaml` and `map.pgm` exist in `~/.lang2sim/projects/active_project/`. Nav2 plans collision-free path.
3. **Archetype 3:** Check Gazebo 3D viewport: verify dusk lighting angle and visual fog. In TUI, ask *"Is the path visible?"* Verify vision agent checks `/camera/image_raw` and confirms fog occlusion.
4. **Archetype 4:** Issue mid-run stress command in CLI. Verify cargo crate drops into Gazebo 3D viewport. RViz costmap turns yellow/red. Robot executes local avoidance detour. Terminal renders final NQI benchmark table:

| Evaluation Metric | Nominal Benchmark Run | Adversarial Stress Run | Performance Delta |
| :--- | :--- | :--- | :--- |
| **Mission Completion Status** | Succeeded | Succeeded | Baseline maintained |
| **Mission Duration ($T$)** | **$24.2\text{s}$** | **$38.8\text{s}$** | **$+60.3\%$** (Velocity reduction) |
| **Traversed Path Length ($L$)** | **$12.4\text{m}$** | **$16.2\text{m}$** | **$+30.6\%$** (Avoidance detour) |
| **Path Traversal Efficiency ($E_p$)** | **$0.96$** | **$0.73$** | **$-23.9\%$** |
| **Trajectory Smoothness ($S_\tau$)** | **$14.2$** | **$62.8$** | **$+342\%$** (Steering corrections) |
| **Minimum Obstacle Margin ($C_{\text{min}}$)** | **$0.48\text{m}$** | **$0.18\text{m}$** | Near-miss envelope detected |
| **Physical Collision Count ($C_{\text{force}}$)** | 0 | 0 | Zero structural impact |
| **Navigation Quality Index ($NQI$)** | **94.8 / 100** | **68.2 / 100** | **-28.0% Performance Degradation** |

---

## 6. Project Directory Structure

```
/home/gigikoneti/Lang2Sim/
├── pyproject.toml                  # Packaging & console script: lang2sim = "sim2lang.cli.app:main"
├── README.md                       # Setup and demonstration guide
├── catalog/                        # Grounded Asset Depot (Dev 1)
│   ├── scene_catalog.json          # Semantic anchors, dimensions, verified costmaps
│   ├── maps/                       # Pre-computed map.yaml and map.pgm grids
│   │   ├── aws_small_warehouse.yaml
│   │   └── aws_small_warehouse.pgm
│   └── models/                     # High-fidelity PBR SDF assets (racks, benches, barrels)
└── sim2lang/                       # Core Python Package
    ├── __init__.py                 # Version and global export
    ├── cli/                        # OpenCode-Style Textual TUI Frontend (Dev 2)
    │   ├── app.py                  # Main Textual App (State machine, Event loop)
    │   ├── screens/                # LandingScreen (Screen 1), WorkspaceScreen (Screen 2)
    │   ├── widgets/                # ThoughtBox, TelemetrySidebar, PromptCard, ActionPill
    │   └── styles/app.tcss         # Dark theme (#0c0c0c), cyan borders (#00d2ff)
    ├── core/                       # Canonical Data Contracts & Configuration (Day 0)
    │   ├── models.py               # Pydantic v2 SimSpec, EnvironmentSpec, SIR schemas
    │   └── config.py               # Runtime paths, API endpoints, display configs
    ├── agent/                      # Nemotron 3 Reasoning & Tool Dispatcher (Dev 2)
    │   ├── nemotron_client.py      # OpenAI-compatible API client with streaming
    │   ├── react_loop.py           # Perception-Reasoning-Action supervisor loop
    │   └── tools.py                # Typed function definitions for LLM tool calling
    ├── compiler/                   # 5-Stage Simulation Compiler Pipeline (Dev 1 & Dev 3)
    │   ├── sir_generator.py        # Compiles MissionState.json -> simulation_ir.json (Dev 1)
    │   ├── sir_validator.py        # Geometric feasibility & aisle corridor checks (Dev 1)
    │   ├── sdf_compiler.py         # Multi-target Jinja2 template engine (Harmonic/Classic) (Dev 1)
    │   └── launch_builder.py       # Parametric ROS 2 sim_launch.py synthesizer (Dev 3)
    ├── world_gen/                  # Spatial Placement & Map Synthesis (Dev 1)
    │   ├── shapely_solver.py       # OBB collision avoidance & Minkowski sum inflation
    │   └── map_slicer.py           # Direct 3D geometric slicing -> map.pgm/map.yaml
    ├── runtime/                    # Simulation Process & Container Management (Dev 3)
    │   ├── docker_runner.py        # Spawns Gazebo/RViz2 with X11 pass-through
    │   ├── hardware_probe.py       # VRAM and system memory introspection
    │   ├── nav2_dispatcher.py      # BasicNavigator action client wrapper
    │   ├── self_healing.py         # Stdout/Stderr log monitor & automated recovery
    │   └── mode_b_fallback.py      # RTF health probe & MCAP deterministic replay
    ├── perturbation/               # In-Process FastMCP Dynamic Controls (Dev 4)
    │   ├── mcp_server.py           # In-process FastMCP tool registry
    │   └── gazebo_bridge.py        # Direct gz service & rclpy mutation dispatcher
    └── telemetry/                  # Live Observation, Scoring & Profiling (Dev 4)
        ├── vision.py               # Dual-stream X11 GUI grabber & ROS camera sampler
        ├── ros_subscribers.py      # Background threads for /odom, /tf, /sim2lang/contact
        └── nqi_scorer.py           # Composite Navigation Quality Index evaluator
```
