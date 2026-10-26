# **Sim2Lang: Natural Language to Executable Robot Simulation System Architecture and Implementation Blueprint**

## **1\. Project Overview and Operational Objectives**

The robotics software lifecycle is constrained by the manual overhead of simulation configuration1. Constructing physics-accurate simulation environments within Gazebo requires days of specialized systems engineering1. Roboticists must draft Simulation Description Format (.sdf) models, program kinematic transform trees (![][image1]), configure sensor plugins with representative noise models, generate 2D occupancy navigation grids, author ROS 2 Python launch scripts, and configure Nav2 behavior trees1. When engineers need to validate autonomous mobile robot (AMR) navigation stacks under adverse operational conditions—such as low ambient illuminance, optical lens degradation, LiDAR beam dropouts, or unmapped dynamic obstacles—the engineering burden compounds significantly1.  

Sim2Lang resolves this friction through a **conversational Autonomous Agent with structured Tool Access**. Rather than relying on a static, one-shot LLM code generator, Sim2Lang implements an interactive, OpenCode-style terminal CLI copilot (`sim2lang>`) powered by NVIDIA Nemotron 3 Ultra 550B A55B. The Agent autonomously generates procedurally placed, collision-free Gazebo .sdf worlds; loads pre-built open-source environments on demand; produces zero-SLAM 2D occupancy navigation maps (map.yaml, map.pgm); launches native Gazebo GUI and RViz2 desktop windows; inspects simulation and tool interfaces visually via multimodal screen capture; dynamically parameterizes Unified Robot Description Format (URDF) models with sensor degradation filters; dispatches programmatic Nav2 missions via the Simple Commander API; and evaluates navigation performance via closed-loop telemetry scoring5.

```
User Prompt (Natural Language at sim2lang> CLI)
    │
    ▼
Autonomous Supervisor Agent (NVIDIA Nemotron 3 Ultra 550B A55B ReAct Loop)
    │
    ├── 1. Procedural World & Map Builder (Shapely 2.0 Placement + Zero-SLAM 2D Slicer)
    ├── 2. Native Simulation Runtime (Docker X11 Display Pass-Through -> Gazebo GUI & RViz2)
    ├── 3. Bidirectional Visual Perception (X11 Desktop GUI Screen Capture + /camera/image_raw)
    ├── 4. Adversarial Stress Injector (FastMCP Runtime Model Spawning & Dynamic Lighting)
    ├── 5. Telemetry & Benchmark Engine (Kinematics, Nav2 Feedback, Bumper Contacts, NQI)
    └── 6. Interactive Chat Support Loop (Stays alive in CLI while Gazebo runs in background)
```

The system operates under strict operational constraints: it must execute reliably on standard Linux workstations running Ubuntu with ROS 2 (Humble/Jazzy) and Gazebo (Classic 11/Harmonic) via native host or verified Docker environments with X11 forwarding, require zero compilation freezes or fragile build pipelines, provide an agile hackathon implementation runway, and guarantee presentation reliability during a 3-minute live pitch demonstration5.

## **2\. Forensic Analysis and Adaptation of Key Reference Architectures**

Sim2Lang leverages architectural patterns, algorithms, and interface boundaries extracted from four open-source codebases, adapting their core components to eliminate manual boilerplate.

### **2.1 robotics-upo/gazebo\_world\_generator**

The robotics-upo/gazebo\_world\_generator package establishes a foundation for procedural environment creation and automated map extraction from textual prompts5.  
The spatial placement engine within gazebo\_world\_generator organizes room synthesis through data models defined in data\_models.py, partitioning assets into architectural boundaries, primary static furniture, and secondary grouped objects5. Spatial synthesis operates via constrained hierarchical rejection sampling5. The outer room boundary is established as a 2D bounding box ![][image2]. Primary entities (such as warehouse pallet racks or workbenches) are assigned oriented bounding polygons ![][image3]. A candidate placement pose ![][image4] is accepted if the candidate polygon does not intersect existing objects and maintains a wall standoff distance:  
![][image5]  
Semantic grouping rules anchor dependent objects to parent frames; for example, pallets are placed along regular grid arrays parallel to rack faces, while chairs are anchored facing desk interaction perimeters3. Sim2Lang refactors data\_models.py into a Pydantic v2 validation schema while replacing naive rejection sampling with Shapely 2.0 polygon buffering to enforce guaranteed navigation corridors:  
![][image6]  
where ![][image7] represents the robot circumscribed radius and ![][image8] represents the Nav2 footprint inflation safety margin.  
A major friction point in robot simulation setup is the requirement to teleoperate a robot through Cartographer or slam\_toolbox to generate navigation maps9. The UPO repository eliminates this requirement by interfacing with gazebo\_ros2\_2Dmap\_plugin5. This plugin attaches directly to Gazebo’s physics engine, slicing the 3D collision space across a horizontal volume bounded by ![][image9] (typically calibrated between ![][image10] and ![][image11], representing standard 2D planar LiDAR scanning planes)11. By casting physical rays or calculating geometric collision intersections across a uniform grid resolution (e.g., ![][image12]), the plugin populates a standard nav\_msgs/msg/OccupancyGrid message directly from world geometry11. The accompanying generate\_map.sh script saves this occupancy grid directly as a P5 grayscale Portable Gray Map (map.pgm) and metadata configuration (map.yaml)5. Sim2Lang embeds this functionality into an automated headless post-generation step, producing verified Nav2 costmap assets within 3.5 seconds of SDF compilation.  
To bridge simulator differences, the UPO generator supports both Gazebo Classic (ROS 2 Humble) and modern Gazebo Harmonic (ROS 2 Jazzy)5. Sim2Lang integrates Jinja2 templating to manage syntactic variations between simulators:

* Gazebo Classic targets SDF version 1.6, utilizing legacy ROS plugins such as libgazebo\_ros\_init.so and libgazebo\_ros\_ray\_sensor.so.  
* Gazebo Harmonic targets SDF version 1.9/1.10, utilizing native system plugins including gz::sim::systems::Physics, gz::sim::systems::UserCommands, and gz::sim::systems::SceneBroadcaster, with communication bridged to ROS 2 through ros\_gz\_bridge.

### **2.2 godrift.ai (Drift CLI / Platform)**

The Drift copilot architecture demonstrates techniques for maintaining execution resilience across complex ROS 2 and simulator workspaces1.  
Drift maintains system context by placing an authoritative context file (DRIFT.md) at the workspace root3. This file stores active robot configurations, transform frame trees, primary topic interfaces, and workspace conventions3. Sim2Lang implements this pattern through an automated context manager that outputs both an internal JSON document (.sim2lang.json) and a human-readable SIM2LANG.md. This configuration anchors downstream LLM prompts to the active ROS distribution, RMW implementation, base coordinate frames (map ![][image13] odom ![][image13] base\_footprint ![][image13] base\_link), and sensor topic names3.  
Prior to task execution, Drift evaluates host hardware and environment health to prevent simulator crashes1. Sim2Lang incorporates this introspection by querying system RAM and GPU VRAM via /proc/meminfo and NVML before initialization3. If available VRAM is below ![][image14], the orchestrator forces Gazebo to launch headlessly (-s), preserving graphics resources for WebSockets and web rendering3.  
Simulation failures often stem from missing software packages, mismatched transform frames, or broken topic connections1. Sim2Lang incorporates an automated supervisor loop that captures standard error streams from ROS 2 launch processes and executes deterministic recovery actions.

| Simulation Failure Mode | Diagnostic Error Signature | Automated Self-Healing Action |
| :---- | :---- | :---- |
| **Missing Package Dependency** | Package 'xyz' not found | Invokes rosdep install \--from-paths src \--ignore-src \-r \-y or installs standard binary packages via apt-get install ros-\$ROS\_DISTRO-xyz. |
| **Transform Frame Discontinuity** | Lookup would require extrapolation into the future or frame 'base\_scan' does not exist | Dynamically injects an ephemeral tf2\_ros static\_transform\_publisher node into the launch graph bridging base\_link to the orphaned sensor link. |
| **Unlinked Sensor Topic** | Nav2 Costmap: Timed out waiting for transform from base\_link to \[scan\] | Scans active topics via ros2 topic list; if laser data is published on an alternative namespace (e.g., /lidar/points), spawns a topic\_tools relay. |
| **Unreachable Goal Pose** | Nav2 Planner: ComputePathToPose Action failed: goal in collision | Evaluates the 2D Euclidean distance transform of map.pgm; shifts the target pose outward along the obstacle boundary normal by ![][image15]. |

### **2.3 sandraschi/gazebo-mcp**

The sandraschi/gazebo-mcp repository implements a Model Context Protocol (MCP) server built with FastMCP 3.4, allowing AI agents to inspect and manipulate Gazebo simulations via structured tool invocations16.  
The repository provides 14 operational tools, categorizing functions as either READ\_ONLY or MUTATING to preserve simulation stability16. Sim2Lang extracts five core tool definitions to control runtime environments:

* gz\_sim\_state (READ\_ONLY): Retrieves the active simulation state, including active models, coordinate poses, and physics clock time16.  
* load\_world (MUTATING): Commands the simulation engine to tear down existing entities and load a newly synthesized SDF world16.  
* spawn\_model (MUTATING): Injects an SDF or URDF entity into an active physics world at a designated 6-DoF pose without restarting the simulator process16.  
* apply\_control (MUTATING): Publishes command velocities (geometry\_msgs/msg/Twist) or joint efforts to entities over gz-transport or ROS 2 bridges16.  
* analyze\_sim\_logs (READ\_ONLY): Parses simulation server output buffers for physics integration instability or contact solver divergence16.

Sim2Lang uses these tools to perform dynamic, runtime adversarial testing. While the robot navigates its planned path, the orchestration engine can call spawn\_model to drop a cargo box or barrel obstacle (![][image16]) ![][image17] along the robot's projected trajectory16. It can introduce moving obstacles by spawning a model and repeatedly calling apply\_control to move it across the robot's heading at ![][image18]16. Additionally, calling gz\_service\_call allows the engine to alter ambient lighting parameters dynamically via Gazebo's /world/\<world\_name\>/set\_physics or lighting services, validating how the robot's visual perception stack reacts to adverse lighting18.

### **2.4 acutronicRobotics/gym-gazebo2**

The acutronicRobotics/gym-gazebo2 framework provides a design pattern for wrapping ROS 2 and Gazebo into a synchronous, Gym-compatible benchmarking loop without incurring the latency of simulator restarts14.  
Cold-restarting Gazebo requires 5 to 12 seconds per iteration, which disrupts rapid benchmarking and interactive pitch demonstrations14. gym-gazebo2 achieves sub-second reset cycles by commanding simulation lifecycle services directly over ROS 219:

* Physics clock execution is paused during state  resets by calling /pause\_physics in Gazebo Classic, or calling /world/\<world\_name\>/control with pause: true in Gazebo Harmonic19.  
* The robot and dynamic obstacles are returned to their initial spawn poses using the /set\_entity\_state service in Gazebo Classic, or /world/\<world\_name\>/set\_pose in Gazebo Harmonic, avoiding the need to reload meshes or reconstruct physics trees21.  
* ROS 2 subscriber buffers for odometry, IMU, and laser scans are flushed to clear stale state observations21.  
* Stepping physics deterministically via /unpause\_physics or by executing a specific number of physics iterations ![][image19] (e.g., advancing 100 physics iterations for a ![][image20] solver to step forward by ![][image21]) maintains reproducible test conditions19.

To detect physical collisions accurately, gym-gazebo2 uses hardware contact sensors rather than relying on costmap inflation heuristics14. Sim2Lang automates this by inspecting the base robot URDF and wrapping the chassis collision geometry with a Gazebo contact sensor plugin (libgazebo\_ros\_bumper.so in Classic or gz::sim::systems::Contact in Harmonic)14. This plugin publishes contact state messages to /sim2lang/contact\_states. The telemetry engine monitors this topic; any non-zero contact force vector exceeding ![][image22] between the robot chassis and an environmental obstacle increments the deterministic collision counter ![][image23], recording an objective failure event21.

## **3\. Supplementary Open-Source Packages and Ecosystem Integration**

To accelerate development within a 72-hour window, Sim2Lang integrates established open-source libraries across its simulation, navigation, geometry, and presentation layers.

### **3.1 Simulation and 3D Visualization Infrastructure**

* **Foxglove Bridge (foxglove\_bridge):** High-performance ROS 2 WebSocket server package written in C++. It streams active topics—including transform trees, occupancy grids, raw camera frames, and global paths—directly to browser clients with low CPU overhead15.  
* **Three.js with urdf-loader:** Client-side 3D rendering pipeline running directly in the browser via WebGL. This renders the robot URDF model and environment assets locally, decoupling visual telemetry display from server-side Gazebo rendering resources.  
* **Gazebo Fuel REST API (gz-fuel-tools):** Cloud asset retrieval interface that downloads pre-built 3D models (such as industrial warehouse racks, pallets, and office furniture) directly from the OSRF Fuel repository, eliminating the need to author custom meshes16.

### **3.2 Robotics and Navigation Stack**

* **Nav2 Simple Commander (nav2\_simple\_commander):** Exposes the BasicNavigator Python interface, providing high-level task control methods such as setInitialPose(), goToPose(), followWaypoints(), cancelTask(), and getFeedback()4. This removes the boilerplate of managing low-level rclpy action clients for NavigateToPose6.  
* **TurtleBot3 / TurtleBot4 Simulation Repositories (turtlebot3\_gazebo / turtlebot4\_simulator):** Verified differential drive and holonomic ground robot models with pre-tuned Nav2 parameters and sensor configurations, providing a stable baseline for navigation benchmarks.  
* **AWS RoboMaker Small Warehouse World (aws-robomaker-small-warehouse-world):** A collection of warehouse assets, textures, and physical layouts that serve as standard visual and collision primitives for the procedural generator.

### **3.3 Geometry and Map Utilities**

* **Shapely 2.0:** GEOS-backed computational geometry engine used to perform 2D polygon intersection tests, convex hull calculations, and Minkowski sum footprint inflations at sub-millisecond speeds.  
* **SDFormat Python Bindings (sdformat13 / sdformat14):** Provides XML schema validation, verifying that procedurally generated SDF strings conform to official Gazebo schemas before writing them to disk.  
* **OpenCV (cv2) and NumPy:** Provides rasterization utilities, converting Shapely polygon layouts directly into 2D NumPy occupancy arrays to generate fallback .pgm and .yaml navigation maps if Gazebo map-slicing plugins are unavailable.

### **3.4 Interface & Presentation Architecture: OpenCode-Style Textual TUI (`lang2sim`)**

Rather than relying on browser-based dashboards that introduce rendering lag and decouple the engineer from the terminal, Sim2Lang adopts a **full-screen Terminal User Interface (TUI)** built with Python's **`Textual`** framework (the Python counterpart to Go's Charmbracelet `Bubble Tea` / `Lip Gloss` stack used in OpenCode).

```
╔═══════════════════════════════════════════════════════════════════════════════╗
║                      LANG2SIM TUI FRONTEND ARCHITECTURE                       ║
╠═══════════════════════════════════════════════════════════════════════════════╣
║ 1. TUI Framework: Textual (Pure Python, CSS-styled, asynchronous)            ║
║    • Equivalent to Go's Bubble Tea + Lip Gloss, running natively in ROS 2.    ║
║    • Zero IPC overhead: directly connects to ROS 2 topics & Nemotron API.     ║
║                                                                               ║
║ 2. Screen 1: Initial Landing View (Single-Command Entry: `lang2sim`)          ║
║    • Centered ASCII/block logo: `lang2sim`                                    ║
║    • Floating Input Card (#161616 background, `border-left: wide #00d2ff`)     ║
║    • Badge pill: `Build · Nemotron 3 Ultra 550B A55B Nvidia · medium`         ║
║    • Keybind hints: `tab agents   ctrl+p commands`                            ║
║    • Tip banner with history navigation shortcuts (pgup / pgdn)               ║
║                                                                               ║
║ 3. Screen 2: Dynamic 80/20 Split-Screen Workspace (Active Simulation Mode)   ║
║    • Left Pane (80% Width):                                                   ║
║        - User prompt with vertical cyan accent bar.                           ║
║        - Collapsible reasoning block (`+ Thought: 1.0s` in orange).           ║
║        - Streaming markdown responses from Nemotron 3.                        ║
║        - Agent tool execution pills (`🗄️ Build · Nemotron · 19.2s`).          ║
║        - Sticky bottom input card with persistent focus.                      ║
║    • Right Sidebar Pane (20% Width):                                          ║
║        - Session Topic & Token Context (tokens used, % capacity, cost).       ║
║        - Gazebo Simulation Status (GUI Active, Sim Time, Real-Time Factor).   ║
║        - Robot Telemetry (Position x/y, linear speed, obstacle clearance).    ║
║        - Bumper Collisions counter (contact sensor events).                   ║
║        - Vision Perception Status (/camera/image_raw FPS, GUI Grounding).     ║
║                                                                               ║
║ 4. Global Single-Command Launch:                                              ║
║    • Executable from any directory via `gigikoneti@Nimbus2026:~$ lang2sim`    ║
║    • Packaged via pyproject.toml console_scripts: `lang2sim = sim2lang.cli:run`║
╚═══════════════════════════════════════════════════════════════════════════════╝
```

| TUI Component | OpenCode Architecture (Go) | `lang2sim` Architecture (Python `Textual`) | Technical Capability |
| :---- | :---- | :---- | :---- |
| **Application Runtime** | `charmbracelet/bubbletea` (Go) | `textual.app.App` (Python) | Full-screen terminal raw mode, reactive state management, asynchronous event loop. |
| **Styling & Colors** | `charmbracelet/lipgloss` (Go) | `Textual.css` + `Rich` (Python) | Hex colors (`#0c0c0c` pitch black, cyan `#00d2ff`), borders, margins, padding. |
| **Input & Viewports** | `bubbles/textinput` & `viewport` | `textual.widgets.Input` & `VerticalScroll` | Non-blocking input card, smooth mouse and keyboard scroll buffers. |
| **Markdown Rendering** | `glamour` (Go) | `rich.markdown.Markdown` | Syntax-highlighted code blocks, tables, and streaming token rendering. |
| **Command Palette** | Custom modal | `textual.command.CommandPalette` | Native `Ctrl+P` modal for fuzzy-searching commands (`/reset`, `/light`, `/telemetry`). |
| **Middleware Bridge** | Bun / Hono HTTP server | Native Python ROS 2 / FastMCP | Direct in-memory access to ROS 2 topics and Nemotron client with zero IPC latency. |

### **3.5 Autonomous Agent Architecture and Tool Registry**

Static single-pass LLM compilation pipelines fail when real-world simulation runtime errors arise (e.g. unlinked topics, transform frame discontinuities, goal poses in collision, or paused physics clocks). Sim2Lang establishes an **Autonomous Agent Architecture with Tool Access** powered by a continuous Perception-Reasoning-Action (ReAct) execution loop.

The Agent operates over a typed, deterministic tool registry:
* **Visual Perception & GUI Tools:**
  * `capture_gazebo_window()`: Grabs a screenshot of the native Gazebo desktop window.
  * `capture_rviz_window()`: Captures the active RViz2 window (Displays panel, 3D viewport, costmaps).
  * `read_robot_camera(topic)`: Samples the active on-board RGB camera stream (`/camera/image_raw`).
  * `click_gui_button(window, button)`: Programmatically clicks toolbar buttons (e.g., Gazebo Play/Pause, RViz 2D Nav Goal).
* **Procedural Synthesis & World Tools:**
  * `solve_spatial_layout(dimensions, objects)`: Runs Shapely 2.0 to calculate collision-free bounding boxes and corridor clearances.
  * `compile_world_sdf(layout_spec)`: Compiles the validated layout into valid Gazebo Classic / Harmonic SDF.
  * `generate_zero_slam_map(sdf_path)`: Slices world collision geometry into `map.yaml` and `map.pgm`.
  * `launch_simulation_stack(world_path, robot_type)`: Dispatches Gazebo and Nav2 processes.
* **Runtime Control & FastMCP Mutation Tools:**
  * `spawn_dynamic_entity(model_name, pose)`: Drops obstacles into the running simulation via FastMCP.
  * `set_ambient_lighting(luminance)`: Adjusts ambient light levels dynamically for stress testing.
  * `reset_simulation(mode)`: Calls Gazebo `/reset_world` service for sub-second resets without closing GUI windows.
* **Introspection & Self-Healing Tools:**
  * `inspect_topic_health(topic_name)`: Verifies topic publication rates and message validity on `/scan`, `/odom`, `/cmd_vel`.
  * `query_robot_telemetry()`: Reads instantaneous speed, distance to goal, and contact bumper collision forces.
  * `auto_repair_tf(parent_frame, child_frame)`: Injects ephemeral static transform publishers if frames disconnect.

### **3.6 Bidirectional Visual Perception & GUI Understanding**

To endow the AI with complete visual knowledge of what is happening in the graphical simulation tools, Sim2Lang integrates a **Dual-Stream Visual Observer**:
1. **Robot Perspective Stream (`/camera/image_raw`):** Captures the forward-facing optical frame of the mobile robot, enabling perception diagnostics (e.g., lens blur, occlusions, target identification).
2. **Desktop Window Stream (Gazebo GUI & RViz2 Screen Grabbing):** Utilizes X11 window capture to snapshot the actual desktop GUI windows.
   * **In the Gazebo GUI:** Visually detects Play/Pause button states, simulation time, real-time factor (RTF), active models in the left-hand scene tree, and 3D visual lighting.
   * **In the RViz2 GUI:** Visually parses the Displays sidebar to verify if `RobotModel`, `LaserScan`, and `Costmaps` are active; verifies top toolbar buttons (e.g., `2D Goal Pose`); and visually confirms global path trajectories.

Snapshots are encoded and passed directly to the multimodal visual reasoning engine, allowing the AI to answer visual queries (*"Why did the robot stop?"*, *"Is the laser scan visible in RViz?"*, *"Is physics paused?"*) with grounded visual proof.

### **3.7 Core Reasoning Engine: NVIDIA Nemotron 3 Ultra 550B A55B API Integration**

Sim2Lang leverages the **NVIDIA Nemotron 3 Ultra 550B A55B** model via an OpenAI-compatible API endpoint as the primary intelligence driver:
* **Structured Output Compilation:** Enforces deterministic Pydantic v2 validation for initial `SimSpec` declarations.
* **Autonomous Tool Selection:** Employs native function calling to evaluate when to trigger spatial solvers, launch processes, inject perturbations, or query telemetry.
* **Multimodal Visual Reasoning:** Processes JPEG window captures of Gazebo and RViz2 to interpret GUI states, scene geometry, and visual navigation hazards.
* **Offline Heuristic Fallback:** Maintains a local rule-based heuristic parser ensuring basic pipeline execution even if network connectivity is interrupted during pitch presentations.

### **3.8 Dockerized Simulation Runtime with X11 Display Pass-Through**

To eliminate host compilation failures and ROS 2 distribution conflicts on developer workstations, Sim2Lang supports a containerized simulation engine (leveraging verified images such as `sih26050_simulation` with ROS 2 Jazzy and Gazebo Harmonic, or `docker-project_zero_sim:latest` / `pipegen:humble-sitl` with ROS 2 Humble):

#### **1. Host Display Server Authorization**
Before launching the simulation container, the host display server must grant access to the containerized X11 clients. Sim2Lang automates this in its pre-flight check:
```bash
# Allow local container connections to the X11 server
xhost +local:root > /dev/null 2>&1
```

#### **2. Battle-Tested Docker Launch Command Matrix**
To guarantee native rendering without OpenGL context failures across varying host hardware, the container runner chooses the appropriate execution template:

| Hardware Environment | Docker Acceleration Flags | Environment Variables & Devices | Rendering Pipeline |
| :--- | :--- | :--- | :--- |
| **NVIDIA Dedicated GPU** | `--gpus all --runtime=nvidia` | `-e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all` | Full NVML + OpenGL hardware acceleration (60+ FPS). |
| **Intel / AMD Integrated** | `--device /dev/dri:/dev/dri` | `-e LIBGL_ALWAYS_INDIRECT=0` | Direct Rendering Infrastructure (DRI) Mesa hardware acceleration. |
| **Headless / VM Fallback** | N/A (CPU only) | `-e LIBGL_ALWAYS_SOFTWARE=1 -e QT_X11_NO_MITSHM=1` | LLVMpipe software rasterizer (safest fail-safe fallback). |

#### **3. Standard Container Execution Command**
```bash
docker run -it --rm \
  --net=host \
  --ipc=host \
  --gpus all \
  -e DISPLAY=${DISPLAY:-:0} \
  -e XAUTHORITY=/tmp/.docker.xauth \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v /tmp/.docker.xauth:/tmp/.docker.xauth:rw \
  -v ~/.lang2sim/projects:/root/.lang2sim/projects:rw \
  -v $(pwd)/catalog:/root/catalog:ro \
  sih26050_simulation:latest \
  ros2 launch /root/.lang2sim/projects/active_project/sim_launch.py
```
This isolates the heavy ROS 2 Gazebo and RViz2 desktop GUI runtime inside the pre-compiled container while projecting native GUI windows seamlessly onto the host desktop alongside the host-resident `lang2sim` TUI.


### **3.9 Hybrid Environment Strategy: Curated Scene Catalog & Catalog-Grounded Procedural Synthesis**

Sim2Lang resolves the fundamental tension between simulation visual realism, physics stability, and generative flexibility by combining a **Curated Scene Catalog** with a **Catalog-Grounded Procedural Synthesizer**:

```
[User Prompt: e.g., "Warehouse with 8 racks, dim lights, navigate to rack 3"]
                               │
               ┌───────────────┴───────────────┐
               ▼                               ▼
    [Standard Archetype Match]       [Custom Bespoke Geometry]
    Path A: Retrieve & Mutate         Path B: Catalog-Grounded Synthesis
    • Pulls `aws_small_warehouse`     • Pulls component blueprints (dimensions,
    • Loads verified base SDF & map     friction, PBR meshes) from catalog
    • Reads semantic anchor: rack_3   • Shapely 2.0 solves OBB placement
    • Injects dynamic delta (lights)  • Enforces catalog aisle buffers (>2.2m)
    • Zero map slicing latency (<1s)  • Emits custom world.sdf & zero-SLAM map
               │                               │
               └───────────────┬───────────────┘
                               ▼
    [Execution in Gazebo: 100% Nav2 Reliability • AAA Mesh Quality • No Physics Jitter]
```

#### **1. Curated Scene Catalog with Semantic Metadata (`scene_catalog.json`)**
Pre-built environments (e.g., AWS RoboMaker Small Warehouse, robotics laboratory offices, retail stores, obstacle arenas) are indexed with rich semantic metadata, allowing the Agent to ground its actions in physical reality:
* **Semantic Anchor Coordinates:** Known collision-free poses for high-level targets (e.g., `dock: [0, 0, 0]`, `rack_1: [2.5, 3.0, 0]`, `rack_3: [7.5, 3.0, 0]`, `packaging_bench: [-3.0, 2.0, 0]`). When a user says *"Navigate to rack 3"*, the Agent dispatches directly to mathematically verified coordinates, eliminating *"Goal in collision"* Nav2 aborts.
* **Verified Nav2 Costmaps:** Pairs each scene with pre-verified `map.yaml` and `map.pgm` grids, enabling instant sub-second simulation start without SLAM or procedural slicing latency.
* **Pre-Tuned Physics & Lighting:** Ensures surface friction ($\mu = 0.8$), inertial tensors, and ambient lighting are physically balanced, preventing simulator jitter or floating assets.

#### **2. Catalog-Grounded Procedural Synthesis (Shapely 2.0 Solver)**
When the user demands custom architectural geometry (*"Make a 18x12m warehouse with 10 racks arranged in two rows and a packaging area"*), the procedural engine does **not hallucinate blind geometry into a vacuum**:
* **Component Blueprints from Catalog:** The solver queries the catalog for verified physical specifications (e.g., standard industrial pallet rack: $2.8\text{m} \times 1.2\text{m} \times 3.6\text{m}$, mass $120\text{kg}$, PBR mesh URI `model://aws_warehouse_rack`). The LLM never invents arbitrary or distorted object bounds.
* **Domain & Safety Corridor Rules:** The catalog supplies domain rules (e.g., minimum warehouse aisle width $W_{\text{aisle}} \ge 2.2\text{m}$, wall standoff distance $\ge 0.5\text{m}$).
* **Constrained Geometric Arrangement:** Shapely 2.0 places the outer room polygon and solves 2D Oriented Bounding Box (OBB) packing for the catalog assets, guaranteeing non-intersecting polygons and collision-free navigation corridors ($W_{\text{corridor}} > 2 \cdot r_{\text{robot}}$).
* **High-Fidelity SDF Emission:** The resulting `.sdf` world is populated with real textured meshes from the catalog asset depot, ensuring custom worlds retain commercial visual quality rather than looking like primitive grey boxes.

### **3.10 Live GUI Mutation Plumbing & Multi-Environment Synthesis**

#### **1. FastMCP Runtime Control Plumbing (Zero-Restart Live Mutation)**

To allow the CLI to modify the running simulation without closing or reloading the native Gazebo desktop window, Sim2Lang implements an **In-Process FastMCP Server & Bridge**:

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                       IN-PROCESS FASTMCP BRIDGE TOPOLOGY                    │
├─────────────────────────────────────────────────────────────────────────────┤
│  Autonomous ReAct Agent (NVIDIA Nemotron 3 Ultra 550B Function Calling)     │
│       │                                                                     │
│       ▼ (Direct Python In-Memory Method Invocation - 0ms IPC Latency)       │
│  FastMCP Tool Registry (`@mcp.tool()`)                                      │
│       │                                                                     │
│       ├── spawn_dynamic_entity(model_name, pose)                            │
│       ├── set_ambient_lighting(luminance)                                   │
│       ├── reset_simulation(mode)                                            │
│       └── apply_chassis_perturbation(velocity_bias)                         │
│       │                                                                     │
│       ▼                                                                     │
│  GazeboBridge Dispatcher (Auto-detects Simulator Target)                    │
│       │                                                                     │
│       ├── [Gazebo Harmonic / GZ Sim] ──> gz.transport / `gz service` IPC   │
│       └── [Gazebo Classic / ROS 2]   ──> rclpy Service Clients / Topics     │
└─────────────────────────────────────────────────────────────────────────────┘
```

* **Why In-Process FastMCP?** Rather than running FastMCP as a decoupled child process communicating over stdio or HTTP (which introduces connection drop risks, socket timeouts, and process synchronization bugs during live pitch demos), Sim2Lang embeds the FastMCP tool registry directly inside the application's memory space. The Agent invokes tools synchronously or asynchronously within the same `asyncio` event loop.
* **Dynamic Entity Ingestion:** When the agent calls `spawn_dynamic_entity("wooden_crate", pose)`, the FastMCP server invokes:
  * *Gazebo Harmonic (GZ Sim):* `gz service -s /world/<name>/create --reqtype gz.msgs.EntityFactory --req "sdf_filename: 'model://wooden_crate', pose: {position: {x: 3.5, y: 2.0, z: 0.5}}"`
  * *Gazebo Classic (ROS 2):* `ros2 service call /spawn_entity gazebo_msgs/srv/SpawnEntity "{name: 'wooden_crate', xml: '...', initial_pose: {...}}"`
  The physics engine dynamically inserts the entity into the scene graph, and the Gazebo GUI instantly renders the new model dropping into the robot's heading.
* **Dynamic Lighting & Weather Adjustment:** Calling `set_ambient_lighting(0.15)` sends a lightweight `gz.msgs.Light` payload to `/world/<name>/set_physics` or `/set_light_properties`, immediately dimming the 3D GUI viewport.
* **Sub-Second World Reset:** Calling `reset_simulation()` invokes `/world/<name>/control` (`reset: {all: true}`) or ROS 2 `/reset_world`, snapping the robot back to the starting pose in $<0.3\text{s}$ without restarting process daemons.


#### **2. Multi-Environment Synthesis (Warehouses, Fields, Ground Terrains & Skies)**
Sim2Lang extends beyond indoor warehouses to synthesize arbitrary open-world operating environments:
* **Atmospheric Sky & Clouds:** Configures Gazebo's `<scene>` and `<sky>` blocks with procedural cloud speed, daylight angles, and night/dusk illumination.
* **Volumetric Fog & Weather Stress:** Injects the `<fog>` element (`linear` or `exponential` with adjustable density and color) to challenge the robot's optical perception with heavy mist or dust storms.
* **Diverse Ground Terrains & Friction:**
  * *Logistics Warehouse:* Polished concrete with standard traction ($\mu = 0.8$).
  * *Agricultural Field:* Grass or tilled soil textures (`Gazebo/Grass`, `Gazebo/Dirt`) with mild slip resistance ($\mu = 0.5$).
  * *Off-Road / Rough Terrain:* Uneven DEM elevation grids via the `<heightmap>` tag for rovers and quadrupeds.
  * *Slippery Surface:* Low-friction surfaces ($\mu = 0.15$) simulating wet pavement or icy floors for traction loss evaluation.

## **4\. Modular System Architecture and Data Contracts**

### **4.1 Component Flow and Pipeline Topology**

The Sim2Lang architecture unifies natural language parsing, agentic tool execution, physics simulation, and real-time visual inspection into a cohesive topology:

```
+─────────────────────────────────────────────────────────────────────────────+
|               OpenCode-Style Interactive Terminal CLI (sim2lang>)           |
|         Captures natural language declarations & multi-turn instructions    |
+──────────────────────────────────────┬──────────────────────────────────────+
                                       │
                                       ▼
+─────────────────────────────────────────────────────────────────────────────+
|            Autonomous Supervisor Agent (NVIDIA Nemotron 3 Ultra 550B)       |
|      Maintains execution loop: Think -> Tool Call -> Observe -> Re-plan     |
+──────────────────┬───────────────────┬───────────────────┬──────────────────+
                   │                   │                   │
                   ▼                   ▼                   ▼
+───────────────────────+ +─────────────────────────+ +───────────────────────+
| Procedural & Map Tools| | Visual Perception Tools | | FastMCP Mutation Tools|
| • Shapely 2.0 solver  | | • X11 Gazebo/RViz grab  | | • spawn_dynamic_entity|
| • SDF world compiler  | | • /camera/image_raw feed| | • set_ambient_lighting|
| • Zero-SLAM 2D slicer | | • GUI button clicker    | | • reset_simulation    |
+──────────────────┬────+ +────────────┬────────────+ +───────────┬───────────+
                   │                   │                          │
                   └───────────────────┼──────────────────────────┘
                                       │
                                       ▼
+─────────────────────────────────────────────────────────────────────────────+
|               Simulation Execution Engine (Docker / Host ROS 2)             |
|   Native Gazebo GUI & RViz2 Desktop Windows + TurtleBot3/4 + Nav2 Stack     |
+──────────────────────────────────────┬──────────────────────────────────────+
                                       │
                                       ▼
+─────────────────────────────────────────────────────────────────────────────+
|                    Telemetry & Scoring Engine (NQI Index)                   |
|   Kinematics (/odom), Nav2 Feedback, Bumper Contacts (/sim2lang/contact)    |
+─────────────────────────────────────────────────────────────────────────────+
```

| Pipeline Subsystem | Primary Functional Responsibilities | Consumed Inputs | Emitted Outputs |
| :---- | :---- | :---- | :---- |
| **Interactive Terminal CLI** | Standalone OpenCode-style prompt interface (`sim2lang>`) with Rich streaming and active command loop. | Human-authored prompt strings and follow-up commands. | Formatted terminal output and agent action logs. |
| **Supervisor Agent** | Drives multi-turn reasoning and tool invocation via NVIDIA Nemotron 3 Ultra 550B. | Natural language prompts, tool execution observations, visual snapshots. | Structured tool invocation calls and conversational responses. |
| **Procedural World Builder** | Validates layout geometry via Shapely 2.0, compiles SDF worlds, and exports zero-SLAM maps. | SimSpec environment/object specifications. | Validated world.sdf, map.pgm, and map.yaml. |
| **Simulation Runtime** | Launches containerized/host Gazebo GUI and RViz2 desktop windows via X11 pass-through. | Compiled SDF, robot URDF, Nav2 configuration files. | Native Gazebo/RViz windows and active ROS 2 node graph. |
| **Visual Observer** | Captures desktop window frames (Gazebo & RViz) and on-board robot camera frames. | X11 desktop display buffer and `/camera/image_raw` ROS 2 topic. | Base64 visual frames passed to Nemotron Multimodal Vision API. |
| **FastMCP Stress Injector** | Coordinates runtime perturbations, dynamic obstacle injection, and lighting attenuation. | Follow-up stress prompts, Nav2 real-time trajectory state. | Dynamic entity spawns and runtime lighting modifications. |
| **Telemetry Engine** | Gathers kinematics, evaluates mission timelines, and monitors contact bumper sensors. | ROS 2 topics: `/odom`, `/tf`, `/sim2lang/contact_states`. | Live performance metrics, comparative NQI benchmark scores. |

### **4.2 The Simulation Compiler Pipeline (Inspired by CraftifAI PipeGen)**

To prevent the common pitfall of AI hallucinations and broken XML/Python configurations, Sim2Lang adopts the compiler philosophy pioneered by **PipeGen** in the CraftifAI Orbit ecosystem. Rather than allowing an LLM to generate unstructured ROS 2 code or Gazebo world files directly, Sim2Lang structures all synthesis around a **deterministic 5-stage sequential compiler pipeline** mediated by a formal **Simulation Intermediate Representation (SIR)**:

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                   5-STAGE COMPILER PIPELINE (PIPEGEN MODEL)                 │
├─────────────────────────────────────────────────────────────────────────────┤
│  [Stage 1: Intent & Mission Gathering]                                      │
│  • Conversational Agent (NVIDIA Nemotron 3 Ultra 550B)                      │
│  • Classifies boundaries, asset counts, robot platform, and mission goals   │
│  • Outputs: `MissionState.json`                                             │
│                                                                             │
│  [Stage 2: Simulation Intermediate Representation (SIR) Synthesis]          │
│  • SIRGenerator: Compiles requirements into a canonical topological graph   │
│  • SIRValidator: Enforces geometric feasibility, corridor rules, and bounds │
│  • Outputs: `simulation_ir.json`                                            │
│                                                                             │
│  [Stage 3: Environment & Zero-SLAM Map Compilation]                         │
│  • Spatial Placement Solver (Shapely 2.0): Calculates collision-free bounds │
│  • Zero-SLAM Slicer: Casts rays across world geometry -> map.yaml & map.pgm │
│  • Multi-Target SDF Compiler: Synthesizes valid Classic / Harmonic world    │
│  • Outputs: `world.sdf`, `map.yaml`, `map.pgm`                              │
│                                                                             │
│  [Stage 4: Robot Behavior & ROS 2 Launch Synthesis]                         │
│  • Parametric Launch Compiler: Assembles ROS 2 Python launch scripts        │
│  • URDF Sensor Noise Injector: Configures camera blur & LiDAR dropouts      │
│  • Nav2 Mission Dispatcher: Synthesizes Simple Commander action goals       │
│  • Outputs: `sim_launch.py`, `robot_configured.urdf`                        │
│                                                                             │
│  [Stage 5: Runtime Execution, FastMCP Mutation & Benchmark Evaluator]       │
│  • Native Desktop Launch: Spawns Gazebo GUI & RViz2 via Docker X11 bridge   │
│  • FastMCP Runtime Control: Injects dynamic crates & dims lights on-the-fly │
│  • Dual-Stream Visual Observer: Grabs GUI window & on-board camera frames   │
│  • Telemetry Profiler: Logs kinematics, bumper collisions, and NQI score    │
│  • Outputs: `benchmark_report.json`                                         │
└─────────────────────────────────────────────────────────────────────────────┘
```

#### **Core Architectural Principles Adapted from PipeGen:**
1. **The Intermediate Representation (SIR) Principle:**
   - Decouples high-level natural language intent from low-level, brittle simulator syntax (SDF XML, URDF xacro, Nav2 YAML).
   - The deterministic `SIRValidator` guarantees that obstacle clearances ($> 2 \cdot r_{\text{robot}}$), coordinate bounds, and transform frames are verified **before** generating files or invoking Gazebo.
2. **Project Isolation & Workspace Storage:**
   - Every compilation run is isolated in a clean project directory matching the PipeGen convention:
     ```
     ~/.lang2sim/projects/<project_name>/
     ├── MissionState.json          # Stage 1: Extracted user requirements
     ├── simulation_ir.json         # Stage 2: Validated canonical IR
     ├── world.sdf                  # Stage 3: Compiled Gazebo world
     ├── map.yaml & map.pgm         # Stage 3: Sliced zero-SLAM occupancy grid
     ├── sim_launch.py              # Stage 4: Synthesized ROS 2 launch file
     ├── robot_configured.urdf      # Stage 4: URDF with sensor noise plugins
     ├── logs/                      # Live ROS 2 & Gazebo execution logs
     └── benchmark_report.json      # Stage 5: Quantitative NQI evaluation report
     ```
3. **Asynchronous Worker Engine:**
   - Following PipeGen's in-process worker model (`QThread`), Sim2Lang executes Stages 1 through 5 inside non-blocking `asyncio` worker tasks. The interactive Textual TUI (`lang2sim`) remains silky smooth at 60 FPS while background workers compile worlds, slice maps, and monitor Gazebo.
4. **Automated Benchmarking & Reporting (`benchmark_report.json`):**
   - Equivalent to PipeGen's `model_report.json` (which compares ONNX vs TensorRT), Sim2Lang outputs a comprehensive benchmark report measuring path efficiency ($E_p$), acceleration jerk ($S_\tau$), obstacle clearance margin ($C_{min}$), and contact collision events across nominal and degraded test runs.
5. **Concrete Repository Source Tree & Entrypoint Specification:**
   - The developer workspace is structured as an installable Python package providing the global `lang2sim` CLI command:
     ```
     /home/gigikoneti/Lang2Sim/
     ├── pyproject.toml                  # Packaging & console script: lang2sim = "sim2lang.cli.app:main"
     ├── README.md                       # Setup and demonstration guide
     ├── catalog/                        # Grounded Asset Depot
     │   ├── scene_catalog.json          # Semantic anchors, dimensions, verified costmaps
     │   ├── maps/                       # Pre-computed map.yaml and map.pgm grids
     │   │   ├── aws_small_warehouse.yaml
     │   │   └── aws_small_warehouse.pgm
     │   └── models/                     # High-fidelity PBR SDF assets (racks, benches, barrels)
     └── sim2lang/                       # Core Python Package
         ├── __init__.py                 # Version and global export
         ├── cli/                        # OpenCode-Style Textual TUI Frontend
         │   ├── app.py                  # Main Textual App (State machine, Event loop)
         │   ├── screens/                # LandingScreen (Screen 1), WorkspaceScreen (Screen 2)
         │   ├── widgets/                # ThoughtBox, TelemetrySidebar, PromptCard, ActionPill
         │   └── styles/app.tcss         # Dark theme (#0c0c0c), cyan borders (#00d2ff)
         ├── core/                       # Canonical Data Contracts & Configuration
         │   ├── models.py               # Pydantic v2 SimSpec, EnvironmentSpec, SIR schemas
         │   └── config.py               # Runtime paths, API endpoints, display configs
         ├── agent/                      # Nemotron 3 Reasoning & Tool Dispatcher
         │   ├── nemotron_client.py      # OpenAI-compatible API client with streaming
         │   ├── react_loop.py           # Perception-Reasoning-Action supervisor loop
         │   └── tools.py                # Typed function definitions for LLM tool calling
         ├── compiler/                   # 5-Stage Simulation Compiler Pipeline
         │   ├── sir_generator.py        # Compiles MissionState.json -> simulation_ir.json
         │   ├── sir_validator.py        # Geometric feasibility & aisle corridor checks
         │   ├── sdf_compiler.py         # Multi-target Jinja2 template engine (Harmonic/Classic)
         │   └── launch_builder.py       # Parametric ROS 2 sim_launch.py synthesizer
         ├── world_gen/                  # Spatial Placement & Map Synthesis
         │   ├── shapely_solver.py       # OBB collision avoidance & Minkowski sum inflation
         │   └── map_slicer.py           # Direct 3D geometric slicing -> map.pgm/map.yaml
         ├── runtime/                    # Simulation Process & Container Management
         │   ├── docker_runner.py        # Spawns Gazebo/RViz2 with X11 pass-through
         │   └── self_healing.py         # Stdout/Stderr log monitor & automated recovery
         ├── perturbation/               # In-Process FastMCP Dynamic Controls
         │   ├── mcp_server.py           # In-process FastMCP tool registry
         │   └── gazebo_bridge.py        # Direct gz service & rclpy mutation dispatcher
         └── telemetry/                  # Live Observation, Scoring & Profiling
             ├── vision.py               # Dual-stream X11 GUI grabber & ROS camera sampler
             ├── ros_subscribers.py      # Background threads for /odom, /tf, /sim2lang/contact
             └── nqi_scorer.py           # Composite Navigation Quality Index evaluator
     ```
   - **`pyproject.toml` Configuration:**
     ```toml
     [build-system]
     requires = ["setuptools>=61.0"]
     build-backend = "setuptools.build_meta"

     [project]
     name = "lang2sim"
     version = "2.0.0"
     description = "Natural Language to Executable Robot Simulation with Dynamic Stress Testing"
     readme = "README.md"
     requires-python = ">=3.10"
     dependencies = [
         "textual>=0.70.0",
         "rich>=13.7.0",
         "pydantic>=2.6.0",
         "openai>=1.14.0",
         "shapely>=2.0.0",
         "numpy>=1.24.0",
         "pillow>=10.0.0",
         "jinja2>=3.1.0",
         "fastmcp>=0.1.0"
     ]

     [project.scripts]
     lang2sim = "sim2lang.cli.app:main"
     ```


### **4.3 Intermediate Specification Schema: SimSpec / SimulationIR**

The interface between natural language parsing and downstream robotics execution is governed by a Pydantic v2 schema (SimSpec), enforcing typed validation, boundary limits, and physical constraints across the configuration pipeline.

Python  
from pydantic import BaseModel, Field  
from typing import List, Optional, Literal

class Pose2D(BaseModel):  
    x: float \= Field(..., description="X coordinate in meters relative to world origin")  
    y: float \= Field(..., description="Y coordinate in meters relative to world origin")  
    yaw: float \= Field(0.0, ge=-3.14159, le=3.14159, description="Yaw orientation in radians")

class Vector3D(BaseModel):  
    x: float \= Field(..., description="Size X in meters")  
    y: float \= Field(..., description="Size Y in meters")  
    z: float \= Field(..., description="Size Z in meters")

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
    id: str \= Field(..., description="Unique alphanumeric identifier")  
    model\_name: str \= Field(..., description="Asset identifier (e.g., 'pallet\_box', 'storage\_rack')")  
    dimensions: Vector3D \= Field(..., description="Bounding dimensions of the object")  
    pose: Pose2D \= Field(..., description="2D ground plane pose")  
    is\_static: bool \= Field(True, description="Static obstacle vs dynamically mutable entity")  
    semantic\_parent\_id: Optional\[str\] \= Field(None, description="Anchor ID for grouped objects")

class CameraPerturbation(BaseModel):  
    enabled: bool \= False  
    motion\_blur\_kernel: int \= Field(0, ge=0, le=31, description="Blur kernel size (odd integer)")  
    brightness\_attenuation: float \= Field(1.0, ge=0.0, le=1.0, description="Illumination multiplier")  
    gaussian\_noise\_stddev: float \= Field(0.0, ge=0.0, le=0.5, description="Additive noise standard deviation")

class LidarPerturbation(BaseModel):  
    enabled: bool \= False  
    dropout\_probability: float \= Field(0.0, ge=0.0, le=0.9, description="Random beam dropout rate")  
    range\_noise\_stddev: float \= Field(0.0, ge=0.0, le=0.2, description="Gaussian noise standard deviation in meters")

class SensorNoiseSpec(BaseModel):  
    camera: CameraPerturbation \= Field(default\_factory=CameraPerturbation)  
    lidar: LidarPerturbation \= Field(default\_factory=LidarPerturbation)

class DynamicObstacleTrigger(BaseModel):  
    trigger\_type: Literal\["time", "waypoint\_distance", "immediate"\]  
    trigger\_value: float \= Field(..., description="Seconds from start or distance in meters to target")  
    model\_name: str \= Field("wooden\_crate", description="Asset identifier to spawn dynamically")  
    spawn\_pose: Pose2D \= Field(..., description="Spawn coordinate in world reference frame")  
    velocity: Optional\[Pose2D\] \= Field(None, description="Linear and angular velocity vector")

class MissionSpec(BaseModel):  
    initial\_pose: Pose2D \= Field(..., description="Robot starting coordinate and heading")  
    waypoints: List\[Pose2D\] \= Field(default\_factory=list, description="Intermediate navigation waypoints")  
    goal\_pose: Pose2D \= Field(..., description="Final navigation goal pose")  
    timeout\_seconds: float \= Field(120.0, ge=10.0, le=600.0, description="Maximum operational allowance")

class SimSpec(BaseModel):  
    spec\_version: str \= "2.0.0"  
    environment: EnvironmentSpec  
    objects: List\[ObjectSpec\] \= Field(default\_factory=list)  
    robot\_model: Literal\["turtlebot3\_waffle", "turtlebot4", "ridgeback"\] \= "turtlebot3\_waffle"  
    sensor\_noise: SensorNoiseSpec \= Field(default\_factory=SensorNoiseSpec)  
    adversarial\_triggers: List\[DynamicObstacleTrigger\] \= Field(default\_factory=list)  
    mission: MissionSpec

## **5\. Technology Stack and Fail-Safe Demonstration Strategy**

### **5.1 Technology Selection Matrix**

To maintain build velocity and eliminate compilation failures, Sim2Lang standardizes on pre-compiled binary packages and stable upstream libraries.

| Architectural Layer | Selected Technology | Version / Distribution | Technical Justification |
| :---- | :---- | :---- | :---- |
| **Operating System** | Ubuntu Linux LTS | 22.04 LTS (Jammy) / 24.04 LTS (Noble) | Native host and Docker base support for target ROS 2 distributions5. |
| **Robotics Middleware** | ROS 2 | Humble Hawksbill / Jazzy Jalisco | Standard LTS distributions with native Nav2 support5. |
| **Simulation Engines** | Gazebo Classic / Gazebo Harmonic | Gazebo 11.10.2 / GZ Sim 8.x | Native physics backends executed via host or Docker with X11 pass-through5. |
| **Primary Interface** | OpenCode-Style Interactive CLI | Python 3.10+ (prompt_toolkit, rich) | Standalone, non-blocking conversational terminal copilot with markdown and streaming. |
| **Visual Perception** | X11 Desktop Screen Grabbing & ROS Camera | mss / xdotool / sensor_msgs/Image | Dual-stream visual perception of Gazebo/RViz GUI windows and robot camera. |
| **Navigation Stack** | Nav2 Simple Commander | Standard binary distribution | Exposes high-level programmatic navigation control via Python6. |
| **Map Synthesizer** | gazebo\_ros2\_2Dmap\_plugin / Shapely Rasterizer | Zero-SLAM geometric slicer | Direct geometry slicing producing PGM/YAML maps without SLAM5. |
| **Agent Tool Protocol** | FastMCP Python | FastMCP 3.4+ | Structured stdio and streamable HTTP tool execution for LLM agents16. |
| **Core Reasoning Agent** | NVIDIA Nemotron 3 Ultra 550B A55B | OpenAI-Compatible API / NIM Endpoint | State-of-the-art agentic tool calling, spatial reasoning, and multimodal vision. |
| **Secondary Dashboard** | FastAPI + React / Three.js | Port 10991 (REST/WS) / Port 10990 (UI) | Optional browser-based telemetry mirror and Mode B deterministic fallback. |

### **5.2 Dual-Mode Demo-Safety Strategy**

Live hackathon demonstrations face common failure points, including X11 display server freezes, OpenGL context crashes, DDS discovery conflicts on shared conference networks, or unexpected Nav2 planning aborts. To guarantee presentation reliability, Sim2Lang implements a dual-mode execution architecture with automated failover handling.

Live Presentation Dashboard  
            │  
            ▼  
Execution Supervisor & Health Probe  
      │                    │  
      ▼ (Nominal Path)     ▼ (Failover Path)  
Mode A: Live Sim     Mode B: Synthetic Replay  
  \* Gazebo physics     \* In-memory MCAP traces  
  \* ROS 2 DDS graph    \* Client-side Three.js  
  \* Real-time Nav2     \* Deterministic plots

#### **Mode A: Live Simulation Mode**

Under nominal conditions, Gazebo runs locally with hardware acceleration. To prevent desktop rendering issues, Gazebo is launched with the \-s flag, running the physics server headlessly (gzserver or gz sim \-s \-r)3. Robot poses, costmaps, and sensor data stream over WebSockets to the web dashboard, isolating physics computation from user interface rendering.

#### **Mode B: Deterministic Web-Rendered Fallback Mode**

An autonomous supervisor process monitors simulation health, tracking the Real-Time Factor (RTF) and node heartbeats via /clock and sim\_status16. If:

> 1. The physics Real-Time Factor drops below ![][image25] for more than three consecutive seconds,  
> 2. An unhandled X11, Wayland, or OpenGL process fault occurs, or  
> 3. Nav2 encounters an unrecoverable planning abort,

the supervisor triggers an immediate hot-switch to Mode B. The frontend seamlessly redirects its WebSocket data pipeline to an in-memory replay engine that streams pre-recorded MCAP telemetry traces matching the requested prompt archetype. The React UI continues rendering kinematic movements in Three.js and updating Plotly performance charts at ![][image26], preventing demonstration crashes during evaluation.

## **6\. Implementation Roadmap: 72-Hour Sprint Execution Plan**

The 72-hour development sprint is structured into four sequential phases, progressing from core data contracts to a validated, demonstration-ready platform:

| Sprint Phase | Operational Milestones | Technical Deliverables | Risk Mitigation Protocols |
| :---- | :---- | :---- | :---- |
| **Phase 1: Hour 0–12** | Core Specification, Prompt Engine, and Procedural World Builder | 1\. Implement Pydantic SimSpec data models. 2\. Build LLM prompt engine with structured output enforcement. 3\. Adapt gazebo\_world\_generator placement engine using Shapely 2.05. 4\. Integrate Jinja2 SDF templates and zero-SLAM 2D map generation5. | Validate generated SDF strings using sdformat Python bindings before saving to disk to prevent parser errors. |
| **Phase 2: Hour 12–24** | ROS 2 Launch and Navigation Automation | 1\. Implement parametric ROS 2 Python launch scripts. 2\. Integrate Nav2 Simple Commander (BasicNavigator) mission dispatcher6. 3\. Implement sub-second reset services based on gym-gazebo2 patterns19. 4\. Validate map ingestion via nav2\_map\_server10. | Standardize on pre-tuned TurtleBot3 Waffle configurations; freeze inflation layer parameters to prevent costmap tuning delays. |
| **Phase 3: Hour 24–48** | Adversarial Perturbation and Telemetry Instrumentation | 1\. Implement FastMCP tool server for runtime spawning and light adjustment16. 2\. Build synthetic sensor degradation nodes (OpenCV image blur, LiDAR dropouts). 3\. Instrument robot chassis collision meshes with contact sensor plugins14. 4\. Implement telemetry recording for kinematics and collision events. | If dynamic model spawning introduces physics instability, fall back to pre-spawned dormant models enabled via physics flags. |
| **Phase 4: Hour 48–72** | Web Dashboard Integration, Analytics, and Demonstration Rehearsals | 1\. Implement FastAPI WebSocket streaming endpoints16. 2\. Build React/Vite dashboard featuring live Plotly performance charts16. 3\. Validate the autonomous dual-mode safety fallback supervisor. 4\. Complete ten consecutive dry-run demonstration cycles. | Record backup video captures of nominal runs to provide fallback visual collateral if hardware issues occur. |

## **7\. Mathematical Formulation and Telemetry Scoring Engine**

To provide quantitative evaluation during demonstration runs, the telemetry engine computes a composite Navigation Quality Index (![][image27]) evaluating both nominal and degraded execution paths.

### **7.1 Path Traversal Efficiency**

Path traversal efficiency evaluates actual traveled distance relative to the theoretical Euclidean A\* shortest path:  
![][image28]  
where ![][image29] represents the shortest collision-free path across the static occupancy grid, and ![][image30] represents the integrated distance traveled by the robot.

### **7.2 Trajectory Smoothness and Control Stability**

Trajectory smoothness evaluates actuator stress and control stability by integrating the squared norm of linear acceleration jerk and angular acceleration rates:  
![][image31]  
Elevated values reflect erratic velocity adjustments or recovery maneuvers triggered by dynamic obstacles or sensor noise.

### **7.3 Minimum Obstacle Clearance**

The minimum obstacle clearance metric tracks the closest distance between the robot chassis boundary and surrounding obstacle geometries throughout navigation:  
![][image32]  
Values below the safety threshold ![][image33] indicate near-miss events, while values of zero denote physical collisions.

### **7.4 Task Success Formulation**

Mission completion requires the robot to reach the target pose within position tolerance ![][image34] without physical collisions:  
![][image35]  
where ![][image23] records the number of force events exceeding ![][image22] logged by the /sim2lang/contact\_states topic23.

### **7.5 Composite Navigation Quality Index**

The composite Navigation Quality Index aggregates normalized performance metrics into a unified benchmark score:  
![][image36]  
Standard normalized weights are defined as ![][image37], ![][image38], ![][image39], and penalty weight ![][image40].

## **8\. The Winning 3-Minute Live Pitch and Demonstration Script**

A winning hackathon pitch balances clear problem definition, rapid technological execution, unexpected system tension, and quantitative proof of performance.

### **8.1 Timeline Beat 1: The Hook (0:00 – 0:30)**

* **Presentation Visual:** The display opens on a terminal showing an unhandled ROS 2 launch traceback with mismatched transform frames and costmap configuration warnings.  
* **Spoken Pitch:** "Judges, autonomous mobile robots are transforming industrial operations, but developing them remains bottlenecked by simulation setup. Preparing a single Gazebo test world—authoring SDF files, aligning transform frames, driving a robot around for 15 minutes to run SLAM, and configuring sensor noise—takes an experienced engineer two to three days1. When an autonomous robot fails in the field due to degraded lighting or unexpected obstacles, reproducing that failure in simulation is slow and labor-intensive1. This is **Sim2Lang**: an AI-driven compiler that turns natural language directly into executable, stress-tested robot simulations in seconds1."

### **8.2 Timeline Beat 2: The Wow (0:30 – 1:15)**

* **Presentation Visual:** The presenter opens the standalone OpenCode-style terminal interface. At the interactive `sim2lang>` prompt, the presenter types:  
  `"Generate a 14x10 meter logistics warehouse with 8 pallet racks, a packaging bench, nominal lighting, and dispatch the AMR to rack 5."`  
* **System Action:** The Agent initiates its ReAct tool loop.  
* **Visual Progression:**  
  * In under 1.5 seconds, the Agent invokes `solve_spatial_layout` and streams the structured `SimSpec` JSON into the terminal.  
  * Within 3.5 seconds, the native 3D Gazebo GUI window and RViz2 desktop window open on the workstation display via the containerized X11 bridge.  
  * Concurrently, the zero-SLAM 2D occupancy grid (`map.yaml`/`map.pgm`) is sliced and ingested into the Nav2 map server.  
  * The TurtleBot3 Waffle initializes its localization in Gazebo and begins navigating smoothly along the green trajectory path toward Rack 5.  
  * The Agent invokes `capture_gazebo_window()` and confirms in the terminal: *"Visually verified: Gazebo simulation is running (RTF: 1.0x, Sim Time: 00:04.2). 8 racks and TurtleBot3 are placed collision-free."*  
* **Spoken Pitch:** "In four seconds, our Agent completed tasks that typically require days of manual setup. It solved spatial placement constraints, compiled valid Gazebo SDF, sliced the world to generate a Nav2 occupancy map without running SLAM, launched native Gazebo and RViz desktop windows, visually verified the scene, and dispatched the AMR via Nav2 Simple Commander. And our terminal CLI remains actively connected."

### **8.3 Timeline Beat 3: The Twist (1:15 – 2:15)**

* **Presentation Visual:** While the robot moves in Gazebo on the desktop, the presenter enters a follow-up stress prompt directly into the active CLI:  
  `sim2lang> Stress-test navigation: drop ambient lighting to 15%, inject camera motion blur, and spawn a moving obstacle directly in the robot’s path.`  
* **Visual Progression:**  
  * The Agent calls `set_ambient_lighting(0.15)`: Gazebo's ambient world lighting dims to low-light conditions.  
  * The synthetic on-board camera stream displays motion blur and optical sensor noise.  
  * The Agent calls `spawn_dynamic_entity("cargo_box", x=4.2, y=2.0)`: A cargo container spawns directly across the planned trajectory.  
  * In the RViz window, the local costmap flashes yellow/red as LiDAR beams hit the new obstacle.  
  * The robot decelerates, calculates an evasive local detour around the barrier, and continues safely.  
  * The Agent calls `capture_rviz_window()` and informs the user: *"Obstacle detected at (4.2, 2.0). Nav2 costmap updated. Robot executed an evasive detour with 18cm safety margin."*  
* **Spoken Pitch:** "Using our FastMCP bridge and dynamic sensor injection pipeline, Sim2Lang acts as an autonomous stress-testing agent. We degraded lighting, injected optical noise, and dropped unexpected obstacles into the robot's heading. The navigation stack dynamically replanned, avoided the hazard, and preserved mission safety—while the AI visually monitored both the robot and developer tools."

### **8.4 Timeline Beat 4: The Payoff (2:15 – 3:00)**

* **Presentation Visual:** The robot reaches the target waypoint. The terminal interface renders a Rich-styled comparative analytics table contrasting the nominal run against the degraded run.

| Evaluation Metric | Nominal Benchmark Run | Adversarial Stress Run | Performance Delta |
| :---- | :---- | :---- | :---- |
| **Mission Completion Status** | Succeeded (![][image41]) | Succeeded (![][image41]) | Baseline maintained (![][image42]) |
| **Mission Duration (![][image43])** | **![][image44]** | **![][image45]** | **![][image46]** (Velocity reduction) |
| **Traversed Path Length (![][image47])** | **![][image48]** | **![][image49]** | **![][image50]** (Avoidance detour) |
| **Path Traversal Efficiency (![][image51])** | **![][image52]** | **![][image53]** | **![][image54]** |
| **Trajectory Smoothness (![][image55])** | **![][image56]** | **![][image57]** | **![][image58]** (Steering corrections) |
| **Minimum Obstacle Margin (![][image59])** | **![][image60]** | **![][image61]** | Near-miss envelope detected |
| **Physical Collision Count (![][image62])** | 0 | 0 | Zero structural impact23 |
| **Navigation Quality Index (![][image27])** | **94.8 / 100** | **68.2 / 100** | **\-28.0% Performance Degradation** |

* **Spoken Pitch:** "This demonstrates the core capability of Sim2Lang. The platform moves beyond static world generation to deliver quantitative, automated regression testing for robotics. The dashboard measures the impact of operational stressors: a 60% increase in transit time, a 340% increase in trajectory jerk, and a near-miss clearance margin of 18 centimeters, with zero physical contact verified by simulated bumper sensors23. Sim2Lang reduces simulation setup time from days to seconds, allowing engineering teams to validate autonomous systems across thousands of edge cases before physical deployment1. Thank you."

### **8.5 Standardized Benchmark Archetype Test Suite**

To ensure deterministic testing and bulletproof validation during live hackathon judging, Sim2Lang defines four standardized reference archetypes covering every subsystem of the compiler:

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

#### **Detailed Specification of Archetype Test Scenarios:**

1. **Archetype 1 (Logistics Warehouse Retrieval):**
   * **Input Prompt:** `lang2sim> Load a logistics warehouse, set lighting to 0.2, and navigate to rack 5`
   * **Target Environment:** `catalog/scenes/aws_small_warehouse.sdf` paired with `catalog/maps/aws_small_warehouse.yaml`.
   * **Target Coordinates:** Resolved automatically via `scene_catalog.json` semantic anchor `rack_5` $\to [7.5, 3.0, 0.0]$.
   * **Pass Criteria:** Desktop Gazebo GUI opens visibly; TurtleBot3 initializes localization instantly without SLAM; reaches waypoint within tolerance $\Delta r \le 0.15\text{m}$ in $<35\text{s}$.

2. **Archetype 2 (Custom Bespoke Factory Layout):**
   * **Input Prompt:** `lang2sim> Build an 18x12m workshop with 6 workbenches, 2 storage racks, and send the robot to rack 2`
   * **Solver Action:** Shapely 2.0 computes outer boundary polygon and packs 8 catalog bounding boxes with safety corridor buffer $W_{\text{aisle}} \ge 2.2\text{m}$.
   * **Zero-SLAM Slicer:** Generates `map.pgm` and `map.yaml` in $<3.5\text{s}$ from world geometry.
   * **Pass Criteria:** Validated with `sdformat` Python bindings; Nav2 global planner finds an unblocked path without collision warnings.

3. **Archetype 3 (Outdoor Agricultural Field with Weather Stress):**
   * **Input Prompt:** `lang2sim> Setup an outdoor field with dirt ground, fog density 0.15, sunset illumination, and follow waypoints`
   * **Environment Specs:** `ground_terrain="dirt_mud"`, `surface_friction=0.5`, `fog_density=0.15`, `sun_elevation=0.2`.
   * **Vision VQA Action:** Agent samples `/camera/image_raw` and desktop screen grab, confirming visual fog occlusion and dusk lighting to the user in the TUI.
   * **Pass Criteria:** Nav2 waypoint follower successfully navigates the 4 GPS/odometry waypoints under low-traction physics.

4. **Archetype 4 (Dynamic Adversarial Collision Injection):**
   * **Trigger Mode:** Executed dynamically while the robot is halfway to its goal in Archetype 1 or 2.
   * **FastMCP Mutation:** In-process call to `spawn_dynamic_entity("wooden_crate", pose=[3.5, 2.0, 0.5])` directly in front of the robot's heading.
   * **Observation & Response:** LiDAR registers obstacle; local costmap marks cells as lethal; Nav2 DWB / TEB controller executes local evasive arc.
   * **Scoring Output:** Mission successfully completes; telemetry engine logs $C_{min} = 0.18\text{m}$, zero contact sensor forces ($C_{\text{force}} = 0$), and outputs the comparative NQI degradation score in the terminal table.

## **9\. Architectural Synthesis**


Sim2Lang demonstrates that simulation environment configuration can be automated through typed intermediate representations, deterministic procedural geometry engines, and modular middleware bridges1. Integrating LLM reasoning with spatial collision solvers and direct physics slicing eliminates the manual setup friction historically associated with Gazebo and ROS 25.  
Grounding generative systems in physical validation constraints and reproducible benchmarking metrics establishes a viable path toward autonomous test engineering for mobile robotics1. By uniting natural language understanding with low-level physics simulation, the platform compresses multi-day engineering configurations into sub-minute compiler operations, ensuring autonomous software is systematically validated across diverse operating conditions before deployment on real hardware1.

#### **Works cited**

> 1. Drift: Simulation based evaluations for 10x faster robot deployments, [https\://godrift.ai/](https://godrift.ai/)  
> 2. Best Robot Simulators for ROS 2: A 2026 Comparison \| Drift, [https\://godrift.ai/blogs/best-robot-simulators-ros2](https://godrift.ai/blogs/best-robot-simulators-ros2)  
> 3. Introduction \- Drift, [https\://docs.godrift.ai/getting-started/introduction](https://docs.godrift.ai/getting-started/introduction)  
> 4. Navigating using GPS Localization \- Nav2 Docs, [https\://docs.nav2.org/jazzy/tutorials/general\_tutorials/navigation2\_with\_gps/navigation2\_with\_gps/](https://docs.nav2.org/jazzy/tutorials/general_tutorials/navigation2_with_gps/navigation2_with_gps/)  
> 5. GitHub \- robotics-upo/gazebo\_world\_generator: A ROS2 package, [https\://github.com/robotics-upo/gazebo\_world\_generator](https://github.com/robotics-upo/gazebo_world_generator)  
> 6. Nav2 Stack Using Simple Commander Python API \- GitHub, [https\://github.com/hello-robot/stretch\_tutorials/blob/master/ros2/navigation\_simple\_commander.md](https://github.com/hello-robot/stretch_tutorials/blob/master/ros2/navigation_simple_commander.md)  
> 7. Simple Commander API \- Nav2 Docs, [https\://docs.nav2.org/rolling/configuration\_and\_development/simple\_commander\_api/simple\_commander\_api/](https://docs.nav2.org/rolling/configuration_and_development/simple_commander_api/simple_commander_api/)  
> 8. gene-su/pgm\_map\_creator: Generate pgm map from Gazebo world, [https\://github.com/gene-su/pgm\_map\_creator](https://github.com/gene-su/pgm_map_creator)  
> 9. Build a Full Robot Simulation under 10 Mins with Drift (ROS 2 \+, [https\://www\.youtube.com/watch?v=lWcuospP9Jg](https://www.youtube.com/watch?v=lWcuospP9Jg)  
> 10. GitHub \- laygond/Adaptive-Monte-Carlo-Localization: A robot uses a, [https\://github.com/laygond/Adaptive-Monte-Carlo-Localization](https://github.com/laygond/Adaptive-Monte-Carlo-Localization)  
> 11. robotics-upo/gazebo\_ros2\_2Dmap\_plugin: Gazebo simulator plugin, [https\://github.com/robotics-upo/gazebo\_ros2\_2Dmap\_plugin](https://github.com/robotics-upo/gazebo_ros2_2Dmap_plugin)  
> 12. JZX-MY/pgm\_map\_creator: Create pgm map from Gazebo world file, [https\://github.com/JZX-MY/pgm\_map\_creator](https://github.com/JZX-MY/pgm_map_creator)  
> 13. 激光slam-- .pgm格式地图分析及修改方法 \- CSDN博客, [https\://blog.csdn.net/GGY1102/article/details/108397727](https://blog.csdn.net/GGY1102/article/details/108397727)  
> 14. gym-gazebo2 is a toolkit for developing and comparing ... \- GitHub, [https\://github.com/acutronicRobotics/gym-gazebo2](https://github.com/acutronicRobotics/gym-gazebo2)  
> 15. GitHub \- ishandutta2007/Awesome-Robotics-Fleet-Management, [https\://github.com/ishandutta2007/Awesome-Robotics-Fleet-Management](https://github.com/ishandutta2007/Awesome-Robotics-Fleet-Management)  
> 16. Bridges the Gazebo robot simulator to AI agents via MCP. · GitHub, [https\://github.com/sandraschi/gazebo-mcp](https://github.com/sandraschi/gazebo-mcp)  
> 17. gazebo-mcp/CHANGELOG.md at master \- GitHub, [https\://github.com/sandraschi/gazebo-mcp/blob/master/CHANGELOG.md](https://github.com/sandraschi/gazebo-mcp/blob/master/CHANGELOG.md)  
> 18. gazebo-mcp/AGENTS.md at master \- GitHub, [https\://github.com/sandraschi/gazebo-mcp/blob/master/AGENTS.md](https://github.com/sandraschi/gazebo-mcp/blob/master/AGENTS.md)  
> 19. GitHub \- ENPH353/gym-gazebo-sim: API extension for Gymnasium, [https\://github.com/ENPH353/gym-gazebo-sim](https://github.com/ENPH353/gym-gazebo-sim)  
> 20. (PDF) MultiROS: ROS-Based Robot Simulation Environment for, [https\://www\.researchgate.net/publication/365098937\_MultiROS\_ROS-Based\_Robot\_Simulation\_Environment\_for\_Concurrent\_Deep\_Reinforcement\_Learning](https://www.researchgate.net/publication/365098937_MultiROS_ROS-Based_Robot_Simulation_Environment_for_Concurrent_Deep_Reinforcement_Learning)  
> 21. AutoRL: A Tightly Synchronized ROS2–Gazebo Pipeline for ... \- MDPI, [https\://www\.mdpi.com/2226-4310/13/9/825](https://www.mdpi.com/2226-4310/13/9/825)  
> 22. pic4rl-gym: a ros2 modular framework for robots autonomous ... \- arXiv, [https\://arxiv.org/pdf/2211.10714](https://arxiv.org/pdf/2211.10714)  
> 23. gym-gazebo2, a toolkit for reinforcement learning using ROS 2 and, [https\://arxiv.org/html/1903.06278v2](https://arxiv.org/html/1903.06278v2)  
> 24. ROS 2 Fleet Management Dashboard for Autonomous Mobile, [https\://robocloud-dashboard.vercel.app/learn/blog/ros-bridge-integration](https://robocloud-dashboard.vercel.app/learn/blog/ros-bridge-integration)  
> 25. I am using in rviz2 the tool nav2 goal. But I want to cancel the robot's, [https\://robotics.stackexchange.com/questions/103296/i-am-using-in-rviz2-the-tool-nav2-goal-but-i-want-to-cancel-the-robots-navigat](https://robotics.stackexchange.com/questions/103296/i-am-using-in-rviz2-the-tool-nav2-goal-but-i-want-to-cancel-the-robots-navigat)  
> 26. An end-to-end Autonomous Smart Wheelchair powered by ROS 2, [https\://github.com/abdulrahimzulfiqar/ROS2-Autonomous-Wheelchair](https://github.com/abdulrahimzulfiqar/ROS2-Autonomous-Wheelchair)  
> 27. rosclaw-dashboard/另一个AI的想法.md at master \- GitHub, [https\://github.com/ros-claw/rosclaw-dashboard/blob/master/%E5%8F%A6%E4%B8%80%E4%B8%AAAI%E7%9A%84%E6%83%B3%E6%B3%95.md](https://github.com/ros-claw/rosclaw-dashboard/blob/master/%E5%8F%A6%E4%B8%80%E4%B8%AAAI%E7%9A%84%E6%83%B3%E6%B3%95.md)  
> 28. Getting Started With the Simple Commander API – ROS 2 Jazzy, [https\://automaticaddison.com/getting-started-with-the-simple-commander-api-ros-2-jazzy/](https://automaticaddison.com/getting-started-with-the-simple-commander-api-ros-2-jazzy/)

[image1]: <data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAB4AAAAVCAYAAABR915hAAAByklEQVR4Xu2VzSsFYRSHT76FBYpIykb+ATtsSFkIWYmFEFkqHykpRRZKkqKsZKGklKQQxUo2KMWObCyI2CHx+3nfO945dy7yVcpTT3fuOee+M/Pec2ZE/vnD5MAVeAOP4DBM9FX8AMnwGLbAdNgDn+CSWxRENqzRwQDy4Bzch5swzsb74VCoyMI6nrxSxX0swD0dDGAXTsMteArTbHwZXsFi+53UiznxjBPzwau+haM6oSgUs1AVzIRJTm7R5pqcWLWNzTsxH0XygS0BA/ABpugEyIB1MMqJ9YlZl7/z0QbX4Rm8t8c01y0C3TZ+DS/sMQ1tcxDR8BBewiyV82CTbOtgALzAiP+Xgt3N3SnTiRAJ8E7CO1LDq+a2cUzeo0DMLLO5IlIqZsFynVBUiKnj51ukitniZp3QDIq543idUPSKOTGbKBKxcA12OTFeCGc8DP63bBTCsVqFMa9pDz4MTnRQMQYnVawEjqjYC1xsXMzJpmCtP+3Bx2HEeQQNYnZkw3oAz+EjbHXqPNh9bAQWNvpTHtwuLtqhEw47YmqCZB99Co4EF9Dz/WN0wnYxjcWXwq+QL+YuJ8S8EN6cye+EDxa+hTiTsyr3JZ4BJjth+v678UUAAAAASUVORK5CYII=>

[image2]: <data:image/png;base64,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>

[image3]: <data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAF4AAAAVCAYAAADPTVnBAAADwUlEQVR4Xu2YeYhPURTHj30vS4jSDIn8ISFkCVlCCmUt+76EbNllEEJ2UrKMJWXJGtkNWZP4A2WriRApWwiJ79e5d+b+7vzG/J55mld+3/rU753zXr/7zr333HOeSFJJJZVUUv+DyoFJ4CJ4Dn56PANnQBP7QARUBIwG18EcY2sHtoF3hqOgmPFFTh3AXTAd9ARPwRvRl+gPZoJ7ohPwDXTRxwpc88AD0XFxAtaA22A4GCH6DvSNsQ9ESTXAFzDIsb0FGc41VRisE32RU56vIDVfdEyZYD8o6vgmG18kA78Y7AKFzHV10cFuyrojW6mivoxYcyBxF+0UDRJTl89JUCLr7rx1UHRMXPmlPN968BpU9OyRVFfRF5noO6AUUd9s35Gg0sASUNaz50dPRMfk7lgGeotoymzk2CMtHlJ8kfa+QzSHZooexEHVVzTwYYoB5lhfiO6SZmAVeAmOgTLZt0Zf+8B3yRncPuA9aOrZE1FxcEli828Y4iHPwDPY9cBZ8MHYyCFQKevuiIu5knmWKg2ag9XG3tDeZNQKXPFs8dQAzPKNIYjnEwPMqsyKJWZrcMf44p1VkVMt0cGyDj4BPoIDoqu9pHOfVRXQyzfGUT8w2DeGoAuiJSOD7Yv9Bt+FNX7kxTqeg+WBxFUaVuPRBizzjflUefAVpHt2K5bJfJeNviOKuiaaUhLRNHBEtDzMSyxVb4oGKyxxBzGwrMJSY12/NQW8AtVAR/BQtItlKbvV3FNBtKxlKqWtm7FzvCuMjb2LTZMtRBvNdLAUbAC7QUtzD8vywJ29XSELfUcccQCNRf9oqOfLTczD5yWc4DMwt0RLyU+i3TRr9u5gGNgr+umjpn0Amit6HtUG441tDxhofrPEfQzqg7Fgs7FTTLejzG9+VuEk2l6DY+AipDhxXIyBNEM08BxYXuIkMeczv1b2fH8S2/r7oIdoJ/y36iS6O1PAANHPBD9Ey8jtEnvYWnFFrnWuWbXxGXdyDot+HuEEDXHsU8Fx83uC6IRZ8X14mFMcF6u3QOIhFGS2eOByoFVFy8VEVRcsFy31/K6VnJacpWwYYuBXOtdMM1xoHI9VBlggmhZHOvZForuVGie6060YeFZ3FAN/2fElJDYfQVYhW3VWNDw0/VY9imJj6AaeOif6UZDiAvosWlikiTaLFAuMG6Lph2JHn1vgO4Orju+fiNUC23KWilFXW9Gc/0g0VVjVES1JeUgydfGQpNi/cEemi65gpkiKzSN3JIPdW/SsYFO5Q7TIYArigW7Pg6Sipl8a9M6GWXbiowAAAABJRU5ErkJggg==>

[image4]: <data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAFQAAAAVCAYAAADYb8kIAAAD/ElEQVR4Xu2YWaiNURTHl3keM483w4OZhIxFGR7MZIpCxsg8PpCEkMiYJCWzZJ7nIeQZydQtMos8IEr8/63vu3efdc43nOu+OO6v/t2z195n3fPtvddae38iBRTwr1EMOgLVtR3/IQug0daYLluhgdaYQXCCbkFPoT6mz1IYugZ1tx1xGQUdtcYMYiF0DiojOkmvoRIJI5JpKDqunO2IgqvxSqJX7V+lNfQCquK1S0O/oX45I4I5Cc2zxij6Q8+gQrYjAygC3YVWObaaohM6zbEFMV50sxW3HWHsgTZbY4bQW3TyWji2rp4tzs5rJDq2h+0I4zE03RozhG2i0ecyR3SShhl7ENnQXGsMooKo8wG2w6MjdAm6B12B2kO7oH3QYmdcXmkGHRD1PTaxSw5Bs4wtHZjC3oo+Xyo1zR0aygVoizUG0UrUeTfbIRomTAdlvfZN6LPod1aLfs/vywvMbzzGVILWQG+cvjqi/sc5tnThM9HHGMfGXPgF+ir6/+PAheWix6KdJOcYn01QPe8zV5uV8rDXXgtN8j77dIFuG1sYfGDucvp+Ap1x+lgM+Lsae20eXZiaaueMiGaRqI+Wjq2XZzvo2KJ87xCN0lj4O5R/w2BoRu2YatBQa4wBj2v0zbOwz25J3LE82k2U9E4i60X9lnRs6zxbZ8cW5Xs7dNUag+A1MyjkXaaKjssy9vyA+Ylh6D84H5CT6UdDXlkJvXPa9PsSOuXY4sBwP2aNQfDmwInqaztEr2rLvc884PLK5pMFbXDaPIKckOTjBfMjd3VYvroBnXXabUR/0wyvPUS0EM7PGZELQzhVuiKTofdOezD0TXLTCAnz7cMbFtNfbLKhmcbGa9kv0YTMH/xTtICQotBeqK3X7uR9ZgGzKYHjODlh1ZonhsveZ4bdadHv8IZTCpogGpJ8cJcmouNYyflix8L8/wOqD1WGHkli3g/z7cL8PsUaw6AzPriFlfc89ED0/suVopiH3F3BtMFw/QhVdexkhejC7Dd2FxYD/n+G4h3Rc+MH0RBlwagoenQZ5H/Bgzmb47jYzU2fzxLRibwvWuhcwnz71BBdtA62IwzuKoZGqlWOCw/Jx6HqknxN4x2aRSYVTDkNTPuTJO6YWqK5sLzoJFoYjm4Yp0OUb57PH1pjFHTGsIl7c0gF31SxwnNXM5Rc+KMYWqngjvwuuTmWtxjuaOZRn9nQRmi4JFZnH/e4lS5Rvhk5YekqEL4oYO7KK3yXuhMaYezMiRcl9eqT56LHEsKcyLTBnOYyUrTgLTN2whc7S60xDcJ8c9czVbjHrtgwX3GX9bQdfwmrfthrQb68YDHjwfm6JJ8SomCOZpHMbzgfrBdxr6fyB+5PxzEAwVGTAAAAAElFTkSuQmCC>

[image5]: <data:image/png;base64,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>

[image6]: <data:image/png;base64,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>

[image7]: <data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAACwAAAAVCAYAAAA98QxkAAAB4UlEQVR4Xu3VT0hVQRTH8VNipKhoKIFI2CYxESwQTMUCdyEaBbYIQwrBJGhluLEQikDDIF3UQiRCEVxo0D/KykUg1qJF0CJtE7iLso0bkfqe5lzeMCDed2kRcX/wgZkz8+47777hXpE0adKkSZMmQS7jFV6jAXNYwE1/07+SakziMH7hI4pxw+b7MltjRT/bgtJw4W/lOppwTlyDJ6x+G302zibnsYaOcCFGjiE/LG6Xe/iOnHAhQV5KsobHJYt/5hMehcWESdLwcWxIzIYPiDsO/eECqcQi3mEY78Wd0xLM4g4m0O62/8lzzOAKxnAXu2ytDo8xIu5zNdiNQXE93EKv7d023eI21wf1KP6v1wsXYRpdtl6AVdTa/Bl6bKx5gk7sxWcctPoRfLG6RnuIdYev4qlk7kKYZnzw5oXYkswXa+YxYGNt+Ky3pnfzIU5ixatrfqLRxrEb3inasB6JKHoc9OJVXm0RQzYOG9YnzhRO4atX1zu7iVabRw3r9+2JNiWJPlf9hjX6ojlj4/3ijsxRm2vDF22s53MZbeJ+6DeU2dppcXc8z+Y/UIFryLVa1qkU93euY1Qy5+0Q3oh7FC2Je5ZH0fPdjUt4gAvemjb5FvfxAuXemp57vZ6+G/6P/AaojFaVwAZisQAAAABJRU5ErkJggg==>

[image8]: <data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAEcAAAAVCAYAAAAU9vPjAAAC10lEQVR4Xu2WWchNURiGXzJnSL8hRUrmZEyG0EmGXLhDyh3lQvgNmYdkiCIhZMyYEkpuiAs3ihtDkjJemAplDEnifX1r2eusnf8cdP7+2vutp7PXt4a993u+9e0F5MqVK1fd0QUyLw7mAsaTH+QVaRr1ZVr1yFVyDmbQquLubGsG2U06ky/kI+lUNCKjakQeko6uvROWPft+j8iwZpPNQbsD+Uy+kq5BPHNqQu6QNlF8Cyx7DkXx/1VvMiwO1lXps702DsLM+kC+IZ09w8ljUhXF/6RJsMx8QE7B5paj0VG7mlyKYhWTskYP3CrucNoIy56jUbw9WQ6rVaXUktyDfQ2PkzEo35yTUXsImR7FKqZFZEUcDKTseQvLnm5RX7nqTm4H7V4obY6MXEhuxB21pWawWtMi7oi0BpY9R4LYanILtt20xbTOYVimbSPr3TjdQ+03sHOTxvdEYo4y9xhZSjaRcS7ehRwgz2HzJsCeUxl8zY2RppGzsLn6wrYmbclFcpMsIRtg92ju5pSlBbCbaw/XxBWYOd9Jj18zTS+R1CLVrfuksWs/gpkghWbEbW2507AM1Yn8tbuWCkhnTl8kc3WtjGzo2qpHMkEaCHtmZa2kj8pMd11SDcgT2AJ/g/5Nr2dIzJlDTgR9d8lQd12TOZLWmEVWknekj4sXkDYn3JLKloNB32DYwbU+6UdeBH17yOKgXXE9RWKOXs7/a5LM0XaT4hoTmjMSdvj09Uxr6sWkAhJz/FcrnLsVxfccC6uNyl4ZrLW8ZI62bq1JW9KbMxdpc0a4a51rQnP04L6tI4SvZdoeypypjkGwdaR17jdcS4ZdhxVvaTuS7O2PtDnLgnZFpRTV6XkH7ECnAqgXmQw7bb+HvfQAspd8ghVW1QDfng8zSgVWBVxGaZvoBXV+0kvvgp2PppB2wVz/OVexPU/OwMxR1mjcftjz6Tknwky8TEbZtFz/pJ8S9akl5mIsKAAAAABJRU5ErkJggg==>

[image9]: <data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAJsAAAAVCAYAAABR25wkAAAEtUlEQVR4Xu2ZeahtUxzHvzwyxzNlSuYpU4QUtXrEH6aSqWTMPCYyxsuQOTJleOF5T2Se55mSlPjLGIpCUZIk/uH36bfWvessdx/77L3PfdezP/XtrrPWuvusvdZ3/dZwpJ6enp6e/z57mP6KOqAo6xnO6prsu4uLsp4pwGwXlJk9IxHUm60WvdnaE9SbrRa92doT1JutFr3Z2hP0PzHb8aYFpmdMr2S6JK80hN5s7QlazM22oulp0xmmpYqyUejN1p6glmabZbrR9KfpcfmAbm66Ka+0CLnNdHiZ2YAqs3Gsf0F+rD8n5h2pbr6zK2ZKG4Namu0q09uma0zPyQ33hmn5vFIGkWZ309plwRjY3jSvzGxIldneMT1kusH0svz97x6oseiZKW0Mamm2tbL0SvKXGGakg0yfm46Nn8+U753GwemmHcvMhlSZbd0svaU8ki6d5dXlUdXfP45KV21sS1BLsyWC6UHT7CJ/KqiXzLZLlu6aW+RRNz8QlLpiovZwqsyWOMl0nWmJsqAmh5jmlJkd07aNbQnqwGxHy0Mzka0O92t8BsshUnQ1gFVmW9J0u8YXlbpgprQxqKXZzjJdrcHZwgPZlwX5PuEy+T5ulVi+UG42zLnA9F7Mh71Nr5nOly/JHEA4dBClbpUfRnaeqD2cLUz3lZkNmcpsLEW8y4FZHu19wrS+fH/0oek805WmN03bmS4y3WE6wv9F28qvZFje4BjTT6a5pstND5gOjWUle5m2KTMzumojXCgf60vl2x/YTH4Y5PBxnNzQ38Z6JUEtzHaq6WN5R82XP+ge01OxnPwNY5pGpMiXzAZ09FcxvbHpS/lSvIZ8mVvHdLa8I2AT0xcxXQf+76gyswFTmY1N96vydjKpeP/XNXni20E+CAwI3Cu/hoGVTT9r8jrmNHkfJh6TT0SgXxjAEvZfPP8HVe/BumwjE363mOaZ+8Y03CV/NkFiqyw/J6ih2XggX0CI3khusF/kM2KZWIeX+0b/nFm52eiwZDYa+mJM57wr73waiui45QZqVEPEZSYyCYZFgH+jNBvRh5kMu5o+lU+CwyZqeIT4LvtMpKB/YFn5IPMXTtSg2R42nRzT65l+y8oSa5p+lF87bV2UQddtXFW+ZaIfiIanxHwgQBB4GMMqghqarQ4byF+WwWaG7BTzc7Ox1CWz0ZDnYzrnI/nmtg1cg3BNUx4QyKtDabY6YIA8IjGQ6RlpINMV0QkaNBuHKPIAs/2elZXcbNq0zKxJ3TauJjfqfrGMMeS0nyDAMHZvqfoAEjRGsz1pWiGmuYebE9P5aZQImcxGh32vyb0dIZs8ZhpXAwn2Fyl6ThdNzIbBqwaSyMxAcu8ITKb5MQ2PaNBsRK8quN9sSt027mn6OlUynpX/KkMggXPlxsWEbK+mImiMZmPpw2SYhSsGltyDTZ/IG8sL3ClfIpL59jG9L98Ys7TALNP18gFg/8YzpptRzcYSN8/0h3wgiAgfyA9KTDo20AzktfJJxfaBCM738H6fyW/9+cyhiLppYHP2V/NT5ihtZAmlPZxqacdc00vycWMs2UKxv+Zq5Vf5XplfLnKCxmi2xYlRzTZdMInb/OY7nQQ1MNvfm/MUhlq9Sc8AAAAASUVORK5CYII=>

[image10]: <data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAC8AAAAVCAYAAADWxrdnAAACFklEQVR4Xu2WT0gVURTGj4mRKEihEIg7RRTbmIKCkX9wkYtIBEEQXAnSSuyPGQYJLUoUFxqkIBW6KWgTCOJCF2oQuBIUtI0rJYgWBbkIse947o0zx5nnPOTxFr4f/Bbz3Tsz37yZd2eIMmQ4F9fhLFyBX2FvcDgWBfCaDVNNIdyE9912CfwGn/sJCciCZSQXuw0HgsOpZ5ykvKYP/oA5Jrd8gGtwDh5TGsrvkZTQNJOUuWXyKOooDeX5WeeTTpv8psuHTR5FWsrXkJx0yuQ3XP7W5FHEKZ8NN+BfeACvwlfwM8lCwXc7D07CJfgFDp7sGcFtCi9f6fJ5k0cRp/wl2AgX4S/4keTOM3wRP+ECyQ/KdJAcs9xtn6KewstXuPy9yaOIU94zQTK3QWWdLnussuKQLABfFU94bfIql/OJ4uDLP7ADIfDqxnPzVeZ/5VaV+f/jkMoC8BrPE/gFpal1+TOTR5FM+VGSuVdU5su3qMyXf6qyU+zCTya7Q7Jjm8mj8OUf2oEQ+PlOpnzCFe8F3CF5W3oekawI+iXFa36X2tb48rzfWfjHJm75hHefD7IO77ntXJK70f1/hlzYb5KDVavc00QyNmIHQpghmVuksh6X3VVZqcteqiwU/qB6At+RHLw9MCqswu8U/Pjqh1vwiORELK/lb9Qcz2WStdvP2ycpzS/IQ5f9gWMk31n8eeLnLlOGDBecfxkEghmOkL+sAAAAAElFTkSuQmCC>

[image11]: <data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAC8AAAAVCAYAAADWxrdnAAACFklEQVR4Xu2WTUhVQRiGvxR/IF3kD4jhQgjaS+JGIZIIXIhgCi3UCFcKLRKyIvcWhYRtdCNSqwLzB8GMlAilhW3aRxtJAxEqNFIi39dvpvs53nu8B/JuvA88i3ln7jnfmTsz54hkyfJfOAWrw/AI2uAMnIIvYdPB7uOnErbAN/BV0BfFZfgCFrt2KfwM2/+NOGZuwk/wKdyReMUvwrog64bvgiwjbEm84r/A3iC7CleCLCPELX4B/oH3TMb132faGSNu8R3wr3MCPoFjMM8OcuSK/iO7cA2egQ/gtOjyuwRPw2E4D5dh//4v0yRu8eSOJB7gt+jpk4wceBHOwR+iG73C9fEhNuEsvOCyVtFrnnftI4lbPE8Zzlwn/CiJh7huxoQMiY6pNxlPJ2a3TXY2SRYJi58MwwhewxumzZPmp+jS4LGZjMeiRRWZzM8yj14P/xVmd00WSZzieaN1mB/kfhavBLnnoWh/ocl88Y0m88XbwyASFs83ZTr0wKUwdKxK6jct13ec4u+bLBIWzzWcjAZ4zbQ5899hickIl8MGLAtyj1826RY/YLKUFMBf8K3oN46Fba5lXqzGZM9FN6rffFVwXHTtp2JU9DrlJutyWbPJzrls0GSH4Pn6QbRwDqb8PuF3jt1U7+E3OTjTfIBboi8rfhI8g7Wm38K9wbPb3+OraNEjkrj3NnwkuiT57/mxvH6WLCeaPVBIgSzI86VJAAAAAElFTkSuQmCC>

[image12]: <data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAGEAAAAVCAYAAABWtYB0AAAEkklEQVR4Xu2YC8ieYxjHL0a2JIe0ITIhhzAZIocPKcec5TCTzCLn8ylsDiHnhhzmvAlraA7Nhm22kKy2ORWiRcghiVgjcf123dfe67m+5/16156+b7X3V//67v99v89zP/d1H677E+nSpcvqydPZWJXYRPW4arbqA9XYanWf7K16SfWOao5q/0qtcb1q81DeUjUllPsD+nlfNleC08W+6wnVrsXrKd49qtOK1xEbqz5SnVvKW6i+VI33Bn2wl+o71Z6lfLBqieqg5S1EBqn+S/pXdV5o0x/cr9onmysBE/UTse85oHh895vFIxgdc7dYECLnqH5RrZ38zIeqCcl7VvVKKK+l+lPsHd+LrYBDQn1/wESYn80GOEqqQYD1i7dCQViseiF5RJQH7Zf8yLZibc5P/jjV72KDD2uqvmhVDwhHqG7NZgMcLr2DsG7xOg4CZwE/eCT5I4t/XfIjJ4m1yXvfJcXvCd5AB4ED2fftJmFF5yAMLl7HQdhd7AcPJH/n4j+Z/MilYm0IRoS9Hv/M4HHG4L+u+lY1TTU01EceVv0h9oxdVJeLHX6LpHVuXah6VfWp6jHVkOLXwaAsyGaBs4ytk63yedXLYtuJs4FYfxaK7fUPFc9pJAjM1rog7Fj8ycmPsErqgsB5gn928Nie2BKAAXtfLCB17Ka6Q+wZc6R16PN7vNekFeBNVUtVZ5VyHaNU12RTOVosQbi4lI8Xez5nJDCYDP5NpbyO6g2xQDmNBIG0rS4IOxS/r7z6KqkPAoOPPyZ4O4W/wdu0O6CPE6uPgzeseKyACKshexHqYnoMnFeLpXpYbyM2yN6ny1R/i+3xjm/BW5dyI0HYTuwHDyafQcO/N/kRUjTanJx8tgx8BrIdfqDl9zrMUup99QCpNN7NwQO2kunJc/jNrGyKpdY8i7S1He+JtanToaVNI0HwD+OiFtmj+H09yJfvGclnBsWO3S52l2CmOT1ibdpd2I4Uq48rxft6Y/CAIDCD62DFXZBN5VSxZ9VtU87Pqq+ymWgkCEDm8mLyDhN7EDO2HcPF2vie6twmdmHzA4yZSDvSXsdnOnt/Hb7/dxqEmclzOEzrEgBSb551Z64IcBH7TbVGrgg0FoRbVJ9L9WVXqH6Q6mWNjp8SysC/KiYmj2xjaihzhb8olIEZSEd7ku94kDoNAoOd4Qxpt0L4Vr45Z02cFR6Y8WLvY1VGrhVLHqCRewIQuXdVx5Qy2QurI+b/dNrTRu8AbCW2ZDcrZbKqX8XOGocMhkHyNgzm16pnlrfozWixd50QvOHFuyt49Iv3s39nCHxMDjKcC/T1huBdLa13rqf6TPWN2MUUSJn5P5lzolif4o7BysNjG14hNhLrwFOqR1XHVmqNeaofxdpGthebnc+J5dEjqtXL4D7CCmFrIiMZJ/avhDo4rEk7+RC2NZID9vafiodYgfuKzWb3PhabBM5s1YahXAf3IfrNv1/YRg+sVi/7P9oM1T+qt8T65s+cINY/3v2X2KCTOrODeJ/eLm1XSxi8mM93GQCulN7nV5d+Zq7YWddlgOA+Mimbqyr/A3wWKTpe29irAAAAAElFTkSuQmCC>

[image13]: <data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAABUAAAAVCAYAAACpF6WWAAAAbklEQVR4XmNgGAWjYFCAAHQBaoANQCyILkgpcAHiCnRBaoAeILZCF6QUMAPxSiCuBGJWNDkwWAjEu8nAF4D4HRAnMlAJiALxeiAWQ5cgFzAB8VYglkSXoAQEA3E0uiClAORtrJFDCdBDFxgFcAAAUUUTNyPN6gQAAAAASUVORK5CYII=>

[image14]: <data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAADwAAAAVCAYAAAAaX42MAAACtUlEQVR4Xu2XS6hOURiGX/drRCK3UIwMlCRCHQxESCllpCgxMRPJdaQkCTMpMWAi14lrp1CiFAoTHUVuA9cQEu/bt/Y5y/fvvf5/GZ72W09/+/vW3nu9a6/1rfUDtWp1e/Ugk3wwoaVkOekVrnX/OrKhs0VaM8kh8oL8IZdJG+lJLpKhod2skK/iJ7mEjL6PISvIVXLW5VLajcaX3yVjozZlGkCOky9kI5lCepNh5Ci5AXvWiNC+0MoQP+zik8k98g42iEltIg/JEdhI5RjeQ56QD+Q22UsG/9OiUTJ2i/wgU11O0tfVwJcZXhTiB11cWgjLPfaJlL4iz/AussYHm2gbrGP7fSJSG/INT4Tlfrl4UrmGdyDPsNb4e1jHJrhcLNWET8gzvASW0+xoWf9jeCc5SdphxWdt3MBpNqxT32FTN6X1pK+LVRmeQ16T+2ScyyWVa3gLuUL6h+tV5DesA2VSgcpeZ5EKwx2wLylUOxTTr4pXlnINj0fX1iFpyuor6+VlKgzf8YkWVfWFtTw08J/JYpdLSobP+WCmihEf7ROwLUO5tz6BrkLliZdIlWGpH3lDPqL5ttipHMPa5F+isdpqpNWpaS4uad3KrPLxzIil5fEK1maGy6UMS6dh+a0+USUZPu+DFSr2vWsuroOH4kNcvNBmWF7rv0odsDY5VVo6BcvrfNCSZPiCDwbNI6uj61HkARkZxfrATk+prUFbTjvskKOjqZcqs6ZlrmENcDF75rtcqbQGtF1chxWfWLqWET1sehTXdNYaK7YYTaVvKJ/OsXS0PAZ73glYodEJbRDZBxsQVXtveBnsHn+01L1nQu6AyzVoAaxqyqxuEM9gXyk+Jt6EjeDwKKZB2g7roKay1m/ZcbFKxZ+Hp7D3Poed3vTcuWRgaNfsz4N4BNsWa9Wq1U31F3MDwTgeDLfYAAAAAElFTkSuQmCC>

[image15]: <data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAC8AAAAVCAYAAADWxrdnAAACZUlEQVR4Xu2WX2iPURjHH3/mT0iWRUlpqUV2wUgLESlcLBKlVquVkgtXZiOyCxeIXCBR/pUbyg0puUCJrJRysWUmSyEliSJcbN+v5zn2vM/evbP0c+P3qc/F+Z5z3vec95zfOT+RMmX+ipnwPLwHO+D2bHUhe2E3vAOvwQ3Z6tIyHT6DO608G76A7alBAexzClZYuR32wS2pQak5Ljp4zw74QQYGNRTs9wVWW3mp6OC5gv+EXng1ZKtFB7Ei5JH7opOcY+Vlov0eWbmkcK/zZWdDXmf5/pBHxsEZrrxLtN9Bl5WMxaIv47711Fp+MeRFcOv0wktwbKZGGQOfwJ/wHZwGj8AbotuMqz0JnhT98XP1Wn/1HIKVkj/4+ZZfCXkek+EZ+AZ2iv7g8xgNV8Hb8LPoycSVJ5zER3hL9IOSzaJjqLHyIOolf/DzLL8c8uHgSn2CC2OF44Tos5e7bKtle1w2KyfLwFmxwemQL7CcLxoJU0X7dcHxoS7B041tuGKJ9JXXuiz9HnmP5MIzng14QXmWWH4g5BFO0g+C9Ij2XRfyxFHR+gkuS4Nf47I0+H0uGwRvx+shWy/asei2TC98HPJXlm8MeYL7eySDLzzxDsHncJTLWkRPBH9J8czf5spNog9/4LIp8Af8Dqtc7knb5k8HX7j6fMhDGfhSE0VXo/F3C50Yb1I+bJFl3C4vYbOVOdEL1ma3ZXmcE23jJ5c+RIPL5lp22GW5VMI20TOaD9+UqVX4hd+Ltk3wWLwJX8Onome2/9F5eKHx7OaA6FvRQfOC/GbZV3hM9D8Tb+7U9q6UKfOf0w+AQo9Pcr68WQAAAABJRU5ErkJggg==>

[image16]: <data:image/png;base64,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>

[image17]: <data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAC8AAAAVCAYAAADWxrdnAAACA0lEQVR4Xu2WP0gcQRjFvyhRwWCI4J8YLISAhUVA7FSQiF2QgBCwOtBGLCz9R/xXqUGxME3S2qWJEQQxMR5IgoiNNlZWCUpAFCIqGETfc3buPkd37xbUIO6DXzFv5mbe7M58eyKRIl2LHoAy1wzQK9AIMr02f98K2hIjbkEl4DX4Cj47fUEaBKcOK+CZGnOj6gDr4D04lnDhh8AG2AM/wDB4dGHELepAwoUfADHX/F8KG75P7nj4fjAF4uAXaNEDlHipV8E/sA2egFEwAxbBS5ALJsE8+Am6zn+ZpsKG5+RcKMdrvwEnoDoxIqkMUAfmwF/wCRR7fdzELpgFVZ7XJKYAlHvtlAobvhQ8Vm2WSj59Xl4/TYgJVaM8bppep/JYsVwvUAw/7ZohxeBc9Knb4WlcTL+uSvYpNyiPb4Vej/ICFSY8P2a/wZjj8xhx0ReOb/VOTL89apQNX688G75XeYFi+C+u6SMuxMm/OT4/UvTzHN+K5ztM+LfKCxTD8/ZfpVrQrNpFYA0UKu8h2BfzpfaTPTbphmdFS6lscAQWxFw8LbYZipNVKp9HhqWRlYTqBofif2Soj2LmKVBezPP4P8nqueeNKO+SWF+XxQTnYLIp5unpS7UE/oB85XHDfK1xMceF571C9Wtlianddo0tMaE/SHJtbpwPpB3sqLHfJVKke64zp9J9NfEgymkAAAAASUVORK5CYII=>

[image18]: <data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAEEAAAAVCAYAAAAZ6IOkAAADOElEQVR4Xu2WWchNURTHFyIyZUzEg0TEiyERmTI+iAylTIkSpZAx9D14QIZMD2aiZCxjpghRxuINiQcZSgpFkvj/rb2/u+6651xOuh6++/3qX3f/z97n7rP23msvkWqqqWpMgoZ5MyutoD3QNegONCv/cVEmQGegU9AxaFT+4//CWailN7PQHHoMzQntttAzqCJ2KMJQ6CjUMLSbQc+hiZU9Sk9r6Lw3s7JBNAiW2dB7qLbzPdw5vZ03E7ruvFIyH5rhzay8hI44bzD0E+rvfM8LaK7zxkP3nVdKbkANvJkF5gJ+7A7n9wj+Cud7rkI/oOXGY35YaNqlpD103JtZ6Sn6sduc3y34+5zvmSLajzoJbRYdk3SMaonukO/QG6gJtBY6LXqsuPvqQ1uhS9BtaMnvkemsFE3MFualCugy9EBySTuVAZIchC7BP+T8JJZKLhDfpHBSkZrQQOgC9Ek0oXInEgbjA3ROdGHIONF3dgrtJO5BdZ3HwKw3bc7noWkX0EeSg9A5+Aec7+GtwJWcKhr1GIzppo9nk2iffsbjbUJvsfHaJHgWJuS93hSdx2rnXXHtPBhl/tF253cNPidcjIuSn5l5M3wW3fLclknwNuK7bTKLq84rNxLz1TLjWTZKcoG0W3Qc6x3WLX+8rlkjcAALJUuv4HNrpcEJv4XqOD+u6nDnR9aJPrfbOAZhiPFiEGzSjdSAHonmGU9j6KDo2CjmmaI8hU44b6To4GLVH4urW94MvJL0sTz/WYKQdEMNgrZ4MzDN/GaC3yX6ng7GL4Dn54lodCOLRDO4zfKsGVijR7gTPkJNjUe4zVlocZclEY/D3wYhaTfy+KbVMFzUdqbNb2DC7mu8AjgZruiY0K4n+qLJlT00QDzrnFR34/H2YCKKSY4lN5Mpc0MaO0Xf08J4XD16o43HlaO3xniER4AVrl00C+d+WHJHhbcNF6VRZY8UuJq86vaLTnJs3lPlJvRO8leeE1kgWjSxVOZZZD5JgrmDd388p69FP56F2tfgfRG93njUOPHYl++PjBA9UmkwWc8TrTX4m9VwXLgqA7M/K9qyhef7rjfLDeaMVd4sN5h0O3rzX/gFoEC5Q1DYVT8AAAAASUVORK5CYII=>

[image19]: <data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAABMAAAAVCAYAAACkCdXRAAABBElEQVR4XmNgGAWDClgB8W4gvg/E/4H4BKo0GOwA4t8MEPlnQNyIKo0JdgLxAwaIBjNUKTAoAuL56ILYADcQXwfiBAaIYetRZCFgChA7oQtiA25APAmI2RgQrlNHVgAEpxgg8gRBNxAHQNnFDBDDQIbDgDwQb0Xi4wVngJgPyuYB4jdA/AmIBaBiaQyQMCMIxIH4EJpYGwPEdSBXgsBKIDZASOMGUUBcgyYmAcTfgPgpEPMC8Q1UadxgDgMkraGDGQwQ1y2GYqLANSBmQRcEAg0GiGEgnIgmhxV4AfF5IGZEl4CC1QwQw6TRJZCBDQMkBmE2PwBiO2QFUGAJxJfQBUcBZQAAcHcw529XylsAAAAASUVORK5CYII=>

[image20]: <data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAEUAAAAVCAYAAAAQAyPeAAADVUlEQVR4Xu2XWahNYRTHl3nIi3ko3YREyJgUKYV4cSVvZok88EQSkpAyJpQphMxjRKb7YIgySzJFhlDKFEqJ//+s77tn7XX3ufc6bng4v/rXt/7r22fvb529v722SIECBaqQalAbbxrGQCehy9A+qCiZztAC2gKVQFehycl0hprQQug8dB1aDtVLzMiyGPqZopFQ5xSf2p058g9pBRVDZ6DDLheZBt2CmoV4PvTGxKQJdEd0LmkNPYIWxAkBXvRO0eLUhY6JFqg8NosueLpPgCGiuYs+kS88CReyFvou6UVpDn2DRhmvOvQCmmm8FaK/ZZkKvYNqhXi46AJsMTsFr4/xPKtF50z0CdBPNMc/tcr5IulFmSR60q7OPwudMvEzaK+JyUDRY/uHeDv0NpvOwDvmBzTH+ZZYlHE+If+oKLyLeFI+DpZD0GfRR4B7CedsSMwQ6Rn8uSHmnXQ/my7lPXTam4b/rij0eFIu3LIn+G2hXmHMAlq6BH9riD9I2UeMcH964k1DPkUpCT4f3wvQtRAfsJMqIldR+A+mFWVX8DtCA8LYFyXuF9xYCcdpRXkFvfSmIRalPPmi8MUwQfStSvaLPqa81kqTqyjHJb0oXCj99lDfMPZFYcHocy8hXyW9KCzIU28a8rlT7plx3ODnGa9SsChHvAl2iP5gS+fz1Uq/MdQhjNclZmR7iVUhfg7dzaZLeS3as+Qin6LEc7JV4Hlvi27qv0WuosQL8o0d59Ln65kn5piNm6V38OM/dEO0d/F8lLKLsuRTlMhB0Xx8A3Y3uQphUY56U/RC+KM9nM9m6ZKJH4pegGWo6LHDQswmjG8aS33ROUudb4lFGe98Ul5R2PkytybEvFNKsumKYVHYXXoaQp+gscarLbqrzzDeIuiBZDc2wuaOj0Zs3gaJXmRR6Qxt2tKKbsmneWsqulc9hhoEr52k75up1BHtWs9JclGRYtFvnhoh5jfNTdEeJcIx7xzOJfye4d0zunSGwm+dlSbmnuX7G8960YWzQ/bEBtF/KpwIPvMRdt2bTJwKD7giWhD+AMV+gVWP1Y0MhpaJLoK3Y6NkOgO92dA2aCM0IpFVuAdNEX1T8XGaFbw08v0g7BbGbC65For7Gb0lUqBAgQJ/gV+CFQTU1DHaVgAAAABJRU5ErkJggg==>

[image21]: <data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAACYAAAAVCAYAAAAq05ytAAABxklEQVR4Xu2VTSgFURTHj6TYUFjYsCIRGx+FktixYkEpZaVkIfkoC4qyJAuUzxAlyUJWPsoCC4WForCxk7KzshD/49w7787pzWvqPRu9X/1q7v/euXNm3rwzREn+IXlwDZ7BK9jjnw5FFszWYTzkwjvYZ8b58BlO2AUxSIFFJDfyAAf90/ExQ1KYSy98h2kq1+zCC7gFvynBhb2QXMClieRC9SoPooYSXBi/W7zhksorTT6m8iASXlgVyYbzKi83+brKgwhbWA7Ju3sCb+AhPHAXWBooemGlJt9WeRBhCxuH0864Hd46Y49ail5Yick3VR5E2ML4KU2p7FSNfykm2XBB5WUmn1V5ELawIT2hWCVZx71yD3b4pyNwD+OF3Fxdqk3Ojz4MYQvjJmxbi3XOt8LhCe6rrJnkpBaVB2ELG9YTim7nmP9gKyTnFTq5B//mjyRd3DICX8nfYLmndTpjF1sYnxcLfggFzpj3/4R1TuaRDi9hqxlnkGzQ5a2Qoj9ILl7h5JZGkrlJPaHgfXdgqhlzu+IvTKa3QsEf31G4AZdhm29WOIdv5P9QD8B7+EWRd+YaLjprXI5gPzw2x/zFiXajSZL8KT8OVGS4yBqRmgAAAABJRU5ErkJggg==>

[image22]: <data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAC0AAAAVCAYAAADSM2daAAACd0lEQVR4Xu2WS8hNURiGX5eEKKEUA7dEUmQgcvlPyIhyGaAM3OlHuQxMlAxE5JIBA8rEpSTKwG2gH6EwkLkJkUu5JkXifc+31jlrf//a55wMdZ566t/v+vbe37/23msdoE2bpoyjm+iAJJtDjybHjZhBr9C7tAt2bo6+9I9T56bsT8aikwoVgbnoXviezk6LSphOX9Np4Xg+/QG7ZhlD6TvYfR7TXsVhTKFP6WE6xI3VmEdf0Tf0OT1NxxcqynlCT7jsPL3mshQ1fYNehTW+rThc5TiaTJpm5YwPW0CvlW661eV76Vfa2+WR2PRo+p1+oiMKFS00XcG/Nb0c1vQql+8IeYfLI3rk18PfG2G1l+rDVdR02bdRpUIv0iOwGfhAj9EeSU2OnbAbqvmULSFf6/JInOnIHVj9kiRrOtP6gl/SMeF4LP1Md9cq8uxBvunNIdeKlMM3PYH+pG/p4JCp6Vm1igwD6UiXnaNfYDcoQ/9Urmk1q3ydyyO+aXEIds7ZcNy06RwHYRdZ4QcSNiBf0xnypS6PqOmbLutHX8DOU7MNm9Z720Uf0Z5JHh99bjmKLIPVrHb5rpBXXB7JNS0WwM57BltGZxaH6/Shv2FLT/8k10epCyxMMs8oWM12lx+AbTCDXB5R07d8GLgAu6be79KZFtqVJrtM75zWz/hhCH3NK5Njoa1bm1GKNpbLLktR07d9GBgGW73ia1LKInoS9ZnW+viLrq9V2Gv0DXaxqUmuDULv4vBwPJF+ROMdtQO2WmnV8lu4WIMWmhb6mLRe3qMP6OLicJX7sN8M6ewLLVn7YGv9KXR/apHcDyadk+MhWmi6TZv/gb9hno7PjT8lwQAAAABJRU5ErkJggg==>

[image23]: <data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAEQAAAAVCAYAAAD/wUjgAAAC2UlEQVR4Xu2XWahNURzGPzJkjCdToTyYSl6MoWN8QKHwwAuKi0xRihCZh4QoRI4hEiJDpvIgZJ5DCveFJy8IyQPfd///be293Xuui4OO89Wv1rj3Od/6r/9aGyiqqKKKKipv6knOkxfkC7ma7C7TGfIZ1v+KLEl2F6bOklLYn+6a7CrTbLIr3Vioqkcek3EwQ44mek2bSb90Y6FqENlEaiFESdv4AOo6rP+/0Foy3MtzYIbIoEityKlYveB1kzT0cn3ymrwljbxtEiyH/IxqkC6kvdf1/N6k2bcR/5iakIupthWwKFG0SAdJ59BdJckMnWJZr48kT8kEr8/0/lw6TBalG/OlMWRBqq0p+UBekgbkSbK7ypqKYIh0AMGQbrFyRRqNP5jQd8DuImlthUXJXudXVIKkIftQuQl/TY9g+zytdjBDxPhUX2tyDpaM9yDkg7HkGFkFO6Ybe7tyUNbLkgyWIYo+zY8ug83JBrIMNn4Y6UROkC0+phrsvTthiX+et2tRH8LmrUR4TpU0mNyBvaQ8HYIZ0iLWVpPcJhmv74Y9Rz/8vvdLyg1RZFVkiKR5z708F3YXkjJkqJenIcyfQrZ7WVJ+0fOlWbD8VNvrz2ALW6l6wU6WKAJKSZ/4AFcP2J+MSyvxiVRPtSsqtGqRlEzfwcblMkSnT2TICPKeHCczECI3vuUuI5gmKfFHV4LpZH+sT9HfPVbPizLkI76PqvVI5pqBsO8frVYuQ7SCkSEtYZfEheQewikXn3+DTPSytJRc8LKSd/w3yJDy8uNvlVZcL+rrdd1fdJTqFLiFYNRGhNXKdcp0QDBE+76jl4eQxV6ejDBfbRonaXteg20jSVGVNkS7Ie9qQ67AomI+qePty8lpcgRmiKJDH4r6Wn4A2xKjYN9NJ8kAsg22TWTQOrIGFiGrYUk2mn+X9Cd1Yd9aWXIJtp0kjVOilwl6h/LOG1iOU+QV9aP6CtSom7YGzRNDAAAAAElFTkSuQmCC>

[image24]: <data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAACsAAAASCAYAAADCKCelAAACV0lEQVR4Xu2WTYhPURjGHx9hoeRrhiKEoiRZm5osTFZESsl3JIWVsmEjsxFmhqwmJeQrH4kIkc8F8rGws8KCUooiSTzPvO9759wz159uWaj51dP/vs977rnv/5xzz7lAP/2UWEg9o+5Ql6kt1MBSC2MldZV6SJ2hJpXTBXuonxVaSs2q8KWTPXf+gTnUfWqEx/NhNx8sWhibqedUk8e7qHdJXEU3rK+teYK0wXJ69l9zCHbTco8HUJ+pL9Rw95qpr9Qyj4VG/g21PfFyOmB9r8sTZB4sdyNPNGI39YNa4rGK/QbrKIpd7/Fsj4Ob1LXMS4liV+cJ1CxWTEyutSzUiQoJYvTTduI8bBaGZX7wT4oNtG4vUY+psYl/AdbxuMQTp9yfmvlBnWJvu/+Bukc98bjEDuoFbK22llO4jupiT7g/M/ODKLaR8mL1Eq+FLUdxFrZMK1kB6yR9KbSdVRV73P3pmR/UGdmXyfUiWJudidcHLQO9/TM8Pga7aXzRwtDeKH905gd1ij3gv2Oo17DZHhzJCehbxBFYR+0ex0OnFC2Mi+5XHSCiTrHBOVi+JQxN63fYKOqfBEdhDfd7rIcpnlu0MLShP8i8lCh2TeaLRsXqpFOuy+OekZ3s5keUT6K77mvNiJHUJ2pV0QIYAntjtyVeTp1DQbvQW+oVevf5aZE8DTsWY11shHWiKU6ndzHsm2CQxxtg3xO/22PFYVhfm/IEeo/1W5l/xX3lg31xMZTqpN5Tj2B7mwqOolIWUHthL5ymaFQ5XVD3QyYOJB00GnHpqXv/D78A7Um52UYwg3gAAAAASUVORK5CYII=>

[image25]: <data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAGsAAAAVCAYAAABBlxC9AAAEAklEQVR4Xu2YaahNURTHl8xTIXPoUeYpZU48EpkyFiKeeUyGTKUkH2RMwgsflJIxRcoHQ888z+WF5IMhRRQfFIn1t/a+b539zr733HteJOdX/7rnv9fbb5+9zl57n0OUkJCQ8N+zmvUzjW6yeqSiSyjP+kGl433axaoS4vu0kf4evVgnWBdZRay+gdb0VGatYF1gnWVtIpkrTZz+f7OTZJKmm+u6rE4kyfzOWml8Sz2S+EWs6sbrarx7NoipSTKww8rLI4l7qTxLZ9Z91n634Q/Rk/WG1d1cD2R9ZQ1IRfjBvSJJB0gSVJ/1nDVXxcTpPwWeZEzgZLeBeUzS1k55rVhX1DXoQBJ3zfG7sM6pazwIiCtWnmYQ64xr5ggmbRSrmdvg4TZrh+MdZJ1yvDCwij6QrC4wnOQ+96Qi4vWfwiZrktvAnCZpW6683qxCdQ18yQJ6tWVKVgXWLdfMkkqsOSSTs5bVINgcSkuScS10fPz9Z5Jx+ahDskJ2Kw8VBxUL8wLi9B8gXbKuk7T1V94QCiYPYOX5koX6bbHJeqo8MFX9LlK/s6Eiax7rAck9NQw2p2U8hVeXJcbv5/gajB0x2BZ8xOk/gC9ZY42POpyJdMnS2GQ9c/y76jdKZzag3M1i3SHZX2sEmyOxlGRcmFTNAuPb/TyMrSQxs1knWZdYx0n2cUuc/gPYZD1knSfZGL+x3rFmsMqVhHppQ7knqz3JQSZbMK5xJGUT94BylCtrKHwycUCAj7LqYz9JDKpFW+NNISlvtgzG6T9A2MrKZ32k6BnPNllhyobBJKevzSQnr7hgRYZNJiYRPh5aH4dIYrYpDw/Se9Yxcx2n/wBhyQJbjK+Xs49sk6VXVjfWF3WdidokqwnlOcrhIQoooxjXBMefb/wxjq/BwQIx7oP9hKRCoSzH6T+AL1m2nuonxkecZIHLznUm8OROZD0iGV/cpNn9ucDxlxk/3/E160liRjg+xga/EcXrP4AvWTONjxfbTMRNVq7gBFjAusHaTjIxuZBHMq7Fjr+B5Fhey/E1NhHYPzXFxscxPs/8zqX/AL5k2eMm/qlFvxxrsk3WC7ehDBhN8gUE7zdNnLYo4BPQPsfDCytOdhocv/GuaanG+sRapTysfLwkY1+1RO0/LSgjYUsUb+M44aCtBasq60ggogR8nkIcjs/paEwS99ZtKCNwjMfegRPtXpInOirNSR4ijBHgwcQhq3UqQj4ZYfxIjmYaySe0pua6gPWa1dEGULT+vfg+5I5UMX1YR0mScJU1VLWBV1T6762wYi3pPuQOU3FlBcojkoaDSJQDkgUVYh3JCa+Q5HulBnsjVox+0bdgj0cy8FKO9rxAq5Cp/4SEhISEhH+aX9X4Q7GaUrDEAAAAAElFTkSuQmCC>

[image26]: <data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAADIAAAAVCAYAAAAElr0/AAAC9ElEQVR4Xu2WWaiOURSGl4ODzJQpQ6YiQ5IpNy6UcqHcoEynTBfKkDJd4CSUuJDhQpQLR6YyRBlTyixThnAhoZxQxIVciPc9a63/33uf71NyztFf56m3/vXuvf9vrz2s7xNppJEGYTR0HLoCnbM4ZY5o2w3oGNQnbq5FGbQWugM9hpbFzXXPNKgaGmvxCugd1LTQQ2Qx9BDqYvF60TEeZ8E+r6Cu0CPoe9xct3SHvkJrAu8m9AvqZTEnwkkwYYer/RZaGXghTaAPojtCpojuaL2xVXTSPQJvMrQhiOeL9hkeeOQydD7xnOaiY5amDfXFC+hjaibslniHnBPQN6hl4hN6DZZIB9GHPYFmQldFjxWPA1fUOWn9ugUeOWJ+/8SvMD/UZtHj+8liXvxtokWDC8lC05ODjc5QJXQJugedgU4H7RGcAP+Ul3YP1AxqD92FDgT9Llq/NJFD5g9OfOI7si7wBkFLzGcV84n3hV5Dz6A25nHcdvtNeD/vB3HEUNE//Sm6Ag5Xi/4Ii89anCZSZf7AxCdZiRB/ZnrkFpi/2mLuwqZicw28k5nwgnMwS2TILPO3WHzQYla4kMPmh4vg5CUyxPw0Ed4/+lw0st/i26LHbrr5mZSLdr6V+DPM53EjOyzmEQg5ZT5LcUpeIjxeWYnwnUXfjw+PuC+ga5e1ZcKBDxJvtuhA35EKi0cWeijXoOuJ57SSv0ukn/m83ITPdIZB+0TbBwR+BCsUX4hhlVouOmiixR1F+8wt9NDdZAXK++xoLX9OJH2RLjKfXxXkJdS72Fwzvx/Q+MCL4Mqx/FaKvo1Zklm1uI2Mnami31j+2bJQdCez3iGEny6c2MbE90Q+Q+PM44uWJZj3oYV5TIR30J83SnTh2lmcCasOS+lz6I3kr/Ik0drPs7sT6hQ3F5gHvRedMCsiPxonWJsnshd6KroYLP+sUG2tD7kgevxY+vn7qNQ+2v+VvDtScuSV35JjjGgi/lVckqyCvkjx7vA99M/8BnD9vVZB92nxAAAAAElFTkSuQmCC>

[image27]: <data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAACQAAAASCAYAAAAzI3woAAABrElEQVR4Xu2VyytFURSHlzxCBlJkqqTIozDC4JYpkUeUMlAoysgzYSSvMlIYGRiIAZGRGMlAKf4BI+U1QMqACb/fXXufc+xkdM8Z3a++2mevfe9eZz/WEUmSJFw64SH8hF/wAq7AzOCgKKiA53AZdsNmOAtf4Tfch6ne6JDpgTew1A2AIvghmlSjEwuFdvgMi91AgEXRhMbcQKLJhfdwzg04MGkmtOEGEs2U6ET/rQ5pkYhW6Fh0ogI34DApOo4HP1R4vTlRhxtw4IE/cjvDYFg0oRfYB/NgLRwV/0Zxu95hmXkOlRS4KpqUlVecfTmwBj6K1qRIqYQxWAezRBPlil3Cen9YnHx4Aq/hBJyH26IvQJrgOpwxsWzTPwLf4JnoWFb/B/njZbkVMWMvXBM9MzzIGd4ohUWSVIuuZol53oIDpt0Ph0yb5YSV37IgfulogF2BWJxB+b1dlFlzsMuukVSJ1i8LJxk3bX5eWkVLygHcsYNEf/cE00RXMD0Q8+Af8CBvin5MbWKnsA0Wim7brfj1qhzemTZhQlxRwpu7ZNpctT3TtlyJXpRp2/EDDjxW+LmBhBAAAAAASUVORK5CYII=>

[image28]: <data:image/png;base64,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>

[image29]: <data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAD8AAAAVCAYAAADxaDaPAAAChUlEQVR4Xu2XS4jNYRjGHxpJkliQshrksrCR62I6jaRZkkuyQhEWEgvXDHKLTOMSCrkUIbcVJYlMLJAoYzYWCoWFkoTE8/S+X+d/3kkz54wzMec89av3+97T9/+e7/L+/weoqqqqqqpQTSO3yHvyi3wlj8jJ7I96uq7CzE+IiZ6u3rCdfxkTlaApsF0/GBOVoE0w87NjohJ0j3wnA2PiP1ENmUjGxkRHkuGfsAUol+pDexXsDfO3JOMa71To71A66jry22LCtYYMjZ1F6kJoTyaLQ19XtQIlmD8EMz89JqjBpCV2FqFesMV7EhNl0DKUYL6VfCP9Q78mfpqszPQtJNfIbtiiDSIzSBu5SzaQZnIbtnC15Dh5QzaTBjKAnCEPYVpEPpItZDu5TlbDnqXTqDloLGkUOUfWkf1kpPdLS1Gk+dGwXY/3rx85QD4jXwTHk2ekj7d1b896rIlr0kmHYYsj5dB+5zXWq0z7MmxBpBGwOU3y9j6y1mNdl6Meq4488FjqtPk6mOGnsAe99bYm+cn7hFY5Sbt9ItNWkdHi6ANpI/JmpTnktcc5tDevqpw1f5Es93g4+ZHJ7XCS9Dmuq9RE3mX6O22+FOmYpZ2WdNw1yb5kPQrNL4AddSmHvPlU9ceg0Px52OQlmddrN0nGd3msE6BN0hXVGPoqTSqreU38MawWSLrb6WTI/BWPJR1h5SX9V3jhse60NA6F5i/hz+Z3kj0e3yFLPJ5KPsAK3TCUWO2LkXbhBuyOypx2XZJ5FTD9EzwC28lUQLVYqgF7yTwyhBwjX2Cvu7mwgqlx9cbRcdaV2+rtFtjdnknmk/uw4qmiqGc2wurDTfKczEI3Kx77f1a/AQrbid3mXnqyAAAAAElFTkSuQmCC>

[image30]: <data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAADUAAAAVCAYAAADmSqZGAAACN0lEQVR4Xu2WTUhVURSFV0KQBgWllBAhWAYhOHASkmkUDiQnFWQ0qUmQoQ0CySikaBAEDvqhSQkqNepnUkQpDgJDJKqRhQYNEoWaBA2kImwt9r743Ci+9zDo0lvwwbl7n/uzztnnnAsUVFBBBeWoOjJIvpA5Mktek97MTmnVY5ip2phIq4pgM/UhJtKsXbBZuhETadYFmKmDMZFmvSQ/yfqY+Ee0mTSS4hBfUjLyG2ZspVVOdsZgHuokv8i2mFhKKjmV3uWYcJ0lm2IwSx0hh2IwT80gB1M3Yab2xQS1gYzEYJaqIO+wcqamkIOp9+QHWRviq0gfOe3Xa8gAOUeukiaPSxXkBblG+mFld4JMkHvkIimB3acBPEnKyBhsLSc6730ukTMZcekzsjS1A/YS/VFkSgvyOvmO+c1jHXlASj3/1duryRvYQpY0EM3e1nPjTD2EmZK2YqEpHSm7vT1EDmTkljW1B/ZClYdMTfu1Pu6bx8T95AaXHtoGOwLUrxr2m6WZ1gEetZgpPTMxtQULTancj5Mu8hb2rkTLmspH9eQj2e7XekkNbIb0v6hyjUpM6eOrPKYSXszURjJJWvxa/dq9Lf0VU9odVVqSSk4z1UqOknGy13Mq08PefkSOkf2YL8875JS3dY+OEw2I+nzyuPSEdMDWl5TT7petVGqj5ArM4F1YKWmEK8kr0gNb7MkhqdJUPxnRQEgN5Cls8+iGlfkwbCN5Rm7DjCj33PvrnFKJ34KV6P+hP5OHcWOOPpFOAAAAAElFTkSuQmCC>

[image31]: <data:image/png;base64,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>

[image32]: <data:image/png;base64,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>

[image33]: <data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAHYAAAAVCAYAAACTxxrlAAAESElEQVR4Xu2ZaahVVRTHV1bOlKFJZohBafjFIdHCSqmEhIpyLg0ltSQVG1DLymejDaZRgZifHErI1BQTccj6EmLhDAZRCKF+CcE+1IcIWz/X3t51t/e9d+7R+wY5f/jD2/+zz+KevfZew34iBQoUKFCgBWK58oTykPIT5cCyp60fdys3KX9Qfq+8r+xp/bhFuV15VnlcuUTZoWyGIa/9mmKocpmynXKO2EeMLpvRunGX8qRySBg/qPxHef+FGZXRWfmLcrqyq3KB8pxyq58k+e3XHOyub1LxCsJPYlHI4wu52EEpFinfSbT1Ys59xGl57dcc7MwWscNqgNvFHDE70euUfymvSXSPbcozynudNknM3uowvhT7TQJ24gnldYne2jFBbOEnJ/oLQR+e6B6bxeY87bTHgvZVGF+K/ZqDXbVU7Ifg4CsJL4p9Fw7wmBV077QU3ZVPKNs47TWx994I42rtX638Wfmv8rTyBuX7YmF7r1jU7KT8VLlT+aNYbq8aA5RHlYuVX4r9mGF+QhPjoNhvyMq37bV6ER2RLvzMoD+b6A0BpxxT/qnsEbRq7bNJRih3iIVqTv5N4RkOJvR/qxwctDFidvqGcSbcpjylHBvGT4oZqbswo/UjVrLpwrPg6NMSvSFQHXPSqHoj8tqnveT5PU4bH7T5TutZQWsUFAf0rVeFMSU9RujV8uB65cdiuenD5FlzYYbYN01M9OeCnrWtu0OsDaR48shr/yOx5xSuEfF0jnQapxntFac1iF5iL8x1GoUTGvkhD95SPqN8SKr4ITVGXKypif5S0EckeiWQBwnBlU5fXvsfiD1v77Ro6wGnRccudFqDGCX2QozlgNsmtEFOqwZr5OKQVC2qzbFv2mv1orfYvOcTnahEi9cl0VNcK1bEzHMajqbHBb0ln33yKe9ldSy5PBOGir1AHxbBKTvixoQTTiEV4JagPaxcoXxdrHnvGHQ2ygHlRrFKkEqbyu4zsXwSb2WaA1zzrUo0qtCv3ZhrQipccpoHqYXv9eBCx6eaLPZTxFCc1bGsdyaQV7kxIWQACqk/pHRJwU7dL6UGO34IOQWHgzqxkBKxVkrlPXbfDX9j+9fwd3PgVuVvypvDuJ9Y9ekrTYoTFpCNGUFvirYn8LBYi/KfWMqJyGI/xeditm902pSgPeo01g7tPac1ij7Kfcrdyt/Fmu8IHE+/xcdw8u4MOiU/8zjdFEm+7/WOpf9ikdhpcJdUvjxvKlD8EHn4vZzA/uWPz0ccHBZDLGBtWNRK9KcKNGY/oq3Y2kQ7dCU4dKVY6Eb7W+xegQNEaxXnfieXATiQnfeymGPZqTiGkEx+AOzaeAMD1knJsVTb9HIFWhi6SflpJI/EqjnmZXq4DVK6heHiOzr2VSnPMcxt58YFmgk4lhDNzQ5OijcoxHpO7WKxVolw/ZRynNj/KykaHhc78YQTHE++5XmBAgXy4H/vcypSZmV/tQAAAABJRU5ErkJggg==>

[image34]: <data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAF0AAAAVCAYAAAAkeuLCAAADdElEQVR4Xu2YWahNURjHP7OQMTNFEfGgJJnimvKCQiHdXJHIUIYyZbiGMlM8mPNgKklSSoir8GAIT56kjCURipD4/3173fPtz9nnHA/OlbN/9X/Y/7XO2md/e61vfWuLpKSkpJQMA6Gz0HWoChoaa02mAbQfeg09g45DnWI9RMZC46E60XUtaBY0t7pHCTIAegH1j65HQZ+hEdU9kjkF7YXaQhOg99ATqKnpUwn9cLoNdTR9So470B7nnYDOO88zXHRlcOYG5okGdbvx1kOPoHfQTWgz1MS0lxzdRYO0wPnroA9QXedblkNfocXGY2rheJztAY5VYa5LnimiQSp3PgNJf5jzLUtE+xw1XovIY44PrJF/MOjToBuis+Yb1C7e/FcJgWPwLfMjf6bzLfVEg2nzN18Sf3fVeAz6WuiY6CbNDTdpXG62d0Xj8Er0JW4VTXXXRPeZxqL7yCXoluiKK5ja0GHRnDpGci/lwH35fVPKpU36s0RWS/ags7KgP8f5+WAlw9/xeQIMCgPUMLqeDH2HBlf3yMCYlEEXRdPbaclMQgb/LXQB6hd5k0Tv1yO6zssK0YG7+oYiwoBkCzqDTZ+lXaH0FA0mZ7alM9TMXHPj5WznpprEbtH7DzEeXxa9ZcZjBeS9RPjWuZv7mTnOdioCs0XvO9X5oQqZ6PwkGkEPJZ7fc8GAc/z2viFip2i7rXLCrB5tPK4CeiuNlwhnBTuznKpJwoPMcP7SyC9zfjY4c09CRyRzAApwFT+Hdjif6Ybj93F+YJtoe0hJJPzXkcYLQV9lvER4EGHnhb4hD3+a0zfozxLpItpvkfNZS/OA1Nz52aiEzonm4wDTA2GAOP4V00Z4OKJvN2EL8zfbCw0696a8tBLNf3yjNQ0POIecx2rhjPNYVg5yHgPBALKiCNQXHZPwpMq00ybT/Kvq+QhdNp4npJdCg+73kUT2QU8lM5taQtMzzUWDKeAx1CG67iVaJdiKgJ8K+HDchwK9RY/990QDyFKPz/NFtDwMMLWwRAwrgQXEJ0lOLeSg6P1aG68i8vgdJ9At8rYYLycsETdCD0S/YeyC+sZ6FA/uMdxf+D84GXxAOGPfSHx28qgf0piX3av4UYzLv0p0VTCf84Vlg6uEtXcY56VosA+Ipjt6fGF8kdzs+Z9CX3s2SElJSUn5j/kJoYTkwda7zWAAAAAASUVORK5CYII=>

[image35]: <data:image/png;base64,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>

[image36]: <data:image/png;base64,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>

[image37]: <data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAFcAAAAVCAYAAAAzWHILAAADTklEQVR4Xu1YaahNURReZqLMY4ZnKv5IfpBCQsofY6TEMysZf5ASkqGkZPxlKFOmMpdZmYsypExliBRSiiJJfN9b+1zrrnvPfe/JfU/3na++uuvba+/OWXudtfa+IgkSJEhQsJgIngFvgofBDunDsRgG3gOvgKfBuWD1NA+RmWAvYzcA14F9jFawmA3eB1sEezn4zthx6AleBxsGexD4C9yS8lBcCrrlHrC2dSpEtAS/gWONxsx7Ay4yWjZsFQ3U+GBXA7+AX0WzM8Jl0ez+BJ4D55mxgsY00QD1cPpF8KzTPFaBP8HRwWZwv4uuZ4PLtYqMXWUQZV87px8VzcK6Tvew81gmuBaDaXFBqmhwj4kGpJXTDwa9s9PjwLp7ErwDNndjDO4U8Lhobb8L9k3zyIHh4FXRReykfuATsJ7oJ8OOesKMlxcMgG8MpXFoycx4nJfswd0f9O5Oz4Yl4APRWjswfagEPEWsNjYb3iuwidGygru0N/zmMWafGdsBvjY2s4Q1ioH+X8AXzxZcvgf1rk7PhQmic6Y63ddzbhj91jg9A4vB/qKfDycsNGMvwZ3G7gI+NTaD3BscYLSKBhODz93a6QeC3tTppYFlgaePbn7AoL7o2g/9QBy4C5zQJtjccdqTIwfRDYiynOWDgX8mmTtdkdgo+pwdnc76SN1fCCzaSuam7BKdtzbYPEm8B8elPDSp6PPBaDnBcxxvOBFmiS7QyWjTwUnGJlinyxrcv6m5Q0pmxqNY1M/eoAheDm44zYLP8kM0S5sZfbfoehuCvTLYK1IeIo2Cdstosagh6rzMaMyIz8amD7spm5tFeYKbDzQWfU676bw5fQTnG629aMmrE+wi0XfmxcDe5NjYqY8I9ijR+l0r5aGNnj68CZYJj8Ft4Tcf+JHoAjwH8jNYCo4J4xaVHVxipGgzZgIQM0S/RHvGPSX6PguMdki0adcMNv9DoA9LSlRO+O68jET/I1BnE2W8fKLFgk3phehVj8cSPvAR8Lno5zXnj2saGFzekiobPLKtF+0JmyXzmMQaypPOYKMxizeJ1tTb4DXRAEebFIF1ebvofwxsePxfgQmYdzC4rMUJ8gAGl59hgn8IHqzZAN+KnjJsM0yQIL/4DXU6xkfSa0IDAAAAAElFTkSuQmCC>

[image38]: <data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAFcAAAAVCAYAAAAzWHILAAADcklEQVR4Xu1YWahOURReZqLMY7i/qXgRXqQM15DyYoyUzFPJ+EBKSIZCZMyDoUyZypyZMhdlSBkeDPGClKJIEt931z6/9a/7n3vdm+vqv+err8769tA5a6+99tpHJEGCBAlyFmPBs+At8DCYl9kci0HgffAqeBqcBVbO6CEyDexm7DrgarC70XIWM8AHYJNgLwHfGjsOXcAbYN1g9wN/gpvTPRSXg265B6xuO+UimoJfwZFGY+S9AecbLRu2iDpqdLArgZ/BL6LRGeGKaHR/BM+Ds01bTmOyqIM6O/0SeM5pHsvBH+DwYNO530Tns87lXCljVxhE0dfK6UdFo7Cm0z3sOKYJzkVnWlyUCurcY6IOaeb0g0Fv5/Q4MO+eBO+CjV0bnTsRPC6a2++BPTJ6FIHB4DXRSeygnuBTsJboluGJesK0lxR0gD8YiuPAgpHxuCDZnbs/6J2cng0LwYeiuTY/s6kArCJWGJsH3iuwgdGygqu0NzyzjNln2naAr43NKGGOoqP/F/DDszmX30G9g9OLwhjRMZOc7vM5F4z9Vjq9EBaAvUS3DwfMM20vwZ3Gbg8+C8/1RJ3PBeEHluQj/iYYGHzv5k4/EPSGTi8OTAusPjr6BoPaonM/8g1x4CpwQItg01m0J0QdRBcgivJtonUhwQWi08uj7tsg+p5tnM78SN1fCCxaSuFF2SU6blWwWUm8A0ele+jOZZ/3RisSrON4w4kwXXSCtkabAo4Lz2fANeE5Jdq3T7DjUJqcO6BgZDzGi/azNyiCl4ObTrPgu3wXjdJGRt8tOt/6YC8L9tJ0D9211G4bLRZVRDsvNhoj4pOx2YenKQ83j3zR8T56/gXqi75ntOgEd9AHcI7RWoumvBrBTom+My8G9ibHg536kGAPE83f1dI99KBnH94E/whPwK3hmS/8WHQC1oHcBovAEaHdg2UPV7y8MFQ09zMAiKmiO9HWuKdEv2eu0Q6JnhtVg81/COzDlBKlE347LyPRfwTqPGPor2yBlhW9wReiVz2WJXzhI+Bz0e0183fXDDBi6NjiivWyBku2taJnwiYpXCYxh7LS6W80RvFG0Zx6B7wu6uBokSIwL28X/cfAA4//FRiAZYqu4DrR1WVuzstsTlBasMRhlPQVzbmsNsoj5+YkeFNjforIU9dvpwQJyga/AEZXzJSSb92ZAAAAAElFTkSuQmCC>

[image39]: <data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAFcAAAAVCAYAAAAzWHILAAADAUlEQVR4Xu2YWahNURjHP0OGUjIP4RrDA8mbQkKihAylZOji3pLxwRCREi8iY15QQqYyi5AyPvCAF8MDCg+UEook8f+f79vnfmfdM9x79u0eR/tfv1rff6393b3X+s7aa1+RRIkSJfpvNRdcBQ/AaVCR2Z1Tk8BjcBtcBstA04wRqmLzl72WgCegs8WbwAcX59IwcA+0tXgs+AP2pkeois1f9uoCfoBZzmPlvQOrnZdN+0Qnc7bFTcA38B20MS9O/rLXQtEJGhr4N8G1wAu1BfwG0y3m5P4UzRdNbpz8Za+o+noG/lnRKmwV+KH8ddwmmIsTFylu/rLWOdGH7xr4J83vF/i5xH33IngEOjk/dv4p4A64AUY4fyR4AVqL/mT4Rr3g+usr3iBvqD5MSF2ZW9cl+8MfN39w4GfTOvBUdK8dk9kVLz9X6ai1ecw45voOgrcu5ipyj+JE/yvi8Snbw/M56A8I/HyaI3pNpfNi5V8DRomWNwevcn1vwCEX9wcvrc1q3g12im7srP5SiIXB++4W+CfM7xD4hcRtgaeDQRY3SP6tooO7W8wVYbwgGiC6AFGVrwcHrL1cdCFKUdG7RO+zT+CfNz/bB0GkHlJ70g6LXrfN4jj50+JXCr9AIlWLXtzXeYvAPGvzjHfF2jyufJHCk1vMnjs+dWVuzRcdNzzw+XFwP/C8eC+/RKu0o/OPiObjL5IqNn9azUQTbHQeV+yrizmGb1NuB6G4iv7axlQ70fuMFp1qAT6BFc7rJbrltbS4t+gzf5bMLy2+2OlPtbiu+fPqOdhvbSZ8JvpHeL5jRW4AM6w/0hDRFeYpItpOSqFpoi9jFgC1WPSX6M+gl0SfZ6XzTom+tJtbXCU6hsXif+51yZ9Xo8FrcEv0WMKEZ8Ar0fJfWjO0lmaC91LaCeaRbbvoO2EPaJ/ZndpDedIZ5zxWMV/KH8FDcFd0gqNJ9CqUv0HFyZ5o7YGiK+6/vxPFEI9ka609WbQqKmq6E8UR/z23A2wWPb7w/6KJEjWO/gJP4tE+KwpGfQAAAABJRU5ErkJggg==>

[image40]: <data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAE0AAAAVCAYAAAAD1GMqAAACZ0lEQVR4Xu2YS0hVURSG/wh7DCuDCHJgRRRWRC+EVKgIahBF4CgioiiiQQ2KogcFKTQIRWpQFBY1EYnQoAZBUdDDkdSgJ6XUQAeNbBaE/uuuvbv7LI91TmhXT+eHD+7+1zrbw+++++xzgVy5cuX6Z6ogk635G80h18lj0kX2RcvZ1RSygpwl38iqSHVklZPX5KAbzyMfofNkXl/JffKQDCJ5aBehoYU6AA2+zPiZ1TGkC62XtBlvPXSOGuNnVmlCk71Meq8Yf6XzTxk/s0oTmvRI7yXjL3V+q/GHaSt5Ct0TqgN/HXlHppNJ5AnpCOpp5f+7adhUuDKZ0oRWh/jQljj/tvEjmk1uuc/PEW2+Rr4E47vkJzTA8ag0ocniiAttsfNvGj8i+UOy6c2HNh8Jaj3QM4zXAvI+GHudI3usWQL50FbbQowWQXsvG7/K+U3Gj1UDtHmuGy90492+ARqsX5Vey8l3TLzQ5IwmveGiEMm14p82fqy6yYNgvB96cWXg7SW7grGohdxDstD+Zk/bWLgymXxoa2xhBH0gd4y3GTrHFuMPk7x22HSbyUAwlp5O6EPBq54sQ/LQxlo+tLW2AH29kq1nauCdh2434R59lPQh4eH2LYrf7xnkDfQG5NVCJj1Jdri6aBY54T6Pl9DOQO95gy1A71FqhwNvGnlGtrmxLAhZfTt/dfxBteQzeUReQSdqJ5+gEx8qthZ0AcVVV+rQbkBP9/4r/QN6PPLvlKJG6JPfBjqTHIfOcZVsj1RHUXJMaYV+nQV5yZW9QfaDXAn1AqVdaRNK8pOMrLR+8hIJnji5/lMNAZ98l4SND0HzAAAAAElFTkSuQmCC>

[image41]: <data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAACwAAAASCAYAAAAg9DzcAAAA40lEQVR4XmNgGAWjYPgAQyCeAMR7gPgNEP/Hgs8AMQtMw0CCUiA+BMSJQOwIxA5AXMcAcWQMlA/CrGDVAwzMgfgxEKsgiTEC8WUg3ockNmiADBDzook5M0BCNxdNfNCC6QwQB8ujSwxWcAuIz6MLEgAg9eiZFB9ugWijHEgwQAzsQJcYrCCKAeJgUDoeEmAeEH8FYnZ0icEImIH4JRAvQpcgApCahpsg2igDHkD8F4hNkMRAlUUIAyRtDwqgwICoyc4C8RokPgiDksgdIOZjGCQAFKLo0QXDr4G4mGGQtB+GJAAAHR5CR84BsqIAAAAASUVORK5CYII=>

[image42]: <data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAACoAAAASCAYAAAAt6kybAAAC1UlEQVR4Xu2WSaiPURjGH2RWxkzhnyRDKGNkTLphYSiuul0sDImilLIwFQslQ4YMJYQMmVNk3CA2yM5UimJh5SalxPPc95z/936n72NnIU/9uuc85/y/857pPRf4r39HPchBso8MTtq8NpCBqSk1IX1T02khuUEekfOkkm9uVHdylNwnT8iyfDPakVdkGulHPpB1wZeakankArkUvKp6kjnkNrmctEWtJM9J11DfRD65utSFvID1lXqT12RL7EAtJc9cfTPZTo6Q07DJnYVNoJfrh9Wwj+8n31EcaDfyjcx3XlPyHrYaUTth3/JaQT6T5qF+GLYgUbOQ/4a0CtlkC/UVxYEuIT/JsMS/Q266+jtyztUlbaN+OzHUtSD6XdRsstbV+5MHsGNYqrJA9XENpq300hlqIK1gZ1N9tGJeI4OviyEtJ0+z5sZjMT2UFdw9MrTaWqKyQOVpMAXjpbMkX5diVChrUl4aVP6xUO9A3pAJsPP9EHaBJJ3fraH8W5UFegvFgerwyx9EJodyGqjSj/xTzhsO+6YmWgleZ1jQ2p0/qizQ6ygOVIPL17kaF8ppoJqE/BOJn+oAmeLquoRXyTznVaVAr6QmdRI2mBK115ngazUGhLIG9BoS/N2J7zUGlp6itpFDpBOyI5NTWaB7YIOlj4H6yleqUg5VWcnea3TwNyZ+lNKWMkfHUG8Pu6DaIalP+JuTAtVyp1oMG2xE4iuN6FxF6cW56OrSDNhvZyZ+1HqywNUnwfrryJRKgV5LTdhsv5BFzmsBS+RrnKcte4l8DlQy/4gs4Xtph9Jnciws0NL/AVrCXp+7KE62emL1xsdUojdcT6G/pSprhdVXag1b5fpqj7wUZJqb25AfZHyoV2KDXo7HsCA1E/EW9szFfxSiasgO2OXaCzvsqeRpO4/DLsjcXGumWtjzXaRdsDG0YOmZ/6tqC7sLcXdS6ZgozWnB6n4B0P+dxOqyRRcAAAAASUVORK5CYII=>

[image43]: <data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAA0AAAASCAYAAACAa1QyAAAAv0lEQVR4XmNgGAUoIASI/wPxXyC+AcS7gfgzVOw3EJ8A4oNQeZAYGGwB4log5ocJAMEuBogCVSQxPSD+BmLwAfF2JAkQ4GCASN5GEweByyAiEYij0CRcGSC2TEUT52SAOB1sPTuqHEMrA0QTyK/IgA2INdHE4OAoA0STALoELsDLAAmxU+gS+IAvA8SWdnQJfGAiA0STC7oEPnAeiL8zQIKdKKDAALEFHKz4gCgDRBEIP2WAaALhe1AxWYRSCgAA/XgoreKa6BoAAAAASUVORK5CYII=>

[image44]: <data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAACoAAAASCAYAAAAt6kybAAACCUlEQVR4Xu2VzUtUYRTGj6mEIEGYQqZImxD/ByNMgkLBrZgIGoEbC0WFiKAgQlzUInHTQhEUQlFDXCi2sy9woQjZwrUbUVyUoEQ9D+fe6cxp7sw7s3LhD34w87zXO4/3vh8i55yToghe92ECL+CEDxOogcvwCH6Hr2BZ2hWBVMM2uArn3Vgm+M/8hFN+IAPlcAc+gBVwGP6BH+xFIfTBLfgWnkhYUV7zW8KKPoMvXTYjWrbV5cHwKeUqegeOSfgTXYIHsNFkHaJFJ02WF7mKFsPPsFLCi/J+LNVtMk4zZu9Nlhe5ij6EA9Hn0KJVsB1eMNlT0aLPTRbDBd0LF+E6/JQ+rGQregl+haXR99CiHr6VbbgPr7oxcgt+M99vmM8pshUdEX1lMYUW5eo/hc1+IOI1/AhL/ICFP77gQ1Anug9aCilaL7qXcjEl0SU6LX6ILsTH6cNKUlFO+gaX5Vv0sugr7/EDDs7lJ/CXaGH6H/xxTmIP8/iPMmlXdCY4r1fgoMlYnHusp0X0YCC1kuWJhpwYV0QL+ifKY7EfXnP5Gzjusptw1GXkHez0oeUiPIZroltENnjksui0y4eifM5k96OM96WbcE/0ZON252FRnpR8GCTeZaQJfhEtGb/KXdFzn+e0ZxYeyr9rN+C9aOyuaAH7SnlvP1Vib5vrYniUPxJdK5wufhGfXf4C2uaEKSfLluUAAAAASUVORK5CYII=>

[image45]: <data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAACoAAAASCAYAAAAt6kybAAACoUlEQVR4Xu2WW4iNURTH/+53k1seXIokZcrDJFEyLhERUsqDPChlSjNkKHJ7ohQPPFDkGlIKSaJ0CA8uuURyyRQPCpGUosT/P+vb56y9fedzXtX86tfMXt+ac/Zl7fUN0EEHmEbv0m/Zz5Xx43Z60AP0A31HT9LhUUZ1BtKD9BI9S/fQQVFGDTTQl7QR9sVn6G+63uWI03QfHUoX06+0jfb3STl0ppfpTBdrpQ9oVxf7J9foHDfWzmkC3+mQLDad3qCdQhJpgi1ot4vlMYteTGJd6Bs6O4lXRavVcb+idS6uI9YkQglspD/p2nKG7b5ytKgi9Bk6sX5J/CHsZGpCW/8R9oVjXFw1pNiabLwuGx8pZwADsphqtggdufIe0ZFZbBJ9T3uFpFqYQGcksauwDw911Y2uQFyPuoDKue5ieWgznqGyqBbYbk7xSQ6V12p6gd6md+LHFUbRH/QW4ppMCeXh67sao+kTWL68gkr9pzTCOk9grPs94jh9i+LWM47+olvSB1VYSk/BbnuYrHY1r2PshZ1SYUdYRj/T+vSBozd9jLhei1BZPYV1E6ENUKfRZPM6hkpMz17A+q5KJUL9tI1OTB84VAramcOwFlMLJfz9ZZq0jvdmEhfqRJtg7THsfpkR9DWd6mLqnYvcWGyn52EfFtBRFaHbPS8NklWwy5IyH5W3luZVXmRP2Op07J7NdIEbL4Hl9XGx7rAXQUDtRq1smIuV6E43Duygu9IgOUSXp0FxlH6B1U2JPqefYFuuSyPGw16Zeu0p7z7swqk7nMhyxAbY351zscmwtrQNtkiVzELYsfd1eQFNVB1icDZWa2y/daEO8tSOCRV9+iyonQnMhXWDrS4mdDmP0Xuwher26yTz2E+bYSWmfq7/E/4P/gC++5ZZ1q78rwAAAABJRU5ErkJggg==>

[image46]: <data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAD4AAAASCAYAAAADr20JAAADdElEQVR4Xu2XWaiNURTHlynzHJLhkpQx8WAmGcqDKMn0QghRHpQxwzUURUpmGUOUIUNkSrcMRWahDClDeDKVF4n/39r7nHXW/c49340Hyq/+3bv+3z7nfmvvtfbeV+Q///lXaQptgTZAHdwzyyKonTd/h9HQOegCdAhqkvtYKkPLoUvQLWgtVD1nRDLDoOFQpRBXgCZD0zMjRGpBT6DBUBvoNTQn+ISfHQgdgY4F74+wDroNNQrxcehA9vEvDkL7RSegGnRSdBIKUQz9cLoBNTNjpkB3TLwUWg1tF32P66KLwQlpbsYlwlna5c0EuCJ8md4hrh3ix5kRIiOC19h4LEd6PYyXxDLR7/oAXYVWSXYlI9tEKy3CCuGKW2ZCM5yXSBXolDcTuAY9dN4CaKSJ90LvTUy48t+hhc73cPUmeNOxEbpoYk70bBO3ha6ItklBqkrhxFuJrtph53vuS24FRLiK573pWCyFE58q2mqRYmho+J3JsqU6Z54WgH1YKPFYwpuhNaKzypLjRmf5KJq85x303JsOJr4E2geVQK+gSXYAqAc9g/qKthNbIm6G7P8V4fdUpEmcL8DE+TITg8fe/RyeRTgmKfE3ohtOWcwTrQq+D+GkskX6ZEYoXUXHcRMrCl5D0UmIn01FmsS5WTCpm87nscGk2C7kqyQnzqRfeNPRAqprYpYuJ5oJFWITNMDEPAJPQKOicVdKHxllKZbOuBDvCXFka/Bjn72EHmQfZ3greqaXFybN7+elJR/dRY+zyErR92oA7TZ+KdKs+BDRF2B/W7jL0h8bYm48T7OPM3yS3GPI01q0KnjZsbCk+f1dnB/hiXQWqh9iVswXqFeIW4afiaRJnBvJN2i983mu2hXfIbqDW2qIjuFFIx+DRMfYo4rwAkO/jvMj86ExJu4vOr698fKSJnFyBjrtvKOifc3LDImVUZQZoRcXet2M10+0fSK89t6T3IsPV5Orl69SWCX+WtpT9G+VdYfPkDZxziJ3cSZHOomurk2AsFx5tY3weGJlRLhpMSE/GfwcT4iKIeZqclLzlTmT5oZoYXXZk8AuQCnSJk5Yktyk2MuPRP9Z8PDFp4n2P0t/bvAsl0VveNyAIjwZ+B9ViWiJs787mucWHnWzvBngpHOyOcE73bMcypP430BN0aMqXlw8bBFOOi9M438C1SvDzL7pCs0AAAAASUVORK5CYII=>

[image47]: <data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAAwAAAATCAYAAACk9eypAAAArUlEQVR4XmNgGAUMVkC8G4hfAfF/IP4OxGeAeB6yImxgPQNEgzG6BDbAxACx4Qa6BC5gwQAxfTK6BC5AsoYaBoiGIHQJXOAQEP8CYn50CWwApOgvA0QTUQDkDJBzmtAloKAYXWAKA0SDM7oEEAgB8VF0wetA/BOIudHEGYF4IRBnIwuqM0BMByUNZMAJxJOA+DMDNCDsGCCKLjBANDyD8s8B8QeoGAgvAykmGQAAINkmAyYlrIoAAAAASUVORK5CYII=>

[image48]: <data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAADIAAAASCAYAAAAZk42HAAAC50lEQVR4Xu2WWaiNURTHFyJkyBApilKkCGXI9IKUIRJKhriEkGTIkBSSMoQH8SAyFFFkSCFzogwhYxEPZEzJgyTx/5+11znrLN89up3rRfdfv+5d/z2cb39777U+kRrV6J+rK7gB7oJXYFJRa2mNB6fACXAUDC9uzqktOAO+gCdgA2hQ1KMa1AV8BKNS3AO8AUPzPSoX+xwBjVPcArwAE/I9RBqBp2Bmal8GfoGTrk/ZqgUegdfB5xt7HLwsXQJ9gscHvuLi1WC9i6lDoouxl1e2OopOeD34s5PP3Sqll2Be8MaB2y4+DT6Dgc7j0eX8+5xXlrqLTng1+NOTPzn4URfBT7DSebwvi118XHSuCueNSR6PZbWog+iE14K/MPlLgh81RbQfOQa2g72gruvTCkwEtZ23SnTMGueZdoGvou3dRJ9hD7gP5qY+C0RfGK/F7uTlgucWJPFNcaItwc/Sciks5rtoFiulOuAh+ATahDaqJ9goOt9l0Dv5I5LHo2q7y/H8zZz6gg9gWIr7id4ZDlprnSoRsxWzz1RwRwoLmub6RDEZ/ABDYoPTWNF5VjivdfK4E17ciLw6idaA86LbPkd0EP+W0lkpPvt8SB4LPihTbVRn0Vrytzo1WvT3uQumlslb5zzqQYiLxMvLQUwGlYk15B2oF3zWEI61HTY1Ez1SM4KfpZHy5xy2kHiv8gvhqveDhoW23O7cc3GWePFi2jaxLvkKz8t/Dix1HhfGGpMluw9VWshB0Q79U9xU9ALNtw5JrAPMPibuCI9Jc+dRrOS8yPxh0zaw08XUILApeCY7WlVaCD8Z+Pbrgyaiqe6waNU38X9Licwq5vEl8JIPSF470SLHu2JiLeK4Cwmm0bei9WeW6+dlaZ3F1dQ+eZudx2fgJ1FO3HZ+QjDl3hLN0T7nm1hr3kvxDnCiRaKFkZ8lB0Av107dFH2ALAa7fqYdoieC7d/AVtEvDWZWG8ff4st7luL/Q78BuWi1ZV+2H0gAAAAASUVORK5CYII=>

[image49]: <data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAADsAAAASCAYAAADlhqZNAAACgUlEQVR4Xu2XS6hNURzGP+9ncU2E4ioTE0VclAlGZh7llQlFMTAwMBJXMTBQCiMGNykjeSSR91vekQxQSgopJhIlvs9/r3P+53/XvnGcPeF+9evu86219trfWmuvvS7Qq179k+pDJkYzo/GkXzT/QJfImuCtI9Pc7+FkF5npvJZoLFlIzpFjoSxpIJlKOskHMr2h9Pe1nPyAhfO6UPieQ7B+W6aN5DHZR76hPOxrcho2IHqQZsJqtl4gH/YieUg+krOw56pUn1EeNmkzmg+7g+yGtV8bys6T9uBVqirDToAF0juYC6sV0x68SlVl2MOwd34W8stYYVeT4+QReUBmN9SoSxvZc9h9jpApZD+5RY7CBk2o7DJskLupqrAK2OWuc2FPwZZ50l7yioxyXpK8ueQteQbbbwYUZTfJPdigjS68PcXfBlUV9gpsx5fKwmp2vCbD6u0Mvtd98okMdp4GSe3UT9JKd12TwmpEelIKOyMWlGgV2eJ+l4WNGgar9yQWON2BzaSXZlHt/CdrmbuuqdVhh5IbZJDzcmEXk3dkqfN0wFG9986L0jt6NXgpbH/nlYY9Ec2gFLYjFmSUgvXEJLK9uN5mzX5pZOEpUJk0q38V9mQ0g1LY3DFuCNlExsUCpyXoPrOLYLt12mSkObB6W50XpWXcVFgttS+wY5uWUJnUuW42PxagPhDa/su0AlZnvfPU3xnUB7AvbHfWTqsBzEltVK5Tl9dB2P1HOK92Dp9HbsOCpqX1Evbd0/EuqQv2KUh1dLTULruhXgULyHfkZ0P/PFwnX2Ht1d811FfBGHIANth3YefitqIsSq+QgqZneQr7JitH8t7ANkeFV5//j34CpfS0hXDkqqMAAAAASUVORK5CYII=>

[image50]: <data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAD4AAAASCAYAAAADr20JAAADMUlEQVR4Xu2XWchNURTHFxKZh4zhI0OIwoPMGUsRkkx5kExRkqEohXgQJTLLVFJkiAyZEpllylhCMiRPiniR+P9bex/r7HPuudf97gP1/evXd/d/7XO/s87eZ+11RSpUof9VzcA2sAl0DmJWy0DH0Cyv6oIGoWnUHGwB18F5MCUeztR4cA5cAAdBExOrBV6AoaAteA8WO5+qAgaDI+CY88qtSqA9mAGegQXxcCSuwlswy40HgG+gSzQjt9aD+6CRGx8HB/6EZTp4YMbLwRqwU3TebdGHxQfSwsxLFZ/SntBM0SFwDewHvyR34jfBaTNeKzo/36qPFJ3Xx41ru/HzaIbIDtGd4DVKdMWt5oI5gZeqquBkaGaol+ROfIRobJLxOohue74eWboBngbeUjDWjDeDi2Y8WuL3wR3JxeHuzKtqUrrEd4nGuoWBPGotet3hwA81U/RV8FoBhrvPTPYS6BpF86i6lC7xO6KxYeAKuAW2SrxApYkrx+s4d53oqnFLs9BZ1QMvQT/QWLR48lWl+P6vcp8LUikTfyUaOyW6tbkKLDyPQE0zL9Q00eveganOY5H84mJW3UVPChaxMuc1FH0IzKVglTLxj6KxMcbjjdKbZ7xQLEacczfweSx9EH0ds8QaMtCMZ4MTYJw3Hor+g0JJ2zo+8YVhAHoiGmtnPB4r9K4aLxSLIefsC/ztzvfvcZp6iu4qr9Wi17HP2Gv8hIpd8bTEL4vG6hivqfMeGy8UawLn8P22YhWnPzHwvXginQX13Ziv11fQ241bub+pKjbxRWEA2iAa8w0I1dJ5WSvOQvUDbAx8nttZK74ETDBjNkuc38l4OVVs4mHjQPUVjfkmhOrhvJXG6y/xs546I/HGhzoKvos2M6HaSLIt9feW1cNH+tvEB0kyESt2gazqNUBl0bOdq+0bGFZ6bkd+Bx+KF1eJVZzbnmKL+1mSD8iLSXM3WfF//hRdAKrMxBIqNPH5op0Vv5g3TViFWUisuDpsU1+DN6K/pHhDVnwQnyT5Q2cIuCfapPD3AH+MpInne65Tgv0+22o+4N1BLKZCE/9XxH6AR5VvXEKx4LEosqeY/BsW2by8/pRZjAAAAABJRU5ErkJggg==>

[image51]: <data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAACMAAAASCAYAAADR/2dRAAABfElEQVR4Xu2UvyuGURTHjx8pyqIUMrzlV1mxyWCUQQxMRBEWg0GKFCn8AzIwKL1+DFIWJLuQX5NBBrOSJBPf73vO03N6XtQzvKbnW5967vfc7j3PvedckUSJEiX6Hw2Bc7AB8sEUOAAXYA0UhVNzq3JwBOrAF7gCEyAPpMzrDybnWqOgB3SIbrzoYiXmTTovp6oBxWBedOMyF2s1r915cVUJGt14FryAAedl6RjcR7xp8AkKI34c9YqevNee/JEMN3sF6xH/EJxGvDhKgWuJmUyL6HWwqwKxq97AsuiiZ+ARLIAZ0U5rtrmdYFX0ClhzrDVqEDyALYsF/jZYEi2NTdBtfkZjosnUOq/JvOCvWDfPoNTGfeBGtOuGwbj5c2DFvqkTyT6ZtGgSFGv2KQzpu3LpDdH2ZjIVNm4Dt2E48yQw3gAKQJdoje2Lbhbot2R4AFQ1eHexH7UD7tyYneWTqRJNhp3CB5LXSY2A3WCShMlw03rzuDbnUfQ/+PEN3VNIvgQfRR4AAAAASUVORK5CYII=>

[image52]: <data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAACAAAAASCAYAAAA6yNxSAAACPklEQVR4Xu2USaiPYRTGH6RQKJFLCImIco0hlJBhYdiZSrcMCTuyoCgkJRaGzMTGwgolC0PIsCGhEMmQITtZWIjn+Z/zfv9zT9fNjoWnfvWd55z3+7//77zvAf7rH1IDOU6ukXtkRfN0q1pCnpCnsLXjm6cr9SYHyG1yhSwtie7kEVnjcV/ygmwtBa1oMflABni8jHwmvaoK0zDyhqzyeAr5VpJ7YBuIWk2+kPbJjxoIe8mZ5N8ih5J3h1wK8W7yswSvybkqZZoGK5ic/KgmWM2u5J8l70kbj+fC6hZVFcBgWDtqvVfycEhKo93fnPyo9bCaHck/4b5aKR3zeGRVETQGltyf/BHun0x+1HJYzc7kn3d/nMf3PZ5BbpC75CDpqeRUT+YN6NDI1+f8nfrBavSPi9qSj+7Pc++lxxdJV1hrjsDP3QRP5g0Mdf908rNWkrekP+zF68hD2NrpXqNboni+x1KjexjiD7UDETTc/b3Jb0mzyFXYKV8I+2paq68oPfZ4kMdSH/dqM0APGkJRY93fkvw/0WXyibTz+DrsXV1KAeqHv6bnsIMTNRtWMCf5WRvJthDrR3UG9gVPz3pXj+DphlQb2E6eoX5vpQ2w3sVBpJkQ77L0jnwP8UzyA9bjokmwH5sYvFHu1dQBNp/LIekI+yrVrIZt7itskRYX6WSXGaJ/9YCsracr6aaothPspmg23IwF3cgmcgp2RRbEpEsL1FvVFmnmHyUXYL1W61pSZ9j4fQWbvBrV2szf1S/b+oP+SkUMjQAAAABJRU5ErkJggg==>

[image53]: <data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAACAAAAASCAYAAAA6yNxSAAACLUlEQVR4Xu2US6iOURSGX0ROMXBJBoxdYuKSTrmUSIwolFySQ4kZiYQMyOiEIlFClBBySVFIUkcxIHUOEgPlKCkZSeJ9v7X2/69v9fdPGXjrqW+/+/KtvffaC/ivf0hjyWnykDwlm+rdbbWC3CI3yBWypN5daRy5Q76RXnKIdJTO0eQl2eLt8eQt2V8GtNFCcpkM9/Yo8o6sbIwAhpE+stH7d5Lf5GYZ0A0LIGoz+UIGJz9LJzYrefrRo9DeRw6GtnQRFkSlD+RSo8s0HzZgTvKz3pOtyVtOnoX2bfIV9bVWwwPQ3evjZOiUpru/J/lZD8gvsjt4yoftoX0dttaG4C11DzP841jolKa6fyb5WWth48Q1chQ2J17dGLKKDAyeNlYFMM8/cgCT3b+Q/FbahWYQP2Cvop0GkVewHEMnWgcwyf1zyc9S9iub15HnaAayPozJUpL+JAvUmACbcDyOoKa4fzj5WXdRv1st/h32Az25rImwWqAkrKQaoB+pCEXNdH9v8qNUA/rJkOSrBmjuouSPgB19V/LxhlxN3mLYIq2qWpEK15Nsuj6iPldJeY/sCJ4CqnSAvCYDmn3VwE+oZ7PesbK5SCeg4xwZPEmVTwmm0y06Qk6EtjS3fAyF7URvU1KN1qmsKQNgweludSrTgqdXouSb7Z7KuBJXuVCkdTTvvvMCtjnVj4a0Cz2ns+QUWRY7XY/JZ9R3rCC2wQqSyu95WP5E9aD5OjJ/V38Ayx5/jHI7EoEAAAAASUVORK5CYII=>

[image54]: <data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAD4AAAASCAYAAAADr20JAAACy0lEQVR4Xu2XS6hOURTH/0iEPIs8b0J5JDHwyCOvgYEYkFdKeeVRkkcMPK4oBpggjxKFoqRMJMQVBiQyQIkREyMDRQbi/2/t/d317Xu+e86933cHt+6/ft2z1l7na6+z1957XaBDHWqvGkzOkzNkfDLmdYCMTZ3Vqg/pnzor6AlZnzoztJgsIV2C3YlsIFtKEUAv8oksJKPIN7I3+CW9O5/cJneCr2ppImPIJvKB7CofztQq8o9sTgcyVA+L9bwiQ13MRvLW2YfJCXKJ3CAvyU3YBxnm4qrSLfKcXINNKi9xrcJnFE/8CPlIfpAX5DgaVzLqInnobFWIVtxrO9mW+Gqi6SiW+DFyCharKsmTVm9d6kx0ljxy9lKUz0MVqcVRddZcRRKvg01wGoonfhD5iaty3ji7niwKz0r2MZlYGq2xiiR+nUxGY2yRUlfih2BbqYF8RdNDsS9s+8wiA2FbIh6G2v9Hw3ObKC9xjV91z0UT30cekO7BXkH+kpmlCJM+qOJ0iKmypAGwjxDfbRPlJf6UDAnPLUl8OOyajFLpatWVUJ7OkbnO1hV4lyx3vqoVk9mdDlBrYY1DVEsSz5KS1vtqWippKuw6i9KhegHWZ1yJTu0NXQdFuIem14lUKfEesIl2c76iiY+E3b0nE79KWu9PSvxRXcl90i/YqpifZEawR4S/NVFMZk8Ff3OMLkWXawFs3F9VkhoY+Xsn/qj9ZKWz58DixzlfzRQTTBuHLC1D9orPJqudPYi8g53UUVpNrZ5vWLxUJWlbGufWXA/fas2D/bg6rTwpOcVudT4dWkpI/inOrzLX9dU52FrNX6hc5kpaB6KXtpu/CercWKu1k7yH/XAs3dewgySV9pY6qD+wuN/kGRr7bj1/R/k/OjobdDA2wEpc+3uCG/fSVbcjdQadhvUC+sCXk7F2rZ6wqyo2Lqm0RdTafiFr/gOkkaKBEZZlfgAAAABJRU5ErkJggg==>

[image55]: <data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAACEAAAASCAYAAADVCrdsAAABo0lEQVR4Xu2VTShEURiGPz+FYkGJsiIbkq2sFAsiPws7VlI2bMnfSonyk4Xyt7NBkYUilJSyY2dLdiJCUUq873znmuPcmbomd1bz1FN3zvfNPe8998wZkRQpgpEGu+ERfIYPcBs2201hkg8P4Cxsgb1wD37BD5gbbQ0HrsCpaAiXC/gIs93Cf9Mm+sRTbgEswBp3MAy8EHzieqeWNNLhrmiQT7gKK391JIlMOCS6NxiGMkyG3ZQAWaKvs9wt2JSKf9N1wXvRIKNmrAxewyqvKSB18Ez0V+cjB27BE1ji1EiHaAjegLB/HBb8dASH34sZYlB0kngbkcFYn3cLCTAicUIsiU6yLv6DiPuA++EGFpqxFfgG283nYrgBJ+EiLIID8AUuix50faZ3WDQEV5H3eIf9LNTCO9Egt3BadIIxeG6sZqPFsURD8NfUZK574Jq5PhQN1gBbzZgXgvfjuZNnxiPwabjcl/AVXsEd2Gg3WXAChuBNGH5O9H3PiK4G2Yed5tqDIfh/9AQrnNqf4QQ82PiKGCLWWcIeb7U8GGITToiGifAN/8hUWJxxK1MAAAAASUVORK5CYII=>

[image56]: <data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAFEAAAASCAYAAAAjQzL0AAAELElEQVR4Xu2YeahVVRTGl5pTKOaYgdJTckQNzSTB8qVJ5kBOfyhOZOVQaCo5NYiikVZWiJGKUxQ4gPOsOKWJFkRpURqR5B+ahCBBIhL6fa6971t3ec59572e/zz8wQf3fnvfvc9ZZ+2197ki9ykvvaAXoNq+obLwELQKOgntglrlN1cIH0G3gioln4tmCpkJnYNqlDSn0hJ615spMIh9odbWbAqtho5Ap6FXbWMZeA76w5ugGbQHugb9Ar0v924pcJ4Pwuci0WzpmWtNZxY01JspLIbqWKMRdAZ6LXxvDv0GzYsdMlId+gG66HxO9iv0CtRQ9GJ5Yztsp3tEsehcLZyfxCmoljdTWARNF83IOywRDaJlIvS3aGCyMg26IXcHcS70nvPWi97cQOdXNBugL7yZwBPQWm8WYKTo6s1xAdpoDdGawpt82vlpNIa+g3ZDf7o2Fverkj8WL4LjZ7nB8jJGdPws2cXlyRpXLhhN3swK5/PJ0H/H+Wl8BvWBdsrdmbhVdKxxxhsUvE3Gq0g6i66wKqIbxqP5zXmwD1diNd+QwoPQeeipaHQVvZll0Qh0DH6WFG8PbQmfk4LYBBoBVTUeHw7Hn2+8yHLoH9H2TtCb0BroRymp21NE5/pZ9DhjNynW3S+hZ0VrIktJoZpYDC11HgM7CdoOfSN6XKLqiQabQedR6g49JTmIDAz9r5yfxD7osfA5KYgeXsRPojX3EddGuojurpz/KNQt+P2Dx/IQs5q/Zx3mphXhjbNf1HUpnGV8aD2cVwx9a77zKHNZdBMmvJaXJMSnuyQHsV3wS6tZHOxj8z1LEHnDN0WPQ2kMEZ1/jvEeDh7nsDAbvZcVbpzMcM8n0GHoAeNtFpN9ljaiF8aaZukQfA6WBp/uCai+8UoLYlvRsyI3lkK8KDo/H1KEWUBvgfEIl9Ze52WFtXmhN8FY0bl4UGfmT4Vq5vUwxAvjQdvyZPALneAnQ687r1AQGWwu45d9QwIDROd/3njxWn0dZRBZUsoDl+Pj3hSt329B/0pJWeDyTt3pudMwVS18ueYP+znfwoDFCZK0LtdTl80BaIbxGFSeIZOI9S9rEDl2WeFm9L03A3yI3KAIXz6Yif+J1sFEmM5MW+5IEd7sJck/bPOcx122ELyopEz8VPSd1vIM9KHzInE5Zw3iQedlYbhotiXBHX+08/ZDs52XgynKbZz1gfAJMTtH5XpogOOxg7tnGlyuDL6F4/B3h4JYyNmHT3a86WfhDfA3w4xXFLzcq5bodf0uevwoK1x9PEMmwSDy4cTdmMn0l5Ty8tFANMrroJXQ4LxW5bjoQOzreVv0zScuZX6OOyvfSf1Sj+od+li4yfHYwnYeT7i5TYCuBI86Jnos4QqK3lnRo1kW6kJfe9PA08ob0DbRUsE/NewDvY/o7st3/f/NbSQN+sNN7SycAAAAAElFTkSuQmCC>

[image57]: <data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAFEAAAASCAYAAAAjQzL0AAAEFUlEQVR4Xu2YW4hWVRTHl6lZkRgqZWI4ivkQYWhpPiiNQuQNFG8oYWoIkoLaFKmBRJSgUUqh6IMoXh5E0fKSlyJ1lErChBQfUqoHK0x80dBIkVq/WWd/Z317vvPNOcPMPAzzgz/MXnv7nX3WWXuttRXpoLmMVY1XPRxPtBceU21Rfac6rHq6fLpF+Fj1X6J2ySaxSIF3VD+rHkynMxmoWhUbM8CJ41SD4wnopBoQG5tghuqQ6oBqr2pC+XQDbK6fG/dX7XHjluSI6qPk7xqxaHmpNJvNctW02JjBWtWjsbGvaorqa9Xn0Vw1XhZzRvdk3Ev1i2pmaYVIZ0lDP+i+arFb01rUij0vT2CcVT0UGzNYo6oTi8gGlqguqDao7koxJ55UvRjZFqjq3biL6rbYM/4Uc/orbr412a3aHhsr8LxqW2yswquqPrExwMsWceJv0jiipqvOufEDqstu3Fa8JubAPNHF8STHtQhFnXhC7Gi+62zkx7fcGNraiUNVn4jleAoGOTgL1nBKSDt5eETsfUbGE4GiTpwjaZ7br/pU7Fh09YuUK2IR+6XqqlgRerxsRcpm1d9ivzlE9bZqq+on1aJkDSmIj3VJrJ3x/Rp5eadqjFhOXC3Vc2Kt6rPIhmPfENvnt2LtEuoh5mycTitVkaJOhBWSOvJfsWodc0s1MfmbF/5ezKGVGCZWXfm9U6oRiZ1/j43e7/XE9qTYM8nDAV7cF7F/pHqU8dFGRbZa1Q9uTCtzTdU7GbOX+apdpRWOok6kKh8Uyz8/SrrxeW4NPBuNF4qtyyowU8XmVzrbE4mNCPQQjbEtL5wYIjxmvViqoigG9kmV6PPgxC9iYxWOSxoVQERwFO+JHass6CVxyMZ4ImGy2HyIXiAKsH3gbMDROhrZ8kJb92FsVOaKPYtGnchfpupWtqIKRZxIj0iIx7cBekQfZfRUf6gGlVZY88uarIZ7kjSO1ODE950NcOKxyJYXjuNzsVGso6BY3pH0dHG881T6BieSU/JAkifpVuJ3SW8uHAs2Ea5iECIt3CxiQv7L68SvIlseyM3nY2MCHzGcpKfEIpEuhDzYJDiRHFeJ0arZbkwk3lT1dDbgOnRD0iS8TrU0nW6AXIdDiMhKBCfndSI3raLMkvLWzEPFp/PwkLooolXhzFPJvhEr8R7Goe2gegYbx4GCEqobX40G11dLKigvydUScMavqh2lFY0JrRONe6AmsZWuWmJ74JpJ+1EUCgU9ZCVwIh8nBAIF6C+xQKoIx4x7Iw4M55+N8eL+kn1G7Id85PESdWJHtl6sPxvu5gMviEU467jNvCfZbQfFhraFfbAnKiXV/HpiQzyLD0fiD7aLqmckH3QVp2Ojgyswp4f6QKrgPzX8B+1ArPq+GRubw/+G1+nlYDHfOgAAAABJRU5ErkJggg==>

[image58]: <data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAEcAAAASCAYAAAAJ88NbAAADYElEQVR4Xu2YWahPURTGV7imSMg83GssJSJjXhDCkzJEGcpUPJAxSshUMiQUT9wMkXLJPJVZiiKKErmRSB7Egyjxfe293XX22eecfe+b+v/qq7O/s++yzp7W/hMpUaJEHNOgM9B8qIH3ztED2uSbdWUt9Aq6Bp2CJiZfB7kJzfVNMAu6DD0QE6s8+TqaUPyF0FmoNbQdug8NUu87Qouhz9Bg5dcZBtsPldn2RugPNNV1CDBdTB8mq2Gsp1Bb214PfVLtWELx60HV0BTbbgY9h5ZAR6Er0B3oBrTT9smkPnTINwM8g75D3W17qJjEOHMhmNRrSSffDvohyUHlB72HVimviKz4Xaw3Unl31TNpAV2Hmnp+Cq6E874Z4Bb0Baqw7RFikuC2CLEF2iWmzwLlz7NeP+URziRnNZas+O2tN1p5t9UzOQKN8bwgjSRucBqKmXUHlymT2KA8R7mYj3WrSyfPrUmPM6ypErMyG3t+iLz45CM02T5zhXGVOMZBx1Q7FyYTMzgabq1qqFLClYD/+ABomKSXPSsIPc6w5qT1WUGKyItPlkIXxBwZrEY8/Am/9RHUxrYLqc3gcBYOQB+gF5KefcKEK9WznzwrXWhwjlu/j+f7FMV3bBVz8C5T3jpJV7ZcajM4msPQVzEzqOH+ZqkkoeQ5o6HB4Wqg38vzfYriZ1EBXVJtrlBWr33OYPlksFhtNn8WhCc++7wUc26RmWJmxxFKngnR66A8csL6vJtkERM/C96l3MB3ht5A/aEh/3oEiF05fcVsK40rpePFlEVettxAkVDye6zXTXmElzb6LOshYuOH4LVhjWrzHOIKLiRmcHjyM4mHnv/W+pOkJtE89YTm2OeBkuSemI/PIja+D1c4K5suHFclsmLFDI77IH2Zag79gn5K9unvBlXPbEvoGzRbebwm8A7FKuNoAi2HOinPJxTf56Ckz0V+LwtAITGDw+3EPepO+jIxt2omttJ1CjBDTJ9Fns+VxssjSy3hPeWJJO84q8X87Wnl+WTFdwyHdvsm2CbJwzmTmMEhLNvs907Mh5yDxiZ61NBVzDbhqmLy/LnAVadXAS9jO8Qc0HuhVuodmQD9FvO7yycmPgf+oqTPScLzjr/leCD39t4liB2c/40Vkv+/BqOgx1DVX54H5n4cpZAyAAAAAElFTkSuQmCC>

[image59]: <data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAACMAAAASCAYAAADR/2dRAAABe0lEQVR4Xu2UPSiFYRTHTz7ykVKKTTYp8jWLstgNFpEkDCiTJJJBBjEpE0o2FiIfAxPFYDAYlJRQBrIw8v/fc17ee+K65b2L7q9+dd5znufe8zzv+zwiadL8Q8rgBDyE1/AEXsBuq8/CJotTyjh8hcewBeZYPhNOwTOr51o+ZSzBdzjoC0YWvIU7vhA1Q6KN8PUkYhcO+2SUlMAXeAWzXc0zA8t9MkrmRHclpStOlkvRZup8ISLW5ffXH4OnhY28iZ6YRLTBUp9MAs5r9smfuBFtKM/lw+TDA5jhC1HD+4PNtPqCUQQ35Wt1xXAfnsMROA2PYA0cg4uww8ZWwy24YM/zov/VDyfhho35pADuwTvYGC6ABtHjXOvy9aI/GpysZdGGSSF8Fr2XyABcsZg8wR6LeauvhmoxOLFLdIWncE30o+uU729b7sJ96Jm7wV0hHM9Gg3l9Et/MI6y0uF105/5ElehtHMBmRi0OmuF3RnolvpkHWGExm9lm8AH7rkTx7nc+AAAAAABJRU5ErkJggg==>

[image60]: <data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAADIAAAASCAYAAAAZk42HAAAC0klEQVR4Xu2WWaiOURSGFxKKCzNFnTtDuDAVGcqQOUMSoXSMuXBFKERImZU5Y5E5MhUSMkShyCylJJEIRZJ433+t7V/fOr9zOp3jRuetp85+19r7fPv79lr7F6lSlf65moGd4BK4BaZmw2VqDDgHLoCDoGk2LC3AWfAJPAYrQJ1MRiWoEbgPZtq4JXgOFqeEMrQW3AWNbXwC7M+HpS54AqaAhmAu+AVOupxK0RrRjXjNAO9BzeBHDRV9qO42rmdjvvWkRWC5G1MHRPOGBb9CegkOBa+P6D/qGfyoG+Bh8OaDUW58GnyQ7FrjRdff67wKibXBBbcFv5P5C4LvVSSacyT4UcdF84qdN8K8w86rkDqLLrgx+O3N3x18r+GiOZvBKnBNtNhZ+F5NwDhQ3Xl8QZy7xHlJW8EX0XgHMBvsAvckX8ezwCnR07CDRm+bEDfS1vx9wffiG2bOKzDJPM77bLG/qQZ4IFqDzUOM6ghWiq59GXQ1f4h5PKppfc7/zj+6WTBupI35pZ1hvh3m3A7+UfAa1Ap+ErvXD9AvBpxYY1yb9ZbElk6PX8IrV6OtRIObsjFpZ/664HvxuDBnT/B5NOgPDD7VWvQuYbGXpnRs+RWSeE3QW+o8KtdxU5CXoVcX8xcG36u/aA7rw4tfl/7Y4NcXPVKTg19Iqa0PcF561lhXf66OZ+CYC1CDRCcNDr4Xi5hHZEPw2QHjF+F9dB7McR43xjumkFI9lGsjy8BTUC0fy/3DN5K9EHkP8Dh58WfHmeDxpXwVvRyT1oMtbkz1kpJfMykdrXJtpDa4LtrbKf4G4leakBJEN5laIrtKEpsCuxSPGcXa+ijZDXMdzrtosI3yJf0E01ye10TROaOdV2TeaufxuV64sTQA80QLdzsY6YOmq+CtaK5XX3BH9PfWIynZjW6KPkAhODeKjYctlfFvog1nOnhnHrkCeoieJI7/D/0GTPq3TR4KBw0AAAAASUVORK5CYII=>

[image61]: <data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAADIAAAASCAYAAAAZk42HAAACo0lEQVR4Xu2WWchOURSGlylkKkMoJCURLsxEMkSGMiSl/DdkyIVkKIoQRcYbChkjRblBUi58pgtThlCI3IncuZAL8b7fWtu/zuqcw+/jRv9bT3373evss9fZe6/9iTSqUf9c3cAxcB3cBUuy3b+lDqBjNE30D4PL4DzYBzplIv6COoOnYIW1e4LXYEsKKFET0Fc08Rdgdba7qqbgCpjkvHXgIWjuvJq1VzQRr+XgE2gR/Khz4DY4Db5LfiKTwcXgNQNvwZTg16R3ohPymig6sXHBL9IoKU5kMXgF2gX/EZgTvD8WzwYnwP3rNdT8jcEvUlki3FLsewx6mTcSvAetU1CtGib6kgPBH2T+ieAXqSwRnoPnov0fwSrR1Rjjg5wOgc+i8YPBWnAcPJH6c7wSXBId9yiN8fZATGSA+WeCX6SyRKg+oueQMeQq6JKJqNcQsEs0rgJGmD/DPFa+ReZ1B1/5Y7R1xkT6m38q+EX6VSLzwVnRapWS4aq090FOc0VjNjivq3lcCS+uivQT7TyY7ZOB5u8PfpFSImtih2jheAZaWrsHuCYavzsFBc0S7ecqJPGaoLfNeVS14qZOXoZew83fFPwilSVSET0XXkzqHrgZ/KSZouNNdV6a61bnUT+vDpbGC66Dmib60PTgFyklwoMZxeqUN85ScCeapnQeGpTIdvBS9JZO4l7mBPyFyDtlgWt7pUT4XFQF7Iim6IR2RtOUtlaDEmkl+mVmW5u1nau0MAWIJplKIqtK1ATJfwnFgsKyuxm0Eb3VOVFuq7YuzqtOdLx5zutt3h7ncV5vXLv6p249OAmOSP6Newt8kOwfQ+59Vo1voi8hD0TvAi8WD1bA+6IHnSvHD5gnFh6WVI71RbTgLBP9GOkdN8BY0Z3E9v+hH1jlpM5esMbWAAAAAElFTkSuQmCC>

[image62]: <data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAABUAAAASCAYAAAC0EpUuAAABHklEQVR4Xu2SPyhFYRiHf0oW3Jgog5HhDiZFMkgGFrPJXYwGViWLxeZPGZRkksGkyGaSlDJgc0tMJgODgee777nd73vLvUcm5amnzvn9zvnOe875pH/+JMN4hg/4iRdpXeEEP2T9M66k9fecYll242BaVVjAXR/WoxXvcFa26FHSGps45sN6TOA6tqg2bV98AVzK+tys4XR2vChbNDykSi8eR+e5uMJCdtyGL/iKHVk2J/umuenCc5etyqYNUwcOcKBWN2YGl1zWjW/4hO14n9aN2ZHtVc+2bNr9zB9xi80+hH7ZosGS6wLzuIdbOBUXk3iNTXEYcShbtMfly7I3CWzI9rBGZH+8OkkZR7OLYobwxmWd+I5Fl/+KcXz04Rd+VzdvfpACGgAAAABJRU5ErkJggg==>