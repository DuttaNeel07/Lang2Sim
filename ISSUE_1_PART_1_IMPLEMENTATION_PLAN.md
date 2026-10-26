# Sim2Lang: Issue #1 (Part 1) Technical Implementation Plan & Architectural Specification

> **Issue Reference:** `[Part 1] Procedural World Builder, Multi-Target SDF Compiler & Curated Asset Catalog`  
> **Source Documents:** `Sim2Lang Robot Simulation Architecture (1).md` & `SIM2LANG_TEAM_OF_4_IMPLEMENTATION_PLAN.md`  
> **Assignee Role:** Developer 1 (Geometry, Compilers & Asset Depot)  
> **Milestone:** Phase 1 (Hour 0–24)  
> **Status:** Approved for Implementation  

---

## 1. Executive Summary & Objective

The primary objective of **Issue #1 (Part 1)** is to eliminate the manual, multi-day engineering overhead of authoring 3D Gazebo simulation environments and 2D navigation occupancy costmaps. 

Part 1 implements:
1. **Canonical Simulation Intermediate Representation (SIR):** Typed Pydantic v2 data models establishing the shared contract for all downstream systems (`models.py`, `config.py`).
2. **Grounded Curated Asset Depot:** High-fidelity scene archetypes indexed with verified semantic anchors, pre-computed Nav2 costmaps, and physical surface friction (`catalog/`).
3. **Procedural Spatial Arrangement Engine:** Shapely 2.0 computational geometry solver executing 2D Oriented Bounding Box (OBB) placement with polygon buffering to mathematically guarantee collision-free navigation corridors ($W_{\text{corridor}} \ge 2.2\text{m}$, wall standoff $\ge 0.5\text{m}$).
4. **Zero-SLAM 2D Map Slicer:** Direct 3D geometric slicing / vectorized rasterizer producing standard P5 grayscale `map.pgm` and `map.yaml` in $<3.5\text{s}$ without running Cartographer or SLAM.
5. **Harmonic SDF Compiler:** Multi-environment Jinja2 template synthesizer generating valid **Gazebo Harmonic (SDF 1.9+)** worlds (aligned with ROS 2 Jazzy; Gazebo Classic dropped per Dev 3 v2 decision) with volumetric fog, dynamic lighting, atmospheric sky, and calibrated ground friction.
6. **5-Stage Simulation Compiler Pipeline:** Formal `SIRGenerator` and `SIRValidator` enforcing physical feasibility before files are written to disk.

---

## 2. Adaptation Matrix of the 4 Open-Source Foundations

Part 1 adapts algorithms, data schemas, and interface patterns from four open-source robotics codebases:

```
┌─────────────────────────────────────────────────────────────────────────────────────────────────────────┐
│                                 HOW ISSUE #1 BUILDS ON THE 4 FOUNDATIONS                                │
├──────────────────────────┬──────────────────────────┬──────────────────────────┬────────────────────────┤
│ 1. gazebo_world_generator│ 2. godrift.ai            │ 3. sandraschi/gazebo-mcp │ 4. gym-gazebo2         │
│    (Robotics-UPO)        │    (Drift Platform)      │    (FastMCP Server)      │    (Acutronic Robotics)│
├──────────────────────────┼──────────────────────────┼──────────────────────────┼────────────────────────┤
│ • Primary architecture   │ • Coordinate frames:     │ • Entity schema matching │ • Calibrated physics   │
│ • Refactors data_models  │   map->odom->base_link   │ • Object specifications  │   ground friction (mu) │
│   into Pydantic v2       │ • Corridor clearance     │   align with FastMCP     │ • Chassis collision    │
│ • Shapely 2.0 OBB solver │   bounds (robot radius)  │   dynamic spawn tools    │   geometry for bumper  │
│ • Zero-SLAM map slicer   │ • Pre-validated spawn    │   used in Phase 3        │   sensors in Phase 3   │
│ • Jinja2 SDF templates   │   coordinates            │                          │                        │
└──────────────────────────┴──────────────────────────┴──────────────────────────┴────────────────────────┘
```

### 2.1 Adaptation of `robotics-upo/gazebo_world_generator`
* **Data Models:** Replaces UPO's `data_models.py` with strict Pydantic v2 schemas (`Pose2D`, `Vector3D`, `ObjectSpec`, `EnvironmentSpec`, `SimSpec`).
* **Spatial Placement:** Replaces naive rejection sampling with Shapely 2.0 polygon buffering to prevent corridor blockages:
  $$W_{\text{corridor}} \ge 2 \cdot r_{\text{robot}} + \delta_{\text{safety}} \quad (\ge 2.2\text{m})$$
  $$\text{Wall standoff} \ge 0.5\text{m}$$
* **Zero-SLAM Map Slicer:** Refactors the `gazebo_ros2_2Dmap_plugin` and `generate_map.sh` workflow into a zero-latency headless rasterizer producing `map.pgm` and `map.yaml` in $<3.5\text{s}$.
* **Jinja2 Harmonic SDF (1.9+):** Exclusively targets ROS 2 Jazzy & Gazebo Harmonic (SDF 1.9+; Classic dropped per Dev 3 v2 decision) with native `gz::sim::systems` system plugins.

### 2.2 Adaptation of `godrift.ai` (Drift CLI / Platform)
* **Coordinate Frame Grounding:** Enforces Drift's standard coordinate tree (`map` $\to$ `odom` $\to$ `base_footprint` $\to$ `base_link`). Note: AMCL publishes `map -> odom`, and the Gazebo diff-drive plugin publishes `odom -> base_footprint`.
* **Safe Pose Verification:** Guarantees that initial spawn poses and Nav2 target waypoints are never generated in collision or outside room boundaries.

### 2.3 Adaptation of `sandraschi/gazebo-mcp`
* **Schema Alignment:** Standardizes asset identifiers, dimensions, and 6-DoF poses in `catalog/` and `models.py` to match FastMCP's `spawn_model` and `load_world` parameter contracts for runtime mutation in Phase 3.

### 2.4 Adaptation of `acutronicRobotics/gym-gazebo2`
* **Physics & Surface Friction:** Incorporates calibrated surface friction ($\mu$) into ground planes:
  * Logistics warehouse concrete: $\mu = 0.8$
  * Agricultural field / mud: $\mu = 0.5$
  * Wet / icy floor: $\mu = 0.15$
* **Collision Meshes:** Pre-instruments robot models and static obstacles with rigid collision boxes required for bumper contact sensors.

---

## 3. Directory Layout & File Ownership

All files implemented under Issue #1 belong to the following tree:

```
Lang2Sim/
├── pyproject.toml                         # Packaging, dependencies & console scripts
├── catalog/                               # Grounded Asset Depot
│   ├── scene_catalog.json                 # Semantic anchors, dimensions, verified costmaps
│   ├── config/                            # Frozen robot configs
│   │   └── nav2_tb3_waffle.yaml           # Frozen Nav2 parameters for TB3 Waffle
│   ├── maps/
│   │   ├── aws_small_warehouse.yaml       # Nav2 metadata config
│   │   └── aws_small_warehouse.pgm        # 0.05m/pixel grayscale occupancy grid
│   └── models/                            # High-fidelity SDF assets (racks, benches, crates, barrels)
└── sim2lang/
    ├── __init__.py                        # Package export
    ├── core/
    │   ├── __init__.py
    │   ├── models.py                      # Pydantic v2 SimSpec, EnvironmentSpec, SIR & Control API schemas
    │   └── config.py                      # Workspace paths (~/.lang2sim/projects/<name>/runs/<timestamp>/)
    ├── world_gen/
    │   ├── __init__.py
    │   ├── shapely_solver.py              # OBB collision avoidance & Minkowski-sum corridors
    │   └── map_slicer.py                  # Direct 3D geometric slicing -> map.pgm/map.yaml
    └── compiler/
        ├── __init__.py
        ├── sir_generator.py               # Compiles MissionState.json -> simulation_ir.json
        ├── sir_validator.py               # Geometric feasibility & aisle corridor checks
        ├── sdf_compiler.py                # Gazebo Harmonic (SDF 1.9+) Jinja2 engine
        └── templates/
            └── world_harmonic.sdf.jinja   # Gazebo Harmonic (SDF 1.9+)
```

---

## 4. Detailed Technical Specifications

### 4.1 Shared Canonical Data Contracts (`sim2lang/core/models.py`)

All components in Sim2Lang adhere to typed Pydantic v2 schemas:

```python
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

### 4.2 Curated Asset Depot (`catalog/`)

The catalog pairs physical properties with pre-verified semantic coordinates:

```json
{
  "scenes": {
    "aws_small_warehouse": {
      "dimensions": {"x": 14.0, "y": 10.0, "z": 4.0},
      "map_yaml": "catalog/maps/aws_small_warehouse.yaml",
      "map_pgm": "catalog/maps/aws_small_warehouse.pgm",
      "surface_friction": 0.8,
      "semantic_anchors": {
        "dock": {"x": 0.0, "y": 0.0, "yaw": 0.0},
        "rack_1": {"x": 2.5, "y": 3.0, "yaw": 0.0},
        "rack_5": {"x": 7.5, "y": 3.0, "yaw": 0.0},
        "packaging_bench": {"x": -3.0, "y": 2.0, "yaw": 1.57}
      }
    }
  },
  "models": {
    "aws_warehouse_rack": {
      "dimensions": {"x": 2.8, "y": 1.2, "z": 3.6},
      "mass": 120.0,
      "static": true
    },
    "wooden_crate": {
      "dimensions": {"x": 0.8, "y": 0.8, "z": 0.8},
      "mass": 15.0,
      "static": false
    }
  }
}
```

### 4.3 Shapely 2.0 Spatial Placement Engine (`sim2lang/world_gen/shapely_solver.py`)

* **Polygon Buffering Algorithm:**
  1. Construct room bounding polygon $\mathcal{R} = [0, W] \times [0, H]$.
  2. For each catalog item, construct an Oriented Bounding Box (OBB) at candidate pose $(x, y, \theta)$.
  3. Validate wall standoff:
     $$\text{dist}(\mathcal{P}_{\text{candidate}}, \partial \mathcal{R}) \ge 0.5\text{m}$$
  4. Validate navigation corridor:
     $$\text{buffer}\left(\mathcal{P}_{\text{candidate}}, \frac{W_{\text{aisle}}}{2}\right) \cap \mathcal{P}_i = \emptyset \quad \forall \mathcal{P}_i \in \text{placed objects}$$
     where $W_{\text{aisle}} \ge 2.2\text{m}$.
  5. Apply semantic grouping: pallets aligned along rack faces; chairs facing desk perimeters.

### 4.4 Zero-SLAM 2D Map Slicer (`sim2lang/world_gen/map_slicer.py`)

* Converts the Shapely layout into a 2D NumPy binary occupancy grid at resolution $0.05\text{m}$/pixel:
  * Grid size: $\text{cols} = \lceil W / 0.05 \rceil$, $\text{rows} = \lceil H / 0.05 \rceil$.
  * Free cells: pixel value `254` (white).
  * Occupied cells: pixel value `0` (black).
  * Unknown/border cells: pixel value `205` (grey).
* Emits standard P5 Portable Gray Map (`map.pgm`) and Nav2 metadata (`map.yaml`):
  ```yaml
  image: map.pgm
  resolution: 0.05
  origin: [-7.0, -5.0, 0.0]
  negate: 0
  occupied_thresh: 0.65
  free_thresh: 0.25
  ```
* Execution latency: $<3.5\text{s}$.

### 4.5 Harmonic SDF Compiler (`sim2lang/compiler/sdf_compiler.py`)

* Employs Jinja2 templates targeting **Gazebo Harmonic (SDF 1.9+)** exclusively (aligned with ROS 2 Jazzy; Gazebo Classic dropped per Dev 3 v2 decision).
* Injects native Gazebo Harmonic system plugins:
  * `gz::sim::systems::Physics` (physics simulation loop)
  * `gz::sim::systems::UserCommands` (dynamic model spawning/deletion via `gz service`)
  * `gz::sim::systems::SceneBroadcaster` (scene rendering for GUI & screen grabbing)
  * `gz::sim::systems::Contact` (chassis contact collision reporting)
* Dynamically populates `<scene>`, atmospheric `<sky>`, volumetric `<fog>`, and calibrated ground `<surface><friction>`.

---

## 5. Implementation Roadmap for Issue #1

```
Step 1: Core Setup & Data Contracts
  ├── Create pyproject.toml with dependencies and console script entrypoint
  ├── Implement sim2lang/core/models.py (Pydantic v2 SimSpec, EnvironmentSpec, SIR schemas, Control API models)
  └── Implement sim2lang/core/config.py (workspace directory resolution ~/.lang2sim/projects/<name>/runs/<timestamp>/)

Step 2: Curated Asset Depot
  ├── Implement catalog/scene_catalog.json (semantic anchors & dimensions)
  ├── Add catalog/config/nav2_tb3_waffle.yaml (frozen Nav2 bringup config)
  └── Add pre-computed catalog/maps/aws_small_warehouse.yaml and .pgm

Step 3: Procedural Spatial Placement Solver
  └── Implement sim2lang/world_gen/shapely_solver.py (OBB packing & corridor buffering)

Step 4: Zero-SLAM 2D Map Slicer
  └── Implement sim2lang/world_gen/map_slicer.py (rasterizer emitting map.pgm and map.yaml)

Step 5: Harmonic SDF Compiler
  ├── Implement sim2lang/compiler/templates/world_harmonic.sdf.jinja
  └── Implement sim2lang/compiler/sdf_compiler.py

Step 6: SIR Generator & Validator Pipeline
  ├── Implement sim2lang/compiler/sir_generator.py
  └── Implement sim2lang/compiler/sir_validator.py

Step 7: Isolated Verification & Pytest Suite
  ├── Implement tests/test_models.py
  ├── Implement tests/test_shapely_solver.py
  ├── Implement tests/test_map_slicer.py
  └── Implement tests/test_sdf_compiler.py
```

---

## 6. Definition of Done for Issue #1

- [ ] `pyproject.toml` installs cleanly with `pip install -e .`.
- [ ] `sim2lang/core/models.py` strictly validates `SimSpec`, `SimulationIR`, and Control API schemas.
- [ ] `catalog/scene_catalog.json` indexes `aws_small_warehouse` with verified anchors (`dock`, `rack_5`).
- [ ] `catalog/config/nav2_tb3_waffle.yaml` provides frozen parameters for TB3 navigation.
- [ ] `shapely_solver.py` places objects collision-free with corridors $\ge 2.2\text{m}$.
- [ ] `map_slicer.py` emits valid `map.yaml` and `map.pgm` readable by `PIL.Image` in $<3.5\text{s}$.
- [ ] `sdf_compiler.py` generates valid Gazebo Harmonic (SDF 1.9+) XML.
- [ ] `SIRValidator` rejects out-of-bound coordinates and blocked waypoints.
- [ ] 100% pass rate across `pytest tests/test_world_gen.py`.
