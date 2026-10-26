"""
Canonical Simulation Specification Schemas and Control API Data Contracts.
File: sim2lang/core/models.py
"""

import math
from typing import List, Optional, Literal, Dict, Any
from pydantic import BaseModel, Field


# ---------------------------------------------------------------------------
# 1. Spatial and Kinematic Primitives
# ---------------------------------------------------------------------------

class Pose2D(BaseModel):
    """2D ground-plane coordinate pose with yaw orientation."""
    x: float = Field(..., description="X coordinate in meters relative to world origin")
    y: float = Field(..., description="Y coordinate in meters relative to world origin")
    yaw: float = Field(0.0, ge=-math.pi, le=math.pi, description="Yaw orientation in radians, [-pi, pi]")

    @classmethod
    def from_anchor(cls, anchor: Any) -> "Pose2D":
        """Convert either a list [x, y, yaw] or a dict {'x':..., 'y':..., 'yaw':...}."""
        if isinstance(anchor, (list, tuple)):
            yaw = float(anchor[2]) if len(anchor) > 2 else 0.0
            return cls(x=float(anchor[0]), y=float(anchor[1]), yaw=yaw)
        elif isinstance(anchor, dict):
            return cls(**anchor)
        elif isinstance(anchor, cls):
            return anchor
        raise ValueError(f"Cannot parse anchor of type {type(anchor)}: {anchor}")


class Vector3D(BaseModel):
    """3D bounding dimensions in meters."""
    x: float = Field(..., description="Bounding size X (length) in meters")
    y: float = Field(..., description="Bounding size Y (width) in meters")
    z: float = Field(..., description="Bounding size Z (height) in meters")


# ---------------------------------------------------------------------------
# 2. Environmental & Weather Specifications
# ---------------------------------------------------------------------------

class SkyWeatherSpec(BaseModel):
    """Procedural atmospheric sky, sunlight angle, and volumetric fog controls."""
    sky_enabled: bool = True
    clouds_speed: float = Field(10.0, ge=0.0, le=50.0, description="Procedural cloud animation speed")
    fog_enabled: bool = False
    fog_density: float = Field(0.0, ge=0.0, le=0.5, description="Volumetric fog density (0.05 = light, 0.2 = heavy mist)")
    sun_elevation: float = Field(0.8, ge=0.0, le=1.57, description="Solar elevation angle in radians (0.0 = sunset/dusk)")


class EnvironmentSpec(BaseModel):
    """Complete physical room and terrain parameters."""
    environment_type: Literal[
        "indoor_warehouse", "indoor_office", "outdoor_field", "outdoor_terrain"
    ] = "indoor_warehouse"
    room_dimensions: Vector3D = Field(..., description="Outer room/field boundaries in meters")
    wall_height: float = Field(2.5, ge=0.0, le=10.0, description="Wall boundary height (0.0 for open outdoor fields)")
    ambient_light_luminance: float = Field(
        1.0, ge=0.0, le=1.0, description="Ambient light factor (1.0 = nominal, 0.15 = degraded night)"
    )
    ground_terrain: Literal[
        "concrete", "grass_soil", "dirt_mud", "asphalt_road", "uneven_heightmap", "tiles"
    ] = "concrete"
    surface_friction: float = Field(
        0.8, ge=0.05, le=1.5, description="Surface friction coefficient (0.8 = standard, 0.15 = slippery ice/mud)"
    )
    sky_weather: SkyWeatherSpec = Field(default_factory=SkyWeatherSpec)


# ---------------------------------------------------------------------------
# 3. Object & Obstacle Specifications
# ---------------------------------------------------------------------------

class ObjectSpec(BaseModel):
    """Physical entity to be placed in the simulation world."""
    id: str = Field(..., description="Unique alphanumeric identifier (e.g., 'rack_1')")
    model_name: str = Field(..., description="Asset identifier matching catalog (e.g., 'aws_warehouse_rack')")
    dimensions: Vector3D = Field(..., description="Bounding dimensions of the object in meters")
    pose: Pose2D = Field(..., description="2D ground plane pose")
    is_static: bool = Field(True, description="Static obstacle vs dynamically mutable entity")
    semantic_parent_id: Optional[str] = Field(None, description="Anchor ID for grouped objects")


# ---------------------------------------------------------------------------
# 4. Sensor Perturbations & Noise Models
# ---------------------------------------------------------------------------

class CameraPerturbation(BaseModel):
    """Optical degradation filters for on-board robot camera."""
    enabled: bool = False
    motion_blur_kernel: int = Field(0, ge=0, le=31, description="Blur kernel size (odd integer, 0 = off)")
    brightness_attenuation: float = Field(1.0, ge=0.0, le=1.0, description="Illumination multiplier")
    gaussian_noise_stddev: float = Field(0.0, ge=0.0, le=0.5, description="Additive noise standard deviation")


class LidarPerturbation(BaseModel):
    """LiDAR beam degradation filters."""
    enabled: bool = False
    dropout_probability: float = Field(0.0, ge=0.0, le=0.9, description="Random beam dropout rate")
    range_noise_stddev: float = Field(0.0, ge=0.0, le=0.2, description="Gaussian noise standard deviation in meters")


class SensorNoiseSpec(BaseModel):
    """Aggregated sensor degradation models."""
    camera: CameraPerturbation = Field(default_factory=CameraPerturbation)
    lidar: LidarPerturbation = Field(default_factory=LidarPerturbation)


# ---------------------------------------------------------------------------
# 5. Dynamic Perturbations & Adversarial Triggers
# ---------------------------------------------------------------------------

class DynamicObstacleTrigger(BaseModel):
    """Runtime perturbation trigger dispatched via FastMCP."""
    trigger_type: Literal["time", "waypoint_distance", "immediate"]
    trigger_value: float = Field(..., description="Seconds from start or distance in meters to target")
    model_name: str = Field("wooden_crate", description="Asset identifier to spawn dynamically")
    spawn_pose: Pose2D = Field(..., description="Spawn coordinate in world reference frame")
    velocity: Optional[Pose2D] = Field(None, description="Linear and angular velocity vector")


# ---------------------------------------------------------------------------
# 6. Mission & Simulation Specifications
# ---------------------------------------------------------------------------

class MissionSpec(BaseModel):
    """Navigation task definition dispatched via Nav2 Simple Commander."""
    initial_pose: Pose2D = Field(..., description="Robot starting coordinate and heading")
    waypoints: List[Pose2D] = Field(default_factory=list, description="Intermediate navigation waypoints")
    goal_pose: Pose2D = Field(..., description="Final navigation goal pose")
    timeout_seconds: float = Field(120.0, ge=10.0, le=600.0, description="Maximum operational allowance")


class SimSpec(BaseModel):
    """Canonical top-level specification emitted by the supervisor agent."""
    spec_version: str = "2.0.0"
    environment: EnvironmentSpec
    objects: List[ObjectSpec] = Field(default_factory=list)
    robot_model: Literal["turtlebot3_waffle", "turtlebot4", "ridgeback"] = "turtlebot3_waffle"
    sensor_noise: SensorNoiseSpec = Field(default_factory=SensorNoiseSpec)
    adversarial_triggers: List[DynamicObstacleTrigger] = Field(default_factory=list)
    mission: MissionSpec


# ---------------------------------------------------------------------------
# 7. Simulation Intermediate Representation (SIR)
# ---------------------------------------------------------------------------

class AisleCorridor(BaseModel):
    """Validated navigation corridor between obstacles."""
    corridor_id: str
    width: float = Field(..., ge=2.2, description="Aisle width in meters (must be >= 2.2m)")
    start_point: Pose2D
    end_point: Pose2D


class SimulationIR(BaseModel):
    """Validated intermediate representation graph linking entities and corridors."""
    spec: SimSpec
    world_sdf_path: Optional[str] = None
    map_yaml_path: Optional[str] = None
    map_pgm_path: Optional[str] = None
    resolved_anchors: Dict[str, List[float]] = Field(default_factory=dict)
    aisle_corridors: List[AisleCorridor] = Field(default_factory=list)
    is_valid: bool = False
    validation_messages: List[str] = Field(default_factory=list)


# ---------------------------------------------------------------------------
# 8. Status Models (PR #7 Aligned)
# ---------------------------------------------------------------------------

class NavStatus(BaseModel):
    """Navigation task state streamed via WebSocket or GET /nav/status."""
    is_active: bool = False
    current_task: Optional[str] = None
    distance_remaining: float = 0.0
    target_pose: Optional[Pose2D] = None
    status: Literal["idle", "navigating", "succeeded", "failed", "canceled"] = "idle"
    waypoint_index: int = Field(0, description="0-based index of the waypoint being driven to")
    waypoints_total: int = Field(0, description="Number of poses in the current task (1 for a single goal)")
    navigation_time: float = Field(0.0, description="Sim seconds spent on the current leg")
    eta_seconds: Optional[float] = Field(None, description="Nav2's estimated time remaining on this leg")
    recoveries: int = Field(0, description="Nav2 recovery behaviours run on this leg")
    message: Optional[str] = Field(None, description="Why a task failed or was canceled")


class SimStatus(BaseModel):
    """Simulation state monitor model (PR #7 and HealthMonitor aligned)."""
    is_running: bool = False
    sim_time: float = 0.0
    real_time_factor: float = 1.0
    robot_pose: Pose2D = Field(default_factory=lambda: Pose2D(x=0.0, y=0.0, yaw=0.0))
    robot_velocity: Optional[Pose2D] = None
    render_mode: Literal["gpu", "dri", "software", "headless"] = "software"
    health_status: Literal["healthy", "degraded", "crashed"] = "healthy"


# ---------------------------------------------------------------------------
# 9. Control API Contracts (Dev 3 Container Server & Dev 4)
# ---------------------------------------------------------------------------

class SimStartRequest(BaseModel):
    """Request payload sent to POST /sim/start."""
    run_dir: str = Field(..., description="Absolute path to active run directory inside container")
    world_sdf_path: str = Field(..., description="Path to world.sdf")
    map_yaml_path: str = Field(..., description="Path to map.yaml")
    robot_model: str = "turtlebot3_waffle"
    initial_pose: Pose2D
    use_gui: bool = True
    use_rviz: bool = Field(True, description="Open RViz2 with the Nav2 panels (ignored without a display)")
    world_name: Optional[str] = Field(None, description="World name; read from the SDF when omitted")
    nav2_params_path: Optional[str] = Field(
        None, description="Nav2 params file inside the container; default: TurtleBot3 Waffle params"
    )


class RunManifest(BaseModel):
    """Contract for one run folder (decision 1): ~/.lang2sim/projects/<project>/runs/<timestamp>/manifest.json.

    Paths are relative to the run folder. Part 1 writes the files, Part 3 only reads them.
    """
    manifest_version: Literal["1.0"] = "1.0"
    world_sdf: str = Field("world.sdf", description="Gazebo Harmonic world (SDF 1.9+), no robot inside")
    world_name: Optional[str] = Field(None, description="<world name=...>; read from the SDF when omitted")
    map_yaml: str = Field("map.yaml", description="Nav2 map whose frame equals the world frame")
    spec_json: Optional[str] = Field("spec.json", description="The SimSpec this run was generated from")
    robot_model: Literal["turtlebot3_waffle"] = Field(
        "turtlebot3_waffle", description="Only TurtleBot3 Waffle is supported for the MVP (decision 4)"
    )
    initial_pose: Pose2D = Field(..., description="Robot spawn pose in the world/map frame")
    nav2_params: Optional[str] = Field(None, description="Custom Nav2 params; default: TurtleBot3 Waffle params")


class SimStopRequest(BaseModel):
    """Request payload sent to POST /sim/stop."""
    force: bool = False


class SimStatusResponse(BaseModel):
    """Response payload returned by GET /sim/status."""
    status: Literal["idle", "launching", "running", "degraded", "stopped", "error"]
    rtf: float = Field(0.0, description="Real-Time Factor (Δsim/Δwall)")
    sim_time: float = Field(0.0, description="Current simulation time in seconds")
    active_nodes: List[str] = Field(default_factory=list)
    active_topics: List[str] = Field(default_factory=list)
    ready: bool = Field(False, description="All readiness checks pass: robot can be given goals")
    readiness: Dict[str, bool] = Field(
        default_factory=dict, description="gazebo_clock, odometry, laser_scan, nav2_active, localized"
    )
    world_report: List[str] = Field(default_factory=list, description="World pre-flight findings for this run")


class NavInitialPoseRequest(BaseModel):
    """Request payload sent to POST /nav/initial_pose."""
    pose: Pose2D


class NavGoalRequest(BaseModel):
    """Request payload sent to POST /nav/goal."""
    goal_pose: Pose2D
    timeout_seconds: float = 120.0


class NavWaypointsRequest(BaseModel):
    """Request payload sent to POST /nav/waypoints."""
    waypoints: List[Pose2D]
    timeout_seconds: float = 180.0


class HealEvent(BaseModel):
    """Log record of self-healing error recovery."""
    timestamp: float
    error_signature: str
    action_taken: str
    success: bool
