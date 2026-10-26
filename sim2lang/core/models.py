"""
Canonical Simulation Specification Schemas and Control API Data Contracts.
File: sim2lang/core/models.py
"""

from typing import List, Optional, Literal, Dict, Any
from pydantic import BaseModel, Field


# ---------------------------------------------------------------------------
# 1. Spatial and Kinematic Primitives
# ---------------------------------------------------------------------------

class Pose2D(BaseModel):
    """2D ground-plane coordinate pose with yaw orientation."""
    x: float = Field(..., description="X coordinate in meters relative to world origin")
    y: float = Field(..., description="Y coordinate in meters relative to world origin")
    yaw: float = Field(0.0, ge=-3.14159, le=3.14159, description="Yaw orientation in radians")


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
    ir_version: str = "2.0.0"
    project_name: str
    sim_spec: SimSpec
    aisle_corridors: List[AisleCorridor] = Field(default_factory=list)
    is_valid: bool = False
    validation_messages: List[str] = Field(default_factory=list)


# ---------------------------------------------------------------------------
# 8. Control API Contracts (Used by Dev 3 Container Runtime & Dev 4)
# ---------------------------------------------------------------------------

class SimStartRequest(BaseModel):
    """Request payload sent to POST /sim/start."""
    run_dir: str = Field(..., description="Absolute path to active run directory inside container")
    world_sdf_path: str = Field(..., description="Path to world.sdf")
    map_yaml_path: str = Field(..., description="Path to map.yaml")
    robot_model: str = "turtlebot3_waffle"
    initial_pose: Pose2D
    use_gui: bool = True


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


class NavStatus(BaseModel):
    """Navigation task state streamed via WebSocket or GET /nav/status."""
    task_active: bool = False
    state: Literal["idle", "navigating", "succeeded", "failed", "canceled"]
    current_pose: Optional[Pose2D] = None
    distance_remaining: float = 0.0
    elapsed_time: float = 0.0


class HealEvent(BaseModel):
    """Log record of self-healing error recovery."""
    timestamp: float
    error_signature: str
    action_taken: str
    success: bool
