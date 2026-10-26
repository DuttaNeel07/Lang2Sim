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
