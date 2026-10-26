"""
Unit tests for Core Data Models and Configuration
File: tests/test_core_models.py
"""

import pytest
from pydantic import ValidationError
from sim2lang.core.models import (
    Pose2D,
    Vector3D,
    SkyWeatherSpec,
    EnvironmentSpec,
    ObjectSpec,
    CameraPerturbation,
    LidarPerturbation,
    SensorNoiseSpec,
    DynamicObstacleTrigger,
    MissionSpec,
    SimSpec,
    SimulationIR,
    NavStatus,
    SimStatus,
)
from sim2lang.core.config import (
    get_projects_dir,
    get_active_project_dir,
    get_run_dir,
    get_catalog_dir,
    DEFAULT_CONTROL_PORT,
    DEFAULT_SIM_IMAGE,
)


def test_pose2d_valid():
    pose = Pose2D(x=2.5, y=-3.0, yaw=1.57)
    assert pose.x == 2.5
    assert pose.y == -3.0
    assert pose.yaw == 1.57


def test_pose2d_yaw_out_of_bounds():
    with pytest.raises(ValidationError):
        Pose2D(x=0.0, y=0.0, yaw=4.0)


def test_vector3d_dimensions():
    v = Vector3D(x=14.0, y=10.0, z=5.0)
    assert v.x == 14.0
    assert v.y == 10.0
    assert v.z == 5.0


def test_environment_spec_defaults():
    env = EnvironmentSpec(
        room_dimensions=Vector3D(x=18.0, y=12.0, z=6.0),
    )
    assert env.environment_type == "indoor_warehouse"
    assert env.surface_friction == 0.8
    assert env.ambient_light_luminance == 1.0
    assert env.sky_weather.sky_enabled is True


def test_object_spec_validation():
    obj = ObjectSpec(
        id="rack_1",
        model_name="warehouse_rack",
        dimensions=Vector3D(x=2.8, y=1.2, z=3.6),
        pose=Pose2D(x=4.0, y=2.0, yaw=0.0),
    )
    assert obj.id == "rack_1"
    assert obj.is_static is True


def test_mission_spec():
    mission = MissionSpec(
        initial_pose=Pose2D(x=0.0, y=0.0, yaw=0.0),
        goal_pose=Pose2D(x=7.5, y=3.0, yaw=1.57),
        timeout_seconds=180.0,
    )
    assert mission.timeout_seconds == 180.0
    assert len(mission.waypoints) == 0


def test_full_simspec_serialization():
    spec = SimSpec(
        environment=EnvironmentSpec(
            room_dimensions=Vector3D(x=14.0, y=10.0, z=5.0),
            surface_friction=0.85,
        ),
        objects=[
            ObjectSpec(
                id="rack_1",
                model_name="pallet_rack",
                dimensions=Vector3D(x=2.8, y=1.2, z=3.6),
                pose=Pose2D(x=3.0, y=2.0, yaw=0.0),
            )
        ],
        robot_model="turtlebot3_waffle",
        mission=MissionSpec(
            initial_pose=Pose2D(x=0.0, y=0.0, yaw=0.0),
            goal_pose=Pose2D(x=6.0, y=4.0, yaw=0.0),
        ),
    )
    data = spec.model_dump_json()
    reconstructed = SimSpec.model_validate_json(data)
    assert reconstructed.environment.surface_friction == 0.85
    assert len(reconstructed.objects) == 1
    assert reconstructed.robot_model == "turtlebot3_waffle"


def test_simulation_ir():
    spec = SimSpec(
        environment=EnvironmentSpec(
            room_dimensions=Vector3D(x=10.0, y=10.0, z=4.0),
        ),
        mission=MissionSpec(
            initial_pose=Pose2D(x=0.0, y=0.0, yaw=0.0),
            goal_pose=Pose2D(x=5.0, y=5.0, yaw=0.0),
        ),
    )
    ir = SimulationIR(
        spec=spec,
        world_sdf_path="/tmp/world.sdf",
        map_yaml_path="/tmp/map.yaml",
        resolved_anchors={"dock": [0.0, 0.0, 0.0], "rack_1": [3.0, 2.0, 0.0]},
    )
    assert ir.resolved_anchors["dock"] == [0.0, 0.0, 0.0]
    assert ir.world_sdf_path == "/tmp/world.sdf"


def test_runtime_status_models():
    nav = NavStatus(
        is_active=True,
        current_task="navigating_to_rack_5",
        distance_remaining=1.85,
        target_pose=Pose2D(x=7.5, y=3.0, yaw=0.0),
        status="navigating",
    )
    assert nav.is_active is True
    assert nav.distance_remaining == 1.85

    sim = SimStatus(
        is_running=True,
        sim_time=42.5,
        real_time_factor=0.98,
        robot_pose=Pose2D(x=2.1, y=1.0, yaw=0.3),
        render_mode="gpu",
        health_status="healthy",
    )
    assert sim.real_time_factor == 0.98
    assert sim.health_status == "healthy"


def test_config_paths(tmp_path, monkeypatch):
    monkeypatch.setenv("HOME", str(tmp_path))
    p_dir = get_projects_dir()
    assert p_dir.exists()
    assert "projects" in str(p_dir)

    act_dir = get_active_project_dir("test_proj")
    assert act_dir.exists()
    assert act_dir.name == "test_proj"

    run_dir = get_run_dir("test_proj", timestamp="run_001")
    assert run_dir.exists()
    assert run_dir.name == "run_001"

    cat_dir = get_catalog_dir()
    assert "catalog" in str(cat_dir)
    assert DEFAULT_CONTROL_PORT == 9090
    assert "lang2sim-sim" in DEFAULT_SIM_IMAGE
