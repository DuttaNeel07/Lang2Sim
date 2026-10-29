"""
Unit Tests for Sim2Lang Core Models and Configuration (Day 0 Seed)
File: tests/test_core_models.py
"""

import json
import unittest

from pydantic import ValidationError

from lang2sim.core.config import (
    DEFAULT_CONTROL_PORT,
    DEFAULT_SIM_IMAGE,
    get_catalog_dir,
    get_projects_dir,
    get_run_dir,
)
from lang2sim.core.models import (
    EnvironmentSpec,
    MissionSpec,
    NavStatus,
    ObjectSpec,
    Pose2D,
    SimSpec,
    SimStartRequest,
    SimStatus,
    Vector3D,
)


class TestCoreModels(unittest.TestCase):
    """Test suite for Pydantic v2 schemas and validation logic."""

    def test_pose2d_valid(self):
        """Test valid 2D pose creation."""
        pose = Pose2D(x=2.5, y=-3.0, yaw=1.57)
        self.assertEqual(pose.x, 2.5)
        self.assertEqual(pose.y, -3.0)
        self.assertAlmostEqual(pose.yaw, 1.57, places=2)

    def test_pose2d_from_anchor(self):
        """Test Pose2D.from_anchor parsing lists and dicts."""
        p1 = Pose2D.from_anchor([2.5, 3.0, 1.57])
        self.assertEqual(p1.x, 2.5)
        self.assertEqual(p1.y, 3.0)

        p2 = Pose2D.from_anchor({"x": 1.0, "y": 2.0, "yaw": 0.0})
        self.assertEqual(p2.x, 1.0)

    def test_pose2d_invalid_yaw(self):
        """Test that yaw out of range [-pi, pi] raises ValidationError."""
        with self.assertRaises(ValidationError):
            Pose2D(x=0.0, y=0.0, yaw=4.0)

    def test_vector3d(self):
        """Test 3D bounding dimension model."""
        vec = Vector3D(x=14.0, y=10.0, z=5.0)
        self.assertEqual(vec.x, 14.0)
        self.assertEqual(vec.y, 10.0)
        self.assertEqual(vec.z, 5.0)

    def test_environment_spec(self):
        """Test environment specification with defaults and constraints."""
        env = EnvironmentSpec(
            environment_type="indoor_warehouse",
            room_dimensions=Vector3D(x=14.0, y=10.0, z=4.0),
            ambient_light_luminance=0.8,
            surface_friction=0.8,
        )
        self.assertEqual(env.environment_type, "indoor_warehouse")
        self.assertEqual(env.surface_friction, 0.8)
        self.assertTrue(env.sky_weather.sky_enabled)

    def test_complete_sim_spec(self):
        """Test a complete SimSpec containing environment, objects, and mission."""
        spec = SimSpec(
            environment=EnvironmentSpec(room_dimensions=Vector3D(x=14.0, y=10.0, z=4.0)),
            objects=[
                ObjectSpec(
                    id="rack_1",
                    model_name="aws_warehouse_rack",
                    dimensions=Vector3D(x=2.8, y=1.2, z=3.6),
                    pose=Pose2D(x=2.5, y=3.0, yaw=0.0),
                    is_static=True,
                )
            ],
            mission=MissionSpec(
                initial_pose=Pose2D(x=0.0, y=0.0, yaw=0.0),
                goal_pose=Pose2D(x=7.5, y=3.0, yaw=0.0),
                timeout_seconds=60.0,
            ),
        )
        self.assertEqual(len(spec.objects), 1)
        self.assertEqual(spec.objects[0].id, "rack_1")
        self.assertEqual(spec.mission.goal_pose.x, 7.5)

    def test_sim_status_and_nav_status(self):
        """Test PR #7 SimStatus and NavStatus models."""
        sim_stat = SimStatus(
            is_running=True,
            sim_time=12.5,
            real_time_factor=1.0,
            render_mode="software",
            health_status="healthy",
        )
        self.assertTrue(sim_stat.is_running)

        nav_stat = NavStatus(
            is_active=True,
            current_task="navigating",
            distance_remaining=3.2,
            status="navigating",
        )
        self.assertTrue(nav_stat.is_active)

    def test_control_api_models(self):
        """Test Dev 3 Control API schemas."""
        start_req = SimStartRequest(
            run_dir="/root/.lang2sim/projects/test/runs/run1",
            world_sdf_path="/root/.lang2sim/projects/test/runs/run1/world.sdf",
            map_yaml_path="/root/.lang2sim/projects/test/runs/run1/map.yaml",
            robot_model="turtlebot3_waffle",
            initial_pose=Pose2D(x=0.0, y=0.0, yaw=0.0),
        )
        self.assertEqual(start_req.robot_model, "turtlebot3_waffle")


class TestConfigAndCatalog(unittest.TestCase):
    """Test suite for directory resolution and asset catalog integrity."""

    def test_config_paths_and_functions(self):
        """Verify standalone config functions and constants matching PR #7."""
        self.assertEqual(DEFAULT_CONTROL_PORT, 9090)
        self.assertIn("lang2sim-sim", DEFAULT_SIM_IMAGE)
        self.assertTrue(get_projects_dir().exists())
        self.assertTrue(get_catalog_dir().exists())

    def test_create_run_dir(self):
        """Verify that get_run_dir builds ~/.lang2sim/projects/<name>/runs/<timestamp>/."""
        run_dir = get_run_dir("test_unit_project")
        self.assertTrue(run_dir.exists())
        self.assertIn("runs", str(run_dir))
        self.assertIn("test_unit_project", str(run_dir))

    def test_scene_catalog_json_validity(self):
        """Verify catalog/scene_catalog.json is valid and contains expected anchors."""
        catalog_path = get_catalog_dir() / "scene_catalog.json"
        self.assertTrue(catalog_path.exists())
        with open(catalog_path) as f:
            data = json.load(f)
        self.assertIn("scenes", data)
        self.assertIn("aws_small_warehouse", data["scenes"])
        warehouse = data["scenes"]["aws_small_warehouse"]
        self.assertIn("anchors", warehouse)
        self.assertIn("dock", warehouse["anchors"])
        self.assertIn("rack_5", warehouse["anchors"])
        self.assertEqual(warehouse["anchors"]["rack_5"][0], 7.5)


if __name__ == "__main__":
    unittest.main()
