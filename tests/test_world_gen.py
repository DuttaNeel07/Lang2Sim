"""
Unit Tests for Procedural World Generation, Map Slicing, and SDF Compilation.
File: tests/test_world_gen.py
"""

import os
import shutil
import tempfile
import unittest
from pathlib import Path

from lang2sim.core.models import (
    Pose2D,
    Vector3D,
    EnvironmentSpec,
    ObjectSpec,
    MissionSpec,
    SimSpec,
)
from lang2sim.world_gen.shapely_solver import ShapelySpatialSolver, solve_spatial_layout
from lang2sim.world_gen.map_slicer import ZeroSlamMapSlicer, slice_map_from_layout
from lang2sim.compiler.sdf_compiler import HarmonicSdfCompiler, compile_world_sdf
from lang2sim.compiler.sir_validator import SIRValidator
from lang2sim.compiler.sir_generator import SIRGenerator


class TestWorldGenAndMapSlicer(unittest.TestCase):
    """Test suite for spatial placement, zero-SLAM slicer, and SDF compiler."""

    def setUp(self):
        self.temp_dir = Path(tempfile.mkdtemp(prefix="l2s_test_world_"))
        self.env = EnvironmentSpec(
            environment_type="indoor_warehouse",
            room_dimensions=Vector3D(x=14.0, y=10.0, z=4.0),
            ambient_light_luminance=0.8,
            surface_friction=0.8,
        )
        self.objects = [
            ObjectSpec(
                id=f"rack_{i}",
                model_name="aws_warehouse_rack",
                dimensions=Vector3D(x=2.8, y=1.2, z=3.6),
                pose=Pose2D(x=0.0, y=0.0, yaw=0.0),  # Unplaced, requires solver
                is_static=True,
            )
            for i in range(1, 7)
        ]
        self.mission = MissionSpec(
            initial_pose=Pose2D(x=0.0, y=0.0, yaw=0.0),
            goal_pose=Pose2D(x=5.0, y=3.0, yaw=0.0),
            timeout_seconds=60.0,
        )

    def tearDown(self):
        shutil.rmtree(self.temp_dir, ignore_errors=True)

    def test_shapely_solver_places_objects_collision_free(self):
        """Verify solver places all racks without overlapping each other or room walls."""
        placed_objects, corridors = solve_spatial_layout(
            environment=self.env,
            objects=self.objects,
            spawn_pose=self.mission.initial_pose,
            goal_pose=self.mission.goal_pose,
        )

        self.assertEqual(len(placed_objects), len(self.objects))
        self.assertTrue(len(corridors) > 0)
        self.assertGreaterEqual(corridors[0].width, 2.20)

        # Check all polygons are non-intersecting
        solver = ShapelySpatialSolver()
        room_poly = solver.create_room_polygon(self.env.room_dimensions)
        polys = [
            solver.create_obb(o.pose.x, o.pose.y, o.dimensions.x, o.dimensions.y, o.pose.yaw)
            for o in placed_objects
        ]

        for i, p1 in enumerate(polys):
            # Must be completely inside the room
            self.assertTrue(room_poly.contains(p1))
            for j, p2 in enumerate(polys):
                if i != j:
                    self.assertFalse(
                        p1.intersects(p2),
                        f"Collision between {placed_objects[i].id} and {placed_objects[j].id}",
                    )

    def test_zero_slam_map_slicer(self):
        """Verify 2D map slicer generates valid P5 map.pgm and map.yaml."""
        placed_objects, _ = solve_spatial_layout(self.env, self.objects)
        pgm_path, yaml_path = slice_map_from_layout(
            room_dim=self.env.room_dimensions,
            objects=placed_objects,
            output_dir=self.temp_dir,
            map_name="test_map",
        )

        self.assertTrue(pgm_path.exists())
        self.assertTrue(yaml_path.exists())

        # Validate P5 header in map.pgm
        with open(pgm_path, "rb") as f:
            header_line = f.readline().decode("ascii").strip()
            dim_line = f.readline().decode("ascii").strip()
            max_val = f.readline().decode("ascii").strip()

        self.assertEqual(header_line, "P5")
        cols, rows = map(int, dim_line.split())
        self.assertEqual(cols, int(14.0 / 0.05))
        self.assertEqual(rows, int(10.0 / 0.05))
        self.assertEqual(max_val, "255")

        # Validate map.yaml content
        with open(yaml_path, "r") as f:
            yaml_text = f.read()
        self.assertIn("image: test_map.pgm", yaml_text)
        self.assertIn("resolution: 0.05", yaml_text)

    def test_sdf_compiler_generates_valid_harmonic_xml(self):
        """Verify SDF compiler writes valid Gazebo Harmonic SDF 1.9 XML."""
        placed_objects, _ = solve_spatial_layout(self.env, self.objects)
        sim_spec = SimSpec(
            environment=self.env,
            objects=placed_objects,
            mission=self.mission,
        )

        sdf_path = self.temp_dir / "world.sdf"
        compile_world_sdf(sim_spec, sdf_path, world_name="test_warehouse")

        self.assertTrue(sdf_path.exists())
        with open(sdf_path, "r", encoding="utf-8") as f:
            xml = f.read()

        self.assertIn('<sdf version="1.9">', xml)
        self.assertIn('<world name="test_warehouse">', xml)
        self.assertIn("gz::sim::systems::Physics", xml)
        self.assertIn("gz::sim::systems::UserCommands", xml)
        self.assertIn("gz::sim::systems::SceneBroadcaster", xml)
        self.assertIn("gz::sim::systems::Contact", xml)
        self.assertIn('<model name="ground_plane">', xml)
        self.assertIn('<model name="rack_1">', xml)

    def test_sir_pipeline_validation(self):
        """Verify SIRGenerator and SIRValidator validate physical feasibility."""
        sim_spec = SimSpec(
            environment=self.env,
            objects=self.objects,
            mission=self.mission,
        )
        generator = SIRGenerator()
        sir = generator.generate(sim_spec)

        self.assertTrue(sir.is_valid)
        self.assertEqual(len(sir.validation_messages), 0)
        self.assertGreaterEqual(len(sir.aisle_corridors), 1)

    def test_sir_validator_catches_collision_errors(self):
        """Verify SIRValidator catches overlapping obstacles."""
        colliding_objects = [
            ObjectSpec(
                id="rack_a",
                model_name="aws_warehouse_rack",
                dimensions=Vector3D(x=2.8, y=1.2, z=3.6),
                pose=Pose2D(x=2.0, y=2.0, yaw=0.0),
                is_static=True,
            ),
            ObjectSpec(
                id="rack_b",
                model_name="aws_warehouse_rack",
                dimensions=Vector3D(x=2.8, y=1.2, z=3.6),
                pose=Pose2D(x=2.0, y=2.0, yaw=0.0),  # Exact same pose = collision!
                is_static=True,
            ),
        ]
        bad_spec = SimSpec(
            environment=self.env,
            objects=colliding_objects,
            mission=self.mission,
        )
        validator = SIRValidator()
        is_valid, errors = validator.validate(bad_spec)

        self.assertFalse(is_valid)
        self.assertTrue(any("Collision detected" in e for e in errors))


if __name__ == "__main__":
    unittest.main()
