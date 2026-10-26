"""
End-to-End Pipeline Demonstration for Developer 1 (World Gen & Compiler).
File: scripts/run_e2e_demo.py

Demonstrates:
1. Asset & Scene Catalog lookup (Scene dimensions, Semantic Anchors, Models)
2. Procedural Geometry Solver (Collision-free layout + 2.2m aisle corridor validation)
3. Zero-SLAM Map Slicer (Instant map.pgm + map.yaml generation)
4. SDF Compiler for Gazebo Harmonic (world.sdf with ground friction & plugins)
"""

import sys
from pathlib import Path

# Ensure repo root is on sys.path
REPO_ROOT = Path(__file__).resolve().parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from lang2sim.world_gen import catalog, solve_spatial_layout, slice_map_from_layout
from lang2sim.compiler.sdf_compiler import compile_world_sdf
from lang2sim.core.models import EnvironmentSpec, MissionSpec, SimSpec


def run_pipeline():
    print("=" * 60)
    print("  Sim2Lang Developer 1: End-to-End Pipeline Test")
    print("=" * 60)

    # 1. Query Catalog for Scene & Anchors
    scene_id = "aws_small_warehouse"
    room_dims = catalog.get_scene_dimensions(scene_id)
    dock_pose = catalog.resolve_anchor(scene_id, "dock")
    goal_pose = catalog.resolve_anchor(scene_id, "rack_5")

    print(f"\n[Step 1] Catalog Query:")
    print(f"  - Target Scene: {scene_id} ({room_dims.x}m x {room_dims.y}m x {room_dims.z}m)")
    print(f"  - Robot Spawn Anchor ('dock'):   ({dock_pose.x}, {dock_pose.y})")
    print(f"  - Mission Goal Anchor ('rack_5'): ({goal_pose.x}, {goal_pose.y})")

    env = EnvironmentSpec(
        environment_type="indoor_warehouse",
        room_dimensions=room_dims,
        ambient_light_luminance=0.8,
        surface_friction=0.8,
    )
    mission = MissionSpec(
        initial_pose=dock_pose,
        goal_pose=goal_pose,
        timeout_seconds=60.0,
    )

    # 2. Query Catalog for Object Models
    print(f"\n[Step 2] Querying Physical Object Blueprints from Catalog:")
    objects = [
        catalog.get_model_spec("aws_warehouse_rack", f"rack_{i}") for i in range(1, 5)
    ] + [
        catalog.get_model_spec("wooden_crate", f"crate_{j}") for j in range(1, 3)
    ]
    for obj in objects:
        print(f"  - Added {obj.id} ({obj.model_name}): {obj.dimensions.x}m x {obj.dimensions.y}m, static={obj.is_static}")

    # 3. Procedural Spatial Solver (Shapely OBB)
    print(f"\n[Step 3] Running Procedural Geometry Solver:")
    placed_objects, corridors = solve_spatial_layout(
        environment=env,
        objects=objects,
        spawn_pose=dock_pose,
        goal_pose=goal_pose,
    )
    print(f"  - Successfully placed {len(placed_objects)}/{len(objects)} objects without collision.")
    print(f"  - Preserved main corridor width: {corridors[0].width:.2f}m (Requires >= 2.2m for TB3)")

    # 4. Zero-SLAM Map Slicer
    output_dir = Path("test_output/e2e")
    output_dir.mkdir(parents=True, exist_ok=True)

    print(f"\n[Step 4] Slicing 2D Occupancy Grid (Zero-SLAM):")
    pgm_path, yaml_path = slice_map_from_layout(
        room_dim=env.room_dimensions,
        objects=placed_objects,
        output_dir=output_dir,
        map_name="e2e_map",
    )
    print(f"  - Generated Map PGM:  {pgm_path} ({pgm_path.stat().st_size} bytes)")
    print(f"  - Generated Map YAML: {yaml_path}")

    # 5. SDF Compiler for Gazebo Harmonic
    print(f"\n[Step 5] Compiling Gazebo Harmonic SDF:")
    sim_spec = SimSpec(
        environment=env,
        objects=placed_objects,
        mission=mission,
        corridors=corridors,
        metadata={"name": "e2e_warehouse_demo"},
    )
    sdf_path = output_dir / "world.sdf"
    compile_world_sdf(sim_spec, sdf_path)
    print(f"  - Compiled SDF World: {sdf_path} ({sdf_path.stat().st_size} bytes)")

    print("\n" + "=" * 60)
    print("  ALL 5 STAGES COMPLETED SUCCESSFULLY!")
    print("=" * 60)


if __name__ == "__main__":
    run_pipeline()
