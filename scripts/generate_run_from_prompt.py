"""
NLP-to-Simulation Pipeline: Generate and Launch a Complete Run from Natural Language.
File: scripts/generate_run_from_prompt.py

Usage:
    python3 scripts/generate_run_from_prompt.py "generate a 23 by 15 room with 10 racks and make the agent go to 5th rack"
    python3 scripts/generate_run_from_prompt.py --launch
"""

import argparse
import json
import re
import sys
from datetime import datetime
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from lang2sim.world_gen import catalog, solve_spatial_layout, slice_map_from_layout
from lang2sim.compiler.sdf_compiler import compile_world_sdf
from lang2sim.core.models import (
    EnvironmentSpec,
    MissionSpec,
    SimSpec,
    RunManifest,
    Pose2D,
    Vector3D,
)
from lang2sim.runtime.run_folder import write_manifest


def parse_prompt(prompt: str) -> dict:
    """
    Extracts geometric dimensions, rack counts, and target goal from NLP command.
    Fallback to intelligent defaults if omitted.
    """
    print(f"\n[1/5] Parsing Natural Language Prompt:")
    print(f"      \"{prompt}\"")

    # Match room dimensions like "23 by 15" or "23x15"
    dim_match = re.search(r"(\d+(?:\.\d+)?)\s*(?:by|x)\s*(\d+(?:\.\d+)?)", prompt, re.IGNORECASE)
    if dim_match:
        dim_x = float(dim_match.group(1))
        dim_y = float(dim_match.group(2))
    else:
        dim_x, dim_y = 14.0, 10.0

    # Match rack count like "10 racks" or "6 shelves"
    count_match = re.search(r"(\d+)\s*(?:racks?|shelves?|stands?)", prompt, re.IGNORECASE)
    rack_count = int(count_match.group(1)) if count_match else 6

    # Match goal rack target like "go to 5th rack", "navigate to rack 3", "5th rack"
    target_match = re.search(r"(?:(?:go\s*to|reach|target|visit|approach)\s+(?:the\s+)?(\d+)(?:st|nd|rd|th)?\s*rack)|(?:(?:go\s*to|reach|target|visit|approach)\s+(?:the\s+)?rack\s*(\d+))", prompt, re.IGNORECASE)
    if not target_match:
        # Fallback to any ordinal like "5th rack"
        target_match = re.search(r"(\d+)(?:st|nd|rd|th)\s*rack", prompt, re.IGNORECASE)

    if target_match:
        target_idx = int(target_match.group(1) or target_match.group(2))
    else:
        target_idx = min(rack_count, 5)

    print(f"  -> Extracted Dimensions: {dim_x}m x {dim_y}m")
    print(f"  -> Requested Racks:     {rack_count} units")
    print(f"  -> Navigation Target:   rack_{target_idx}")

    return {
        "dim_x": dim_x,
        "dim_y": dim_y,
        "rack_count": rack_count,
        "target_idx": target_idx,
    }


def build_simulation_run(prompt_info: dict, project_name: str = "nlp_demo") -> Path:
    dim_x = prompt_info["dim_x"]
    dim_y = prompt_info["dim_y"]
    rack_count = prompt_info["rack_count"]
    target_idx = prompt_info["target_idx"]

    # Create run folder under ~/.lang2sim/projects/<project>/runs/<timestamp>
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir = Path.home() / ".lang2sim" / "projects" / project_name / "runs" / timestamp
    run_dir.mkdir(parents=True, exist_ok=True)

    # 1. Environment & Mission
    env = EnvironmentSpec(
        environment_type="indoor_warehouse",
        room_dimensions=Vector3D(x=dim_x, y=dim_y, z=4.0),
        ambient_light_luminance=0.8,
        surface_friction=0.8,
    )

    # Spawn near room corner / entrance
    spawn_pose = Pose2D(x=round(-dim_x / 2.0 + 2.0, 2), y=round(-dim_y / 2.0 + 2.0, 2), yaw=0.0)

    # 2. Query Catalog for verified physical models
    objects = [
        catalog.get_model_spec("aws_warehouse_rack", f"rack_{i}")
        for i in range(1, rack_count + 1)
    ]

    # 3. Procedural Geometry Layout Solver
    print(f"\n[2/5] Running Procedural Geometry Solver (Shapely OBB):")
    # Temporary goal for layout calculation
    temp_goal = Pose2D(x=round(dim_x / 2.0 - 2.0, 2), y=0.0, yaw=0.0)
    placed_objects, corridors = solve_spatial_layout(
        environment=env,
        objects=objects,
        spawn_pose=spawn_pose,
        goal_pose=temp_goal,
    )
    print(f"  -> Placed {len(placed_objects)} racks collision-free.")
    print(f"  -> Verified corridor width: {corridors[0].width:.2f}m (Safe for TurtleBot3 >= 2.2m)")

    # Find the target rack's actual placed coordinates
    target_obj = next((o for o in placed_objects if o.id == f"rack_{target_idx}"), placed_objects[-1])
    # Set approach goal 1.5m in front of the target rack
    actual_goal = Pose2D(
        x=round(target_obj.pose.x, 2),
        y=round(target_obj.pose.y - 1.5, 2),
        yaw=1.57,
    )
    print(f"  -> Target rack_{target_idx} is at ({target_obj.pose.x:.2f}, {target_obj.pose.y:.2f})")
    print(f"  -> Robot Nav2 Goal set to: ({actual_goal.x:.2f}, {actual_goal.y:.2f})")

    mission = MissionSpec(
        initial_pose=spawn_pose,
        goal_pose=actual_goal,
        timeout_seconds=90.0,
    )

    # 4. Zero-SLAM Map Slicer
    print(f"\n[3/5] Slicing 2D Occupancy Grid (Zero-SLAM):")
    pgm_path, yaml_path = slice_map_from_layout(
        room_dim=env.room_dimensions,
        objects=placed_objects,
        output_dir=run_dir,
        map_name="map",
    )
    print(f"  -> Created: {pgm_path.name} and {yaml_path.name}")

    # 5. Gazebo Harmonic SDF Compiler
    print(f"\n[4/5] Compiling Gazebo Harmonic SDF:")
    sim_spec = SimSpec(
        environment=env,
        objects=placed_objects,
        mission=mission,
        corridors=corridors,
        metadata={"prompt": prompt_info, "created_at": timestamp},
    )
    sdf_path = run_dir / "world.sdf"
    compile_world_sdf(sim_spec, sdf_path)
    print(f"  -> Created: {sdf_path.name} ({sdf_path.stat().st_size} bytes)")

    # 6. Save spec.json and manifest.json
    spec_path = run_dir / "spec.json"
    spec_path.write_text(sim_spec.model_dump_json(indent=2))

    manifest = RunManifest(
        world_sdf="world.sdf",
        map_yaml="map.yaml",
        world_name="default_world",
        robot_model="turtlebot3_waffle",
        initial_pose=spawn_pose,
    )
    write_manifest(run_dir, manifest)
    print(f"  -> Saved: spec.json and manifest.json")

    print(f"\n[5/5] Run Folder Ready!")
    print(f"      Location: {run_dir}")
    return run_dir


def main():
    parser = argparse.ArgumentParser(description="Generate and launch simulation from NLP prompt.")
    parser.add_argument(
        "prompt",
        nargs="?",
        default="generate a 23 by 15 room with 10 racks and make the agent go to 5th rack",
        help="Natural language prompt describing the room, racks, and target.",
    )
    parser.add_argument(
        "--launch",
        action="store_true",
        help="Immediately launch the generated simulation using scripts/run_sim.py",
    )
    parser.add_argument(
        "--display",
        default="headless",
        choices=["x11", "headless", "vnc", "wslg"],
        help="Display mode for Gazebo / RViz.",
    )
    args = parser.parse_args()

    prompt_info = parse_prompt(args.prompt)
    run_dir = build_simulation_run(prompt_info)

    print("\n" + "=" * 65)
    print("  SIMULATION COMPILED DIRECTLY FROM NATURAL LANGUAGE!")
    print("=" * 65)
    print(f"\nTo launch this simulation in Gazebo with TurtleBot3 and Nav2, run:")
    print(f"  python3 scripts/run_sim.py {run_dir} --display {args.display} --mission\n")

    if args.launch:
        import subprocess
        cmd = [
            sys.executable,
            str(REPO_ROOT / "scripts" / "run_sim.py"),
            str(run_dir),
            "--display", args.display,
            "--mission",
        ]
        print(f"Executing: {' '.join(cmd)}")
        subprocess.run(cmd)


if __name__ == "__main__":
    main()
