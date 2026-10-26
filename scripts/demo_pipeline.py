#!/usr/bin/env python3
"""
Full-pipeline demo: scenario spec -> Part 1 world + map -> run folder -> Part 3 sim -> Nav2 mission.

    python scripts/demo_pipeline.py            # Gazebo + RViz windows on Ubuntu
    python scripts/demo_pipeline.py --display vnc   # macOS: http://localhost:6080

The LLM step (Part 2) is not wired yet, so the parsed scenario below stands in for
what the agent will produce from: "a warehouse with racks and crates, drive from
the dock past the racks to the far corner".
"""

import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))

import importlib  # noqa: E402

from lang2sim.compiler import compile_world_sdf, generate_simulation_ir  # noqa: E402
from lang2sim.core.models import (  # noqa: E402
    EnvironmentSpec,
    MissionSpec,
    ObjectSpec,
    Pose2D,
    RunManifest,
    SimSpec,
    Vector3D,
)
from lang2sim.runtime.run_folder import write_manifest  # noqa: E402
from lang2sim.world_gen import slice_map_from_layout  # noqa: E402

cfg = importlib.import_module("lang2sim.core.config")


def box(i, model, x, y, dx, dy, dz):
    return ObjectSpec(id=i, model_name=model, dimensions=Vector3D(x=dx, y=dy, z=dz), pose=Pose2D(x=x, y=y))


def main() -> int:
    print("[1/4] Scenario (what the agent will output): warehouse 12x10 m, 4 racks, 1 crate, 4-stop mission")
    spec = SimSpec(
        environment=EnvironmentSpec(room_dimensions=Vector3D(x=12, y=10, z=3)),
        objects=[box("rack_1", "aws_warehouse_rack", -2.5, 2.5, 3.0, 0.8, 2.0),
                 box("rack_2", "aws_warehouse_rack", 2.5, 2.5, 3.0, 0.8, 2.0),
                 box("rack_3", "aws_warehouse_rack", -2.5, -0.5, 3.0, 0.8, 2.0),
                 box("rack_4", "aws_warehouse_rack", 2.5, -0.5, 3.0, 0.8, 2.0),
                 box("crate_1", "wooden_crate", 4.5, -3.5, 0.8, 0.8, 0.8)],
        mission=MissionSpec(initial_pose=Pose2D(x=-4.5, y=-3.5),
                            waypoints=[Pose2D(x=0, y=1), Pose2D(x=4.5, y=1), Pose2D(x=0, y=-3)],
                            goal_pose=Pose2D(x=-4, y=3.5, yaw=1.5708), timeout_seconds=300))

    print("[2/4] Part 1: layout solver + physical validation")
    ir = generate_simulation_ir(spec)
    if not ir.is_valid:
        print("  rejected:", ir.validation_messages)
        return 1
    print("  valid: no overlaps, spawn and goal clear")

    run = cfg.get_run_dir("pipeline_demo")
    print(f"[3/4] Part 1: Gazebo Harmonic world + Nav2 map -> {run}")
    compile_world_sdf(ir.spec, run / "world.sdf", "pipeline_demo")
    slice_map_from_layout(ir.spec.environment.room_dimensions, ir.spec.objects, run)
    (run / "spec.json").write_text(ir.spec.model_dump_json(indent=2))
    write_manifest(run, RunManifest(world_name="pipeline_demo", initial_pose=ir.spec.mission.initial_pose))

    print("[4/4] Part 3: container -> pre-flight -> Gazebo + TurtleBot3 + Nav2 -> mission")
    cmd = [sys.executable, str(REPO / "scripts" / "run_sim.py"), str(run), "--mission", *sys.argv[1:]]
    return subprocess.call(cmd)


if __name__ == "__main__":
    sys.exit(main())
