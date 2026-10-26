#!/usr/bin/env python3
"""
Regenerate the Part 3 fixture runs in catalog/fixtures/.
File: scripts/fixtures/make_fixtures.py

    python scripts/fixtures/make_fixtures.py

Each fixture is a complete run folder (decision 1): world.sdf, map.yaml/.pgm,
spec.json (with a mission for the navigation tests) and manifest.json.

* empty_room, small_warehouse: generated with Part 1's own pipeline
  (SIRGenerator -> HarmonicSdfCompiler -> ZeroSlamMapSlicer), so they follow
  the real contract. Needs the host deps (jinja2, shapely).
* tb3_world: the TurtleBot3 world and map (ROBOTIS, Apache-2.0) copied from the
  sim image's turtlebot3_gazebo / turtlebot3_navigation2 packages; only its
  spec.json and manifest.json are written here.

Every mission waypoint is checked for >= 0.5 m clearance from obstacles.
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import List, Sequence, Tuple

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))

from sim2lang.core.models import (  # noqa: E402
    EnvironmentSpec,
    MissionSpec,
    ObjectSpec,
    Pose2D,
    RunManifest,
    SimSpec,
    Vector3D,
)
from sim2lang.runtime.run_folder import write_manifest  # noqa: E402

OUT = REPO / "catalog" / "fixtures"
MIN_CLEARANCE = 0.5


def obj(i: str, model: str, x: float, y: float, dx: float, dy: float, dz: float, yaw: float = 0.0) -> ObjectSpec:
    return ObjectSpec(id=i, model_name=model, dimensions=Vector3D(x=dx, y=dy, z=dz), pose=Pose2D(x=x, y=y, yaw=yaw))


def mission(start: Tuple[float, float, float], waypoints: Sequence[Tuple[float, float]],
            goal: Tuple[float, float, float]) -> MissionSpec:
    return MissionSpec(initial_pose=Pose2D(x=start[0], y=start[1], yaw=start[2]),
                       waypoints=[Pose2D(x=x, y=y) for x, y in waypoints],
                       goal_pose=Pose2D(x=goal[0], y=goal[1], yaw=goal[2]), timeout_seconds=300)


def check_clearance(spec: SimSpec) -> None:
    from shapely.geometry import Point

    from sim2lang.world_gen.shapely_solver import ShapelySpatialSolver

    solver = ShapelySpatialSolver()
    room = solver.create_room_polygon(spec.environment.room_dimensions)
    polys = [solver.create_obb(o.pose.x, o.pose.y, o.dimensions.x, o.dimensions.y, o.pose.yaw)
             for o in spec.objects]
    m = spec.mission
    for p in [m.initial_pose, *m.waypoints, m.goal_pose]:
        pt = Point(p.x, p.y)
        clear = min([room.exterior.distance(pt), *(poly.distance(pt) for poly in polys)])
        if clear < MIN_CLEARANCE:
            raise SystemExit(f"waypoint ({p.x}, {p.y}) has only {clear:.2f} m clearance")


def generate(name: str, room: Tuple[float, float], objects: List[ObjectSpec], m: MissionSpec) -> None:
    from sim2lang.compiler import compile_world_sdf, generate_simulation_ir
    from sim2lang.world_gen import slice_map_from_layout

    spec = SimSpec(environment=EnvironmentSpec(room_dimensions=Vector3D(x=room[0], y=room[1], z=3.0)),
                   objects=objects, mission=m)
    ir = generate_simulation_ir(spec)
    if not ir.is_valid:
        raise SystemExit(f"{name}: Part 1 validator rejected the layout: {ir.validation_messages}")
    check_clearance(ir.spec)
    run = OUT / name
    run.mkdir(parents=True, exist_ok=True)
    compile_world_sdf(ir.spec, run / "world.sdf", name)
    slice_map_from_layout(ir.spec.environment.room_dimensions, ir.spec.objects, run)
    (run / "spec.json").write_text(ir.spec.model_dump_json(indent=2) + "\n")
    write_manifest(run, RunManifest(world_name=name, initial_pose=m.initial_pose))
    print(f"wrote {run.relative_to(REPO)}")


def tb3_world() -> None:
    run = OUT / "tb3_world"
    if not (run / "world.sdf").is_file():
        raise SystemExit("catalog/fixtures/tb3_world/world.sdf missing: copy it from the sim image "
                         "(turtlebot3_gazebo/worlds/turtlebot3_world.world, map from turtlebot3_navigation2)")
    m = mission((-2.0, -0.5, 0.0), [(0.55, -0.55), (2.0, 0.5)], (0.5, 1.8, 3.14159))
    spec = SimSpec(environment=EnvironmentSpec(room_dimensions=Vector3D(x=5.6, y=5.2, z=1.0)), mission=m)
    (run / "spec.json").write_text(spec.model_dump_json(indent=2) + "\n")
    write_manifest(run, RunManifest(world_name="default", initial_pose=m.initial_pose))
    print(f"wrote {run.relative_to(REPO)} (spec + manifest)")


def main() -> int:
    generate("empty_room", (8.0, 6.0), [],
             mission((-2.5, -1.5, 0.0), [(2.5, -1.5), (2.5, 1.5)], (-2.5, 1.5, 3.14159)))
    generate("small_warehouse", (12.0, 10.0), [
        obj("rack_1", "aws_warehouse_rack", -2.5, 2.5, 3.0, 0.8, 2.0),
        obj("rack_2", "aws_warehouse_rack", 2.5, 2.5, 3.0, 0.8, 2.0),
        obj("rack_3", "aws_warehouse_rack", -2.5, -0.5, 3.0, 0.8, 2.0),
        obj("rack_4", "aws_warehouse_rack", 2.5, -0.5, 3.0, 0.8, 2.0),
        obj("crate_1", "wooden_crate", 4.5, -3.5, 0.8, 0.8, 0.8),
    ], mission((-4.5, -3.5, 0.0), [(0.0, 1.0), (4.5, 1.0), (0.0, -3.0)], (-4.0, 3.5, 1.5708)))
    tb3_world()
    return 0


if __name__ == "__main__":
    sys.exit(main())
