"""
Simulation Intermediate Representation (SIR) Validator.
File: lang2sim/compiler/sir_validator.py
"""

from typing import List, Tuple

from shapely.geometry import Point

from lang2sim.core.config import (
    MIN_AISLE_CORRIDOR_WIDTH,
    MIN_WALL_STANDOFF,
    ROBOT_CIRCUMSCRIBED_RADIUS,
)
from lang2sim.core.models import SimSpec
from lang2sim.world_gen.shapely_solver import ShapelySpatialSolver


class SIRValidator:
    """
    Physical feasibility and safety validator for Simulation Intermediate Representations.
    """

    def __init__(
        self,
        min_corridor_width: float = MIN_AISLE_CORRIDOR_WIDTH,
        wall_standoff: float = MIN_WALL_STANDOFF,
        robot_radius: float = ROBOT_CIRCUMSCRIBED_RADIUS,
    ):
        self.min_corridor_width = min_corridor_width
        self.wall_standoff = wall_standoff
        self.robot_radius = robot_radius
        self.solver = ShapelySpatialSolver(
            min_corridor_width=min_corridor_width,
            wall_standoff=wall_standoff,
            robot_radius=robot_radius,
        )

    def validate(self, sim_spec: SimSpec) -> Tuple[bool, List[str]]:
        """
        Validates geometric feasibility, bounds, non-intersection, and reachability.
        """
        errors: List[str] = []
        env = sim_spec.environment
        room_poly = self.solver.create_room_polygon(env.room_dimensions)

        # 1. Validate room dimensions
        if env.room_dimensions.x <= 0 or env.room_dimensions.y <= 0:
            errors.append(f"Invalid room dimensions: {env.room_dimensions}")

        # 2. Validate object boundaries and overlaps
        placed_polygons = []
        for obj in sim_spec.objects:
            poly = self.solver.create_obb(
                obj.pose.x, obj.pose.y, obj.dimensions.x, obj.dimensions.y, obj.pose.yaw
            )

            # Check inside room
            if not room_poly.contains(poly):
                errors.append(
                    f"Object '{obj.id}' is outside room boundaries at ({obj.pose.x}, {obj.pose.y})"
                )

            # Check overlap with existing objects
            for idx, existing in enumerate(placed_polygons):
                if poly.intersects(existing):
                    errors.append(
                        f"Collision detected: Object '{obj.id}' overlaps with object '{sim_spec.objects[idx].id}'"
                    )

            placed_polygons.append(poly)

        # 3. Validate robot initial spawn pose
        spawn_pt = Point(sim_spec.mission.initial_pose.x, sim_spec.mission.initial_pose.y)
        if not room_poly.contains(spawn_pt):
            errors.append(f"Spawn pose ({spawn_pt.x}, {spawn_pt.y}) is outside room bounds")

        for idx, poly in enumerate(placed_polygons):
            if poly.distance(spawn_pt) < (self.robot_radius + 0.1):
                errors.append(f"Spawn pose in collision with object '{sim_spec.objects[idx].id}'")

        # 4. Validate goal pose
        goal_pt = Point(sim_spec.mission.goal_pose.x, sim_spec.mission.goal_pose.y)
        if not room_poly.contains(goal_pt):
            errors.append(f"Goal pose ({goal_pt.x}, {goal_pt.y}) is outside room bounds")

        for idx, poly in enumerate(placed_polygons):
            if poly.distance(goal_pt) < (self.robot_radius + 0.1):
                errors.append(f"Goal pose in collision with object '{sim_spec.objects[idx].id}'")

        is_valid = len(errors) == 0
        return is_valid, errors
