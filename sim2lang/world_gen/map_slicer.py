"""
Zero-SLAM 2D Occupancy Map Slicer.
File: sim2lang/world_gen/map_slicer.py

Converts 3D world geometric bounds and Shapely object polygons directly into
standard Nav2 P5 Portable Gray Map (map.pgm) and YAML metadata (map.yaml)
in < 2 seconds, completely eliminating Cartographer / SLAM overhead.
"""

import math
from pathlib import Path
from typing import List, Tuple, Optional
from shapely.geometry import Polygon, Point

from sim2lang.core.models import Vector3D, ObjectSpec
from sim2lang.core.config import MAP_RESOLUTION


class ZeroSlamMapSlicer:
    """
    Direct geometric rasterizer converting room and obstacle polygons
    into 2D occupancy grids for Nav2.
    """

    def __init__(self, resolution: float = MAP_RESOLUTION):
        self.resolution = resolution  # 0.05 m/pixel

    def slice_and_export(
        self,
        room_dim: Vector3D,
        objects: List[ObjectSpec],
        output_dir: Path,
        map_name: str = "map",
    ) -> Tuple[Path, Path]:
        """
        Rasters the 2D floorplan and writes map.pgm + map.yaml.
        """
        output_dir.mkdir(parents=True, exist_ok=True)
        pgm_path = output_dir / f"{map_name}.pgm"
        yaml_path = output_dir / f"{map_name}.yaml"

        width_m = room_dim.x
        height_m = room_dim.y

        cols = int(math.ceil(width_m / self.resolution))
        rows = int(math.ceil(height_m / self.resolution))

        origin_x = -(width_m / 2.0)
        origin_y = -(height_m / 2.0)

        # 254 = Free space (white), 0 = Occupied (black)
        # Using a flat bytearray of size rows * cols
        grid = bytearray([254] * (rows * cols))

        def set_pixel(c: int, r: int, val: int):
            if 0 <= c < cols and 0 <= r < rows:
                grid[r * cols + c] = val

        # 1. Draw outer walls (wall thickness = 0.2m -> 4 pixels)
        wall_px = max(2, int(0.20 / self.resolution))
        for r in range(rows):
            for c in range(cols):
                if (
                    r < wall_px
                    or r >= rows - wall_px
                    or c < wall_px
                    or c >= cols - wall_px
                ):
                    set_pixel(c, r, 0)

        # 2. Rasterize each obstacle
        for obj in objects:
            cx = obj.pose.x
            cy = obj.pose.y
            dx = obj.dimensions.x
            dy = obj.dimensions.y
            yaw = obj.pose.yaw

            # Create Shapely OBB for precise polygon point-in-polygon containment
            from sim2lang.world_gen.shapely_solver import ShapelySpatialSolver
            solver = ShapelySpatialSolver()
            poly = solver.create_obb(cx, cy, dx, dy, yaw)
            min_x, min_y, max_x, max_y = poly.bounds

            c_min = max(0, int((min_x - origin_x) / self.resolution))
            c_max = min(cols, int(math.ceil((max_x - origin_x) / self.resolution)))
            r_min = max(0, int((min_y - origin_y) / self.resolution))
            r_max = min(rows, int(math.ceil((max_y - origin_y) / self.resolution)))

            for r in range(r_min, r_max):
                for c in range(c_min, c_max):
                    world_x = origin_x + (c + 0.5) * self.resolution
                    world_y = origin_y + (r + 0.5) * self.resolution
                    if poly.contains(Point(world_x, world_y)):
                        set_pixel(c, r, 0)

        # 3. Write P5 Grayscale PGM file directly
        # Standard P5 format: Header\n<cols> <rows>\n<max_val>\n<raw_bytes>
        header = f"P5\n{cols} {rows}\n255\n".encode("ascii")
        with open(pgm_path, "wb") as f:
            f.write(header)
            f.write(grid)

        # 4. Write Nav2 map.yaml metadata
        yaml_content = f"""image: {map_name}.pgm
mode: trinary
resolution: {self.resolution}
origin: [{origin_x:.3f}, {origin_y:.3f}, 0.000]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
"""
        with open(yaml_path, "w", encoding="utf-8") as f:
            f.write(yaml_content)

        return pgm_path, yaml_path


def slice_map_from_layout(
    room_dim: Vector3D,
    objects: List[ObjectSpec],
    output_dir: Path,
    map_name: str = "map",
    resolution: float = MAP_RESOLUTION,
) -> Tuple[Path, Path]:
    """Convenience function to generate zero-SLAM map assets."""
    slicer = ZeroSlamMapSlicer(resolution=resolution)
    return slicer.slice_and_export(room_dim, objects, output_dir, map_name)
