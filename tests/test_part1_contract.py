"""
Part 1 -> Part 3 contract: Part 1's map must match its world as Nav2 reads it.
File: tests/test_part1_contract.py

Nav2's map_server reads the PGM's first row as the map's maximum y. These tests
place one obstacle off-centre and check it appears where the world has it.
"""

import pytest

pytest.importorskip("shapely")

from lang2sim.core.models import ObjectSpec, Pose2D, Vector3D  # noqa: E402
from lang2sim.world_gen import slice_map_from_layout  # noqa: E402


def _pgm(path):
    data = path.read_bytes()
    parts = data.split(b"\n", 3)
    w, h = (int(v) for v in parts[1].split())
    return parts[3][: w * h], w, h


def _pixel(pix, w, h, x, y, origin, res=0.05):
    c = int((x - origin[0]) / res)
    r = h - 1 - int((y - origin[1]) / res)   # row 0 = top = max y (Nav2 convention)
    return pix[r * w + c]


@pytest.mark.parametrize("ox,oy", [(2.5, 2.0), (-3.0, -1.5), (3.0, -2.5)])
def test_obstacle_lands_where_the_world_has_it(tmp_path, ox, oy):
    box = ObjectSpec(id="b", model_name="wooden_crate", dimensions=Vector3D(x=0.8, y=0.8, z=0.8),
                     pose=Pose2D(x=ox, y=oy))
    _, yaml_path = slice_map_from_layout(Vector3D(x=10, y=8, z=3), [box], tmp_path)
    pix, w, h = _pgm(tmp_path / "map.pgm")
    origin = (-5.0, -4.0)
    assert _pixel(pix, w, h, ox, oy, origin) == 0, "obstacle missing at its world position"
    assert _pixel(pix, w, h, ox, -oy, origin) == 254, "obstacle mirrored across y = 0"
