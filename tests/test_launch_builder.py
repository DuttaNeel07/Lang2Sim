"""
Launch generator: renders valid Python with this run's settings.
File: tests/test_launch_builder.py
"""

import ast
import math

import pytest

from lang2sim.runtime.launch_builder import LAUNCH_FILE, LaunchConfig, render, write


def cfg(**kw):
    base = dict(run_dir="/lang2sim/projects/p/runs/1", world_sdf="/lang2sim/projects/p/runs/1/world.l2s.sdf",
                world_name="warehouse", map_yaml="/lang2sim/projects/p/runs/1/map.yaml",
                x=-3.0, y=-2.0, yaw=math.pi / 2)
    base.update(kw)
    return LaunchConfig(**base)


def _config_of(src: str) -> dict:
    tree = ast.parse(src)
    for node in tree.body:
        if isinstance(node, ast.Assign) and getattr(node.targets[0], "id", "") == "CONFIG":
            return ast.literal_eval(node.value)
    raise AssertionError("no CONFIG in generated launch file")


def test_render_is_valid_python_with_config():
    src = render(cfg(resource_paths=["/lang2sim/catalog/models"]))
    c = _config_of(src)
    assert c["world_name"] == "warehouse" and c["x"] == -3.0 and c["yaw"] == pytest.approx(math.pi / 2)
    assert c["resource_paths"] == ["/lang2sim/catalog/models"]
    assert "def generate_launch_description" in src


def test_render_contains_every_component():
    src = render(cfg())
    for needle in ("gz_sim.launch.py", "robot_state_publisher", '"create"', "parameter_bridge",
                   "image_bridge", "bringup_launch.py", "set_initial_pose", "rviz2", '"-world"', '"-Y"'):
        assert needle in src, needle


def test_flags_are_baked_in():
    c = _config_of(render(cfg(gui=False, rviz=False, camera=False, nav2_params="/x/nav2.yaml")))
    assert (c["gui"], c["rviz"], c["camera"], c["nav2_params"]) == (False, False, False, "/x/nav2.yaml")


def test_world_name_required():
    with pytest.raises(ValueError):
        render(cfg(world_name=""))


def test_write(tmp_path):
    path = write(cfg(run_dir=str(tmp_path)))
    assert path == tmp_path / LAUNCH_FILE
    ast.parse(path.read_text())
