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


# --- Nav2 tuning (robust profile) ---------------------------------------------------------------
MINI_TB3 = """
amcl: {ros__parameters: {}}
bt_navigator: {ros__parameters: {default_nav_to_pose_bt_xml: stock.xml}}
controller_server:
  ros__parameters:
    controller_frequency: 10.0
    progress_checker: {plugin: nav2_controller::SimpleProgressChecker}
    FollowPath: {plugin: dwb_core::DWBLocalPlanner}
local_costmap: {local_costmap: {ros__parameters: {robot_radius: 0.15, width: 3, height: 3,
  plugins: [obstacle_layer, inflation_layer], inflation_layer: {inflation_radius: 1.0}}}}
global_costmap: {global_costmap: {ros__parameters: {robot_radius: 0.15, update_frequency: 1.0,
  plugins: [static_layer, obstacle_layer, inflation_layer], inflation_layer: {inflation_radius: 1.0}}}}
collision_monitor: {ros__parameters: {FootprintApproach: {time_before_collision: 2.0}}}
"""


def _nav_params(tmp_path, monkeypatch, profile):
    """Run the generated launch file's _nav2_params() against a minimal TurtleBot3 params file."""
    import sys
    import types

    import yaml

    (tmp_path / "param").mkdir()
    (tmp_path / "param" / "waffle.yaml").write_text(MINI_TB3)
    src = render(cfg(run_dir=str(tmp_path)))
    head = src.split("def generate_launch_description")[0]
    stub = types.SimpleNamespace(LaunchDescription=None, AppendEnvironmentVariable=None,
                                 IncludeLaunchDescription=None, TimerAction=None,
                                 PythonLaunchDescriptionSource=None, Node=None, get_package_share_directory=None)
    for mod in ("launch", "launch.actions", "launch.launch_description_sources", "launch_ros.actions",
                "ament_index_python.packages"):
        monkeypatch.setitem(sys.modules, mod, stub)
    monkeypatch.setenv("L2S_NAV_PROFILE", profile)
    ns: dict = {}
    exec(compile(head, "sim_launch.py", "exec"), ns)
    return yaml.safe_load(open(ns["_nav2_params"](str(tmp_path))))


def test_robust_profile_uses_the_real_footprint_and_faster_replanning(tmp_path, monkeypatch):
    p = _nav_params(tmp_path, monkeypatch, "robust")
    for key in ("local_costmap", "global_costmap"):
        cm = p[key][key]["ros__parameters"]
        assert "robot_radius" not in cm and "-0.197" in cm["footprint"]           # the Waffle chassis outline
        assert cm["inflation_layer"]["inflation_radius"] == 0.55                 # path keeps to the middle of a gap
    assert p["global_costmap"]["global_costmap"]["ros__parameters"]["update_frequency"] == 2.0
    ctl = p["controller_server"]["ros__parameters"]
    rpp = ctl["FollowPath"]
    assert rpp["plugin"].endswith("RegulatedPurePursuitController") and rpp["use_rotate_to_heading"]
    assert rpp["use_collision_detection"] and rpp["inflation_cost_scaling_factor"] == 3.0  # matches the costmaps
    planner = p["planner_server"]["ros__parameters"]
    assert planner["GridBased"]["plugin"] == "nav2_smac_planner::SmacPlanner2D"
    assert planner["GridBased"]["cost_travel_multiplier"] > 1.0
    assert ctl["progress_checker"]["plugin"] == "nav2_controller::PoseProgressChecker"
    bt = p["bt_navigator"]["ros__parameters"]["default_nav_to_pose_bt_xml"]
    xml = (tmp_path / "navigate_l2s.xml").read_text()
    assert bt == str(tmp_path / "navigate_l2s.xml") and 'RateController hz="2.0"' in xml
    assert xml.index("<BackUp") < xml.index("<Spin")                              # back up before spinning
    assert p["collision_monitor"]["ros__parameters"]["FootprintApproach"]["time_before_collision"] == 1.5


def test_stock_profile_keeps_the_turtlebot3_settings(tmp_path, monkeypatch):
    p = _nav_params(tmp_path, monkeypatch, "stock")
    assert p["controller_server"]["ros__parameters"]["FollowPath"]["plugin"] == "dwb_core::DWBLocalPlanner"
    assert p["local_costmap"]["local_costmap"]["ros__parameters"]["robot_radius"] == 0.15
    assert not (tmp_path / "navigate_l2s.xml").exists()


def test_robust_profile_aims_inside_the_mission_tolerance_and_scores_against_it(tmp_path, monkeypatch):
    import json

    from lang2sim.telemetry.benchmark import scoring_params

    p = _nav_params(tmp_path, monkeypatch, "robust")
    ctl = p["controller_server"]["ros__parameters"]
    assert ctl["goal_checker"]["xy_goal_tolerance"] == 0.12                       # Nav2 drives closer...
    amcl = p["amcl"]["ros__parameters"]
    assert amcl["alpha1"] == 0.05 and amcl["update_min_d"] == 0.1                 # ...and localizes tighter
    side = json.loads((tmp_path / "scoring.l2s.json").read_text())
    assert side == {"goal_tolerance_m": 0.25, "goal_tolerance_source": "mission", "nav2_goal_checker_m": 0.12}
    params, scoring = scoring_params(tmp_path)                                   # ...the score keeps 0.25 m
    assert params.eps_pos == 0.25 and scoring["goal_tolerance_source"] == "mission"
    assert scoring_params(tmp_path, 0.15)[0].eps_pos == 0.15                       # explicit still wins
