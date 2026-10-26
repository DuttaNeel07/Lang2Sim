"""
World pre-flight check and normalizer, including Part 1's compiled output.
File: tests/test_world_check.py
"""

from pathlib import Path

import pytest

from sim2lang.runtime.world_check import (
    NORMALIZED_NAME,
    REQUIRED_SYSTEMS,
    check_world,
    normalize_world,
    prepare_world,
)

ALL_PLUGINS = "".join(f'<plugin filename="{f}" name="{c}"/>' for _, f, c, _ in REQUIRED_SYSTEMS)


def sdf(body: str, version: str = "1.9", name: str = "w") -> str:
    return f'<?xml version="1.0"?><sdf version="{version}"><world name="{name}">{body}</world></sdf>'


def codes(report):
    return {i.code for i in report.issues}


def test_complete_world_is_clean():
    r = check_world(sdf(ALL_PLUGINS + '<model name="box"><pose>1 2 0.5 0 0 0</pose></model>'))
    assert r.ok and not r.issues and r.world_name == "w"


def test_missing_systems_are_fixed_not_errors():
    text = sdf('<plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>')
    r = check_world(text)
    assert r.ok and "sensors" in r.missing_systems and "imu" in r.missing_systems
    assert "physics" not in r.missing_systems and codes(r) == {"systems"}
    fixed = normalize_world(text, r.missing_systems)
    again = check_world(fixed)
    assert again.ok and not again.missing_systems
    assert "<render_engine>ogre2</render_engine>" in fixed


@pytest.mark.parametrize("pose", ["04.0 1.25 0 0 0", "1 2 3", "a b c d e f"])
def test_malformed_pose_is_error(pose):
    r = check_world(sdf(ALL_PLUGINS + f'<model name="wall_south"><pose>{pose}</pose></model>'))
    assert not r.ok and "pose" in codes(r)


def test_quaternion_pose_allowed():
    r = check_world(sdf(ALL_PLUGINS + '<model name="m"><pose rotation_format="quat_xyzw">0 0 0 0 0 0 1</pose></model>'))
    assert r.ok


def test_robot_in_world_rejected_but_tb3_world_model_allowed():
    r = check_world(sdf(ALL_PLUGINS + '<include><uri>model://turtlebot3_waffle</uri></include>'))
    assert "robot-in-world" in codes(r)
    r = check_world(sdf(ALL_PLUGINS + '<include><uri>model://turtlebot3_world</uri></include>'))
    assert "robot-in-world" not in codes(r)


def test_structure_errors_and_versions():
    assert "xml" in codes(check_world("<sdf><world"))
    assert "root" in codes(check_world("<world name='x'/>"))
    assert "world-count" in codes(check_world('<sdf version="1.9"></sdf>'))
    assert "world-name" in codes(check_world('<sdf version="1.9"><world/></sdf>'))
    assert "sdf-version" in codes(check_world(sdf(ALL_PLUGINS, version="1.6")))


def test_model_resolution(tmp_path):
    (tmp_path / "rack").mkdir()
    body = ALL_PLUGINS + "<include><uri>model://rack</uri></include><include><uri>model://ghost</uri></include>"
    r = check_world(sdf(body), model_paths=[tmp_path])
    assert [i.message for i in r.errors if i.code == "model-missing"] == [f"model://ghost not found in {tmp_path}"]
    assert check_world(sdf(body)).ok  # no search paths given: not checked
    fuel = check_world(sdf(ALL_PLUGINS + "<include><uri>https://fuel.gazebosim.org/1.0/x/models/Sun</uri></include>"))
    assert fuel.ok and "fuel" in codes(fuel)


def test_prepare_world_writes_normalized_copy(tmp_path):
    w = tmp_path / "world.sdf"
    w.write_text(sdf(""))
    report, path = prepare_world(w)
    assert report.ok and path == tmp_path / NORMALIZED_NAME and w.read_text() == sdf("")
    w.write_text(sdf(ALL_PLUGINS))
    assert prepare_world(w)[1] == w
    w.write_text(sdf(ALL_PLUGINS + '<model name="m"><pose>1 2</pose></model>'))
    report, path = prepare_world(w)
    assert path is None and "REJECTED" in report.summary()


def test_part1_compiled_world_passes_after_normalizing(tmp_path):
    """Part 1's Harmonic template: valid poses, and Sensors/Imu get added."""
    pytest.importorskip("jinja2")
    pytest.importorskip("shapely")
    from sim2lang.compiler import compile_world_sdf
    from sim2lang.core.models import (
        EnvironmentSpec,
        MissionSpec,
        Pose2D,
        SimSpec,
        Vector3D,
    )

    spec = SimSpec(environment=EnvironmentSpec(room_dimensions=Vector3D(x=10, y=8, z=3)),
                   mission=MissionSpec(initial_pose=Pose2D(x=-3, y=-2), goal_pose=Pose2D(x=3, y=2)))
    world = compile_world_sdf(spec, tmp_path / "world.sdf", "warehouse")
    report, path = prepare_world(world)
    assert report.ok, report.summary()
    assert set(report.missing_systems) == {"sensors", "imu"}
    assert path is not None and Path(path).name == NORMALIZED_NAME
