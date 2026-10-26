"""
Pure logic of the ROS bridge (no ROS needed).
File: tests/test_ros_bridge.py
"""

import math

import pytest

from lang2sim.runtime.ros_bridge import (
    NAV2_NODES,
    RosBridge,
    RosSnapshot,
    readiness,
    rtf_from_samples,
    yaw_from_quaternion,
)


@pytest.mark.parametrize("yaw", [0.0, 0.7, -2.0, math.pi])
def test_yaw_from_quaternion(yaw):
    got = yaw_from_quaternion(0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))
    assert math.isclose(abs(got), abs(yaw), abs_tol=1e-9)


def test_rtf():
    assert rtf_from_samples([]) is None
    assert rtf_from_samples([(10.0, 1.0), (10.1, 1.1)]) is None   # span too short
    assert rtf_from_samples([(10.0, 1.0), (12.0, 2.0)]) == pytest.approx(0.5)
    assert rtf_from_samples([(10.0, 5.0), (11.0, 7.0)]) == pytest.approx(2.0)


def test_readiness_all_green_and_stale():
    now = 100.0
    s = RosSnapshot(now=now, clock_wall=now - 0.1, odom_wall=now - 0.2, scan_wall=now - 0.5,
                    map_pose=(0.0, 0.0, 0.0), lifecycle={n: "active" for n in NAV2_NODES})
    assert all(readiness(s).values())
    s.scan_wall = now - 10
    s.lifecycle["amcl"] = "inactive"
    r = readiness(s)
    assert r["laser_scan"] is False and r["nav2_active"] is False and r["odometry"] is True
    assert readiness(RosSnapshot(now=now)) == {k: False for k in r}


def test_try_create_without_ros_returns_none():
    try:
        import rclpy  # noqa: F401
    except ImportError:
        assert RosBridge.try_create() is None
    else:
        pytest.skip("ROS is installed here")
