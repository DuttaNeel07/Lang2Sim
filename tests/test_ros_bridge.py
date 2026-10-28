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
    assert rtf_from_samples([(10.0, 1.0), (10.1, 1.1)]) is None  # span too short
    assert rtf_from_samples([(10.0, 1.0), (12.0, 2.0)]) == pytest.approx(0.5)
    assert rtf_from_samples([(10.0, 5.0), (11.0, 7.0)]) == pytest.approx(2.0)


def test_readiness_all_green_and_stale():
    now = 100.0
    s = RosSnapshot(
        now=now,
        clock_wall=now - 0.1,
        odom_wall=now - 0.2,
        scan_wall=now - 0.5,
        map_pose=(0.0, 0.0, 0.0),
        lifecycle={n: "active" for n in NAV2_NODES},
    )
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


# --- telemetry plumbing (no rclpy: drive the callbacks with stand-in messages) ----------------
import collections  # noqa: E402
import threading  # noqa: E402
from types import SimpleNamespace as NS  # noqa: E402

from lang2sim.runtime.ros_bridge import contact_pairs, stamp_s  # noqa: E402
from lang2sim.runtime.telemetry_recorder import ContactMonitor, TelemetryRecorder  # noqa: E402


def test_stamp_and_contact_pairs():
    assert stamp_s(NS(sec=12, nanosec=500_000_000)) == pytest.approx(12.5)
    pairs = contact_pairs(
        [
            ("waffle::base_link::base_collision", "crate_1::link::collision", 7.0),
            ("rack_2::link::collision", "waffle::base_link::base_collision", None),
            ("waffle::a::c", "waffle::b::c", 3.0),
        ],
        robot="waffle",
    )
    assert pairs == [("crate_1::link::collision", 7.0), ("rack_2::link::collision", None)]


def _bridge(map_pose=None):
    b = object.__new__(RosBridge)  # skip rclpy init; set what the callbacks use
    b._lock = threading.Lock()
    b._s = RosSnapshot(spawn=(1.0, 2.0, 0.0))
    b._clock_samples = collections.deque(maxlen=10)
    b.robot_name = "waffle"
    b.recorder = TelemetryRecorder(ContactMonitor(robot_name="waffle"))
    b._map_pose_now = lambda: map_pose
    b.expect_ground_truth = False
    b._record_wall0 = 0.0
    return b


def _odom(t, x, y=0.0, v=0.2):
    return NS(
        header=NS(stamp=NS(sec=int(t), nanosec=int(round((t % 1) * 1e9)))),
        pose=NS(pose=NS(position=NS(x=x, y=y, z=0.0), orientation=NS(x=0.0, y=0.0, z=0.0, w=1.0))),
        twist=NS(twist=NS(linear=NS(x=v, y=0.0, z=0.0), angular=NS(x=0.0, y=0.0, z=0.05))),
    )


def test_bridge_records_ground_truth_in_sim_time_with_amcl_estimate():
    b = _bridge(map_pose=(3.05, 2.0, 0.0))
    b.expect_ground_truth = True  # instrumented robot: never mix in odom/map poses
    b.record_start("nominal", goal=(5.0, 2.0, 0.0), rate_hz=10)
    for i in range(20):
        t = 40.0 + 0.05 * i
        b._on_odom(_odom(t, 0.01 * i))  # odom ignored while ground truth is fresh
        b._on_ground_truth(_odom(t, 3.0 + 0.01 * i, 2.0))
    data = b.record_stop()
    assert data["meta"]["pose_source"] == "ground_truth"
    assert data["samples"][0]["t"] == pytest.approx(40.0)
    assert data["samples"][0]["x"] == pytest.approx(3.0) and "slam_x" in data["samples"][0]
    assert 9 <= len(data["samples"]) <= 11


def test_bridge_falls_back_to_odom_plus_spawn_without_ground_truth():
    b = _bridge(map_pose=None)
    b.record_start("nominal")
    for i in range(5):
        b._on_odom(_odom(10.0 + 0.1 * i, 0.5))
    data = b.record_stop()
    assert data["meta"]["pose_source"] == "odom"
    assert data["samples"][0]["x"] == pytest.approx(1.5) and data["samples"][0][
        "y"
    ] == pytest.approx(2.0)


def test_bridge_counts_contacts_and_reports_live_fields():
    b = _bridge()
    b._on_scan(NS(ranges=[math.inf, 0.9, 0.4], range_min=0.12, range_max=3.5))
    wrench = NS(body_1_wrench=NS(force=NS(x=6.0, y=8.0, z=0.0)))
    msg = NS(
        header=NS(stamp=NS(sec=7, nanosec=0)),
        contacts=[
            NS(
                collision1=NS(name="waffle::base_link::base_collision"),
                collision2=NS(name="crate::link::c"),
                wrenches=[wrench],
            )
        ],
    )
    b._on_contacts(msg)
    b.snapshot = lambda: RosSnapshot(
        now=1.0,
        sim_time=7.0,
        rtf=1.0,
        scan_min=0.4,
        spawn=(1.0, 2.0, 0.0),
        odom_pose=(0.0, 0.0, 0.0),
        velocity=(0.1, 0.0, 0.0),
    )
    live = b.live(goal=(4.0, 6.0, 0.0))
    assert live["collision_count"] == 1 and live["last_contact_force_n"] == pytest.approx(10.0)
    assert live["contact_force_source"] == "wrench"
    assert live["pose_source"] == "odom" and live["distance_to_goal_m"] == pytest.approx(5.0)
    assert live["min_scan_range_m"] == pytest.approx(0.4)


def test_bridge_uses_odom_when_expected_ground_truth_never_arrives():
    b = _bridge(map_pose=(1.0, 2.0, 0.0))
    b.expect_ground_truth = True
    b.record_start("nominal")
    b._on_odom(_odom(1.0, 0.0))
    assert b.recorder.sample_count == 0  # within the grace window: wait for ground truth
    b._record_wall0 -= 5.0  # ground truth silent for 5 s
    b._on_odom(_odom(1.1, 0.0))
    data = b.record_stop()
    assert data["meta"]["pose_source"] == "map" and len(data["samples"]) == 1
