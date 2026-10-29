"""In-container telemetry recorder: pure logic, no ROS needed."""

import math

import pytest

from lang2sim.core.models import TelemetryLive
from lang2sim.runtime.telemetry_recorder import (
    ContactMonitor,
    TelemetryRecorder,
    best_pose,
    build_live,
    min_valid_range,
    world_from_odom,
)


def test_world_from_odom_applies_spawn_pose():
    # Spawned at (2, 1) facing +y; driving 1 m "forward" in odom moves +y in the world.
    x, y, yaw = world_from_odom((1.0, 0.0, 0.0), (2.0, 1.0, math.pi / 2))
    assert x == pytest.approx(2.0)
    assert y == pytest.approx(2.0)
    assert yaw == pytest.approx(math.pi / 2)


def test_best_pose_prefers_ground_truth_then_map_then_odom():
    gt, mp, od, sp = (1, 1, 0), (1.1, 1, 0), (0.5, 0, 0), (1.0, 0.0, 0.0)
    assert best_pose(gt, mp, od, sp) == (gt, "ground_truth")
    assert best_pose(None, mp, od, sp) == (mp, "map")
    pose, src = best_pose(None, None, od, sp)
    assert src == "odom" and pose[0] == pytest.approx(1.5)
    assert best_pose(None, None, None, sp) == (None, "none")


def test_min_valid_range_skips_inf_nan_and_out_of_range():
    assert min_valid_range([math.inf, 0.05, 2.0, math.nan, 0.8], 0.12, 3.5) == 0.8
    assert min_valid_range([math.inf, math.inf], 0.12, 3.5) is None


def test_contacts_count_rising_edges_in_sim_time():
    m = ContactMonitor(robot_name="waffle")
    assert m.process([("crate_1::link::collision", 12.0)], t=10.0) is True
    # Still touching: no new collision.
    assert m.process([("crate_1::link::collision", 12.0)], t=10.1) is False
    # Contact released (no messages for > release window), touches again after cooldown.
    m.tick(11.0)
    assert m.process([("crate_1::link::collision", 9.0)], t=11.0) is True
    assert m.count == 2 and m.force_source == "wrench"
    assert [e.t for e in m.events] == [10.0, 11.0]


def test_contacts_ignore_ground_self_and_light_touches():
    m = ContactMonitor(robot_name="waffle")
    assert not m.process([("ground_plane::link::collision", 50.0)], t=1.0)
    assert not m.process([("waffle::wheel_left_link::wheel_left_collision", 50.0)], t=1.1)
    assert not m.process([("rack_2::link::collision", 2.0)], t=1.2)  # below 5 N
    assert m.count == 0


def test_contacts_geometric_fallback_when_gazebo_reports_no_force():
    m = ContactMonitor(robot_name="waffle")
    assert m.process([("wall_3::link::collision", None)], t=4.0) is True
    assert m.force_source == "geometric"
    strict = ContactMonitor(robot_name="waffle", geometric_fallback=False)
    assert strict.process([("wall_3::link::collision", None)], t=4.0) is False


def _drive(rec, t0=100.0, n=50, dt=0.05, gt=True):
    for i in range(n):
        t = t0 + i * dt
        pose = (0.1 * i, 0.0, 0.0)
        rec.offer(
            t,
            pose,
            "ground_truth" if gt else "map",
            v=2.0,
            w=0.0,
            estimate=(0.1 * i + 0.02, 0.0, 0.0),
            scan_min=1.0 - 0.01 * i,
        )


def test_recorder_throttles_in_sim_time_and_outputs_trajectory_dict():
    from lang2sim.telemetry.trajectory import Trajectory

    rec = TelemetryRecorder(ContactMonitor(robot_name="waffle"))
    rec.start("nominal", goal=(5.0, 0.0, 0.0), rate_hz=10.0, spawn=(0.0, 0.0, 0.0))
    _drive(rec)  # 20 Hz offers over 2.45 s
    data = rec.stop()
    assert not rec.recording
    assert data["label"] == "nominal" and data["goal"] == [5.0, 0.0]
    assert 24 <= len(data["samples"]) <= 26  # kept at 10 Hz
    assert data["meta"]["time_base"] == "sim" and data["meta"]["pose_source"] == "ground_truth"
    assert data["meta"]["min_scan_range_m"] == pytest.approx(0.51)
    traj = Trajectory.from_dict(data)  # host-side scorer can load it unchanged
    assert traj.t[0] == pytest.approx(100.0)
    assert traj.ate is not None and traj.ate_mean == pytest.approx(0.02)


def test_recorder_counts_only_collisions_during_the_run():
    mon = ContactMonitor(robot_name="waffle")
    rec = TelemetryRecorder(mon)
    mon.process([("box::link::c", 20.0)], t=1.0)  # before recording
    rec.start("adversarial")
    mon.tick(50.0)
    mon.process([("crate::link::c", 20.0)], t=50.0)
    _drive(rec, t0=50.0, n=5)
    data = rec.stop()
    assert data["collisions"] == 1
    assert data["meta"]["contact_events"][0]["entity"] == "crate::link::c"


def test_recorder_drops_estimate_columns_unless_on_every_sample_and_ignores_time_reset():
    rec = TelemetryRecorder()
    rec.start("r", rate_hz=100)
    rec.offer(1.0, (0, 0, 0), "ground_truth", 0, 0, estimate=(0, 0, 0))
    rec.offer(1.1, (0.1, 0, 0), "map", 0, 0, estimate=(0.1, 0, 0))
    assert rec.offer(0.5, (0, 0, 0), "map", 0, 0) is False  # sim time went backwards
    data = rec.stop()
    assert all("slam_x" not in s for s in data["samples"])
    assert rec.stop() is None


def test_build_live_matches_the_pydantic_contract():
    rec = TelemetryRecorder(ContactMonitor(robot_name="waffle"))
    rec.start("nominal", goal=(3.0, 4.0, 0.0))
    live = build_live(
        sim_time=12.3456,
        rtf=0.97,
        pose=(0.0, 0.0, 0.1),
        source="ground_truth",
        velocity=(0.26, 0.0, 0.1),
        ground_truth=(0.0, 0.0, 0.1),
        map_pose=(0.03, 0.04, 0.1),
        scan_min=0.5,
        recorder=rec,
    )
    model = TelemetryLive(**live)
    assert model.distance_to_goal_m == pytest.approx(5.0)
    assert model.localization_error_m == pytest.approx(0.05)
    assert model.obstacle_clearance_m == pytest.approx(0.37)
    assert model.recording and model.recording_label == "nominal"
