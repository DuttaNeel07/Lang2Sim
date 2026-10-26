"""
Scoring recorded runs and the nominal-vs-adversarial benchmark flow (no Docker, fake control API).
File: tests/test_benchmark_runs.py
"""

from __future__ import annotations

import asyncio
import json
import math
import shutil
from pathlib import Path

import pytest

from lang2sim.core.models import NavStatus, TelemetryLive, TelemetryRecordResult
from lang2sim.runtime.session import SimSession, due_triggers
from lang2sim.runtime.telemetry_recorder import ContactMonitor, TelemetryRecorder
from lang2sim.telemetry.benchmark import (
    REPORT_NAME,
    default_stress_trigger,
    mission_optimal_length,
    mission_targets,
    score_recorded_run,
)
from lang2sim.telemetry.nqi_scorer import NQIParams, OccupancyGrid

FIXTURES = Path(__file__).resolve().parents[1] / "catalog" / "fixtures"


@pytest.fixture
def run_dir(tmp_path):
    dst = tmp_path / "run"
    shutil.copytree(FIXTURES / "empty_room", dst)
    return dst


def spec_of(run_dir):
    from lang2sim.core.models import SimSpec
    return SimSpec.model_validate_json((run_dir / "spec.json").read_text())


def recorded(spec, label="nominal", detour=0.0, rate=10.0, speed=0.25, collisions=0):
    """Drive the mission legs at constant speed through the recorder, like the container would."""
    m = spec.mission
    pts = [(m.initial_pose.x, m.initial_pose.y)] + mission_targets(spec)
    rec = TelemetryRecorder(ContactMonitor(robot_name="waffle"))
    rec.start(label, goal=(m.goal_pose.x, m.goal_pose.y, m.goal_pose.yaw), rate_hz=rate)
    t = 50.0
    for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
        n = max(2, int(math.hypot(x1 - x0, y1 - y0) / (speed / rate)))
        yaw = math.atan2(y1 - y0, x1 - x0)
        for i in range(n):
            f = i / n
            bump = detour * math.sin(math.pi * f)       # sideways bulge on each leg
            x = x0 + f * (x1 - x0) - bump * math.sin(yaw)
            y = y0 + f * (y1 - y0) + bump * math.cos(yaw)
            rec.offer(t, (x, y, yaw), "ground_truth", speed, 0.0)
            t += 1.0 / rate
    rec.offer(t, (pts[-1][0], pts[-1][1], 0.0), "ground_truth", 0.0, 0.0)
    for k in range(collisions):
        rec.contacts.tick(100.0 + 10 * k)
        rec.contacts.process([("crate::link::c", 20.0)], t=100.0 + 10 * k)
    return rec.stop()


def test_mission_optimal_length_follows_waypoints(run_dir):
    spec = spec_of(run_dir)
    grid = OccupancyGrid.from_yaml(run_dir / "map.yaml")
    start = (spec.mission.initial_pose.x, spec.mission.initial_pose.y)
    length, method = mission_optimal_length(start, mission_targets(spec), grid)
    assert method == "astar"
    assert length == pytest.approx(5.0 + 3.0 + 5.0, rel=0.05)        # the three legs, not start->goal (3 m)


def test_score_recorded_run_uses_mission_map_and_spawned_obstacles(run_dir):
    spec = spec_of(run_dir)
    data = recorded(spec)
    nominal = score_recorded_run(data, run_dir=run_dir, spec=spec, nav_status="succeeded")
    assert nominal["E_p"] > 0.95 and nominal["S_mission"] == 1
    assert nominal["optimal_length_method"] == "mission_astar"
    assert nominal["clearance_source"] == "map" and nominal["pose_source"] == "ground_truth"
    assert nominal["time_base"] == "sim" and nominal["final_goal_error_m"] < 0.05
    # A crate 0.6 m from the path shrinks the clearance term.
    crate = [{"x": 0.0, "y": -0.9, "r": 0.42}]
    stressed = score_recorded_run(recorded(spec, detour=0.3), run_dir=run_dir, spec=spec, spawned=crate)
    assert stressed["clearance_source"] == "obstacles+map"
    assert stressed["C_min_m"] < nominal["C_min_m"]
    assert stressed["E_p"] < nominal["E_p"]
    assert stressed["nqi"] < nominal["nqi"]


def test_collisions_fail_the_mission_and_cost_nqi(run_dir):
    spec = spec_of(run_dir)
    hit = score_recorded_run(recorded(spec, collisions=1), run_dir=run_dir, spec=spec)
    assert hit["C_force"] == 1 and hit["S_mission"] == 0 and hit["terms"]["collision_penalty"] == -50.0


def test_default_stress_trigger_drops_crate_ahead_on_the_path(run_dir):
    data = recorded(spec_of(run_dir))
    tr = default_stress_trigger(data)
    assert tr["trigger_type"] == "time" and tr["model_name"] == "wooden_crate"
    duration = data["samples"][-1]["t"] - data["samples"][0]["t"]
    assert tr["trigger_value"] == pytest.approx(0.35 * duration, abs=0.01)
    # 60% of a 13 m U-shaped path lies on the second (x = 2.5) leg.
    assert tr["spawn_pose"]["x"] == pytest.approx(2.5, abs=0.05)


def test_due_triggers():
    trs = [{"trigger_type": "immediate", "trigger_value": 0},
           {"trigger_type": "time", "trigger_value": 5.0},
           {"trigger_type": "waypoint_distance", "trigger_value": 1.5}]
    assert due_triggers(trs, set(), 0.0, None) == [0]
    assert due_triggers(trs, {0}, 5.0, 3.0) == [1]
    assert due_triggers(trs, {0, 1}, 6.0, 1.2) == [2]
    assert due_triggers(trs, {0, 1, 2}, 9.0, 0.1) == []


class FakeApi:
    """Control API stand-in: each mission 'drives' for 8 polls, record_stop returns a recorded run."""

    def __init__(self, spec):
        self.spec = spec
        self.calls, self.spawned, self.lights = [], [], []
        self.sim_time, self.polls, self.label = 0.0, 0, None

    async def record_start(self, label, goal=None, rate_hz=20.0):
        self.calls.append(("record_start", label))
        self.label, self.polls = label, 0
        return TelemetryLive(sim_time=self.sim_time, recording=True, recording_label=label)

    async def nav_waypoints(self, poses, timeout_seconds=180.0):
        self.calls.append(("nav_waypoints", len(poses)))

    async def nav_status(self):
        self.polls += 1
        self.sim_time += 5.0          # 8 polls ~ a 40 s mission
        if self.polls < 8:
            return NavStatus(is_active=True, status="navigating", distance_remaining=10.0 - self.polls)
        return NavStatus(is_active=False, status="succeeded")

    async def telemetry_live(self):
        return TelemetryLive(sim_time=self.sim_time)

    async def world_spawn(self, req):
        self.spawned.append(req)
        return {"ok": True}

    async def world_light(self, req):
        self.lights.append(req["intensity"])

    async def record_stop(self):
        self.calls.append(("record_stop", self.label))
        adv = self.label == "adversarial"
        data = recorded(self.spec, label=self.label, detour=0.35 if adv else 0.0, speed=0.2 if adv else 0.25)
        spawned = [{"name": "wooden_crate_1", "model": "wooden_crate", "x": r["x"], "y": r["y"], "r": 0.42}
                   for r in self.spawned] if adv else []
        return TelemetryRecordResult(label=self.label, samples=len(data["samples"]), duration_s=30.0,
                                     collisions=0, pose_source="ground_truth", path=f"/x/trajectory_{self.label}.json",
                                     trajectory=data, spawned_entities=spawned)

    async def nav_goal(self, pose, timeout_seconds=120.0):
        self.calls.append(("nav_goal", (pose.x, pose.y)))

    async def wait_nav_done(self, poll_s=1.0, timeout=1800.0):
        return NavStatus(status="succeeded")


def session_with(api):
    s = object.__new__(SimSession)
    s.api, s.run_dir = api, None
    return s


def test_run_benchmark_writes_nominal_vs_adversarial_report(run_dir):
    spec = spec_of(run_dir)
    api = FakeApi(spec)
    phases, fired = [], []
    report = asyncio.run(session_with(api).run_benchmark(
        run_dir, light=0.1, poll_s=0, on_phase=phases.append, on_trigger=lambda tr, err: fired.append((tr, err))))
    assert phases == ["nominal run", "returning to start", "adversarial run"]
    assert [c for c in api.calls if c[0] != "nav_waypoints"] == [
        ("record_start", "nominal"), ("record_stop", "nominal"),
        ("nav_goal", (spec.mission.initial_pose.x, spec.mission.initial_pose.y)),
        ("record_start", "adversarial"), ("record_stop", "adversarial")]
    assert len(api.spawned) == 1 and fired[0][1] is None            # default crate fired once
    assert api.lights == [0.1, 1.0]                                 # dimmed for the stress run, restored
    saved = json.loads((run_dir / REPORT_NAME).read_text())
    assert set(saved["runs"]) == {"nominal", "adversarial"}
    assert saved["stress"]["source"] == "default_crate"
    assert saved["runs"]["adversarial"]["nqi"] < saved["runs"]["nominal"]["nqi"]
    assert saved["deltas"]["path_length_pct"] > 0
    assert saved["runs"]["adversarial"]["dynamic_obstacles"][0]["model"] == "wooden_crate"


def test_run_benchmark_uses_spec_triggers_when_given(run_dir):
    spec_json = json.loads((run_dir / "spec.json").read_text())
    spec_json["adversarial_triggers"] = [{"trigger_type": "waypoint_distance", "trigger_value": 4.0,
                                          "model_name": "cargo_box", "spawn_pose": {"x": 1.0, "y": -1.5, "yaw": 0}}]
    (run_dir / "spec.json").write_text(json.dumps(spec_json))
    api = FakeApi(spec_of(run_dir))
    report = asyncio.run(session_with(api).run_benchmark(run_dir, poll_s=0))
    assert api.spawned == [{"model_name": "cargo_box", "x": 1.0, "y": -1.5, "yaw": 0.0}]
    assert report["stress"]["source"] == "spec.adversarial_triggers" and api.lights == []


def test_score_single_run_writes_nominal_only_report(run_dir):
    api = FakeApi(spec_of(run_dir))
    report = asyncio.run(session_with(api).score_single_run(run_dir, params=NQIParams(eps_pos=0.25), poll_s=0))
    saved = json.loads((run_dir / REPORT_NAME).read_text())
    assert set(saved["runs"]) == {"nominal"} and saved["params"]["eps_pos"] == 0.25
    assert report["nav"]["status"] == "succeeded"


def test_recording_stops_even_when_the_mission_errors(run_dir):
    api = FakeApi(spec_of(run_dir))

    async def boom(poses, timeout_seconds=180.0):
        raise RuntimeError("nav2 down")

    api.nav_waypoints = boom
    with pytest.raises(RuntimeError):
        asyncio.run(session_with(api).run_recorded_mission(run_dir))
    assert api.calls[-1] == ("record_stop", "nominal")


def test_default_trigger_keeps_the_crate_off_waypoints(run_dir):
    spec = spec_of(run_dir)
    targets = mission_targets(spec)
    tr = default_stress_trigger(recorded(spec), targets=targets)
    p = tr["spawn_pose"]
    start = (spec.mission.initial_pose.x, spec.mission.initial_pose.y)
    assert all(math.hypot(p["x"] - a[0], p["y"] - a[1]) >= 1.2 for a in [start, *targets])
    # Still on the nominal path, aligned with it (the second leg runs along +y).
    assert p["x"] == pytest.approx(2.5, abs=0.05) and p["yaw"] == pytest.approx(math.pi / 2, abs=0.05)


def test_box_footprint_clearance_is_measured_to_the_box_edge():
    from lang2sim.telemetry.benchmark import footprint_points
    from lang2sim.telemetry.nqi_scorer import min_clearance
    from lang2sim.telemetry.trajectory import Trajectory

    crate = {"x": 0.0, "y": 0.0, "yaw": 0.0, "shape": "box", "size": [0.6, 0.6], "r": 0.4243}
    # Robot passes 0.5 m from the crate centre diagonally: 0.5 - 0.3*sqrt(2) would be a hit for
    # the bounding circle, but the box corner is at 0.4243 and the closest edge point ~0.2 m away.
    traj = Trajectory.from_samples([{"t": i * 0.1, "x": -2.0 + 0.1 * i, "y": 0.5} for i in range(41)])
    exact = min_clearance(traj, footprint_points(crate), r_robot=0.0)
    circle = min_clearance(traj, [(0.0, 0.0, 0.4243)], r_robot=0.0)
    assert exact == pytest.approx(0.2, abs=0.026)
    assert circle == pytest.approx(0.0757, abs=0.01)
    assert footprint_points({"x": 1.0, "y": 2.0, "r": 0.3}) == [(1.0, 2.0, 0.3)]
