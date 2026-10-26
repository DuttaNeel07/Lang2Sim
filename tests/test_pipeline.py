"""Prompt -> spec -> world -> run folder -> recorded, scored mission (PR #17 pipeline on the telemetry engine)."""

from __future__ import annotations

import asyncio
import json
import math

import pytest

from lang2sim.core.models import NavStatus, SimStatusResponse, TelemetryLive, TelemetryRecordResult
from lang2sim.pipeline import build_run, describe, launch_and_drive, prompt_to_spec
from lang2sim.runtime.telemetry_recorder import TelemetryRecorder


def run_folder(monkeypatch, tmp_path):
    import lang2sim.pipeline as pipeline

    def get_run_dir(project_name="active_project", timestamp=None):
        d = tmp_path / project_name / "runs" / "001"
        d.mkdir(parents=True, exist_ok=True)
        return d

    monkeypatch.setattr(pipeline.cfg, "get_run_dir", get_run_dir)


def test_prompt_to_run_folder(tmp_path, monkeypatch):
    run_folder(monkeypatch, tmp_path)
    spec, source = asyncio.run(prompt_to_spec("a dark warehouse with 3 racks and 2 crates", use_llm=False))
    assert "parser" in source and len([o for o in spec.objects if o.id.startswith("rack")]) == 3
    assert "dim lighting" in describe(spec)
    run, steps = build_run(spec)
    for f in ("world.sdf", "map.yaml", "map.pgm", "spec.json", "manifest.json"):
        assert (run / f).is_file()
    assert str(run).startswith(str(tmp_path))


class FakeAPI:
    def __init__(self, spec_json):
        self.spec = json.loads(spec_json)
        self.polls = 0

    async def record_start(self, label, goal=None, rate_hz=20.0):
        return TelemetryLive(sim_time=10.0, recording=True, recording_label=label)

    async def nav_waypoints(self, poses, timeout_seconds=180.0):
        self.poses = poses

    async def nav_status(self):
        self.polls += 1
        if self.polls < 4:
            return NavStatus(is_active=True, status="navigating", distance_remaining=4.0 - self.polls)
        return NavStatus(is_active=False, status="succeeded")

    async def telemetry_live(self):
        return TelemetryLive(sim_time=12.0)

    async def record_stop(self):
        m = self.spec["mission"]
        pts = [(m["initial_pose"]["x"], m["initial_pose"]["y"])] + [(w["x"], w["y"]) for w in m["waypoints"]] + \
              [(m["goal_pose"]["x"], m["goal_pose"]["y"])]
        rec = TelemetryRecorder()
        rec.start("nominal", rate_hz=10)
        t = 10.0
        for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
            n = max(2, int(math.hypot(x1 - x0, y1 - y0) / 0.025))
            for i in range(n):
                rec.offer(t, (x0 + (x1 - x0) * i / n, y0 + (y1 - y0) * i / n, math.atan2(y1 - y0, x1 - x0)),
                          "ground_truth", 0.25, 0.0)
                t += 0.1
        rec.offer(t, (pts[-1][0], pts[-1][1], 0.0), "ground_truth", 0.0, 0.0)
        data = rec.stop()
        return TelemetryRecordResult(label="nominal", samples=len(data["samples"]), duration_s=t - 10.0, collisions=0,
                                     pose_source="ground_truth", path=None, trajectory=data, spawned_entities=[])

    async def telemetry(self):
        raise RuntimeError("not used")


class FakeSession:
    def __init__(self, run):
        from lang2sim.runtime.session import SimSession

        self.api = FakeAPI((run / "spec.json").read_text())
        self.run_dir = run
        self.run_recorded_mission = SimSession.run_recorded_mission.__get__(self)

    async def open_run(self, run, gui=True, rviz=True, on_progress=None, poll_s=2.0):
        st = SimStatusResponse(status="running", ready=True, sim_time=5.0, rtf=0.9,
                               readiness={"gazebo_clock": True})
        if on_progress:
            on_progress(st, 1.0)
        return st


def test_launch_and_drive_records_and_scores(tmp_path, monkeypatch):
    run_folder(monkeypatch, tmp_path)
    spec, _ = asyncio.run(prompt_to_spec("an empty room", use_llm=False))
    run, _ = build_run(spec)
    events = []
    nav, scored = asyncio.run(launch_and_drive(FakeSession(run), run, on_event=events.append, poll_s=0))
    assert nav.status == "succeeded" and scored["S_mission"] == 1 and scored["nqi"] > 80
    assert scored["pose_source"] == "ground_truth" and scored["time_base"] == "sim"
    assert scored["report_path"].endswith("benchmark_report.json") and (run / "benchmark_report.json").is_file()
    assert any(e.startswith("READY") for e in events) and any("recording telemetry" in e for e in events)
