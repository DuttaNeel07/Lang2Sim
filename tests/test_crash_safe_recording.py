"""Crash-safe recording: streamed partial files, recovery and rescoring of a run folder."""

from __future__ import annotations

import json
import math
import shutil
from pathlib import Path

import pytest

from lang2sim.runtime.telemetry_recorder import (
    ContactMonitor,
    TelemetryRecorder,
    load_partial,
    partial_path,
)
from lang2sim.telemetry.benchmark import load_recorded_runs, main, score_run_folder

FIXTURES = Path(__file__).resolve().parents[1] / "catalog" / "fixtures"


def drive(rec, n=60, t0=10.0, y=-1.5, crash=False):
    for i in range(n):
        rec.offer(
            t0 + 0.1 * i,
            (-2.5 + 0.08 * i, y, 0.0),
            "ground_truth",
            0.8,
            0.0,
            estimate=(-2.5 + 0.08 * i + 0.03, y, 0.0),
            scan_min=1.0,
        )
    if crash:
        rec._run.stream.write('{"type": "sample", "t": 99')  # torn write at the crash point


@pytest.fixture
def run_dir(tmp_path):
    d = tmp_path / "run"
    shutil.copytree(FIXTURES / "empty_room", d)
    (d / "logs").mkdir()
    return d


def test_stream_mirrors_recording_and_is_removed_on_clean_stop(run_dir):
    rec = TelemetryRecorder(ContactMonitor(robot_name="waffle"))
    p = partial_path(run_dir / "logs", "nominal")
    rec.start("nominal", goal=(2.5, -1.5, 0.0), rate_hz=10, stream_path=p)
    drive(rec)
    rec.contacts.process([("crate::link::c", 20.0)], t=12.0)
    recovered = load_partial(p)  # readable while the run is still going
    assert len(recovered["samples"]) == 60 and recovered["collisions"] == 1
    assert recovered["meta"]["partial"] and recovered["meta"]["pose_source"] == "ground_truth"
    final = rec.stop()
    assert not p.exists() and final["collisions"] == 1 and not final["meta"]["partial"]
    assert [s["x"] for s in final["samples"]] == [s["x"] for s in recovered["samples"]]


def test_crashed_run_is_recovered_and_scored(run_dir, capsys):
    rec = TelemetryRecorder(ContactMonitor(robot_name="waffle"))
    p = partial_path(run_dir / "logs", "adversarial")
    rec.start("adversarial", goal=(2.5, -1.5, 0.0), rate_hz=10, stream_path=p)
    drive(rec, crash=True)  # process dies: stop() never runs
    spawn = {
        "name": "wooden_crate_1",
        "model": "wooden_crate",
        "x": 0.5,
        "y": -0.9,
        "yaw": 0.0,
        "shape": "box",
        "size": [0.6, 0.6],
        "r": 0.4243,
        "wall_time": load_partial(p)["meta"]["recording_wall_time"] + 1,
    }
    (run_dir / "logs" / "spawns.jsonl").write_text(json.dumps(spawn) + "\n")

    runs = load_recorded_runs(run_dir)
    assert list(runs) == ["adversarial"] and runs["adversarial"]["meta"]["partial"]
    assert runs["adversarial"]["meta"]["spawned_entities"][0]["name"] == "wooden_crate_1"
    path, report = score_run_folder(run_dir)
    run = report["runs"]["nominal"]  # the only run scored, reported in the first slot
    assert run["label"] == "adversarial" and run["nav_status"] == "interrupted" and run["partial"]
    assert run["dynamic_obstacles"][0]["name"] == "wooden_crate_1" and run["samples"] == 60
    assert report["partial_runs"] == ["adversarial"] and path.name == "benchmark_report.json"

    assert main([str(run_dir)]) == 0
    assert "recovered from a partial recording" in capsys.readouterr().out


def test_final_trajectory_wins_over_partial(run_dir):
    rec = TelemetryRecorder()
    rec.start("nominal", rate_hz=10, stream_path=partial_path(run_dir / "logs", "nominal"))
    drive(rec)
    data = rec.stop(keep_partial=True)  # what the API does before writing the final file
    (run_dir / "logs" / "trajectory_nominal.json").write_text(json.dumps(data))
    runs = load_recorded_runs(run_dir)
    assert runs["nominal"]["meta"]["partial"] is False


def test_cli_without_recordings(run_dir, capsys):
    assert main([str(run_dir)]) == 2
    assert "no recorded trajectories" in capsys.readouterr().out


def test_partial_run_obstacles_are_scoped_to_its_sim_session(run_dir):
    rec = TelemetryRecorder()
    p = partial_path(run_dir / "logs", "adversarial")
    rec.start("adversarial", rate_hz=10, stream_path=p)
    drive(rec, crash=True)
    t0 = load_partial(p)["meta"]["recording_wall_time"]
    box = {
        "model": "cargo_box",
        "y": -0.9,
        "yaw": 0.0,
        "shape": "box",
        "size": [1.0, 0.6],
        "r": 0.58,
    }
    lines = [
        {"name": "old_session_box", "x": 0.0, "wall_time": t0 - 100, **box},  # earlier sim, gone
        {"type": "session_start", "wall_time": t0 - 50},
        {"name": "left_from_last_run", "x": 1.0, "wall_time": t0 - 10, **box},  # still in the world
        {"name": "dropped_now", "x": 2.0, "wall_time": t0 + 5, **box},
    ]
    (run_dir / "logs" / "spawns.jsonl").write_text("".join(json.dumps(r) + "\n" for r in lines))
    found = load_recorded_runs(run_dir)["adversarial"]["meta"]["spawned_entities"]
    assert [(e["name"], e["during_run"]) for e in found] == [
        ("left_from_last_run", False),
        ("dropped_now", True),
    ]
