"""Benchmark dashboard: map/trajectory/report data and the routes on the control API."""

from __future__ import annotations

import base64
import json
import shutil
import sys
from pathlib import Path

import pytest

from lang2sim.runtime import dashboard as db
from lang2sim.runtime.telemetry_recorder import TelemetryRecorder

FIXTURES = Path(__file__).resolve().parents[1] / "catalog" / "fixtures"


@pytest.fixture
def run_dir(tmp_path):
    d = tmp_path / "projects" / "empty_room" / "runs" / "001"
    shutil.copytree(FIXTURES / "empty_room", d)
    (d / "logs").mkdir()
    return d


@pytest.mark.parametrize("fixture", ["empty_room", "small_warehouse", "tb3_world"])
def test_read_map_matches_the_scorer(fixture):
    pytest.importorskip("PIL")
    from lang2sim.telemetry.nqi_scorer import OccupancyGrid

    m = db.read_map(FIXTURES / fixture)
    cells = base64.b64decode(m["cells_b64"])
    assert len(cells) == m["width"] * m["height"]
    grid = OccupancyGrid.from_yaml(FIXTURES / fixture / "map.yaml")  # tb3_world's PGM has a comment
    assert grid.occupied.shape == (m["height"], m["width"])
    assert sum(1 for c in cells if c == 1) == int(grid.occupied.sum())
    assert m["resolution"] == grid.resolution and tuple(m["origin"]) == grid.origin


def test_read_pgm_ascii_and_errors(tmp_path):
    (tmp_path / "a.pgm").write_bytes(b"P2\n# c\n2 1\n255\n0 255\n")
    assert db.read_pgm(tmp_path / "a.pgm") == (2, 1, 255, bytes([0, 255]))
    (tmp_path / "b.pgm").write_bytes(b"P6\n1 1\n255\n\x00\x00\x00")
    with pytest.raises(db.MapError):
        db.read_pgm(tmp_path / "b.pgm")


def test_downsample_keeps_ends():
    samples = [{"t": i * 0.05, "x": float(i), "y": 0.0} for i in range(5001)]
    out = db.downsample(samples, 1000)
    assert len(out) <= 1001 and out[0][0] == 0.0 and out[-1][1] == 5000.0


def test_trajectories_and_mission(run_dir):
    rec = TelemetryRecorder()
    rec.start("nominal", rate_hz=10)
    for i in range(30):
        rec.offer(i * 0.1, (-2.5 + 0.1 * i, -1.5, 0.0), "ground_truth", 0.2, 0.0)
    (run_dir / "logs" / "trajectory_nominal.json").write_text(json.dumps(rec.stop()))
    payload = db.trajectories_payload(run_dir)
    assert payload["runs"]["nominal"]["samples"][0] == [0.0, -2.5, -1.5, 0.0, 0.2]
    mission = db.read_mission(run_dir)
    assert mission["initial_pose"]["x"] == -2.5 and len(mission["waypoints"]) == 2
    assert db.project_path(run_dir, str(run_dir.parents[2])) == "empty_room/runs/001"


fastapi = pytest.importorskip("fastapi")
from fastapi.testclient import TestClient  # noqa: E402

from lang2sim.core.models import Pose2D, SimStartRequest  # noqa: E402
from lang2sim.runtime.control_api.server import SimProcess, create_app  # noqa: E402


class FakeSim(SimProcess):
    def build_command(self, req):
        return [sys.executable, "-c", "import time; time.sleep(60)"]


def test_dashboard_routes_on_the_control_api(run_dir, monkeypatch):
    monkeypatch.setenv("L2S_PROJECTS_DIR", str(run_dir.parents[2]))
    sim = FakeSim()
    app = create_app(sim, use_ros=False)
    req = SimStartRequest(
        run_dir=str(run_dir),
        world_sdf_path=str(run_dir / "world.sdf"),
        map_yaml_path=str(run_dir / "map.yaml"),
        initial_pose=Pose2D(x=-2.5, y=-1.5),
    )
    try:
        with TestClient(app) as c:
            page = c.get("/dashboard")
            assert page.status_code == 200 and "Navigation Benchmark" in page.text
            assert c.get("/v1/run").json() == {"loaded": False, "mode": "live"}
            assert c.get("/v1/run/map").status_code == 404
            assert c.post("/v1/sim/start", json=req.model_dump(mode="json")).status_code == 202
            info = c.get("/v1/run").json()
            assert info["loaded"] and info["mode"] == "live" and info["world_name"] == "empty_room"
            assert info["project_path"] == "empty_room/runs/001" and not info["has_report"]
            assert c.get("/v1/run/map").json()["width"] == 160
            assert c.get("/v1/run/report").status_code == 404
            (run_dir / "benchmark_report.json").write_text(
                json.dumps(
                    {"runs": {"nominal": {"nqi": 80.0}}, "generated_at": "2026-10-27T00:00:00Z"}
                )
            )
            assert c.get("/v1/run").json()["has_report"]
            assert c.get("/v1/run/report").json()["runs"]["nominal"]["nqi"] == 80.0
            assert c.get("/v1/run/trajectories").json() == {"runs": {}}
    finally:
        sim.stop(force=True)
