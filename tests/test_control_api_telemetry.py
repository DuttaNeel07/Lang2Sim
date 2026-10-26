"""
Telemetry engine endpoints: live metrics, record start/stop, spawn footprints, WS 'live' key.
Real RosBridge telemetry methods, driven without rclpy (stand-in messages).
File: tests/test_control_api_telemetry.py
"""

from __future__ import annotations

import asyncio
import collections
import json
import sys
import threading
import time
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

fastapi = pytest.importorskip("fastapi")
httpx = pytest.importorskip("httpx")

from fastapi.testclient import TestClient  # noqa: E402

from lang2sim.core.models import Pose2D, SimStartRequest  # noqa: E402
from lang2sim.perturbation.routers import world as world_router  # noqa: E402
from lang2sim.runtime.control_api.client import ControlAPIError, ControlClient  # noqa: E402
from lang2sim.runtime.control_api.server import SimProcess, create_app  # noqa: E402
from lang2sim.runtime.ros_bridge import NAV2_NODES, RosBridge, RosSnapshot  # noqa: E402
from lang2sim.runtime.telemetry_recorder import ContactMonitor, TelemetryRecorder  # noqa: E402

WORLD = '<?xml version="1.0"?><sdf version="1.9"><world name="w"></world></sdf>'


class FakeSim(SimProcess):
    def build_command(self, req):
        return [sys.executable, "-c", "import time; time.sleep(60)"]


def make_bridge():
    """A RosBridge without rclpy: real telemetry logic, canned readiness snapshot."""
    b = object.__new__(RosBridge)
    b._lock = threading.Lock()
    b._s = RosSnapshot()
    b._clock_samples = collections.deque(maxlen=10)
    b.robot_name = "waffle"
    b.recorder = TelemetryRecorder(ContactMonitor(robot_name="waffle"))
    b.expect_ground_truth = False
    b._record_wall0 = 0.0
    b._map_pose_now = lambda: None
    real_snapshot = RosBridge.snapshot

    def snapshot():
        s = real_snapshot(b)
        now = time.time()
        s.clock_wall = s.odom_wall = s.scan_wall = now
        s.sim_time, s.rtf = 30.0, 1.0
        s.map_pose = (-2.0, -0.5, 0.0)
        s.lifecycle = {n: "active" for n in NAV2_NODES}
        return s

    b.snapshot = snapshot
    return b


def odom(t, x, y=0.0):
    return NS(header=NS(stamp=NS(sec=int(t), nanosec=int(round((t % 1) * 1e9)))),
              pose=NS(pose=NS(position=NS(x=x, y=y, z=0.0), orientation=NS(x=0.0, y=0.0, z=0.0, w=1.0))),
              twist=NS(twist=NS(linear=NS(x=0.2, y=0.0, z=0.0), angular=NS(x=0.0, y=0.0, z=0.0))))


@pytest.fixture
def setup(tmp_path):
    run = tmp_path / "run"
    run.mkdir()
    (run / "world.sdf").write_text(WORLD)
    (run / "map.yaml").write_text("image: map.pgm\n")
    req = SimStartRequest(run_dir=str(run), world_sdf_path=str(run / "world.sdf"),
                          map_yaml_path=str(run / "map.yaml"), initial_pose=Pose2D(x=-2, y=-0.5))
    sim = FakeSim()
    app = create_app(sim, use_ros=False)
    bridge = make_bridge()
    app.state.ros["bridge"] = bridge
    world_router.clear_spawn_log()
    yield app, bridge, req, run
    sim.stop(force=True)


def test_live_and_record_roundtrip_writes_trajectory(setup):
    app, bridge, req, run = setup
    with TestClient(app) as c:
        assert c.post("/v1/telemetry/record/start", json={"label": "nominal"}).status_code == 409  # no sim
        assert c.post("/v1/sim/start", json=req.model_dump(mode="json")).status_code == 202
        assert bridge._s.spawn == (-2.0, -0.5, 0.0)          # odom origin handed to the bridge
        r = c.post("/v1/telemetry/record/start",
                   json={"label": "nominal", "goal": {"x": 3.0, "y": -0.5, "yaw": 0.0}, "rate_hz": 10})
        assert r.status_code == 202 and r.json()["recording"] and r.json()["recording_label"] == "nominal"
        for i in range(30):                                 # 3 s of /odom at 10 Hz
            bridge._on_odom(odom(100.0 + 0.1 * i, 0.1 * i))
        live = c.get("/v1/telemetry/live").json()
        assert live["pose_source"] == "map" and live["samples_recorded"] >= 29
        assert live["distance_to_goal_m"] == pytest.approx(5.0)
        res = c.post("/v1/telemetry/record/stop").json()
        assert res["label"] == "nominal" and res["samples"] >= 29
        assert res["duration_s"] == pytest.approx(2.9, abs=0.11)
        assert res["trajectory"]["meta"]["time_base"] == "sim"
        saved = Path(res["path"])
        assert saved == run / "logs" / "trajectory_nominal.json"
        assert json.loads(saved.read_text())["label"] == "nominal"
        assert c.post("/v1/telemetry/record/stop").status_code == 409


def test_record_stop_reports_every_obstacle_in_the_world(setup):
    app, bridge, req, run = setup
    with TestClient(app) as c:
        c.post("/v1/sim/start", json=req.model_dump(mode="json"))
        world_router.record_spawn("old_box", "cargo_box", 9.0, 9.0)   # before the recording
        time.sleep(0.01)
        c.post("/v1/telemetry/record/start", json={"label": "adversarial"})
        world_router.record_spawn("wooden_crate_1", "wooden_crate", 3.5, 2.0)
        for i in range(3):
            bridge._on_odom(odom(5.0 + 0.1 * i, 0.0))
        res = c.post("/v1/telemetry/record/stop").json()
        # The box from before the recording is still in the world: an obstacle for this run too.
        assert [(e["name"], e["during_run"]) for e in res["spawned_entities"]] == [
            ("old_box", False), ("wooden_crate_1", True)]
        assert res["spawned_entities"][1]["r"] == pytest.approx(0.4243)
        assert res["trajectory"]["meta"]["spawned_entities"][1]["x"] == 3.5


def test_websocket_carries_live_telemetry(setup):
    app, bridge, req, run = setup
    with TestClient(app) as c:
        c.post("/v1/sim/start", json=req.model_dump(mode="json"))
        with c.websocket_connect("/v1/nav/feedback?hz=20") as ws:
            msg = ws.receive_json()
        assert {"nav", "telemetry", "live", "sim"} <= set(msg)
        assert msg["live"]["pose_source"] == "map"


def test_without_ros_bridge_live_is_empty_and_recording_is_503(tmp_path):
    app = create_app(FakeSim(), use_ros=False)
    with TestClient(app) as c:
        assert c.get("/v1/telemetry/live").json()["pose_source"] == "none"
        assert c.post("/v1/telemetry/record/stop").status_code == 503
        assert c.get("/v1/telemetry/setup").json()["enabled"] is False


def test_client_methods(setup):
    app, bridge, req, run = setup

    async def go():
        async with ControlClient("http://test", transport=httpx.ASGITransport(app=app)) as api:
            await api.sim_start(req)
            live = await api.record_start("nominal", goal=Pose2D(x=1.0, y=0.0))
            assert live.recording
            for i in range(4):
                bridge._on_odom(odom(1.0 + 0.1 * i, 0.0))
            res = await api.record_stop()
            assert res.samples == 4 and res.pose_source == "odom"   # recording uses TF, absent here
            assert (await api.telemetry_live()).recording is False
            with pytest.raises(ControlAPIError):
                await api.record_stop()

    asyncio.run(go())


def test_recording_streams_to_partial_file_until_stop(setup):
    app, bridge, req, run = setup
    with TestClient(app) as c:
        c.post("/v1/sim/start", json=req.model_dump(mode="json"))
        c.post("/v1/telemetry/record/start", json={"label": "nominal", "rate_hz": 10})
        for i in range(5):
            bridge._on_odom(odom(20.0 + 0.1 * i, 0.1 * i))
        partial = run / "logs" / "trajectory_nominal.partial.jsonl"
        assert partial.is_file() and partial.read_text().count('"type": "sample"') == 5
        world_router.record_spawn("wooden_crate_1", "wooden_crate", 1.0, 0.0)
        assert "wooden_crate_1" in (run / "logs" / "spawns.jsonl").read_text()
        res = c.post("/v1/telemetry/record/stop").json()
        assert res["samples"] == 5 and not partial.exists()
        assert (run / "logs" / "trajectory_nominal.json").is_file()
