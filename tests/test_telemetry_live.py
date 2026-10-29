"""Host consumers of live telemetry: fetch helper, TUI sidebar fields, agent tool, collector backend."""

from __future__ import annotations

import json
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer

import pytest

from lang2sim.telemetry import live as tl

LIVE = {
    "sim_time": 75.25,
    "real_time_factor": 0.93,
    "pose": {"x": 1.5, "y": -2.0, "yaw": 0.3},
    "pose_source": "ground_truth",
    "linear_speed": 0.22,
    "angular_speed": -0.1,
    "localization_error_m": 0.04,
    "min_scan_range_m": 0.61,
    "obstacle_clearance_m": 0.48,
    "distance_to_goal_m": 3.2,
    "collision_count": 2,
    "last_contact_force_n": 11.5,
    "last_contact_entity": "crate_1::link::collision",
    "contact_force_source": "wrench",
    "recording": True,
    "recording_label": "adversarial",
    "samples_recorded": 400,
}
LEGACY = {
    "is_running": True,
    "sim_time": 9.0,
    "real_time_factor": 1.0,
    "robot_pose": {"x": 0.5, "y": 0.25, "yaw": 0.0},
    "robot_velocity": {"x": 0.1, "y": 0.0, "yaw": 0.2},
}


def serve(routes):
    """Tiny control API stand-in on a free port. routes: path -> dict (mutable, read per request)."""

    class H(BaseHTTPRequestHandler):
        def do_GET(self):  # noqa: N802
            body = routes.get(self.path)
            if body is None:
                self.send_response(404)
                self.end_headers()
                return
            data = json.dumps(body).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(data)

        def log_message(self, *a):
            pass

    srv = HTTPServer(("127.0.0.1", 0), H)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    return srv, f"http://127.0.0.1:{srv.server_port}"


@pytest.fixture
def api():
    routes = {"/v1/telemetry/live": dict(LIVE)}
    srv, url = serve(routes)
    yield routes, url
    srv.shutdown()


def test_fetch_live_and_legacy_fallback(api):
    routes, url = api
    assert tl.fetch_live(url)["pose_source"] == "ground_truth"
    del routes["/v1/telemetry/live"]
    routes["/v1/telemetry"] = LEGACY
    legacy = tl.fetch_live(url)
    assert legacy["legacy"] and legacy["pose"]["x"] == 0.5 and legacy["angular_speed"] == 0.2
    assert tl.fetch_live("http://127.0.0.1:1", timeout=0.2) is None


def test_sidebar_fields():
    f = tl.sidebar_fields(LIVE)
    assert (
        f["sim_state"] == "Active (recording)"
        and f["sim_time"] == "01:15.2"
        and f["rtf"] == "0.93x"
    )
    assert (f["robot_x"], f["robot_y"], f["clearance"], f["collision_count"]) == (
        1.5,
        -2.0,
        0.48,
        2,
    )
    assert tl.sidebar_fields(None) == {"sim_state": "No simulation"}
    assert tl.sidebar_fields({**LIVE, "pose": None})["sim_state"] == "Starting"


def test_agent_tool_reads_live_telemetry(api, monkeypatch):
    from lang2sim.agent.tools import query_robot_telemetry

    _routes, url = api
    monkeypatch.setenv("LANG2SIM_CONTROL_API", url)
    out = query_robot_telemetry()
    assert out["status"] == "ok" and out["distance_to_goal"] == 3.2
    assert (
        out["bumper_force_newtons"] == 11.5 and out["collision_count"] == 2 and out["speed"] == 0.22
    )
    monkeypatch.setenv("LANG2SIM_CONTROL_API", "http://127.0.0.1:1")
    assert query_robot_telemetry()["status"] == "unavailable"


def test_collector_control_api_backend_uses_sim_time_and_container_collisions(api):
    from lang2sim.telemetry.ros_subscribers import TelemetryCollector

    routes, url = api
    c = TelemetryCollector(api_base_url=url, poll_interval=0.02)
    try:
        c.start_run("nominal")
        for i in range(5):
            routes["/v1/telemetry/live"] = {
                **LIVE,
                "sim_time": 75.25 + i,
                "pose": {"x": 1.5 + 0.1 * i, "y": -2.0, "yaw": 0.3},
            }
            time.sleep(0.08)
        snap = c.snapshot()
        assert snap["backend"] == "control_api" and snap["collisions"] == 2
        assert snap["pose_source"] == "ground_truth" and snap["obstacle_clearance_m"] == 0.48
        traj = c.stop_run()
        assert traj.t[0] >= 75.0  # sim time, not wall time
        assert len(set(traj.t.tolist())) == traj.n  # paused sim does not duplicate stamps
    finally:
        c.close()
