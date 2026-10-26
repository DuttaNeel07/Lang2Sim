"""Replay fallback (architecture §5.2 Mode B): a recorded run served through the control API routes."""

from __future__ import annotations

from pathlib import Path

import pytest

from lang2sim.core.models import NavStatus, TelemetryLive
from lang2sim.telemetry.replay import PAUSE_BETWEEN_RUNS_S, load_replay, snapshot

DEMO = Path(__file__).resolve().parents[1] / "catalog" / "demo_runs" / "empty_room_benchmark"


@pytest.fixture(scope="module")
def clock():
    return load_replay(DEMO, speed=10.0)


def at(clock, wall_s):
    return clock.started + wall_s


def test_demo_run_ships_both_recorded_runs(clock):
    assert [r.label for r in clock.runs] == ["nominal", "adversarial"]
    nominal, adversarial = clock.runs
    assert nominal.pose_source == adversarial.pose_source == "ground_truth"
    assert nominal.nav_status == adversarial.nav_status == "succeeded"
    assert adversarial.crates and adversarial.crates[0]["appear_t"] > adversarial.t[0]
    assert not nominal.crates


def test_clock_walks_nominal_pause_adversarial_then_holds(clock):
    nominal, adversarial = clock.runs
    i, k, playing = clock.position(at(clock, 0.5))
    assert (i, playing) == (0, True) and k > 0
    span = nominal.duration / clock.speed
    assert clock.position(at(clock, span + PAUSE_BETWEEN_RUNS_S / 2)) == (0, len(nominal.t) - 1, False)
    i, k, playing = clock.position(at(clock, span + PAUSE_BETWEEN_RUNS_S + 1))
    assert (i, playing) == (1, True)
    assert clock.position(at(clock, 10_000)) == (1, len(adversarial.t) - 1, False)


def test_snapshot_matches_the_live_contracts(clock):
    snap = snapshot(clock, at(clock, 2.0))
    live = TelemetryLive(**snap["live"])
    nav = NavStatus(**snap["nav"])
    assert live.recording and live.recording_label == "nominal" and live.pose is not None
    assert live.obstacle_clearance_m is not None and live.obstacle_clearance_m > 0
    assert nav.status == "navigating" and nav.waypoints_total == 3
    end = snapshot(clock, at(clock, 10_000))
    assert end["nav"]["status"] == "succeeded" and not end["live"]["recording"]
    assert end["live"]["distance_to_goal_m"] < 0.3


def test_crate_appears_when_it_was_dropped(clock):
    nominal, adversarial = clock.runs
    start_adv = nominal.duration / clock.speed + PAUSE_BETWEEN_RUNS_S
    drop = (adversarial.crates[0]["appear_t"] - adversarial.t[0]) / clock.speed
    assert snapshot(clock, at(clock, start_adv + drop - 0.5))["crates"] == []
    assert [c["model"] for c in snapshot(clock, at(clock, start_adv + drop + 0.5))["crates"]] == ["wooden_crate"]


def test_loop_restarts(clock):
    looping = load_replay(DEMO, speed=10.0, loop=True)
    assert looping.position(looping.started + looping.cycle + 0.5)[0] == 0


def test_missing_runs(tmp_path):
    with pytest.raises(FileNotFoundError):
        load_replay(tmp_path)
    with pytest.raises(FileNotFoundError):
        load_replay(DEMO, which="night_run")


fastapi = pytest.importorskip("fastapi")
from fastapi.testclient import TestClient  # noqa: E402

from lang2sim.telemetry.replay import create_replay_app  # noqa: E402


def test_replay_serves_the_control_api_routes():
    app = create_replay_app(DEMO, speed=10.0)
    with TestClient(app) as c:
        assert c.get("/v1/health").json()["mode"] == "replay"
        assert c.get("/v1/sim/status").json()["ready"] is True
        TelemetryLive(**c.get("/v1/telemetry/live").json())
        NavStatus(**c.get("/v1/nav/status").json())
        assert c.get("/v1/telemetry/setup").json()["enabled"]
        run = c.get("/v1/run").json()
        assert run["mode"] == "replay" and run["world_name"] == "empty_room" and run["has_report"]
        assert c.get("/v1/run/report").json()["runs"]["adversarial"]["nqi"] < 50
        assert set(c.get("/v1/run/trajectories").json()["runs"]) == {"nominal", "adversarial"}
        assert "Navigation Benchmark" in c.get("/dashboard").text
        for path in ("/v1/world/spawn", "/v1/telemetry/record/start", "/v1/nav/goal"):
            r = c.post(path, json={})
            assert r.status_code == 409 and "replay" in r.json()["detail"]
        with c.websocket_connect("/v1/nav/feedback?hz=20") as ws:
            msg = ws.receive_json()
        assert {"nav", "live", "sim", "telemetry"} <= set(msg)
        assert c.post("/v1/replay/restart").json() == {"restarted": True}
