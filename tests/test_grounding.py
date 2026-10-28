"""Chat grounding: live telemetry + the latest NQI report as LLM context."""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from lang2sim.agent.grounding import fetch_context, format_context, with_context

DEMO = Path(__file__).resolve().parents[1] / "catalog" / "demo_runs" / "empty_room_benchmark"
REPORT = json.loads((DEMO / "benchmark_report.json").read_text())


def test_context_carries_the_numbers_that_explain_the_score():
    live = {
        "sim_time": 120.0,
        "real_time_factor": 0.6,
        "pose": {"x": 2.5, "y": -0.4, "yaw": 1.57},
        "pose_source": "ground_truth",
        "linear_speed": 0.12,
        "obstacle_clearance_m": 0.09,
        "distance_to_goal_m": 6.1,
        "collision_count": 0,
        "localization_error_m": 0.05,
        "recording": True,
        "recording_label": "adversarial",
    }
    nav = {
        "status": "navigating",
        "is_active": True,
        "waypoint_index": 1,
        "waypoints_total": 3,
        "distance_remaining": 1.9,
        "recoveries": 2,
    }
    run = {
        "loaded": True,
        "mode": "live",
        "world_name": "empty_room",
        "mission": {
            "initial_pose": {"x": -2.5, "y": -1.5},
            "waypoints": [{"x": 2.5, "y": -1.5}],
            "goal_pose": {"x": -2.5, "y": 1.5},
        },
        "spawned_entities": [{"model": "wooden_crate", "x": 2.69, "y": 0.05}],
    }
    text = format_context({"mode": None}, live, nav, run, REPORT)
    assert "LIVE simulation" in text and "empty_room" in text
    assert "wooden_crate at (2.69, 0.05)" in text and "recording the 'adversarial' run" in text
    assert "waypoint 2/3" in text and "2 recoveries" in text
    assert "NQI 81.6/100" in text and "NQI 39.8/100" in text  # both runs
    assert "clearance 40.0" in text  # nominal clearance term saturated
    assert "C_min 0.06 m" in text and "NQI = 100*" in text
    assert "dropped at (2.686, 0.05)" in text and "sun dimmed to 10%" in text
    assert "/stress" in text  # it can't act, the user can


def test_no_simulation_means_no_context(monkeypatch):
    assert format_context(None, None, None, None, None) is None
    monkeypatch.setenv("LANG2SIM_CONTROL_API", "http://127.0.0.1:1")
    assert fetch_context(timeout=0.2) is None


def test_context_goes_right_before_the_latest_user_turn():
    history = [
        {"role": "system", "content": "sys"},
        {"role": "user", "content": "hi"},
        {"role": "assistant", "content": "hello"},
        {"role": "user", "content": "why did NQI drop?"},
    ]
    out = with_context(history, "CTX")
    assert [m["role"] for m in out] == ["system", "user", "assistant", "system", "user"]
    assert out[3]["content"] == "CTX" and out[-1]["content"] == "why did NQI drop?"
    assert with_context(history, None) == history and history[-1]["role"] == "user"  # not mutated


fastapi = pytest.importorskip("fastapi")


def test_fetch_context_from_a_replay(monkeypatch):
    import threading
    import time

    import uvicorn

    from lang2sim.telemetry.replay import create_replay_app

    import socket

    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        port = s.getsockname()[1]
    server = uvicorn.Server(
        uvicorn.Config(
            create_replay_app(DEMO, speed=10), host="127.0.0.1", port=port, log_level="warning"
        )
    )
    threading.Thread(target=server.run, daemon=True).start()
    try:
        for _ in range(100):
            if server.started:
                break
            time.sleep(0.05)
        text = fetch_context(f"http://127.0.0.1:{port}")
        assert "REPLAY of a recorded run" in text and "Robot now" in text and "NQI 39.8/100" in text
    finally:
        server.should_exit = True


@pytest.mark.asyncio
async def test_tui_sends_the_context_to_the_model(monkeypatch, tmp_path):
    monkeypatch.setenv("HOME", str(tmp_path))
    import lang2sim.agent.grounding as g

    monkeypatch.setattr(g, "fetch_context", lambda *a, **k: "SIM CONTEXT: NQI 81.6 -> 39.8")
    from lang2sim.cli.app import Lang2SimApp
    from lang2sim.cli.screens.workspace_screen import WorkspaceScreen

    sent = {}

    async def fake_stream(messages, tools=None, temperature=0.2):
        sent["messages"] = messages
        yield "The clearance term fell because the robot passed 6 cm from the crate."

    app = Lang2SimApp()
    async with app.run_test(size=(120, 40)) as pilot:
        screen = WorkspaceScreen(initial_prompt="")
        await app.push_screen(screen)
        await pilot.pause()
        monkeypatch.setattr(screen.client, "is_configured", lambda: True)
        monkeypatch.setattr(screen.client, "stream_chat", fake_stream)
        screen.handle_user_prompt("why did the NQI drop?")
        await app.workers.wait_for_complete()
        await pilot.pause()
    roles = [m["role"] for m in sent["messages"]]
    assert sent["messages"][-1] == {"role": "user", "content": "why did the NQI drop?"}
    assert sent["messages"][-2] == {"role": "system", "content": "SIM CONTEXT: NQI 81.6 -> 39.8"}
    assert roles.count("system") == 2  # base prompt + context
