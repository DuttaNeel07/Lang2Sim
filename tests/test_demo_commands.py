"""TUI demo commands: parsing, natural-language stress requests, crate placement, and the TUI wiring."""

from __future__ import annotations

import math
from pathlib import Path

import pytest

from lang2sim.cli.demo_commands import (
    Command,
    parse_command,
    parse_stress_intent,
    pick_crate_spot,
    stress_targets,
)
from lang2sim.runtime.dashboard import read_map

FIXTURES = Path(__file__).resolve().parents[1] / "catalog" / "fixtures"
DEMO = Path(__file__).resolve().parents[1] / "catalog" / "demo_runs" / "empty_room_benchmark"


def test_parse_command():
    c = parse_command("/stress light=0.2 model=barrel")
    assert c.name == "stress" and c.opts == {"light": "0.2", "model": "barrel"}
    assert parse_command("/mission stress").args == ["stress"]
    assert parse_command("/MISSION").name == "mission"
    assert parse_command("/nope") is None and parse_command("stress the robot") is None


@pytest.mark.parametrize("text,light,model", [
    # architecture §8.3 Beat 3 and §8.5 Archetype 4 prompts
    ("Stress-test navigation: drop ambient lighting to 15%, inject camera motion blur, and spawn a moving "
     "obstacle directly in the robot's path.", "0.15", "wooden_crate"),
    ("While AMR is navigating, drop light to 10%, inject camera blur, and drop a dynamic wooden crate across "
     "the robot's planned path.", "0.10", "wooden_crate"),
    ("spawn a barrel in front of the robot", None, "barrel"),
    ("dim the lights", "0.15", None),
    ("set lighting to 0.3", "0.30", None),
    ("drop a cargo box ahead", None, "cargo_box"),
])
def test_stress_intents(text, light, model):
    c = parse_stress_intent(text)
    assert c is not None and c.name == "stress" and c.natural
    assert c.opts.get("light") == light and c.opts.get("model") == model
    assert c.note.startswith("Interpreted as /stress")


def test_blur_is_reported_as_unsupported():
    c = parse_stress_intent("drop light to 10% and inject camera blur")
    assert "camera blur" in c.note and "not available" in c.note


@pytest.mark.parametrize("text", [
    "why did the robot slow down?", "what is the NQI score", "explain the crate detour",
    "how do I add a crate?", "build me a warehouse with 8 racks", "drop it",
])
def test_questions_and_other_prompts_go_to_the_llm(text):
    assert parse_stress_intent(text) is None


def test_stress_targets():
    assert stress_targets(Command("stress")) == (0.1, "wooden_crate")
    assert stress_targets(Command("stress", opts={"light": "off"})) == (None, "wooden_crate")
    assert stress_targets(Command("stress", opts={"light": "20%", "model": "none"})) == (0.2, None)
    assert stress_targets(Command("stress", opts={"model": "barrel"}, natural=True)) == (None, "barrel")
    with pytest.raises(ValueError):
        stress_targets(Command("stress", opts={"model": "piano"}))


def test_crate_goes_ahead_toward_the_target_and_off_the_waypoints():
    m = read_map(FIXTURES / "empty_room")
    waypoints = [(2.5, -1.5), (2.5, 1.5), (-2.5, 1.5)]
    # Robot on leg 1 heading east toward W1 (2.5, -1.5).
    spot = pick_crate_spot({"x": -1.0, "y": -1.5, "yaw": 0.0}, (2.5, -1.5), waypoints, m)
    assert spot is not None
    x, y, yaw = spot
    assert y == pytest.approx(-1.5) and 0.4 < x < 1.3 and yaw == pytest.approx(0.0)
    assert all(math.hypot(x - a, y - b) >= 1.2 for a, b in waypoints)
    # Close to W1: the crate would block it, so nothing fits ahead.
    assert pick_crate_spot({"x": 1.6, "y": -1.5, "yaw": 0.0}, (2.5, -1.5), waypoints, m) is None
    # Facing a wall: no free cells ahead.
    assert pick_crate_spot({"x": 3.0, "y": 0.0, "yaw": 0.0}, None, [], m) is None


# --- the TUI, end to end against the replay server ------------------------------------------
fastapi = pytest.importorskip("fastapi")
uvicorn = pytest.importorskip("uvicorn")


@pytest.fixture
def replay_api(unused_tcp_port_factory=None):
    import socket
    import threading
    import time

    from lang2sim.telemetry.replay import create_replay_app

    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        port = s.getsockname()[1]
    server = uvicorn.Server(uvicorn.Config(create_replay_app(DEMO, speed=20, loop=True), host="127.0.0.1",
                                           port=port, log_level="warning"))
    threading.Thread(target=server.run, daemon=True).start()
    for _ in range(100):
        if server.started:
            break
        time.sleep(0.05)
    yield f"http://127.0.0.1:{port}"
    server.should_exit = True


@pytest.mark.asyncio
async def test_tui_runs_demo_commands_against_the_api(replay_api, monkeypatch, tmp_path):
    from textual.widgets import Static

    monkeypatch.setenv("LANG2SIM_CONTROL_API", replay_api)
    monkeypatch.setenv("HOME", str(tmp_path))
    from lang2sim.cli.app import Lang2SimApp
    from lang2sim.cli.screens.workspace_screen import WorkspaceScreen

    app = Lang2SimApp()
    async with app.run_test(size=(140, 60)) as pilot:
        screen = WorkspaceScreen(initial_prompt="")
        await app.push_screen(screen)
        await pilot.pause()

        async def ask(text: str) -> list:
            before = len(screen.query(".demo-line, .demo-table"))
            screen.handle_user_prompt(text)
            for _ in range(60):
                await pilot.pause(0.1)
                if len(screen.query(".demo-line, .demo-table")) > before and not screen.demo.mission_lock.locked():
                    await app.workers.wait_for_complete()
                    break
            return list(screen.query(".demo-line, .demo-table"))[before:]

        out = await ask("/report")
        assert any("demo-table" in w.classes for w in out)                    # the NQI table
        assert any("81.6" in str(w.render()) for w in out if "demo-line" in w.classes)
        out = await ask("/status")
        assert "REPLAY" in str(out[0].render())
        out = await ask("drop the lights to 10% and spawn a crate in front of the robot")
        text = " ".join(str(w.render()) for w in out)
        assert "Interpreted as /stress" in text and "replay" in text.lower()   # read-only, clearly said
        assert not screen.is_streaming                                         # the LLM was never called
