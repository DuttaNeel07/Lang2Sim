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
    assert c.note.startswith("Understood: ")


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
    for k in ("NVIDIA_API_KEY", "OPENAI_API_KEY"):
        monkeypatch.setenv(k, "")                                            # the offline parser path
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

        out = await ask("show me the score")                                   # plain English, no LLM key
        assert any("demo-table" in w.classes for w in out)                    # the NQI table
        assert any("81.6" in str(w.render()) for w in out if "demo-line" in w.classes)
        out = await ask("where is the robot?")
        assert "REPLAY" in " ".join(str(w.render()) for w in out)
        out = await ask("drop the lights to 10% and spawn a crate in front of the robot")
        text = " ".join(str(w.render()) for w in out)
        assert "Understood" in text and "replay" in text.lower()               # read-only, clearly said
        out = await ask("/report")                                              # shortcuts still work
        assert any("demo-table" in w.classes for w in out)
        assert not screen.is_streaming                                         # the LLM was never called


# --- merged pitch pipeline (PR #17) intents ----------------------------------------------------
from lang2sim.cli.demo_commands import (  # noqa: E402
    is_goto,
    is_halt,
    is_sim_request,
    is_stop_sim,
    rack_number,
    resolve_goal,
)


@pytest.mark.parametrize("text,expected", [
    ("a dark warehouse with 3 racks and 2 crates", False),          # no verb: chat about it, don't launch
    ("build a dark warehouse with 3 racks and 2 crates", True),
    ("simulate an empty room with a slippery floor", True),
    ("/sim warehouse", True),
    ("how do I build a warehouse?", False),
    ("what does the rack do", False),
    ("spawn a crate", False),                                         # a stress request, not a world
])
def test_sim_requests(text, expected):
    assert is_sim_request(text) is expected


def test_stop_halt_goto():
    assert is_stop_sim("/stop") and is_stop_sim("stop simulation") and is_stop_sim("Stop")
    assert not is_stop_sim("stop the robot")
    assert is_halt("stop the robot") and is_halt("halt") and not is_halt("why did it halt?")
    assert is_goto("detour and reach the 3rd rack") and is_goto("resume") and is_goto("go to rack 2")
    assert not is_goto("how does nav2 avoid obstacles?")


def test_rack_spawn_coordinates_and_orientation():
    c = parse_stress_intent("put a rack across the aisle at 1.5, 0")
    assert c.opts == {"model": "rack", "x": "1.5", "y": "0", "across": "1"}
    c = parse_stress_intent("drop a barrel at x=2 y=-1.5")
    assert (c.opts["model"], c.opts["x"], c.opts["y"]) == ("barrel", "2", "-1.5")
    assert stress_targets(Command("stress", opts={"model": "rack"}, natural=True)) == (None, "rack")


def test_rack_number_and_goal_in_front_of_it():
    assert rack_number("detour and reach the 3rd rack") == 3 and rack_number("go to rack 2") == 2
    assert rack_number("resume") is None
    racks = [{"id": "rack_1", "pose": {"x": -2.5, "y": 2.5}, "dimensions": {"x": 3.0, "y": 0.8}},
             {"id": "rack_3", "pose": {"x": -2.5, "y": -0.5}, "dimensions": {"x": 3.0, "y": 0.8}}]
    goal, desc = resolve_goal("reach rack 1", racks, None)
    assert goal[0] == -2.5 and goal[1] < 2.5 - 0.4 and "rack_1" in desc        # on the room side of it
    goal, _ = resolve_goal("go to the 3rd rack", racks, None)
    assert goal[1] > -0.5 + 0.4                                                   # rack below 0 -> aisle above
    assert resolve_goal("go to rack 9", racks, None)[0] is None
    assert resolve_goal("resume", racks, {"x": 1.0, "y": 2.0, "yaw": 0.5})[0] == (1.0, 2.0, 0.5)


def test_rack_across_the_path_keeps_off_the_robot_and_walls():
    m = read_map(FIXTURES / "small_warehouse")
    spot = pick_crate_spot({"x": -4.5, "y": -3.5, "yaw": 0.0}, (0.0, 1.0), [], m, "rack", 1.5, across=True)
    if spot is not None:                                                          # free space permitting
        x, y, yaw = spot
        assert math.hypot(x + 4.5, y + 3.5) >= 0.6 + 0.6                          # rack edge >= 0.6 m away


def test_obstacle_goes_on_the_recorded_lane_when_straight_ahead_is_blocked():
    from lang2sim.cli.demo_commands import pick_spot_fan, pick_spot_on_path

    m = read_map(FIXTURES / "empty_room")
    # Recorded nominal lane along y = -1.5 from x = -2.5 to 2.5, then up to (2.5, 1.5).
    lane = [(-2.5 + 0.05 * i, -1.5) for i in range(101)] + [(2.5, -1.5 + 0.05 * i) for i in range(1, 61)]
    spot = pick_spot_on_path({"x": -1.0, "y": -1.5, "yaw": 0.0}, lane, [(2.5, -1.5), (2.5, 1.5)], m)
    assert spot is not None and spot[1] == pytest.approx(-1.5) and 0.4 < spot[0] < 1.3     # on the lane, ahead
    # Robot far off the recorded lane (already detoured): no lane placement.
    assert pick_spot_on_path({"x": 0.0, "y": 1.0, "yaw": 0.0}, lane, [], m) is None
    # Fan search turns away from a wall dead ahead.
    fan = pick_spot_fan({"x": 2.0, "y": 0.0, "yaw": 0.6}, None, [], m)
    assert fan is not None


# --- obstacles must leave the robot a way around (route_exists) --------------------------------
def test_route_exists_closes_only_when_the_gap_is_too_narrow():
    from lang2sim.cli.demo_commands import route_exists

    m = read_map(FIXTURES / "empty_room")
    start, goal = (-2.5, -1.5), (2.5, -1.5)
    assert route_exists(m, start, [goal])
    assert route_exists(m, start, [goal], [(0.0, -1.5, 0.0, (0.3, 0.3))])          # a crate: drive around it
    # a wall of racks across the whole room: no way past
    wall = [(0.0, y, math.pi / 2, (1.4, 0.6)) for y in (-3.0, -0.5, 2.0, 4.5)]
    assert not route_exists(m, start, [goal], wall)
    # leave a 0.3 m slot (narrower than the TurtleBot3): still closed; a 0.8 m slot: open
    def wall_with_gap(gap):
        y0, y1 = -3.6, 3.6                                                           # room interior
        mid = -1.5
        return [(0.0, (y0 + mid - gap / 2) / 2, 0.0, (0.1, (mid - gap / 2 - y0) / 2)),
                (0.0, (mid + gap / 2 + y1) / 2, 0.0, (0.1, (y1 - mid - gap / 2) / 2))]
    assert not route_exists(m, start, [goal], wall_with_gap(0.3))
    assert route_exists(m, start, [goal], wall_with_gap(0.8))
    assert route_exists(None, start, [goal], wall)                                  # no map: can't tell, allow


def test_auto_placement_skips_spots_that_seal_the_robot_in():
    from lang2sim.cli.demo_commands import pick_spot_on_path

    m = read_map(FIXTURES / "empty_room")
    lane = [(-2.5 + 0.05 * i, -1.5) for i in range(101)]
    calls = []

    def never(x, y, yaw):
        calls.append((x, y))
        return False
    assert pick_spot_on_path({"x": -1.0, "y": -1.5, "yaw": 0.0}, lane, [], m, accept=never) is None
    assert calls                                                                     # it asked for each candidate
