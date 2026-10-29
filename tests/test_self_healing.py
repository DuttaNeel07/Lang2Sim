"""
Self-healer: fault classification, recovery plans, rate limits, goal snapping,
heal_log.jsonl. Fault injection without ROS.
File: tests/test_self_healing.py
"""

import json
from pathlib import Path

import pytest

from lang2sim.core.models import NavStatus, Pose2D
from lang2sim.runtime.ros_bridge import NAV2_NODES, RosSnapshot
from lang2sim.runtime.self_healing import (
    Fault,
    Observation,
    RateLimit,
    SelfHealer,
    classify,
    nearest_free,
    plan_recovery,
    read_map,
)

FIXTURES = Path(__file__).resolve().parents[1] / "catalog" / "fixtures"
ALIVE = {"robot_state_publisher": True, "bridge": True}


def healthy_snap(now=100.0, **kw):
    base = dict(
        now=now,
        clock_wall=now,
        odom_wall=now,
        scan_wall=now,
        map_pose=(0.0, 0.0, 0.0),
        lifecycle={n: "active" for n in NAV2_NODES},
    )
    base.update(kw)
    return RosSnapshot(**base)


def kinds(faults):
    return sorted(f.kind for f in faults)


# --- classify: one fault-injection case per rule --------------------------------


def test_healthy_run_has_no_faults():
    assert (
        classify(
            Observation(
                lines=["[INFO] all good"], snap=healthy_snap(), alive=ALIVE, was_localized=True
            )
        )
        == []
    )


def test_missing_package_from_log():
    obs = Observation(
        lines=[
            "[ERROR] [launch]: Caught exception: \"package 'turtlebot4_description' not found\"",
            "ModuleNotFoundError: No module named 'foo_msgs'",
        ],
        snap=None,
        alive=ALIVE,
    )
    f = classify(obs)
    assert kinds(f) == ["missing_package", "missing_package"]
    assert {x.signature for x in f} == {"turtlebot4_description", "foo_msgs"}


def test_missing_tf_when_rsp_dies_or_lookups_fail():
    died = Observation(
        lines=["[ERROR] [robot_state_publisher-3]: process has died [pid 41, exit code -9]"],
        snap=healthy_snap(),
        alive={**ALIVE, "robot_state_publisher": False},
        was_localized=True,
    )
    assert set(kinds(classify(died))) == {"missing_tf"}
    warn = ["[WARN] Timed out waiting for transform from base_link to map"]
    tf = Observation(
        lines=warn,
        snap=healthy_snap(),
        alive={**ALIVE, "robot_state_publisher": False},
        was_localized=True,
    )
    assert set(kinds(classify(tf))) == {"missing_tf"}
    # stale warnings while the publisher runs (e.g. right after a restart): report, never restart
    stale = Observation(lines=warn, snap=healthy_snap(), alive=ALIVE, was_localized=True)
    assert kinds(classify(stale)) == ["tf_warning"]
    assert plan_recovery(classify(stale)[0]).kind == "report"
    # the same warning during startup (never localized yet) is normal, not a fault
    assert classify(Observation(lines=warn, snap=healthy_snap(), alive=ALIVE)) == []


def test_unlinked_scan_when_bridge_dies_or_scan_stops():
    dead = Observation(lines=[], snap=healthy_snap(), alive={**ALIVE, "bridge": False})
    assert kinds(classify(dead)) == ["unlinked_scan"]
    stale = Observation(
        lines=[], snap=healthy_snap(now=100, scan_wall=80), alive=ALIVE, was_localized=True
    )
    assert kinds(classify(stale)) == ["unlinked_scan"]
    # clock stopped too (sim paused / dead): not the bridge's fault
    paused = Observation(
        lines=[],
        snap=healthy_snap(now=100, scan_wall=80, clock_wall=80),
        alive=ALIVE,
        was_localized=True,
    )
    assert classify(paused) == []


def test_goal_in_collision_only_while_navigating():
    line = "[ERROR] [planner_server]: GridBased plugin failed to plan from (0.00, 0.00) to (2.50, 2.50): Failed to create plan"
    nav = NavStatus(is_active=True, status="navigating", target_pose=Pose2D(x=2.5, y=2.5))
    assert kinds(
        classify(Observation(lines=[line], snap=healthy_snap(), alive=ALIVE, nav=nav))
    ) == ["goal_in_collision"]
    assert (
        classify(Observation(lines=[line], snap=healthy_snap(), alive=ALIVE, nav=NavStatus())) == []
    )


def test_plans():
    assert plan_recovery(Fault("missing_package", "x", "")).kind == "report"
    assert plan_recovery(Fault("missing_tf", "x", "")).target == "robot_state_publisher"
    assert plan_recovery(Fault("unlinked_scan", "x", "")).target == "bridge"
    assert plan_recovery(Fault("goal_in_collision", "x", "")).kind == "snap_goal"


# --- executor ---------------------------------------------------------------


class Clock:
    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t


def test_executor_restarts_and_rate_limits(tmp_path):
    clock, restarts = Clock(), []
    h = SelfHealer(
        tmp_path,
        restart_node=lambda n: restarts.append(n) or True,
        limit=RateLimit(cooldown_s=10, max_attempts=2, window_s=100),
        clock=clock,
    )
    f = Fault("unlinked_scan", "ros_gz_bridge not running", "process not found")
    assert h.handle([f])[0]["success"] is True and restarts == ["bridge"]
    assert h.handle([f]) == []  # cooldown
    clock.t += 11
    h.handle([f])
    assert restarts == ["bridge", "bridge"]
    clock.t += 11
    gave_up = h.handle([f])  # attempt limit -> one give-up record
    assert gave_up[0]["action_taken"].startswith("give_up") and restarts == ["bridge", "bridge"]
    assert h.handle([f]) == []
    clock.t += 200  # window passed: allowed again
    h.handle([f])
    assert restarts == ["bridge"] * 3

    lines = [json.loads(x) for x in (tmp_path / "logs" / "heal_log.jsonl").read_text().splitlines()]
    assert {"timestamp", "error_signature", "action_taken", "success", "evidence", "note"} <= set(
        lines[0]
    )
    assert lines[0]["error_signature"] == "unlinked_scan: ros_gz_bridge not running"


def test_missing_package_is_reported_once_never_installed(tmp_path):
    restarts = []
    h = SelfHealer(tmp_path, restart_node=lambda n: restarts.append(n) or True)
    f = Fault("missing_package", "turtlebot4_description", "package not found")
    ev = h.handle([f, f])
    assert (
        len(ev) == 1
        and ev[0]["success"] is False
        and "never" in ev[0]["note"]
        or "nothing is installed" in ev[0]["note"]
    )
    assert h.handle([f]) == [] and restarts == []


def test_failed_recovery_is_logged_not_raised(tmp_path):
    def boom(_):
        raise RuntimeError("pkill failed")

    h = SelfHealer(tmp_path, restart_node=boom)
    ev = h.handle([Fault("missing_tf", "robot_state_publisher not running", "")])
    assert ev[0]["success"] is False and "pkill failed" in ev[0]["note"]


# --- goal snapping on a real map ------------------------------------------------


def test_read_map_and_nearest_free_on_warehouse():
    m = FIXTURES / "small_warehouse" / "map.yaml"
    rows, w, h, res, origin, _ = read_map(m)
    assert (w, h, res, origin) == (240, 200, 0.05, (-6.0, -5.0))
    # rack_1 spans x -4..-1, y 2.1..2.9: a goal on it moves out to a free cell nearby
    p = nearest_free(m, -2.5, 2.5, clearance=0.3)
    assert p is not None and (p.y < 2.1 - 0.3 or p.y > 2.9 + 0.3 or p.x > -1 + 0.3)
    assert abs(p.x + 2.5) + abs(p.y - 2.5) < 2.0
    # an already-free goal stays where it is (within one cell)
    q = nearest_free(m, 0.0, 1.0, clearance=0.3)
    assert abs(q.x) <= 0.05 and abs(q.y - 1.0) <= 0.05


def test_snap_goal_resends(tmp_path):
    sent = []
    h = SelfHealer(
        tmp_path,
        restart_node=lambda n: True,
        resend_goal=lambda p: sent.append(p) or True,
        current_goal=lambda: Pose2D(x=-2.5, y=2.5, yaw=1.0),
        map_yaml=FIXTURES / "small_warehouse" / "map.yaml",
    )
    ev = h.handle([Fault("goal_in_collision", "goal not plannable", "planner failed")])
    assert (
        ev[0]["success"] and len(sent) == 1 and sent[0].yaw == 1.0 and "goal moved" in ev[0]["note"]
    )


@pytest.mark.parametrize("missing", [{"current_goal": None}, {"map_yaml": None}])
def test_snap_goal_without_goal_or_map(tmp_path, missing):
    kw = dict(
        restart_node=lambda n: True,
        resend_goal=lambda p: True,
        current_goal=lambda: Pose2D(x=0, y=0),
        map_yaml=FIXTURES / "small_warehouse" / "map.yaml",
    )
    kw.update(missing)
    ev = SelfHealer(tmp_path, **kw).handle([Fault("goal_in_collision", "g", "")])
    assert ev[0]["success"] is False


# --- wired into the API server -----------------------------------------------------


def test_server_heal_tick_restarts_dead_node_and_logs(tmp_path):
    import asyncio
    import sys

    pytest.importorskip("fastapi")
    httpx = pytest.importorskip("httpx")
    from lang2sim.core.models import SimStartRequest
    from lang2sim.runtime.control_api.client import ControlClient
    from lang2sim.runtime.control_api.server import SimProcess, create_app

    class DyingRsp(SimProcess):
        def build_command(self, req):
            code = (
                "import time; print('[INFO] [robot_state_publisher-3]: process started', flush=True); "
                "time.sleep(0.3); print('[ERROR] [robot_state_publisher-3]: process has died "
                "[pid 41, exit code -9]', flush=True); time.sleep(60)"
            )
            return [sys.executable, "-c", code]

    class FakeNodes:
        def __init__(self, run_dir):
            self.state = {"robot_state_publisher": False, "bridge": True}
            self.restarted = []

        def alive(self):
            return dict(self.state)

        def restart(self, name):
            self.restarted.append(name)
            self.state[name] = True
            return True

        def stop_all(self):
            pass

    made = {}

    def factory(run_dir):
        made["nodes"] = FakeNodes(run_dir)
        return made["nodes"]

    run = tmp_path / "run"
    run.mkdir()
    (run / "world.sdf").write_text('<sdf version="1.9"><world name="w"></world></sdf>')
    (run / "map.yaml").write_text("image: map.pgm\n")
    req = SimStartRequest(
        run_dir=str(run),
        world_sdf_path=str(run / "world.sdf"),
        map_yaml_path=str(run / "map.yaml"),
        initial_pose=Pose2D(x=0, y=0),
    )
    sim = DyingRsp()
    app = create_app(sim, use_ros=False, node_control=factory)

    async def go():
        async with ControlClient("http://t", transport=httpx.ASGITransport(app=app)) as api:
            await api.sim_start(req)
            for _ in range(50):
                if any("process has died" in x for x in sim.tail(10)):
                    break
                await asyncio.sleep(0.05)
            events = app.state.heal_tick()
            assert made["nodes"].restarted == ["robot_state_publisher"]
            assert events and events[0]["error_signature"].startswith("missing_tf")
            assert app.state.heal_tick() == []  # healed: nothing more to do
            log = await api._request("GET", "/heal/log")
            assert log["enabled"] and log["events"][0]["success"] is True

    try:
        asyncio.run(go())
    finally:
        sim.stop(force=True)
    assert "missing_tf" in (run / "logs" / "heal_log.jsonl").read_text()


def test_two_symptoms_one_restart(tmp_path):
    restarts = []
    h = SelfHealer(tmp_path, restart_node=lambda n: restarts.append(n) or True)
    ev = h.handle(
        [
            Fault("unlinked_scan", "ros_gz_bridge not running", ""),
            Fault("unlinked_scan", "no /scan while the clock runs", ""),
            Fault("missing_tf", "robot_state_publisher died", ""),
        ]
    )
    assert restarts == ["bridge", "robot_state_publisher"] and len(ev) == 2


def test_restarted_node_is_not_killed_again_by_lingering_warnings(tmp_path):
    """Regression (seen on macOS): Nav2 TF warnings after a restart re-triggered a restart."""
    restarts = []
    clock = Clock()
    h = SelfHealer(
        tmp_path,
        restart_node=lambda n: restarts.append(n) or True,
        limit=RateLimit(cooldown_s=15, max_attempts=3, window_s=300),
        clock=clock,
    )
    died = Observation(
        lines=["[ERROR] [robot_state_publisher-3]: process has died [pid 41, exit code -9]"],
        snap=healthy_snap(),
        alive={**ALIVE, "robot_state_publisher": False},
        was_localized=True,
    )
    h.handle(classify(died))
    assert restarts == ["robot_state_publisher"]
    for _ in range(10):  # 10 later passes, publisher alive, warnings still coming
        clock.t += 20
        h.handle(
            classify(
                Observation(
                    lines=["[WARN] Timed out waiting for transform from base_link to map"],
                    snap=healthy_snap(),
                    alive=ALIVE,
                    was_localized=True,
                )
            )
        )
    assert restarts == ["robot_state_publisher"]
