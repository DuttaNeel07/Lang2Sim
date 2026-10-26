"""
Navigation + telemetry endpoints and the feedback WebSocket, in-process, with a
fake ROS bridge and a fake BasicNavigator.
File: tests/test_control_api_nav.py
"""

from __future__ import annotations

import asyncio
import sys
import time
from types import SimpleNamespace

import pytest

fastapi = pytest.importorskip("fastapi")
httpx = pytest.importorskip("httpx")

from fastapi.testclient import TestClient  # noqa: E402

from sim2lang.core.models import Pose2D, SimStartRequest  # noqa: E402
from sim2lang.runtime.control_api.client import ControlAPIError, ControlClient  # noqa: E402
from sim2lang.runtime.control_api.server import SimProcess, create_app  # noqa: E402
from sim2lang.runtime.nav2_dispatcher import NavDispatcher  # noqa: E402
from sim2lang.runtime.ros_bridge import NAV2_NODES, RosSnapshot  # noqa: E402

WORLD = '<?xml version="1.0"?><sdf version="1.9"><world name="w"></world></sdf>'


class FakeSim(SimProcess):
    def build_command(self, req):
        return [sys.executable, "-c", "import time; time.sleep(60)"]


class FakeBridge:
    def __init__(self, ready=True):
        self.ready = ready

    def reset(self):
        pass

    def snapshot(self):
        now = time.time()
        if not self.ready:
            return RosSnapshot(now=now, clock_wall=now)
        return RosSnapshot(now=now, clock_wall=now, odom_wall=now, scan_wall=now, sim_time=12.0, rtf=0.9,
                           odom_pose=(0.1, 0.2, 0.0), map_pose=(-2.0, -0.5, 3.1416),
                           velocity=(0.22, 0.0, -0.4), lifecycle={n: "active" for n in NAV2_NODES})


class FakeNav:
    def __init__(self, steps=5):
        self.steps, self.goals, self.initial, self._left, self._c = steps, [], [], 0, False

    def goToPose(self, p):
        self.goals.append(p)
        self._left, self._c = self.steps, False
        return True

    def isTaskComplete(self):
        if self._c or self._left <= 0:
            return True
        self._left -= 1
        return False

    def getFeedback(self):
        return SimpleNamespace(distance_remaining=float(self._left), navigation_time=1.0,
                               estimated_time_remaining=2.0, number_of_recoveries=0)

    def getResult(self):
        return SimpleNamespace(name="CANCELED" if self._c else "SUCCEEDED")

    def cancelTask(self):
        self._c = True

    def setInitialPose(self, p):
        self.initial.append(p)


@pytest.fixture
def setup(tmp_path):
    run = tmp_path / "run"
    run.mkdir()
    (run / "world.sdf").write_text(WORLD)
    (run / "map.yaml").write_text("image: map.pgm\n")
    req = SimStartRequest(run_dir=str(run), world_sdf_path=str(run / "world.sdf"),
                          map_yaml_path=str(run / "map.yaml"), initial_pose=Pose2D(x=-2, y=-0.5))
    sim, nav = FakeSim(), FakeNav(steps=10_000)
    disp = NavDispatcher(navigator_factory=lambda: nav, make_pose=lambda _n, p: p)
    app = create_app(sim, use_ros=False, nav=disp)
    bridge = FakeBridge(ready=False)
    app.state.ros["bridge"] = bridge
    yield app, bridge, nav, req, disp
    disp.shutdown()
    sim.stop(force=True)


def test_nav_requires_ready_then_runs(setup):
    app, bridge, nav, req, disp = setup

    async def go():
        async with ControlClient("http://t", transport=httpx.ASGITransport(app=app)) as api:
            with pytest.raises(ControlAPIError) as e:
                await api.nav_goal(Pose2D(x=1, y=1))
            assert e.value.status_code == 409 and "not ready" in str(e.value)
            await api.sim_start(req)
            with pytest.raises(ControlAPIError) as e:
                await api.nav_goal(Pose2D(x=1, y=1))
            assert "waiting for" in str(e.value) and "laser_scan" in str(e.value)

            bridge.ready = True
            st = await api.nav_goal(Pose2D(x=1, y=1), timeout_seconds=300)
            assert st.is_active and st.status == "navigating" and st.target_pose == Pose2D(x=1, y=1)
            for _ in range(100):
                if (await api.nav_status()).distance_remaining > 0:
                    break
                await asyncio.sleep(0.02)
            assert (await api.nav_status()).distance_remaining > 0
            st = await api.nav_cancel()
            done = await api.wait_nav_done(poll_s=0.05, timeout=5)
            assert done.status == "canceled"

            nav.steps = 3
            st = await api.nav_waypoints([Pose2D(x=1, y=0), Pose2D(x=2, y=0)])
            assert st.waypoints_total == 2
            done = await api.wait_nav_done(poll_s=0.05, timeout=5)
            assert done.status == "succeeded" and nav.goals[-2:] == [Pose2D(x=1, y=0), Pose2D(x=2, y=0)]

            await api.nav_initial_pose(Pose2D(x=-2, y=-0.5, yaw=0.1))
            for _ in range(100):
                if nav.initial:
                    break
                await asyncio.sleep(0.02)
            assert nav.initial == [Pose2D(x=-2, y=-0.5, yaw=0.1)]
    asyncio.run(go())


def test_telemetry_and_empty_waypoints(setup):
    app, bridge, nav, req, disp = setup

    async def go():
        async with ControlClient("http://t", transport=httpx.ASGITransport(app=app)) as api:
            idle = await api.telemetry()
            assert idle.is_running is False and idle.health_status == "healthy"
            bridge.ready = True
            await api.sim_start(req)
            t = await api.telemetry()
            assert t.is_running and t.health_status == "healthy" and t.real_time_factor == 0.9
            assert (t.robot_pose.x, t.robot_pose.y) == (-2.0, -0.5)          # map pose preferred over odom
            assert t.robot_velocity.x == 0.22 and t.robot_velocity.yaw == -0.4
            with pytest.raises(ControlAPIError) as e:
                await api.nav_waypoints([])
            assert e.value.status_code == 422
    asyncio.run(go())


def test_feedback_websocket_streams_nav_and_telemetry(setup):
    app, bridge, nav, req, disp = setup
    bridge.ready = True
    with TestClient(app) as tc:
        assert tc.post("/v1/sim/start", json=req.model_dump(mode="json")).status_code == 202
        tc.post("/v1/nav/goal", json={"goal_pose": {"x": 3, "y": 1, "yaw": 0}})
        with tc.websocket_connect("/v1/nav/feedback?hz=20") as ws:
            msgs = [ws.receive_json() for _ in range(3)]
    m = msgs[-1]
    assert set(m) == {"nav", "telemetry", "sim"}
    assert m["nav"]["target_pose"]["x"] == 3 and m["nav"]["status"] == "navigating"
    assert m["telemetry"]["robot_pose"]["x"] == -2.0 and m["sim"]["ready"] is True
