"""
Control API client <-> server, in-process via httpx.ASGITransport.
File: tests/test_control_api.py

Needs the `sim` extra (fastapi, uvicorn): pip install -e ".[sim,dev]".
Skipped otherwise so test_runtime_launch.py still runs everywhere.
"""

from __future__ import annotations

import asyncio
import sys

import pytest

# ---------------------------------------------------------------------------

fastapi = pytest.importorskip("fastapi")
httpx = pytest.importorskip("httpx")

from lang2sim.core.models import Pose2D, SimStartRequest  # noqa: E402
from lang2sim.runtime.control_api.client import ControlAPIError, ControlClient  # noqa: E402
from lang2sim.runtime.control_api.server import SimProcess, create_app  # noqa: E402


class FakeSim(SimProcess):
    def build_command(self, req):
        return [
            sys.executable,
            "-u",
            "-c",
            "import time; print('gz up', flush=True); time.sleep(60)",
        ]


GOOD_WORLD = '<?xml version="1.0"?><sdf version="1.9"><world name="w"></world></sdf>'
BAD_WORLD = '<sdf version="1.9"><world name="w"><model name="wall"><pose>04.0 1 0 0 0</pose></model></world></sdf>'


def _client(sim):
    return ControlClient(
        "http://test", transport=httpx.ASGITransport(app=create_app(sim, use_ros=False))
    )


def _run(tmp_path, world=GOOD_WORLD):
    run = tmp_path / "run"
    run.mkdir(parents=True, exist_ok=True)
    (run / "world.sdf").write_text(world)
    (run / "map.yaml").write_text("image: map.pgm\n")
    return SimStartRequest(
        run_dir=str(run),
        world_sdf_path=str(run / "world.sdf"),
        map_yaml_path=str(run / "map.yaml"),
        initial_pose=Pose2D(x=1, y=2, yaw=0.5),
        world_name=None,
    )


def test_api_health_info_status():
    async def go():
        async with _client(FakeSim()) as api:
            h = await api.health()
            assert h["status"] == "ok"
            assert "display_mode" in await api.info()
            assert (await api.sim_status()).status == "idle"

    asyncio.run(go())


def test_api_start_logs_conflict_stop(tmp_path):
    req = _run(tmp_path)
    sim = FakeSim()

    async def go():
        async with _client(sim) as api:
            st = await api.sim_start(req)
            assert st.status in ("launching", "running")
            for _ in range(50):
                if "gz up" in await api.sim_logs():
                    break
                await asyncio.sleep(0.1)
            assert "gz up" in await api.sim_logs()
            with pytest.raises(ControlAPIError) as e:
                await api.sim_start(req)
            assert e.value.status_code == 409
            assert (await api.sim_stop()).status == "stopped"

    try:
        asyncio.run(go())
    finally:
        sim.stop(force=True)
    run = tmp_path / "run"
    assert (run / "logs" / "sim.log").read_text().startswith("gz up")
    # pre-flight added the missing systems and the launch file points at the normalized world
    launch = (run / "sim_launch.py").read_text()
    assert "world.l2s.sdf" in launch and "'world_name': 'w'" in launch and "'x': 1.0" in launch
    assert "added missing Gazebo systems" in (run / "logs" / "world_check.txt").read_text()


def test_api_rejects_bad_world_with_report(tmp_path):
    req = _run(tmp_path, BAD_WORLD)

    async def go():
        async with _client(FakeSim()) as api:
            with pytest.raises(ControlAPIError) as e:
                await api.sim_start(req)
            assert e.value.status_code == 422 and "malformed pose" in str(e.value)
            st = await api.sim_status()
            assert st.status == "idle" and any("pose" in line for line in st.world_report)

    asyncio.run(go())


def test_status_uses_readiness_when_ros_is_present(tmp_path):
    from lang2sim.runtime.ros_bridge import NAV2_NODES, RosSnapshot

    class FakeBridge:
        def __init__(self):
            self.ready = False

        def reset(self, spawn=None):
            pass

        def snapshot(self):
            import time as _t

            now = _t.time()
            if not self.ready:
                return RosSnapshot(now=now, clock_wall=now, sim_time=1.0)
            return RosSnapshot(
                now=now,
                clock_wall=now,
                odom_wall=now,
                scan_wall=now,
                sim_time=9.5,
                rtf=0.8,
                map_pose=(1.0, 2.0, 0.5),
                lifecycle={n: "active" for n in NAV2_NODES},
            )

    sim = FakeSim()
    app = create_app(sim, use_ros=False)
    bridge = FakeBridge()
    app.state.ros["bridge"] = bridge
    req = _run(tmp_path)

    async def go():
        async with ControlClient("http://test", transport=httpx.ASGITransport(app=app)) as api:
            st = await api.sim_start(req)
            assert st.status == "launching" and not st.ready and st.readiness["gazebo_clock"]
            bridge.ready = True
            st = await api.sim_status()
            assert st.status == "running" and st.ready and st.rtf == 0.8 and st.sim_time == 9.5
            bridge.ready = False
            assert (await api.sim_status()).status == "degraded"

    try:
        asyncio.run(go())
    finally:
        sim.stop(force=True)


def test_api_start_missing_world_is_404(tmp_path):
    req = SimStartRequest(
        run_dir=str(tmp_path),
        world_sdf_path="/nope/world.sdf",
        map_yaml_path="/nope/map.yaml",
        initial_pose=Pose2D(x=0, y=0),
    )

    async def go():
        async with _client(FakeSim()) as api:
            with pytest.raises(ControlAPIError) as e:
                await api.sim_start(req)
            assert e.value.status_code == 404

    asyncio.run(go())


def test_client_unavailable_is_typed():
    from lang2sim.runtime.control_api.client import ControlAPIUnavailable

    async def go():
        async with ControlClient("http://127.0.0.1:9", timeout=1) as api:
            with pytest.raises(ControlAPIUnavailable):
                await api.wait_until_ready(timeout=1, interval=0.2)

    asyncio.run(go())
