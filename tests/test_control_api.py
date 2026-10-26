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
        return [sys.executable, "-u", "-c", "import time; print('gz up', flush=True); time.sleep(60)"]


def _client(sim):
    return ControlClient("http://test", transport=httpx.ASGITransport(app=create_app(sim)))


def test_api_health_info_status():
    async def go():
        async with _client(FakeSim()) as api:
            h = await api.health()
            assert h["status"] == "ok"
            assert "display_mode" in await api.info()
            assert (await api.sim_status()).status == "idle"
    asyncio.run(go())


def test_api_start_logs_conflict_stop(tmp_path):
    world = tmp_path / "world.sdf"
    world.write_text("<sdf/>")
    req = SimStartRequest(run_dir=str(tmp_path / "run"), world_sdf_path=str(world),
                          map_yaml_path=str(tmp_path / "map.yaml"), initial_pose=Pose2D(x=0, y=0))
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
    assert (tmp_path / "run" / "logs" / "sim.log").read_text().startswith("gz up")


def test_api_start_missing_world_is_404(tmp_path):
    req = SimStartRequest(run_dir=str(tmp_path), world_sdf_path="/nope/world.sdf",
                          map_yaml_path="/nope/map.yaml", initial_pose=Pose2D(x=0, y=0))

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
