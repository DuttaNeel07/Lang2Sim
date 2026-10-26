"""
Docker integration: real sim container, real control API, real gz sim.
File: tests/integration/test_container.py

Runs only when asked (needs Docker + a built image), so CI unit runs skip it:

    docker buildx build -f docker/sim/Dockerfile --target sim -t lang2sim-sim:dev --load .
    L2S_INTEGRATION=1 pytest -q tests/integration

    # macOS / noVNC image:
    L2S_INTEGRATION=1 L2S_TEST_IMAGE=lang2sim-sim:dev-vnc pytest -q tests/integration
"""

from __future__ import annotations

import asyncio
import os
import socket
import time

import pytest

from sim2lang.core.models import Pose2D, SimStartRequest
from sim2lang.runtime.control_api.client import ControlClient
from sim2lang.runtime.display import DisplayPlan
from sim2lang.runtime.docker_runner import (
    DockerRunner,
    RunOptions,
    host_to_container_path,
)
from sim2lang.runtime.platform import detect_docker

pytestmark = pytest.mark.skipif(os.getenv("L2S_INTEGRATION") != "1",
                                reason="set L2S_INTEGRATION=1 to run Docker integration tests")

IMAGE = os.getenv("L2S_TEST_IMAGE", "lang2sim-sim:dev")
NAME = "lang2sim-sim-itest"

EMPTY_WORLD = """<?xml version="1.0"?>
<sdf version="1.9">
  <world name="l2s_itest">
    <physics name="1ms" type="ignored"><max_step_size>0.001</max_step_size><real_time_factor>1.0</real_time_factor></physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <model name="ground"><static>true</static><link name="l"><collision name="c"><geometry><plane><normal>0 0 1</normal></plane></geometry></collision></link></model>
  </world>
</sdf>
"""


def _free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


@pytest.fixture(scope="module")
def runner():
    if not detect_docker().server_reachable:
        pytest.skip("docker daemon not reachable")
    r = DockerRunner()
    if not r.image_present(IMAGE):
        pytest.skip(f"image {IMAGE} not built")
    yield r
    r.remove(NAME)


@pytest.mark.parametrize("mode", ["headless", "vnc"])
def test_container_api_and_gz(runner, tmp_path, mode):
    if mode == "vnc" and not IMAGE.endswith("-vnc"):
        pytest.skip("vnc mode needs the sim-vnc image")
    projects = tmp_path / "projects"
    run_dir = projects / "itest" / "runs" / "001"
    run_dir.mkdir(parents=True)
    (run_dir / "world.sdf").write_text(EMPTY_WORLD)
    port = _free_port()

    plan = DisplayPlan(mode=mode, gpu="software", foxglove=(mode == "headless"))  # type: ignore[arg-type]
    opts = RunOptions(image=IMAGE, plan=plan, projects_dir=projects, name=NAME,
                      host_uid=os.getuid(), host_gid=os.getgid(), api_port=port,
                      foxglove_port=_free_port(), novnc_port=_free_port())
    runner.start(opts)

    async def go():
        async with ControlClient(f"http://127.0.0.1:{port}") as api:
            await api.wait_until_ready(timeout=90)
            info = await api.info()
            assert info["display_mode"] == mode and info["gz_version"] == "harmonic"

            st = await api.sim_start(SimStartRequest(
                run_dir=host_to_container_path(run_dir, projects),
                world_sdf_path=host_to_container_path(run_dir / "world.sdf", projects),
                map_yaml_path="/dev/null", initial_pose=Pose2D(x=0, y=0), use_gui=(mode == "vnc")))
            assert st.status in ("launching", "running")

            deadline = time.monotonic() + 60
            while time.monotonic() < deadline:
                if (await api.sim_status()).status == "running":
                    break
                await asyncio.sleep(1)
            assert (await api.sim_status()).status == "running", await api.sim_logs(50)

            await asyncio.sleep(3)
            assert (await api.sim_status()).status == "running", await api.sim_logs(50)
            assert (await api.sim_stop()).status == "stopped"

    try:
        asyncio.run(go())
    except BaseException:
        print(runner.logs(NAME, tail=80))
        raise
    finally:
        runner.remove(NAME)

    log = run_dir / "logs" / "sim.log"
    assert log.exists(), "sim.log should land in the host's project dir via the bind mount"
    assert log.stat().st_uid == os.getuid(), "files written in the container should be owned by the host user"
