"""
Docker integration: real sim container, real control API, display modes.
File: tests/integration/test_container.py

Runs only when asked (needs Docker + a built image), so CI unit runs skip it:

    docker buildx build -f docker/sim/Dockerfile --target sim -t lang2sim-sim:dev --load .
    L2S_INTEGRATION=1 pytest -q tests/integration

    # macOS / noVNC image (also runs the GUI variant):
    L2S_INTEGRATION=1 L2S_TEST_IMAGE=lang2sim-sim:dev-vnc pytest -q tests/integration

Covers: container boot and info per display mode, a bad world rejected with a
422 report, the empty_room fixture reaching ready (with the Gazebo GUI and RViz
in vnc mode), clean stop, and run-folder files owned by the host user.
"""

from __future__ import annotations

import asyncio
import os
import shutil
import socket
from pathlib import Path

import pytest

from sim2lang.core.models import Pose2D, SimStartRequest
from sim2lang.runtime.control_api.client import ControlAPIError
from sim2lang.runtime.display import DisplayPlan
from sim2lang.runtime.docker_runner import (
    DockerRunner,
    RunOptions,
    host_to_container_path,
)
from sim2lang.runtime.platform import detect_docker
from sim2lang.runtime.session import FIXTURES_DIR, SimSession

pytestmark = pytest.mark.skipif(os.getenv("L2S_INTEGRATION") != "1",
                                reason="set L2S_INTEGRATION=1 to run Docker integration tests")

IMAGE = os.getenv("L2S_TEST_IMAGE", "lang2sim-sim:dev")
REPO = Path(__file__).resolve().parents[2]
NAME = "lang2sim-sim-itest"
BAD_WORLD = '<sdf version="1.9"><world name="w"><model name="wall"><pose>04.0 1 0 0 0</pose></model></world></sdf>'


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
def test_container_modes_reject_and_run(runner, tmp_path, mode):
    if mode == "vnc" and not IMAGE.endswith("-vnc"):
        pytest.skip("vnc mode needs the sim-vnc image")
    projects = tmp_path / "projects"
    run = projects / "itest" / "runs" / "001"
    shutil.copytree(FIXTURES_DIR / "empty_room", run)
    bad = projects / "itest" / "runs" / "bad"
    bad.mkdir(parents=True)
    (bad / "world.sdf").write_text(BAD_WORLD)
    (bad / "map.yaml").write_text("image: map.pgm\n")

    plan = DisplayPlan(mode=mode, gpu="software", foxglove=(mode == "headless"))  # type: ignore[arg-type]
    opts = RunOptions(image=IMAGE, plan=plan, projects_dir=projects, name=NAME, src_dir=REPO,
                      host_uid=os.getuid(), host_gid=os.getgid(), api_port=_free_port(),
                      foxglove_port=_free_port(), novnc_port=_free_port(), extra_env={"L2S_CAMERA": "0"})
    session = SimSession(plan, opts=opts, runner=runner)

    async def go():
        await session.start(pull=False)
        try:
            info = await session.api.info()
            assert info["display_mode"] == mode and info["gz_version"] == "harmonic"
            assert (await session.api.health())["ros"] is True

            c_bad = host_to_container_path(bad, projects)
            with pytest.raises(ControlAPIError) as e:
                await session.api.sim_start(SimStartRequest(
                    run_dir=c_bad, world_sdf_path=f"{c_bad}/world.sdf", map_yaml_path=f"{c_bad}/map.yaml",
                    initial_pose=Pose2D(x=0, y=0)))
            assert e.value.status_code == 422 and "pose" in str(e.value)

            st = await session.open_run(run, gui=(mode == "vnc"), rviz=(mode == "vnc"), timeout=240)
            assert st.ready
            if mode == "vnc":
                cp = runner.exec(["bash", "-c", "pgrep -f 'gz sim -g' && pgrep -x rviz2"], name=NAME)
                assert cp.returncode == 0, "Gazebo GUI and RViz should be running in vnc mode"
            assert (await session.api.sim_stop()).status == "stopped"
        finally:
            await session.stop()

    try:
        asyncio.run(go())
    except BaseException:
        print(runner.logs(NAME, tail=80))
        raise

    log = run / "logs" / "sim.log"
    assert log.exists(), "sim.log should land in the host's project dir via the bind mount"
    assert log.stat().st_uid == os.getuid(), "files written in the container should be owned by the host user"
