"""
Acceptance test A5: every fixture run loads and reaches ready in the real container.
File: tests/integration/test_run_folders.py

    L2S_INTEGRATION=1 L2S_TEST_IMAGE=lang2sim-sim:dev pytest -q tests/integration -k run_folders

Uses this checkout's code inside the container (dev mount), headless, no camera,
so it measures the runtime rather than rendering speed.
"""

from __future__ import annotations

import asyncio
import os
import socket
from pathlib import Path

import pytest

from lang2sim.runtime.display import DisplayPlan
from lang2sim.runtime.docker_runner import DockerRunner, RunOptions
from lang2sim.runtime.platform import detect_docker
from lang2sim.runtime.session import FIXTURES_DIR, SimSession

pytestmark = pytest.mark.skipif(
    os.getenv("L2S_INTEGRATION") != "1",
    reason="set L2S_INTEGRATION=1 to run Docker integration tests",
)

IMAGE = os.getenv("L2S_TEST_IMAGE", "lang2sim-sim:dev")
REPO = Path(__file__).resolve().parents[2]
READY_TIMEOUT_S = float(os.getenv("L2S_READY_TIMEOUT", "240"))


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
    return r


@pytest.mark.parametrize("fixture", sorted(p.name for p in FIXTURES_DIR.iterdir() if p.is_dir()))
def test_fixture_reaches_ready(runner, tmp_path, fixture):
    import shutil

    projects = tmp_path / "projects"
    run = projects / fixture / "runs" / "001"
    shutil.copytree(FIXTURES_DIR / fixture, run)
    plan = DisplayPlan("headless", "software", False)
    opts = RunOptions(
        image=IMAGE,
        plan=plan,
        projects_dir=projects,
        src_dir=REPO,
        host_uid=os.getuid(),
        host_gid=os.getgid(),
        name=f"l2s-itest-{fixture}",
        api_port=_free_port(),
        extra_env={"L2S_CAMERA": "0"},
    )
    session = SimSession(plan, opts=opts, runner=runner)

    async def go():
        await session.start(pull=False)
        try:
            st = await session.open_run(run, gui=False, rviz=False, timeout=READY_TIMEOUT_S)
            assert st.ready and all(st.readiness.values())
            assert st.sim_time > 0
        finally:
            await session.stop()

    try:
        asyncio.run(go())
    except BaseException:
        print(runner.logs(opts.name, tail=60))
        raise
    logs = run / "logs"
    assert (logs / "sim.log").stat().st_uid == os.getuid()
    assert (run / "sim_launch.py").is_file() and (logs / "world_check.txt").is_file()
