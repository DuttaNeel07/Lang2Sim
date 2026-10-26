"""
Acceptance test A7: kill the TF publisher, then the bridge, mid-run; the
self-healer must bring both back and log it.
File: tests/integration/test_self_healing_live.py

    L2S_INTEGRATION=1 L2S_TEST_IMAGE=lang2sim-sim:dev pytest -q tests/integration -k self_healing
"""

from __future__ import annotations

import asyncio
import json
import os
import shutil
import socket
import time
from pathlib import Path

import pytest

from sim2lang.runtime.display import DisplayPlan
from sim2lang.runtime.docker_runner import DockerRunner, RunOptions
from sim2lang.runtime.platform import detect_docker
from sim2lang.runtime.session import FIXTURES_DIR, SimSession

pytestmark = pytest.mark.skipif(os.getenv("L2S_INTEGRATION") != "1",
                                reason="set L2S_INTEGRATION=1 to run Docker integration tests")

IMAGE = os.getenv("L2S_TEST_IMAGE", "lang2sim-sim:dev")
REPO = Path(__file__).resolve().parents[2]
NAME = "l2s-itest-heal"
FAULTS = [
    # (what we kill,                         healer target,           readiness that must come back)
    ("lib/robot_state_publisher/robot_state_publisher", "robot_state_publisher", "localized"),
    ("lib/ros_gz_bridge/parameter_bridge", "bridge", "laser_scan"),
]


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


def test_killed_nodes_are_restarted_and_logged(runner, tmp_path):
    projects = tmp_path / "projects"
    run = projects / "heal" / "runs" / "001"
    shutil.copytree(FIXTURES_DIR / "empty_room", run)
    plan = DisplayPlan("headless", "software", False)
    opts = RunOptions(image=IMAGE, plan=plan, projects_dir=projects, src_dir=REPO, name=NAME,
                      host_uid=os.getuid(), host_gid=os.getgid(), api_port=_free_port(),
                      extra_env={"L2S_CAMERA": "0"})
    session = SimSession(plan, opts=opts, runner=runner)

    async def wait_for(pred, timeout=120.0):
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            v = await pred()
            if v:
                return v
            await asyncio.sleep(1)
        raise AssertionError("timed out")

    async def go():
        await session.start(pull=False)
        try:
            await session.open_run(run, gui=False, rviz=False, timeout=240)
            for pattern, target, check in FAULTS:
                cp = runner.exec(["pkill", "-9", "-f", pattern], name=NAME, user="root")
                assert cp.returncode == 0, f"nothing to kill for {pattern}"

                async def healed():
                    log = await session.api._request("GET", "/heal/log")
                    return [e for e in log["events"] if e["action_taken"] == f"restart_node {target}"
                            and e["success"]]
                events = await wait_for(healed, timeout=60)
                assert events, f"no restart of {target} logged"

                async def back():
                    st = await session.api.sim_status()
                    return st.readiness.get(check) and st.ready
                await wait_for(back, timeout=120)

                async def running():
                    return runner.exec(["pgrep", "-f", pattern], name=NAME).returncode == 0
                await wait_for(running, timeout=30)

            # and it stays up: no restart flapping from lingering warnings
            await asyncio.sleep(40)
            log = await session.api._request("GET", "/heal/log")
            restarts = [e for e in log["events"] if e["action_taken"].startswith("restart_node")]
            assert len(restarts) == len(FAULTS), f"expected one restart per fault, got {restarts}"
        except BaseException:
            print(runner.logs(NAME, tail=80))      # before stop() removes the container
            raise
        finally:
            await session.stop()

    asyncio.run(go())
    lines = [json.loads(x) for x in (run / "logs" / "heal_log.jsonl").read_text().splitlines()]
    targets = {e["action_taken"] for e in lines if e["success"]}
    assert {"restart_node robot_state_publisher", "restart_node bridge"} <= targets
