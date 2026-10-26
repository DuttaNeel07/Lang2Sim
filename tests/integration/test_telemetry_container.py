"""
Telemetry engine against the real sim container: instrumented robot, recorded + scored mission,
chassis bumper.
File: tests/integration/test_telemetry_container.py

    L2S_INTEGRATION=1 L2S_TEST_IMAGE=lang2sim-sim:dev pytest -q tests/integration -k telemetry

Dev-mounts this checkout, headless, no camera (software rendering is fine; scores
are in sim time so a slow RTF does not change them much).
"""

from __future__ import annotations

import asyncio
import json
import os
import shutil
import socket
from pathlib import Path

import pytest

from lang2sim.runtime.display import DisplayPlan
from lang2sim.runtime.docker_runner import DockerRunner, RunOptions
from lang2sim.runtime.platform import detect_docker
from lang2sim.runtime.session import FIXTURES_DIR, SimSession

pytestmark = pytest.mark.skipif(os.getenv("L2S_INTEGRATION") != "1",
                                reason="set L2S_INTEGRATION=1 to run Docker integration tests")

IMAGE = os.getenv("L2S_TEST_IMAGE", "lang2sim-sim:dev")
REPO = Path(__file__).resolve().parents[2]
READY_TIMEOUT_S = float(os.getenv("L2S_READY_TIMEOUT", "300"))


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


def _session(runner, tmp_path, fixture, name):
    projects = tmp_path / "projects"
    run = projects / fixture / "runs" / "001"
    shutil.copytree(FIXTURES_DIR / fixture, run)
    if os.getuid() == 0:  # rootful host: the container keeps its user at 1000, so open the folder up
        for p in [tmp_path, *tmp_path.rglob("*")]:
            p.chmod(0o777)
    plan = DisplayPlan("headless", "software", False)
    opts = RunOptions(image=IMAGE, plan=plan, projects_dir=projects, src_dir=REPO,
                      host_uid=os.getuid(), host_gid=os.getgid(), name=name,
                      api_port=_free_port(), extra_env={"L2S_CAMERA": "0"})
    return SimSession(plan, opts=opts, runner=runner), run, opts


def _run(runner, opts, coro):
    try:
        asyncio.run(coro)
    except BaseException:
        print(runner.logs(opts.name, tail=80))
        raise


def test_instrumented_robot_scored_mission(runner, tmp_path):
    session, run, opts = _session(runner, tmp_path, "empty_room", "l2s-itest-telemetry")
    out = {}

    async def go():
        await session.start(pull=False)
        try:
            await session.open_run(run, gui=False, rviz=False, timeout=READY_TIMEOUT_S)
            out["setup"] = await session.api.telemetry_setup()
            live = await session.api.telemetry_live()
            out["live0"] = live
            out["report"] = await session.score_single_run(run)
        finally:
            await session.stop()

    _run(runner, opts, go())
    assert out["setup"]["enabled"], out["setup"]
    live0 = out["live0"]
    assert live0.pose_source == "ground_truth"
    # Ground truth at the spawn pose, and AMCL agrees with it.
    assert live0.pose.x == pytest.approx(-2.5, abs=0.1) and live0.pose.y == pytest.approx(-1.5, abs=0.1)
    assert live0.localization_error_m is not None and live0.localization_error_m < 0.3
    nominal = json.loads((run / "benchmark_report.json").read_text())["runs"]["nominal"]
    assert nominal["pose_source"] == "ground_truth" and nominal["time_base"] == "sim"
    assert nominal["samples"] > 100 and nominal["C_force"] == 0
    assert nominal["nav_status"] == "succeeded"
    assert nominal["optimal_length_method"] == "mission_astar"
    assert 0.6 < nominal["E_p"] <= 1.05 and nominal["nqi"] > 50
    traj = json.loads((run / "logs" / "trajectory_nominal.json").read_text())
    assert traj["meta"]["contact_events"] == []


def test_bumper_counts_a_crate_dropped_on_the_robot(runner, tmp_path):
    session, run, opts = _session(runner, tmp_path, "empty_room", "l2s-itest-bumper")
    out = {}

    async def go():
        await session.start(pull=False)
        try:
            await session.open_run(run, gui=False, rviz=False, timeout=READY_TIMEOUT_S)
            await session.api.record_start("bump")
            live = await session.api.telemetry_live()
            await session.api.world_spawn({"model_name": "wooden_crate", "x": live.pose.x + 0.25,
                                           "y": live.pose.y, "yaw": 0.0})
            for _ in range(40):
                await asyncio.sleep(0.5)
                if (await session.api.telemetry_live()).collision_count:
                    break
            out["live"] = await session.api.telemetry_live()
            out["result"] = await session.api.record_stop()
        finally:
            await session.stop()

    _run(runner, opts, go())
    live = out["live"]
    assert live.collision_count >= 1, live
    assert "crate" in live.last_contact_entity
    assert out["result"].collisions >= 1
    assert out["result"].spawned_entities and out["result"].spawned_entities[0]["model"] == "wooden_crate"
