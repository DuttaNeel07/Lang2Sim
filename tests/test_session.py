"""
SimSession helpers (no Docker): fixture copy, run options, view hints.
File: tests/test_session.py
"""

import importlib

import pytest

from lang2sim.runtime.display import DisplayPlan
from lang2sim.runtime.run_folder import load_manifest
from lang2sim.runtime.session import SimSession, copy_fixture

cfg = importlib.import_module("lang2sim.core.config")


@pytest.fixture
def projects(tmp_path, monkeypatch):
    monkeypatch.setattr(cfg, "PROJECTS_DIR", tmp_path / "projects")
    monkeypatch.setattr(cfg, "DEFAULT_LANG2SIM_ROOT", tmp_path)
    return tmp_path / "projects"


def test_copy_fixture_makes_a_fresh_valid_run(projects):
    run = copy_fixture("small_warehouse")
    assert run.parent.parent == projects / "small_warehouse"
    assert load_manifest(run).world_name == "small_warehouse"
    with pytest.raises(FileNotFoundError, match="available: .*empty_room"):
        copy_fixture("nope")


class NoDocker:
    def ensure_image(self, image):
        raise AssertionError("not called in these tests")


@pytest.mark.parametrize(
    "mode,hint",
    [("vnc", "localhost:6080"), ("headless", "ws://localhost:8765"), ("x11", "windows")],
)
def test_view_hint_and_image(projects, mode, hint):
    plan = DisplayPlan(mode, "software", mode == "headless")
    s = SimSession(plan, image="lang2sim-sim:dev", runner=NoDocker(), extra_env={"L2S_CAMERA": "0"})
    assert hint in s.view_hint
    assert s.opts.image == ("lang2sim-sim:dev-vnc" if mode == "vnc" else "lang2sim-sim:dev")
    assert s.opts.extra_env["L2S_CAMERA"] == "0"
    assert s.opts.projects_dir == projects


def test_run_mission_sends_waypoints_then_goal(projects):
    import asyncio

    from lang2sim.core.models import NavStatus

    run = copy_fixture("small_warehouse")

    class FakeAPI:
        def __init__(self):
            self.sent, self.polls = None, 0

        async def nav_waypoints(self, poses, timeout_seconds):
            self.sent = (poses, timeout_seconds)
            return NavStatus(is_active=True, status="navigating", waypoints_total=len(poses))

        async def nav_status(self):
            self.polls += 1
            if self.polls < 3:
                return NavStatus(
                    is_active=True,
                    status="navigating",
                    waypoint_index=self.polls,
                    waypoints_total=4,
                    distance_remaining=3.0 / self.polls,
                )
            return NavStatus(
                is_active=False, status="succeeded", waypoint_index=3, waypoints_total=4
            )

    s = SimSession(DisplayPlan("headless", "software", True), runner=NoDocker())
    s.api = FakeAPI()
    seen = []
    st = asyncio.run(s.run_mission(run, on_progress=seen.append, poll_s=0))
    poses, timeout = s.api.sent
    assert st.status == "succeeded" and len(poses) == 4 and timeout == 300
    assert (poses[-1].x, poses[-1].y) == (-4.0, 3.5) and len(seen) == 3
