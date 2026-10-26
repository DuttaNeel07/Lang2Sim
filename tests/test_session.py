"""
SimSession helpers (no Docker): fixture copy, run options, view hints.
File: tests/test_session.py
"""

import importlib

import pytest

from sim2lang.runtime.display import DisplayPlan
from sim2lang.runtime.run_folder import load_manifest
from sim2lang.runtime.session import SimSession, copy_fixture

cfg = importlib.import_module("sim2lang.core.config")


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


@pytest.mark.parametrize("mode,hint", [("vnc", "localhost:6080"), ("headless", "ws://localhost:8765"),
                                       ("x11", "windows")])
def test_view_hint_and_image(projects, mode, hint):
    plan = DisplayPlan(mode, "software", mode == "headless")
    s = SimSession(plan, image="lang2sim-sim:dev", runner=NoDocker(), extra_env={"L2S_CAMERA": "0"})
    assert hint in s.view_hint
    assert s.opts.image == ("lang2sim-sim:dev-vnc" if mode == "vnc" else "lang2sim-sim:dev")
    assert s.opts.extra_env["L2S_CAMERA"] == "0"
    assert s.opts.projects_dir == projects
