"""
Every fixture run is a valid run folder and passes the world pre-flight check.
File: tests/test_fixtures.py
"""

from pathlib import Path

import pytest

from sim2lang.core.models import SimSpec
from sim2lang.runtime.run_folder import load_manifest
from sim2lang.runtime.world_check import check_world

FIXTURES = Path(__file__).resolve().parents[1] / "catalog" / "fixtures"
NAMES = ["empty_room", "small_warehouse", "tb3_world"]


@pytest.mark.parametrize("name", NAMES)
def test_fixture_is_a_valid_run(name):
    run = FIXTURES / name
    manifest = load_manifest(run)
    spec = SimSpec.model_validate_json((run / "spec.json").read_text())
    assert manifest.initial_pose == spec.mission.initial_pose
    report = check_world((run / manifest.world_sdf).read_text())
    assert report.ok, report.summary()
    assert report.world_name == manifest.world_name
    assert spec.mission.waypoints, "fixtures carry a waypoint mission for the nav tests"
    assert (run / "map.pgm").read_bytes()[:2] == b"P5"
