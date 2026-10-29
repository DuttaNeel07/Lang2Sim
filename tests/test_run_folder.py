"""
Run-folder contract (decision 1): manifest loading, inference from spec.json,
host -> container start requests.
File: tests/test_run_folder.py
"""

import json

import pytest

from lang2sim.core.models import (
    EnvironmentSpec,
    MissionSpec,
    Pose2D,
    RunManifest,
    SimSpec,
    Vector3D,
)
from lang2sim.runtime.run_folder import (
    RunFolderError,
    load_manifest,
    to_start_request,
    write_manifest,
)


def _run(tmp_path, files=("world.sdf", "map.yaml", "map.pgm")):
    run = tmp_path / "projects" / "demo" / "runs" / "001"
    run.mkdir(parents=True)
    for f in files:
        (run / f).write_text("x")
    return run


def test_manifest_round_trip(tmp_path):
    run = _run(tmp_path)
    m = RunManifest(initial_pose=Pose2D(x=1.0, y=-2.0, yaw=0.5), world_name="warehouse")
    write_manifest(run, m)
    back = load_manifest(run)
    assert back == m and back.world_sdf == "world.sdf"


def test_manifest_inferred_from_spec(tmp_path):
    run = _run(tmp_path)
    spec = SimSpec(
        environment=EnvironmentSpec(room_dimensions=Vector3D(x=10, y=8, z=3)),
        mission=MissionSpec(initial_pose=Pose2D(x=-3, y=-2), goal_pose=Pose2D(x=3, y=2)),
    )
    (run / "spec.json").write_text(spec.model_dump_json())
    assert load_manifest(run).initial_pose == Pose2D(x=-3, y=-2)


def test_missing_files_and_bad_manifest(tmp_path):
    run = _run(tmp_path, files=("world.sdf",))
    write_manifest(run, RunManifest(initial_pose=Pose2D(x=0, y=0)))
    with pytest.raises(RunFolderError, match="map.yaml"):
        load_manifest(run)
    (run / "manifest.json").write_text(
        json.dumps({"initial_pose": {"x": 0, "y": 0}, "robot_model": "ridgeback"})
    )
    with pytest.raises(RunFolderError, match="invalid"):
        load_manifest(run)
    with pytest.raises(RunFolderError, match="neither"):
        load_manifest(_run(tmp_path / "empty", files=()))


def test_start_request_maps_paths_into_container(tmp_path):
    run = _run(tmp_path)
    write_manifest(run, RunManifest(initial_pose=Pose2D(x=1, y=2), nav2_params="nav2.yaml"))
    req = to_start_request(run, projects_dir=tmp_path / "projects", gui=False, rviz=False)
    assert req.run_dir == "/lang2sim/projects/demo/runs/001"
    assert req.world_sdf_path == "/lang2sim/projects/demo/runs/001/world.sdf"
    assert req.map_yaml_path == "/lang2sim/projects/demo/runs/001/map.yaml"
    assert req.nav2_params_path == "/lang2sim/projects/demo/runs/001/nav2.yaml"
    assert (req.use_gui, req.use_rviz) == (False, False)
    with pytest.raises(RunFolderError, match="projects folder"):
        to_start_request(run, projects_dir=tmp_path / "elsewhere")
