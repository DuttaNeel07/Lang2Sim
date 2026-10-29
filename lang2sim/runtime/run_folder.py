"""
Run folder: the one input Part 3 accepts (decision 1).
File: lang2sim/runtime/run_folder.py

    ~/.lang2sim/projects/<project>/runs/<timestamp>/
        world.sdf        Gazebo Harmonic world (Part 1)
        map.yaml/.pgm    Nav2 map, map frame == world frame (Part 1)
        spec.json        the SimSpec (Part 2 / Part 1)
        manifest.json    RunManifest: file names, world name, robot, initial pose
        logs/            written by Part 3 (sim.log, heal_log.jsonl)

Host side only: load the manifest (or infer it from spec.json), check the files
exist, and turn it into the SimStartRequest the container API expects.
"""

from __future__ import annotations

import importlib
import json
from pathlib import Path
from typing import Optional

from lang2sim.core.models import Pose2D, RunManifest, SimSpec, SimStartRequest
from lang2sim.runtime.docker_runner import host_to_container_path

cfg = importlib.import_module("lang2sim.core.config")

MANIFEST = "manifest.json"


class RunFolderError(ValueError):
    """The folder does not satisfy the run-folder contract."""


def load_manifest(run_dir: Path) -> RunManifest:
    """manifest.json if present, else inferred from spec.json (initial pose from its mission)."""
    run_dir = Path(run_dir)
    if not run_dir.is_dir():
        raise RunFolderError(f"run folder not found: {run_dir}")
    mpath = run_dir / MANIFEST
    if mpath.is_file():
        try:
            manifest = RunManifest.model_validate_json(mpath.read_text())
        except ValueError as e:
            raise RunFolderError(f"{mpath} is invalid: {e}") from e
    else:
        spec_path = run_dir / "spec.json"
        if not spec_path.is_file():
            raise RunFolderError(f"{run_dir} has neither {MANIFEST} nor spec.json")
        spec = SimSpec.model_validate_json(spec_path.read_text())
        manifest = RunManifest(initial_pose=spec.mission.initial_pose)

    missing = [n for n in (manifest.world_sdf, manifest.map_yaml) if not (run_dir / n).is_file()]
    if missing:
        raise RunFolderError(f"{run_dir} is missing {', '.join(missing)}")
    return manifest


def write_manifest(run_dir: Path, manifest: RunManifest) -> Path:
    path = Path(run_dir) / MANIFEST
    path.write_text(json.dumps(manifest.model_dump(mode="json"), indent=2) + "\n")
    return path


def to_start_request(
    run_dir: Path,
    manifest: Optional[RunManifest] = None,
    *,
    projects_dir: Optional[Path] = None,
    gui: bool = True,
    rviz: bool = True,
    initial_pose: Optional[Pose2D] = None,
) -> SimStartRequest:
    """Container-side SimStartRequest for a host run folder under projects_dir."""
    run_dir = Path(run_dir)
    manifest = manifest or load_manifest(run_dir)
    projects = Path(projects_dir) if projects_dir else cfg.get_projects_dir()
    try:
        c_run = host_to_container_path(run_dir, projects)
    except ValueError as e:
        raise RunFolderError(f"{run_dir} must be inside the projects folder {projects}") from e
    return SimStartRequest(
        run_dir=c_run,
        world_sdf_path=f"{c_run}/{manifest.world_sdf}",
        map_yaml_path=f"{c_run}/{manifest.map_yaml}",
        robot_model=manifest.robot_model,
        initial_pose=initial_pose or manifest.initial_pose,
        use_gui=gui,
        use_rviz=rviz,
        world_name=manifest.world_name,
        nav2_params_path=f"{c_run}/{manifest.nav2_params}" if manifest.nav2_params else None,
    )
