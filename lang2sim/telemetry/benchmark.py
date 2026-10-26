"""
Benchmark scoring for recorded simulation runs (Stage 5: benchmark_report.json).
File: lang2sim/telemetry/benchmark.py

Turns a trajectory recorded inside the sim container (control API
/v1/telemetry/record/stop, runtime/telemetry_recorder.py) into the scored run
dict of nqi_scorer.score_trajectory, using what the run folder knows:

  * goal + waypoints from spec.json: L_optimal is the A* length of the whole
    mission (start -> waypoints -> goal) over the inflated map, so a waypoint
    patrol is not penalised against a straight start->goal line;
  * static obstacles from the run's map.yaml/map.pgm (zero-SLAM map);
  * dynamic obstacles = entities spawned during the recording (they are not in
    map.pgm), each a circle of its footprint radius;
  * C_force from the chassis bumper events counted in the container.

and writes benchmark_report.json into the run folder (architecture §4.2).
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple

from lang2sim.telemetry.nqi_scorer import (
    NQIParams,
    OccupancyGrid,
    build_report,
    optimal_length,
    score_trajectory,
    write_report,
)
from lang2sim.telemetry.trajectory import Trajectory

REPORT_NAME = "benchmark_report.json"

XY = Tuple[float, float]


def mission_targets(spec: Any) -> List[XY]:
    """Waypoints then goal, as (x, y), from a SimSpec (or its dict)."""
    m = spec.mission if hasattr(spec, "mission") else spec["mission"]
    get = (lambda p, k: getattr(p, k)) if hasattr(m, "goal_pose") else (lambda p, k: p[k])
    waypoints = m.waypoints if hasattr(m, "waypoints") else m.get("waypoints", [])
    goal = m.goal_pose if hasattr(m, "goal_pose") else m["goal_pose"]
    return [(float(get(p, "x")), float(get(p, "y"))) for p in [*waypoints, goal]]


def mission_optimal_length(start: Sequence[float], targets: Sequence[XY], occupancy: Optional[OccupancyGrid],
                           r_robot: float = NQIParams.r_robot) -> Tuple[float, str]:
    """Sum of shortest collision-free legs start -> t1 -> ... -> goal, and the weakest method used."""
    total, methods = 0.0, []
    prev = (float(start[0]), float(start[1]))
    for t in targets:
        length, method = optimal_length(prev, t, occupancy, r_robot)
        total += length
        methods.append(method)
        prev = t
    method = "astar" if methods and all(m == "astar" for m in methods) else (
        "euclidean" if occupancy is None else "mixed")
    return total, method


def load_occupancy(run_dir: Path, map_name: str = "map.yaml") -> Optional[OccupancyGrid]:
    path = Path(run_dir) / map_name
    if not path.is_file():
        return None
    try:
        return OccupancyGrid.from_yaml(path)
    except Exception:  # noqa: BLE001 - a broken map must not lose the run's score
        return None


def dynamic_obstacles(trajectory: Mapping[str, Any],
                      spawned: Optional[Sequence[Mapping[str, Any]]] = None) -> List[Tuple[float, float, float]]:
    entries = spawned if spawned is not None else (trajectory.get("meta") or {}).get("spawned_entities", [])
    return [(float(e["x"]), float(e["y"]), float(e.get("r", 0.0))) for e in entries or []]


def score_recorded_run(trajectory: Mapping[str, Any], *, run_dir: Optional[Path] = None, spec: Any = None,
                       spawned: Optional[Sequence[Mapping[str, Any]]] = None,
                       nav_status: Optional[str] = None, params: NQIParams = NQIParams()) -> Dict[str, Any]:
    """Score one recorded run. `trajectory` is Trajectory.to_dict() format (TelemetryRecordResult.trajectory)."""
    traj = Trajectory.from_dict(trajectory)
    occupancy = load_occupancy(run_dir) if run_dir is not None else None
    goal: Optional[XY] = traj.goal
    l_opt: Optional[float] = None
    l_method = None
    if spec is not None:
        targets = mission_targets(spec)
        goal = targets[-1]
        l_opt, l_method = mission_optimal_length(traj.start, targets, occupancy, params.r_robot)
    obstacles = dynamic_obstacles(trajectory, spawned)
    scored = score_trajectory(traj, goal=goal, obstacles=obstacles, occupancy=occupancy, l_optimal=l_opt,
                              params=params)
    if l_method is not None:
        scored["optimal_length_method"] = f"mission_{l_method}"
    meta = trajectory.get("meta") or {}
    final_err = (math.hypot(traj.end[0] - goal[0], traj.end[1] - goal[1]) if goal is not None else None)
    scored.update({
        "nav_status": nav_status,
        "final_goal_error_m": None if final_err is None else round(final_err, 4),
        "goal_tolerance_m": params.eps_pos,
        "pose_source": meta.get("pose_source"),
        "time_base": meta.get("time_base"),
        "contact_force_source": meta.get("contact_force_source"),
        "contact_events": meta.get("contact_events", []),
        "dynamic_obstacles": [{"x": x, "y": y, "r": r} for x, y, r in obstacles],
        "min_scan_range_m": meta.get("min_scan_range_m"),
    })
    # ATE columns here are AMCL-vs-ground-truth localization error, not SLAM drift.
    if scored.get("ate_mean_m") is not None:
        scored["localization_error_mean_m"] = scored["ate_mean_m"]
        scored["localization_error_max_m"] = scored["ate_max_m"]
    return scored


def write_benchmark(run_dir: Path, nominal: Mapping[str, Any], adversarial: Optional[Mapping[str, Any]] = None,
                    params: NQIParams = NQIParams(), extra: Optional[Mapping[str, Any]] = None) -> Tuple[Path, Dict[str, Any]]:
    """Build the report from scored runs and write <run_dir>/benchmark_report.json."""
    report = build_report(nominal, adversarial, params)
    if extra:
        report.update(extra)
    path = write_report(report, Path(run_dir) / REPORT_NAME)
    return path, report


def default_stress_trigger(nominal: Mapping[str, Any], *, model_name: str = "wooden_crate",
                           at_path_fraction: float = 0.6, fire_at_time_fraction: float = 0.35) -> Dict[str, Any]:
    """Archetype 4 default: drop a crate on the nominal path, ahead of the robot.

    The crate goes where the nominal run was at `at_path_fraction` of its path
    length and is spawned once `fire_at_time_fraction` of the nominal duration has
    elapsed, i.e. while the robot is still well short of it. Returned in
    core.models.DynamicObstacleTrigger form.
    """
    samples = nominal["samples"]
    pts = [(s["x"], s["y"]) for s in samples]
    seg = [math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:])]
    total = sum(seg)
    target, acc, spot = at_path_fraction * total, 0.0, pts[-1]
    for i, d in enumerate(seg):
        if acc + d >= target:
            spot = pts[i + 1]
            break
        acc += d
    duration = samples[-1]["t"] - samples[0]["t"] if len(samples) >= 2 else 0.0
    return {"trigger_type": "time", "trigger_value": round(max(1.0, fire_at_time_fraction * duration), 2),
            "model_name": model_name, "spawn_pose": {"x": round(spot[0], 3), "y": round(spot[1], 3), "yaw": 0.0}}
