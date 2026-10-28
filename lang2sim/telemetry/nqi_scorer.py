"""Composite Navigation Quality Index (NQI) evaluator.

Implements the scoring model of the Lang2Sim architecture (Section 7 / Issue #4 §3.5):

1. Path traversal efficiency   ``E_p = L_optimal / L_actual``                (clipped to [0, 1])
2. Trajectory smoothness       ``S_tau = ∫ (‖ȧ(t)‖² + ω̇(t)²) dt``           (jerk + angular accel.)
3. Minimum obstacle clearance  ``C_min = min_t min_o ‖p(t) − p_o‖ − r_o − r_robot``
                               (or, with ``NQIParams.footprint``, the distance from the robot's
                               chassis rectangle at pose (x, y, yaw) to each obstacle boundary)
4. Mission success             ``S_mission = 1[‖p(T) − p_goal‖ ≤ ε] · 1[C_force == 0]``
5. Composite score             ``NQI = 100·[w1·E_p + w2/(1+α·S_tau) + w3·min(1, C_min/C_safe)] − P·C_force``

Outputs a ``benchmark_report.json`` and a Rich comparison table (nominal vs adversarial).

Run ``python -m lang2sim.telemetry.nqi_scorer --demo`` for a synthetic demonstration.
"""

from __future__ import annotations

import argparse
import heapq
import json
import math
import os
import sys
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable, Mapping, Optional, Sequence

import numpy as np

from lang2sim.telemetry.trajectory import Trajectory

__all__ = [
    "NQIParams",
    "OccupancyGrid",
    "path_length",
    "optimal_length",
    "path_efficiency",
    "trajectory_smoothness",
    "min_clearance",
    "mission_success",
    "compute_nqi",
    "score_trajectory",
    "build_report",
    "write_report",
    "default_report_path",
    "render_comparison_table",
    "render_to_console",
    "demo_trajectories",
]

REPORT_SCHEMA_VERSION = "1.0"

# np.trapz was renamed to np.trapezoid in NumPy 2.0.
_trapezoid = getattr(np, "trapezoid", None) or getattr(np, "trapz")


# --------------------------------------------------------------------------- params
@dataclass(frozen=True)
class NQIParams:
    """Tunable constants of the NQI model (defaults from Issue #4)."""

    w1: float = 0.35  # path efficiency weight
    w2: float = 0.25  # smoothness weight
    w3: float = 0.40  # clearance weight
    alpha: float = 0.05  # smoothness scaling
    c_safe: float = 0.5  # m, clearance at which the clearance term saturates
    p_collision: float = 50.0  # penalty per force-qualified collision
    r_robot: float = 0.22  # m, TurtleBot3 Waffle circumscribed radius (approx.)
    eps_pos: float = 0.15  # m, goal position tolerance
    # Chassis rectangle (x_min, x_max, y_min, y_max) in the robot base frame, metres. When set,
    # C_min is measured from this oriented rectangle ("robot chassis boundary", architecture §7.3)
    # instead of a circle of r_robot, which over-reports near-misses along the robot's sides.
    footprint: Optional[tuple] = None
    resample_dt: float = 0.05  # s, uniform grid used for differentiation
    smooth_window: int = 5  # samples, moving-average window before differentiating


# --------------------------------------------------------------------------- occupancy grid
@dataclass
class OccupancyGrid:
    """Minimal Nav2-style occupancy grid (``map.yaml`` + ``map.pgm``)."""

    occupied: np.ndarray  # bool (H, W); row 0 is the TOP of the image
    resolution: float  # m / cell
    origin: tuple[float, float]  # world coords of the bottom-left cell corner

    @classmethod
    def from_yaml(cls, yaml_path: str | Path) -> "OccupancyGrid":
        """Load a Nav2 map. Uses a tiny ``key: value`` parser to avoid a PyYAML dependency."""
        from PIL import Image  # local import: Pillow only needed for maps

        yaml_path = Path(yaml_path)
        meta: dict[str, str] = {}
        for raw in yaml_path.read_text(encoding="utf-8").splitlines():
            line = raw.split("#", 1)[0].strip()
            if ":" in line:
                key, val = line.split(":", 1)
                meta[key.strip()] = val.strip().strip("'\"")
        image_path = Path(meta["image"])
        if not image_path.is_absolute():
            image_path = yaml_path.parent / image_path
        resolution = float(meta["resolution"])
        origin_vals = [float(v) for v in meta.get("origin", "[0,0,0]").strip("[]").split(",")]
        negate = int(float(meta.get("negate", "0")))
        occ_thresh = float(meta.get("occupied_thresh", "0.65"))

        img = np.asarray(Image.open(image_path).convert("L"), dtype=float)
        p = img / 255.0 if negate else (255.0 - img) / 255.0
        return cls(
            occupied=p > occ_thresh, resolution=resolution, origin=(origin_vals[0], origin_vals[1])
        )

    # world <-> grid (row, col)
    def world_to_cell(self, x: float, y: float) -> tuple[int, int]:
        h = self.occupied.shape[0]
        col = int(math.floor((x - self.origin[0]) / self.resolution))
        row_from_bottom = int(math.floor((y - self.origin[1]) / self.resolution))
        return h - 1 - row_from_bottom, col

    def cell_to_world(self, row: int, col: int) -> tuple[float, float]:
        h = self.occupied.shape[0]
        x = self.origin[0] + (col + 0.5) * self.resolution
        y = self.origin[1] + (h - 1 - row + 0.5) * self.resolution
        return x, y

    def in_bounds(self, row: int, col: int) -> bool:
        h, w = self.occupied.shape
        return 0 <= row < h and 0 <= col < w

    def inflated(self, radius_m: float) -> np.ndarray:
        """Return the occupancy mask dilated by a disk of ``radius_m``."""
        r = int(math.ceil(radius_m / self.resolution))
        if r <= 0:
            return self.occupied.copy()
        src = self.occupied
        out = src.copy()
        h, w = src.shape
        for dr in range(-r, r + 1):
            for dc in range(-r, r + 1):
                if dr * dr + dc * dc > r * r or (dr == 0 and dc == 0):
                    continue
                r0, r1 = max(0, dr), min(h, h + dr)
                c0, c1 = max(0, dc), min(w, w + dc)
                out[r0:r1, c0:c1] |= src[r0 - dr : r1 - dr, c0 - dc : c1 - dc]
        return out


# --------------------------------------------------------------------------- metrics
def path_length(traj: Trajectory) -> float:
    """Integrated travelled distance Σ‖Δp‖ (meters)."""
    d = np.diff(traj.positions, axis=0)
    return float(np.sum(np.hypot(d[:, 0], d[:, 1])))


def _astar_length(
    grid: np.ndarray, start: tuple[int, int], goal: tuple[int, int], res: float
) -> Optional[float]:
    """8-connected A* on a boolean blocked-grid; returns path length in meters or None."""
    h, w = grid.shape
    if not (0 <= start[0] < h and 0 <= start[1] < w and 0 <= goal[0] < h and 0 <= goal[1] < w):
        return None
    if grid[start] or grid[goal]:
        return None
    sq2 = math.sqrt(2.0)
    moves = [
        (-1, 0, 1.0),
        (1, 0, 1.0),
        (0, -1, 1.0),
        (0, 1, 1.0),
        (-1, -1, sq2),
        (-1, 1, sq2),
        (1, -1, sq2),
        (1, 1, sq2),
    ]

    def heur(r: int, c: int) -> float:  # octile distance (admissible)
        dr, dc = abs(r - goal[0]), abs(c - goal[1])
        return (dr + dc) + (sq2 - 2.0) * min(dr, dc)

    g = np.full(grid.shape, np.inf)
    g[start] = 0.0
    closed = np.zeros(grid.shape, dtype=bool)
    heap: list[tuple[float, float, int, int]] = [(heur(*start), 0.0, start[0], start[1])]
    while heap:
        _, cost, r, c = heapq.heappop(heap)
        if closed[r, c]:
            continue
        if (r, c) == goal:
            return cost * res
        closed[r, c] = True
        for dr, dc, step in moves:
            nr, nc = r + dr, c + dc
            if not (0 <= nr < h and 0 <= nc < w) or grid[nr, nc] or closed[nr, nc]:
                continue
            if dr and dc and (grid[r + dr, c] or grid[r, c + dc]):
                continue  # no corner cutting
            nc_cost = cost + step
            if nc_cost < g[nr, nc]:
                g[nr, nc] = nc_cost
                heapq.heappush(heap, (nc_cost + heur(nr, nc), nc_cost, nr, nc))
    return None


def optimal_length(
    start: Sequence[float],
    goal: Sequence[float],
    occupancy: Optional[OccupancyGrid] = None,
    r_robot: float = NQIParams.r_robot,
) -> tuple[float, str]:
    """Shortest collision-free path length and the method used.

    Uses A* over the occupancy grid inflated by ``r_robot`` when a grid is given,
    otherwise (or if A* fails) the straight-line Euclidean distance.
    """
    euclid = float(math.hypot(goal[0] - start[0], goal[1] - start[1]))
    if occupancy is None:
        return euclid, "euclidean"
    blocked = occupancy.inflated(r_robot)
    s = occupancy.world_to_cell(start[0], start[1])
    gl = occupancy.world_to_cell(goal[0], goal[1])
    length = _astar_length(blocked, s, gl, occupancy.resolution)
    if length is None:
        return euclid, "euclidean_fallback"
    return max(length, euclid), "astar"


def path_efficiency(l_optimal: float, l_actual: float) -> float:
    """``E_p = L_optimal / L_actual`` clipped to [0, 1]."""
    if l_actual < 1e-6:
        return 0.0
    return float(np.clip(l_optimal / l_actual, 0.0, 1.0))


def _moving_average(a: np.ndarray, window: int) -> np.ndarray:
    if window <= 1 or a.size < window:
        return a
    pad = window // 2
    padded = np.pad(a, (pad, window - 1 - pad), mode="edge")
    return np.convolve(padded, np.ones(window) / window, mode="valid")


def trajectory_smoothness(traj: Trajectory, params: NQIParams = NQIParams()) -> float:
    """``S_tau = ∫ (‖ȧ‖² + ω̇²) dt`` on a uniformly resampled, lightly smoothed trajectory."""
    tr = traj.resample(params.resample_dt)
    if tr.n < 3:
        return 0.0
    t = tr.t
    vx = _moving_average(tr.v * np.cos(tr.yaw), params.smooth_window)  # type: ignore[operator]
    vy = _moving_average(tr.v * np.sin(tr.yaw), params.smooth_window)  # type: ignore[operator]
    w = _moving_average(tr.w, params.smooth_window)  # type: ignore[arg-type]
    ax, ay = np.gradient(vx, t), np.gradient(vy, t)
    jx, jy = np.gradient(ax, t), np.gradient(ay, t)
    wdot = np.gradient(w, t)
    integrand = jx**2 + jy**2 + wdot**2
    return float(_trapezoid(integrand, t))


def _normalize_obstacles(obstacles: Optional[Iterable[Any]]) -> np.ndarray:
    """Accept ``[(x, y, r)]``, ``[(x, y)]`` or ``[{"x":..,"y":..,"r":..}]`` -> (M, 3) array."""
    rows: list[tuple[float, float, float]] = []
    for ob in obstacles or []:
        if isinstance(ob, Mapping):
            rows.append((float(ob["x"]), float(ob["y"]), float(ob.get("r", ob.get("radius", 0.0)))))
        else:
            vals = list(ob)
            rows.append((float(vals[0]), float(vals[1]), float(vals[2]) if len(vals) > 2 else 0.0))
    return np.asarray(rows, dtype=float).reshape(-1, 3)


def _grid_clearance(points: np.ndarray, grid: OccupancyGrid, search_m: float = 3.0) -> float:
    occ_rows, occ_cols = np.nonzero(grid.occupied)
    if occ_rows.size == 0:
        return math.inf
    h = grid.occupied.shape[0]
    occ_x = grid.origin[0] + (occ_cols + 0.5) * grid.resolution
    occ_y = grid.origin[1] + (h - 1 - occ_rows + 0.5) * grid.resolution
    best = math.inf
    for px, py in points:
        m = (np.abs(occ_x - px) <= search_m) & (np.abs(occ_y - py) <= search_m)
        if m.any():
            d = float(np.min(np.hypot(occ_x[m] - px, occ_y[m] - py))) - grid.resolution / 2
            best = min(best, d)
    return best


def _grid_points(grid: OccupancyGrid) -> np.ndarray:
    """Occupied cell centres as (N, 3) obstacles of radius half a cell."""
    occ_rows, occ_cols = np.nonzero(grid.occupied)
    h = grid.occupied.shape[0]
    xs = grid.origin[0] + (occ_cols + 0.5) * grid.resolution
    ys = grid.origin[1] + (h - 1 - occ_rows + 0.5) * grid.resolution
    return np.column_stack([xs, ys, np.full(xs.shape, grid.resolution / 2)])


def clearance_series(
    traj: Trajectory,
    footprint: Sequence[float],
    obstacles: Optional[Iterable[Any]] = None,
    occupancy: Optional[OccupancyGrid] = None,
    search_m: float = 2.0,
) -> np.ndarray:
    """Per-sample distance between the robot's chassis rectangle and the nearest obstacle boundary.

    ``footprint`` is (x_min, x_max, y_min, y_max) in the robot frame; each obstacle point is
    rotated into the robot frame at every sample, so heading matters (a robot sliding past
    a crate side-on is closer than one approaching it head-on). Overlap gives 0 minus the
    obstacle radius, i.e. a negative value; ``inf`` where nothing is within ``search_m``.
    """
    x0, x1, y0, y1 = (float(v) for v in footprint)
    cx, cy, hx, hy = (x0 + x1) / 2, (y0 + y1) / 2, (x1 - x0) / 2, (y1 - y0) / 2
    obs = _normalize_obstacles(obstacles)
    if occupancy is not None:
        grid_pts = _grid_points(occupancy)
        obs = np.vstack([obs, grid_pts]) if obs.size else grid_pts
    out = np.full(traj.n, np.inf)
    if obs.size == 0:
        return out
    yaw = traj.yaw if traj.yaw is not None else np.zeros(traj.n)
    for i, (px, py, th) in enumerate(zip(traj.x, traj.y, yaw)):
        near = (np.abs(obs[:, 0] - px) <= search_m) & (np.abs(obs[:, 1] - py) <= search_m)
        if not near.any():
            continue
        o = obs[near]
        c, s = math.cos(th), math.sin(th)
        dx, dy = o[:, 0] - px, o[:, 1] - py
        lx, ly = c * dx + s * dy, -s * dx + c * dy  # obstacle in the robot frame
        ex = np.maximum(np.abs(lx - cx) - hx, 0.0)
        ey = np.maximum(np.abs(ly - cy) - hy, 0.0)
        out[i] = float(np.min(np.hypot(ex, ey) - o[:, 2]))
    return out


def footprint_clearance(
    traj: Trajectory,
    footprint: Sequence[float],
    obstacles: Optional[Iterable[Any]] = None,
    occupancy: Optional[OccupancyGrid] = None,
    search_m: float = 2.0,
) -> Optional[float]:
    """Minimum of clearance_series: chassis rectangle to nearest obstacle over the run.
    ``None`` when there is no obstacle information (or none within reach)."""
    series = clearance_series(traj, footprint, obstacles, occupancy, search_m)
    if series.size == 0 or not np.isfinite(series).any():
        return None
    return float(series[np.isfinite(series)].min())


def min_clearance(
    traj: Trajectory,
    obstacles: Optional[Iterable[Any]] = None,
    occupancy: Optional[OccupancyGrid] = None,
    r_robot: float = NQIParams.r_robot,
    footprint: Optional[Sequence[float]] = None,
) -> Optional[float]:
    """Minimum distance between the robot boundary and any obstacle boundary.

    The robot is a circle of ``r_robot`` unless ``footprint`` (chassis rectangle) is given.
    Returns ``None`` when no obstacle information is available.
    """
    if footprint is not None:
        return footprint_clearance(traj, footprint, obstacles, occupancy)
    pts = traj.positions
    best = math.inf
    obs = _normalize_obstacles(obstacles)
    if obs.size:
        d = (
            np.hypot(pts[:, None, 0] - obs[None, :, 0], pts[:, None, 1] - obs[None, :, 1])
            - obs[None, :, 2]
        )
        best = min(best, float(d.min()))
    if occupancy is not None:
        best = min(best, _grid_clearance(pts, occupancy))
    if math.isinf(best):
        return None
    return best - r_robot


def mission_success(
    final_pos: Sequence[float],
    goal: Optional[Sequence[float]],
    collisions: int,
    eps_pos: float = NQIParams.eps_pos,
) -> Optional[int]:
    """1 if the goal was reached within ``eps_pos`` with zero collisions; ``None`` if no goal."""
    if goal is None:
        return None
    reached = math.hypot(final_pos[0] - goal[0], final_pos[1] - goal[1]) <= eps_pos
    return int(reached and collisions == 0)


def compute_nqi(
    e_p: float,
    s_tau: float,
    c_min: Optional[float],
    c_force: int,
    params: NQIParams = NQIParams(),
) -> tuple[float, dict[str, float]]:
    """Return ``(nqi_raw, term_breakdown)``. ``c_min=None`` counts the clearance term as satisfied."""
    t_eff = params.w1 * e_p
    t_smooth = params.w2 / (1.0 + params.alpha * s_tau)
    clear_ratio = 1.0 if c_min is None else float(np.clip(c_min / params.c_safe, 0.0, 1.0))
    t_clear = params.w3 * clear_ratio
    penalty = params.p_collision * c_force
    raw = 100.0 * (t_eff + t_smooth + t_clear) - penalty
    terms = {
        "efficiency": 100.0 * t_eff,
        "smoothness": 100.0 * t_smooth,
        "clearance": 100.0 * t_clear,
        "collision_penalty": -penalty,
    }
    return raw, terms


def score_trajectory(
    traj: Trajectory,
    *,
    goal: Optional[Sequence[float]] = None,
    obstacles: Optional[Iterable[Any]] = None,
    occupancy: Optional[OccupancyGrid] = None,
    l_optimal: Optional[float] = None,
    collisions: Optional[int] = None,
    params: NQIParams = NQIParams(),
) -> dict[str, Any]:
    """Compute every metric for one run and return a JSON-serializable dict."""
    goal = tuple(goal) if goal is not None else traj.goal
    c_force = int(traj.collisions if collisions is None else collisions)
    l_actual = path_length(traj)

    if l_optimal is None:
        l_optimal = traj.l_optimal
    if l_optimal is not None:
        l_opt, l_opt_method = float(l_optimal), "provided"
    else:
        target = goal if goal is not None else traj.end
        l_opt, l_opt_method = optimal_length(traj.start, target, occupancy, params.r_robot)

    e_p = path_efficiency(l_opt, l_actual)
    s_tau = trajectory_smoothness(traj, params)
    c_min = min_clearance(traj, obstacles, occupancy, params.r_robot, params.footprint)
    s_mission = mission_success(traj.end, goal, c_force, params.eps_pos)
    nqi_raw, terms = compute_nqi(e_p, s_tau, c_min, c_force, params)

    slam_metrics = None
    if traj.ate is not None:
        mean_ate = float(traj.ate_mean) if traj.ate_mean is not None else 0.0
        max_ate = float(traj.ate_max) if traj.ate_max is not None else 0.0
        rmse_ate = float(traj.ate_rmse) if traj.ate_rmse is not None else 0.0
        diverged = bool(max_ate > 0.35)
        slam_metrics = {
            "mean_ate": round(mean_ate, 4),
            "max_ate": round(max_ate, 4),
            "rmse_ate": round(rmse_ate, 4),
            "diverged": diverged,
        }

    return {
        "label": traj.label,
        "samples": traj.n,
        "duration_s": round(traj.duration, 4),
        "path_length_m": round(l_actual, 4),
        "optimal_length_m": round(l_opt, 4),
        "optimal_length_method": l_opt_method,
        "goal": list(goal) if goal is not None else None,
        "final_position": [round(v, 4) for v in traj.end],
        "E_p": round(e_p, 4),
        "S_tau": round(s_tau, 4),
        "C_min_m": None if c_min is None else round(c_min, 4),
        "clearance_source": _clearance_source(obstacles, occupancy),
        "C_force": c_force,
        "S_mission": s_mission,
        "terms": {k: round(v, 3) for k, v in terms.items()},
        "nqi_raw": round(nqi_raw, 3),
        "nqi": round(float(np.clip(nqi_raw, 0.0, 100.0)), 3),
        "ate_mean_m": None if slam_metrics is None else slam_metrics["mean_ate"],
        "ate_max_m": None if slam_metrics is None else slam_metrics["max_ate"],
        "ate_rmse_m": None if slam_metrics is None else slam_metrics["rmse_ate"],
        "slam_diverged": None if slam_metrics is None else slam_metrics["diverged"],
        "slam_metrics": slam_metrics,
    }


def _clearance_source(
    obstacles: Optional[Iterable[Any]], occupancy: Optional[OccupancyGrid]
) -> str:
    has_obs = _normalize_obstacles(obstacles).size > 0
    if has_obs and occupancy is not None:
        return "obstacles+map"
    if has_obs:
        return "obstacles"
    if occupancy is not None:
        return "map"
    return "none (clearance term assumed satisfied)"


# --------------------------------------------------------------------------- report
_DELTA_KEYS = {
    "duration_pct": "duration_s",
    "path_length_pct": "path_length_m",
    "E_p_pct": "E_p",
    "S_tau_pct": "S_tau",
    "C_min_pct": "C_min_m",
    "nqi_pct": "nqi",
    "ate_mean_pct": "ate_mean_m",
}


def _pct(new: Optional[float], old: Optional[float]) -> Optional[float]:
    if new is None or old is None or abs(old) < 1e-12:
        return None
    return round((new - old) / abs(old) * 100.0, 2)


def build_report(
    nominal: Mapping[str, Any],
    adversarial: Optional[Mapping[str, Any]] = None,
    params: NQIParams = NQIParams(),
) -> dict[str, Any]:
    """Assemble the ``benchmark_report.json`` structure from scored runs."""
    runs: dict[str, Any] = {"nominal": dict(nominal)}
    deltas: Optional[dict[str, Optional[float]]] = None
    if adversarial is not None:
        runs["adversarial"] = dict(adversarial)
        deltas = {k: _pct(adversarial.get(src), nominal.get(src)) for k, src in _DELTA_KEYS.items()}
        deltas["C_force_delta"] = int(adversarial.get("C_force", 0)) - int(
            nominal.get("C_force", 0)
        )
    return {
        "schema_version": REPORT_SCHEMA_VERSION,
        "generated_at": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "params": asdict(params),
        "runs": runs,
        "deltas": deltas,
    }


def default_report_path(
    project_name: str = "active_project", timestamp: Optional[str] = None
) -> Path:
    run_dir = os.environ.get("LANG2SIM_RUN_DIR")
    if run_dir:
        return Path(run_dir).expanduser() / "benchmark_report.json"
    try:
        from lang2sim.core.config import get_run_dir

        return get_run_dir(project_name=project_name, timestamp=timestamp) / "benchmark_report.json"
    except Exception:
        base = os.environ.get("LANG2SIM_PROJECT_DIR") or str(
            Path.home() / ".lang2sim" / "projects" / project_name
        )
        return Path(base).expanduser() / "benchmark_report.json"


def write_report(report: Mapping[str, Any], path: Optional[str | Path] = None) -> Path:
    out = Path(path) if path is not None else default_report_path()
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(report, indent=2), encoding="utf-8")
    return out


# --------------------------------------------------------------------------- rich output
def _fmt(v: Any, unit: str = "", digits: int = 2) -> str:
    if v is None:
        return "n/a"
    if isinstance(v, float):
        return f"{v:.{digits}f}{unit}"
    return f"{v}{unit}"


def _mission_str(s: Any) -> str:
    return {1: "[green]Succeeded[/]", 0: "[red]Failed[/]"}.get(s, "n/a")


def _delta_str(pct: Optional[float], higher_is_better: bool, note: str = "") -> str:
    if pct is None:
        return "n/a"
    good = (pct >= 0) == higher_is_better or abs(pct) < 1e-9
    color = "green" if good else "red"
    suffix = f" ({note})" if note else ""
    return f"[{color}]{pct:+.1f}%[/]{suffix}"


def render_comparison_table(report: Mapping[str, Any]):
    """Build a Rich table mirroring the architecture's §8.4 benchmark table."""
    from rich.table import Table

    runs = report["runs"]
    nom = runs["nominal"]
    adv = runs.get("adversarial")
    deltas = report.get("deltas") or {}

    table = Table(
        title="Lang2Sim Navigation Benchmark (NQI)", header_style="bold cyan", title_style="bold"
    )
    table.add_column("Evaluation Metric", style="bold")
    table.add_column("Nominal Run", justify="right")
    if adv is not None:
        table.add_column("Adversarial Stress Run", justify="right")
        table.add_column("Performance Delta", justify="right")

    def row(name: str, n_val: str, a_val: str = "", delta: str = "") -> None:
        if adv is not None:
            table.add_row(name, n_val, a_val, delta)
        else:
            table.add_row(name, n_val)

    a = adv or {}
    mission_delta = ""
    if adv is not None:
        mission_delta = (
            "Baseline maintained"
            if a.get("S_mission") == nom.get("S_mission")
            else "[red]Changed[/]"
        )

    def mission(run: Mapping[str, Any]) -> str:
        if run.get("nav_status") == "interrupted":
            return "[yellow]Interrupted (partial)[/]"
        return _mission_str(run.get("S_mission"))

    row(
        "Mission Completion Status",
        mission(nom),
        mission(a) if adv is not None else "",
        mission_delta,
    )
    row(
        "Mission Duration (T)",
        _fmt(nom.get("duration_s"), " s", 1),
        _fmt(a.get("duration_s"), " s", 1),
        _delta_str(deltas.get("duration_pct"), False),
    )
    row(
        "Traversed Path Length (L)",
        _fmt(nom.get("path_length_m"), " m"),
        _fmt(a.get("path_length_m"), " m"),
        _delta_str(deltas.get("path_length_pct"), False),
    )
    row(
        "Path Traversal Efficiency (E_p)",
        _fmt(nom.get("E_p")),
        _fmt(a.get("E_p")),
        _delta_str(deltas.get("E_p_pct"), True),
    )
    row(
        "Trajectory Smoothness (S_tau)",
        _fmt(nom.get("S_tau")),
        _fmt(a.get("S_tau")),
        _delta_str(deltas.get("S_tau_pct"), False),
    )
    row(
        "Minimum Obstacle Margin (C_min)",
        _fmt(nom.get("C_min_m"), " m"),
        _fmt(a.get("C_min_m"), " m"),
        _delta_str(deltas.get("C_min_pct"), True),
    )
    c_delta = deltas.get("C_force_delta")
    row(
        "Physical Collision Count (C_force)",
        str(nom.get("C_force", 0)),
        str(a.get("C_force", "")),
        "" if c_delta is None else ("[green]0[/]" if c_delta == 0 else f"[red]{c_delta:+d}[/]"),
    )
    row(
        "Navigation Quality Index (NQI)",
        f"[bold]{_fmt(nom.get('nqi'), ' / 100', 1)}[/]",
        f"[bold]{_fmt(a.get('nqi'), ' / 100', 1)}[/]" if adv is not None else "",
        _delta_str(deltas.get("nqi_pct"), True),
    )

    if nom.get("ate_mean_m") is not None or (adv and adv.get("ate_mean_m") is not None):
        # Recorded runs (telemetry/benchmark.py) carry AMCL-vs-ground-truth error in these columns;
        # synthetic/SLAM runs carry SLAM drift. Label the row for what it actually measures.
        localization = "localization_error_mean_m" in nom or (
            adv is not None and "localization_error_mean_m" in adv
        )
        row(
            "Localization Error (AMCL vs truth)" if localization else "SLAM Drift (ATE Mean)",
            _fmt(nom.get("ate_mean_m"), " m"),
            _fmt(a.get("ate_mean_m"), " m"),
            _delta_str(deltas.get("ate_mean_pct"), False),
        )

        def _health_str(div: Any) -> str:
            if div is None:
                return "n/a"
            return "[red]Diverged (>0.35 m)[/]" if div else "[green]Healthy (≤0.35 m)[/]"

        row(
            "Localization Health" if localization else "SLAM Tracking Health",
            _health_str(nom.get("slam_diverged")),
            _health_str(a.get("slam_diverged")),
            "",
        )
    return table


def render_to_console(report: Mapping[str, Any], console: Any = None) -> None:
    from rich.console import Console

    (console or Console()).print(render_comparison_table(report))


# --------------------------------------------------------------------------- demo data
def demo_trajectories(adversarial_collisions: int = 0, seed: int = 7) -> dict[str, Any]:
    """Synthetic nominal (straight) vs adversarial (crate detour + jitter) runs on a 10 m corridor."""
    rng = np.random.default_rng(seed)
    goal = (10.0, 0.0)

    # Nominal: straight line at a constant 0.5 m/s.
    t_n = np.arange(0.0, 20.0 + 1e-9, 0.05)
    nominal = Trajectory(t=t_n, x=0.5 * t_n, y=np.zeros_like(t_n), goal=goal, label="nominal")

    # Adversarial: slower, Gaussian detour around a crate at (5, 0) plus small steering jitter.
    t_a = np.arange(0.0, 30.0 + 1e-9, 0.05)
    x_a = 10.0 * t_a / t_a[-1]
    y_a = 0.9 * np.exp(-(((x_a - 5.0) / 1.2) ** 2)) + 0.03 * np.sin(2 * np.pi * x_a / 0.8)
    y_a += rng.normal(0.0, 0.005, size=y_a.shape)
    y_a[0] = 0.0
    y_a[-1] = 0.0
    adversarial = Trajectory(
        t=t_a, x=x_a, y=y_a, goal=goal, collisions=adversarial_collisions, label="adversarial"
    )

    static_obstacles = [(5.0, 1.6, 0.3)]  # rack beside the corridor
    return {
        "goal": goal,
        "nominal": nominal,
        "adversarial": adversarial,
        "obstacles_nominal": static_obstacles,
        "obstacles_adversarial": static_obstacles + [(5.0, 0.0, 0.3)],  # + spawned crate
    }


# --------------------------------------------------------------------------- CLI
def _load_traj(path: str) -> Trajectory:
    return (
        Trajectory.from_csv(path) if path.lower().endswith(".csv") else Trajectory.from_json(path)
    )


def _load_obstacles(path: Optional[str]) -> Optional[list[Any]]:
    if not path:
        return None
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    return data.get("obstacles", data) if isinstance(data, dict) else data


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(
        prog="python -m lang2sim.telemetry.nqi_scorer", description=__doc__.splitlines()[0]
    )
    ap.add_argument(
        "--nominal", help="nominal run trajectory (.json from ros_subscribers --record, or .csv)"
    )
    ap.add_argument("--adversarial", help="adversarial run trajectory (.json or .csv)")
    ap.add_argument(
        "--goal", nargs=2, type=float, metavar=("X", "Y"), help="mission goal (overrides file)"
    )
    ap.add_argument("--obstacles", help='JSON file: [[x, y, r], ...] or {"obstacles": [...]} ')
    ap.add_argument("--map", help="Nav2 map.yaml (enables A* optimal length and map clearance)")
    ap.add_argument("--l-optimal", type=float, help="override optimal path length (m)")
    ap.add_argument(
        "--out", help="report path (default: $LANG2SIM_PROJECT_DIR/benchmark_report.json)"
    )
    ap.add_argument(
        "--demo", action="store_true", help="score synthetic nominal vs adversarial runs"
    )
    ap.add_argument(
        "--demo-collisions", type=int, default=0, help="collisions injected in demo adversarial run"
    )
    ap.add_argument(
        "--save-demo-trajectories", metavar="DIR", help="also write demo trajectories as JSON"
    )
    ap.add_argument("--quiet", action="store_true", help="don't print the Rich table")
    args = ap.parse_args(argv)

    params = NQIParams()
    occupancy = OccupancyGrid.from_yaml(args.map) if args.map else None
    goal = tuple(args.goal) if args.goal else None

    if args.demo:
        d = demo_trajectories(args.demo_collisions)
        if args.save_demo_trajectories:
            out_dir = Path(args.save_demo_trajectories)
            d["nominal"].to_json(out_dir / "nominal.json")
            d["adversarial"].to_json(out_dir / "adversarial.json")
            (out_dir / "obstacles.json").write_text(
                json.dumps({"obstacles": d["obstacles_adversarial"]}), encoding="utf-8"
            )
        nom = score_trajectory(d["nominal"], obstacles=d["obstacles_nominal"], params=params)
        adv = score_trajectory(
            d["adversarial"], obstacles=d["obstacles_adversarial"], params=params
        )
    else:
        if not args.nominal:
            ap.error("--nominal is required unless --demo is used")
        obstacles = _load_obstacles(args.obstacles)
        common = dict(
            goal=goal,
            obstacles=obstacles,
            occupancy=occupancy,
            l_optimal=args.l_optimal,
            params=params,
        )
        nom = score_trajectory(_load_traj(args.nominal), **common)  # type: ignore[arg-type]
        adv = score_trajectory(_load_traj(args.adversarial), **common) if args.adversarial else None  # type: ignore[arg-type]

    report = build_report(nom, adv, params)
    out = write_report(report, args.out)
    if not args.quiet:
        render_to_console(report)
    print(f"benchmark report written to: {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
