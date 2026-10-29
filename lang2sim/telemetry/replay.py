"""
Replay fallback (architecture §5.2, Mode B): serve a recorded run as if it were live.
File: lang2sim/telemetry/replay.py

    python -m lang2sim.telemetry.replay <run_dir>                 # http://localhost:9090/dashboard
    python -m lang2sim.telemetry.replay catalog/demo_runs/empty_room_benchmark --loop --speed 2

If the live simulation dies on stage (GPU, Docker, DDS), stop the sim container and
start this on the same port. It plays the run folder's recorded trajectories
(nominal, then adversarial) in sim time and answers the same read-only control API
routes the TUI and the dashboard use:

  GET /v1/health, /v1/info, /v1/sim/status, /v1/telemetry, /v1/telemetry/live,
      /v1/telemetry/setup, /v1/nav/status        WS /v1/nav/feedback
  GET /dashboard, /v1/run, /v1/run/map, /v1/run/trajectories, /v1/run/report

so neither needs to know it is a replay (both show a REPLAY badge from /v1/run's
"mode"). Commands that would change a simulation (/v1/world/*, /v1/nav/*,
/v1/telemetry/record/*) answer 409.

Clearance during replay is geometric (robot chassis to map + spawned crates), not
lidar-based; everything else comes straight from the recording.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import math
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence

import numpy as np

try:  # module level: with postponed annotations FastAPI resolves `ws: WebSocket` from these globals
    from fastapi import APIRouter, FastAPI, HTTPException, WebSocket, WebSocketDisconnect
except ImportError:  # snapshot/clock logic stays importable without the [sim] extra
    APIRouter = FastAPI = HTTPException = WebSocket = WebSocketDisconnect = None  # type: ignore

from lang2sim.runtime.dashboard import make_router, read_report
from lang2sim.runtime.telemetry_recorder import load_recorded_runs
from lang2sim.telemetry.benchmark import (
    WAFFLE_FOOTPRINT,
    footprint_points,
    load_occupancy,
    mission_targets,
)
from lang2sim.telemetry.nqi_scorer import clearance_series
from lang2sim.telemetry.trajectory import Trajectory

PAUSE_BETWEEN_RUNS_S = 2.0  # wall seconds between the nominal and the stress run
REACHED_M = 0.35  # a waypoint counts as reached within this distance


@dataclass
class PlayedRun:
    label: str
    t: np.ndarray  # sim seconds (as recorded)
    x: np.ndarray
    y: np.ndarray
    yaw: np.ndarray
    v: np.ndarray
    w: np.ndarray
    clearance: np.ndarray  # chassis-to-obstacle, per sample (crate only after it appears)
    loc_err: Optional[np.ndarray]
    targets: List[tuple]
    target_idx: np.ndarray  # index of the waypoint being driven to, per sample
    contacts: List[Dict[str, Any]]
    crates: List[Dict[str, Any]]  # spawned entities, each with "appear_t" (sim seconds)
    nav_status: str
    contact_force_source: str
    pose_source: str

    @property
    def duration(self) -> float:
        return float(self.t[-1] - self.t[0]) if len(self.t) > 1 else 0.0


def _crate_times(
    label: str, run_meta: Dict[str, Any], report: Optional[Dict[str, Any]], t0: float
) -> List[Dict[str, Any]]:
    crates = [dict(e) for e in run_meta.get("spawned_entities") or []]
    fired = iter(
        [tr.get("fired_at_s") for tr in ((report or {}).get("stress") or {}).get("triggers") or []]
    )
    for c in crates:
        # Already in the world when the run began -> visible from the start; dropped during the
        # run -> at the recorded trigger time (benchmark runs), else from the start.
        at = next(fired, None) if label == "adversarial" and c.get("during_run", True) else None
        c["appear_t"] = t0 + float(at or 0.0)
    return crates


def build_played_run(
    label: str, data: Dict[str, Any], run_dir: Path, spec: Any, report: Optional[Dict[str, Any]]
) -> PlayedRun:
    traj = Trajectory.from_dict(data)
    meta = data.get("meta") or {}
    occupancy = load_occupancy(run_dir)
    crates = _crate_times(label, meta, report, float(traj.t[0]))
    static = clearance_series(traj, WAFFLE_FOOTPRINT, None, occupancy)
    clearance = static.copy()
    for c in crates:
        with_crate = clearance_series(traj, WAFFLE_FOOTPRINT, footprint_points(c), None)
        visible = traj.t >= c["appear_t"]
        clearance[visible] = np.minimum(clearance[visible], with_crate[visible])
    targets = (
        mission_targets(spec)
        if spec is not None
        else ([tuple(traj.goal)] if traj.goal else [traj.end])
    )
    idx, k = np.zeros(traj.n, dtype=int), 0
    for i in range(traj.n):
        while (
            k < len(targets) - 1
            and math.hypot(traj.x[i] - targets[k][0], traj.y[i] - targets[k][1]) <= REACHED_M
        ):
            k += 1
        idx[i] = k
    scored = (report or {}).get("runs") or {}
    nav_status = next(
        (r.get("nav_status") for r in scored.values() if r.get("label") == label), None
    )
    loc = traj.ate if traj.ate is not None else None
    return PlayedRun(
        label=label,
        t=traj.t,
        x=traj.x,
        y=traj.y,
        yaw=traj.yaw,
        v=traj.v,
        w=traj.w,
        clearance=clearance,
        loc_err=loc,
        targets=list(targets),
        target_idx=idx,
        contacts=list(meta.get("contact_events") or []),
        crates=crates,
        nav_status=nav_status or ("interrupted" if meta.get("partial") else "succeeded"),
        contact_force_source=meta.get("contact_force_source") or "none",
        pose_source=meta.get("pose_source") or "none",
    )


@dataclass
class ReplayClock:
    """Which run and which sample to show at a given wall time."""

    runs: List[PlayedRun]
    speed: float = 1.0
    loop: bool = False
    started: float = field(default_factory=time.monotonic)

    @property
    def cycle(self) -> float:
        return sum(r.duration / self.speed + PAUSE_BETWEEN_RUNS_S for r in self.runs)

    def position(self, now: Optional[float] = None) -> tuple:
        """(run index, sample index, playing?) at wall time `now`."""
        elapsed = (now if now is not None else time.monotonic()) - self.started
        if self.loop and self.cycle > 0:
            elapsed %= self.cycle
        for i, r in enumerate(self.runs):
            span = r.duration / self.speed
            if elapsed < span:
                t = r.t[0] + elapsed * self.speed
                return i, max(0, int(np.searchsorted(r.t, t, side="right")) - 1), True
            elapsed -= span
            if elapsed < PAUSE_BETWEEN_RUNS_S and i < len(self.runs) - 1:
                return i, len(r.t) - 1, False
            elapsed -= PAUSE_BETWEEN_RUNS_S
        last = self.runs[-1]
        return len(self.runs) - 1, len(last.t) - 1, False

    def restart(self) -> None:
        self.started = time.monotonic()


def _f(v: Any, d: int = 3) -> Optional[float]:
    return None if v is None or not math.isfinite(float(v)) else round(float(v), d)


def snapshot(clock: ReplayClock, now: Optional[float] = None) -> Dict[str, Any]:
    """{"live": TelemetryLive fields, "nav": NavStatus fields, "sim": {...}} at wall time `now`."""
    ri, i, playing = clock.position(now)
    r = clock.runs[ri]
    t = float(r.t[i])
    goal = r.targets[-1]
    k = int(r.target_idx[i])
    hits = [c for c in r.contacts if c.get("t", 0) <= t]
    last = hits[-1] if hits else None
    done = not playing and i == len(r.t) - 1
    status = r.nav_status if done else "navigating"
    live = {
        "sim_time": round(t, 3),
        "real_time_factor": round(clock.speed, 3),
        "pose": {"x": _f(r.x[i]), "y": _f(r.y[i]), "yaw": _f(r.yaw[i], 4)},
        "pose_source": r.pose_source,
        "linear_speed": _f(r.v[i]) or 0.0,
        "angular_speed": _f(r.w[i]) or 0.0,
        "localization_error_m": _f(r.loc_err[i]) if r.loc_err is not None else None,
        "min_scan_range_m": None,
        "obstacle_clearance_m": _f(r.clearance[i]),
        "distance_to_goal_m": _f(math.hypot(goal[0] - r.x[i], goal[1] - r.y[i])),
        "collision_count": len(hits),
        "last_contact_force_n": (last or {}).get("force_n") or 0.0,
        "last_contact_entity": (last or {}).get("entity", ""),
        "contact_force_source": r.contact_force_source,
        "recording": playing,
        "recording_label": r.label if playing else None,
        "samples_recorded": i + 1 if playing else 0,
    }
    target = r.targets[k]
    nav = {
        "is_active": playing,
        "current_task": "waypoints" if len(r.targets) > 1 else "goal",
        "distance_remaining": _f(math.hypot(target[0] - r.x[i], target[1] - r.y[i])) or 0.0,
        "target_pose": {"x": target[0], "y": target[1], "yaw": 0.0},
        "status": status,
        "waypoint_index": k,
        "waypoints_total": len(r.targets),
        "navigation_time": round(t - float(r.t[0]), 2),
        "eta_seconds": None,
        "recoveries": 0,
        "message": None,
    }
    sim = {"status": "running", "ready": True, "rtf": round(clock.speed, 3)}
    return {
        "live": live,
        "nav": nav,
        "sim": sim,
        "run": r.label,
        "crates": [c for c in r.crates if c["appear_t"] <= t],
    }


def load_replay(
    run_dir: Path, which: str = "both", speed: float = 1.0, loop: bool = False
) -> ReplayClock:
    from lang2sim.core.models import SimSpec

    run_dir = Path(run_dir)
    recorded = load_recorded_runs(run_dir)
    if not recorded:
        raise FileNotFoundError(f"no recorded trajectories in {run_dir / 'logs'}")
    labels = [label for label in recorded if which in ("both", label)]
    if not labels:
        raise FileNotFoundError(
            f"no '{which}' run in {run_dir / 'logs'} (have: {', '.join(recorded)})"
        )
    spec_path = run_dir / "spec.json"
    spec = SimSpec.model_validate_json(spec_path.read_text()) if spec_path.is_file() else None
    report = read_report(run_dir)
    runs = [build_played_run(label, recorded[label], run_dir, spec, report) for label in labels]
    return ReplayClock(runs=[r for r in runs if len(r.t) >= 2], speed=max(0.05, speed), loop=loop)


def create_replay_app(run_dir: Path, which: str = "both", speed: float = 1.0, loop: bool = False):
    if FastAPI is None:
        raise ImportError("the replay server needs FastAPI: pip install -e '.[sim]'")
    import lang2sim

    run_dir = Path(run_dir).resolve()
    clock = load_replay(run_dir, which, speed, loop)
    try:
        world = json.loads((run_dir / "manifest.json").read_text()).get("world_name", "")
    except (OSError, ValueError):
        world = ""
    app = FastAPI(title="Lang2Sim replay", version=lang2sim.__version__)
    app.state.clock = clock
    v1 = APIRouter(prefix="/v1")
    readiness = {
        k: True for k in ("gazebo_clock", "odometry", "laser_scan", "nav2_active", "localized")
    }

    @v1.get("/health")
    def health() -> dict:
        return {
            "status": "ok",
            "mode": "replay",
            "version": lang2sim.__version__,
            "run_dir": str(run_dir),
            "ros": False,
        }

    @v1.get("/info")
    def info() -> dict:
        return {
            "mode": "replay",
            "run_dir": str(run_dir),
            "runs": [r.label for r in clock.runs],
            "speed": clock.speed,
            "loop": clock.loop,
        }

    @v1.get("/sim/status")
    def sim_status() -> dict:
        snap = snapshot(clock)
        return {
            "status": "running",
            "rtf": clock.speed,
            "sim_time": snap["live"]["sim_time"],
            "ready": True,
            "readiness": readiness,
            "world_report": ["[info] replay: " + str(run_dir)],
            "active_nodes": [],
            "active_topics": [],
        }

    @v1.get("/telemetry/live")
    def telemetry_live() -> dict:
        return snapshot(clock)["live"]

    @v1.get("/telemetry")
    def telemetry() -> dict:
        live = snapshot(clock)["live"]
        return {
            "is_running": True,
            "sim_time": live["sim_time"],
            "real_time_factor": clock.speed,
            "robot_pose": live["pose"],
            "robot_velocity": {"x": live["linear_speed"], "y": 0.0, "yaw": live["angular_speed"]},
            "render_mode": "headless",
            "health_status": "healthy",
        }

    @v1.get("/telemetry/setup")
    def telemetry_setup() -> dict:
        return {
            "enabled": True,
            "collision": "base_collision",
            "reason": "replay of a recorded run",
        }

    @v1.get("/nav/status")
    def nav_status() -> dict:
        return snapshot(clock)["nav"]

    @v1.post("/replay/restart")
    def restart() -> dict:
        clock.restart()
        return {"restarted": True}

    def read_only() -> None:
        raise HTTPException(
            409, "replay mode: this is a recorded run, the simulation cannot be changed"
        )

    for path in (
        "/world/spawn",
        "/world/light",
        "/world/reset",
        "/world/chassis",
        "/nav/goal",
        "/nav/waypoints",
        "/nav/cancel",
        "/nav/initial_pose",
        "/telemetry/record/start",
        "/telemetry/record/stop",
        "/sim/start",
        "/sim/stop",
    ):
        v1.add_api_route(path, read_only, methods=["POST"], include_in_schema=False)

    @v1.websocket("/nav/feedback")
    async def nav_feedback(ws: WebSocket, hz: float = 5.0) -> None:
        await ws.accept()
        period = 1.0 / max(0.5, min(hz, 20.0))
        try:
            while True:
                snap = snapshot(clock)
                await ws.send_json(
                    {
                        "nav": snap["nav"],
                        "live": snap["live"],
                        "sim": snap["sim"],
                        "telemetry": telemetry(),
                    }
                )
                await asyncio.sleep(period)
        except (WebSocketDisconnect, RuntimeError):
            pass

    app.include_router(v1)
    app.include_router(
        make_router(
            lambda: run_dir,
            mode="replay",
            get_world=lambda: world,
            get_spawned=lambda: snapshot(clock)["crates"],
        )
    )
    return app


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(
        description="Replay a recorded Lang2Sim run through the control API "
        "(demo fallback when the live sim is down)."
    )
    ap.add_argument("run_dir", type=Path)
    ap.add_argument("--run", default="both", help="nominal | adversarial | both (default)")
    ap.add_argument(
        "--speed", type=float, default=1.0, help="playback speed (sim seconds per wall second)"
    )
    ap.add_argument("--loop", action="store_true", help="start over after the last run")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument(
        "--port", type=int, default=9090, help="same port as the control API (default 9090)"
    )
    a = ap.parse_args(argv)
    try:
        app = create_replay_app(a.run_dir.expanduser(), a.run, a.speed, a.loop)
    except FileNotFoundError as e:
        print(f"[replay] {e}")
        return 2
    import uvicorn

    runs = ", ".join(f"{r.label} ({r.duration:.0f} s)" for r in app.state.clock.runs)
    print(f"[replay] {a.run_dir}: {runs} at {a.speed}x{' looping' if a.loop else ''}")
    print(
        f"[replay] dashboard: http://localhost:{a.port}/dashboard   (TUI and API clients work unchanged)"
    )
    uvicorn.run(app, host=a.host, port=a.port, log_level="warning")
    return 0


if __name__ == "__main__":
    sys.exit(main())
