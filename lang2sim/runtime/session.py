"""
SimSession: host-side "open this run folder and show it" in one object.
File: lang2sim/runtime/session.py

What `scripts/run_sim.py` does, and what Part 2's `lang2sim run` should call:

    plan = choose_display(platform.detect(), hardware_probe.probe())
    async with SimSession(plan) as s:          # starts the container, waits for the API
        await s.open_run(run_dir)              # pre-flight, launch, wait until ready
        ...                                    # s.api is a ControlClient for /v1/*
    # leaving the block stops the sim and removes the container
"""

from __future__ import annotations

import asyncio
import dataclasses
import importlib
import shutil
import time
from pathlib import Path
from typing import Any, Callable, Dict, List, Mapping, Optional, Sequence, Set

from lang2sim.core.models import NavStatus, Pose2D, SimSpec, SimStatusResponse, TelemetryRecordResult
from lang2sim.runtime.control_api.client import ControlClient
from lang2sim.runtime.display import DisplayPlan
from lang2sim.runtime.docker_runner import DockerRunner, RunOptions, default_run_options
from lang2sim.runtime.run_folder import load_manifest, to_start_request

cfg = importlib.import_module("lang2sim.core.config")

FIXTURES_DIR = Path(__file__).resolve().parents[2] / "catalog" / "fixtures"
ProgressFn = Callable[[SimStatusResponse, float], None]
NavProgressFn = Callable[[NavStatus], None]
TriggerFn = Callable[[Dict[str, Any], Optional[str]], None]   # (trigger, error or None)


def due_triggers(triggers: Sequence[Mapping[str, Any]], fired: Set[int], elapsed_s: float,
                 distance_remaining: Optional[float]) -> List[int]:
    """Indices of adversarial triggers (DynamicObstacleTrigger dicts) that should fire now.

    immediate: at once; time: `trigger_value` sim seconds after the run started;
    waypoint_distance: when Nav2's distance remaining on the current leg drops to
    `trigger_value` metres.
    """
    due = []
    for i, tr in enumerate(triggers):
        if i in fired:
            continue
        kind, value = tr.get("trigger_type"), float(tr.get("trigger_value", 0.0))
        if (kind == "immediate"
                or (kind == "time" and elapsed_s >= value)
                or (kind == "waypoint_distance" and distance_remaining is not None
                    and elapsed_s > 0 and distance_remaining <= value)):
            due.append(i)
    return due


@dataclasses.dataclass
class RecordedRun:
    """One mission driven with telemetry recording on, and its NQI score."""
    nav: NavStatus
    result: TelemetryRecordResult
    scored: Dict[str, Any]
    triggers_fired: List[Dict[str, Any]] = dataclasses.field(default_factory=list)


class SimNotReady(RuntimeError):
    def __init__(self, status: SimStatusResponse, logs: str = ""):
        failed = [k for k, v in status.readiness.items() if not v]
        super().__init__(f"simulation {status.status}, not ready: {', '.join(failed) or 'unknown'}")
        self.status, self.logs = status, logs


def copy_fixture(name: str, project: Optional[str] = None) -> Path:
    """Copy catalog/fixtures/<name> into a fresh ~/.lang2sim/projects/<project>/runs/<ts>/."""
    src = FIXTURES_DIR / name
    if not src.is_dir():
        available = sorted(p.name for p in FIXTURES_DIR.iterdir() if p.is_dir()) if FIXTURES_DIR.is_dir() else []
        raise FileNotFoundError(f"no fixture '{name}' (available: {', '.join(available) or 'none'})")
    run = cfg.get_run_dir(project or name)
    shutil.copytree(src, run, dirs_exist_ok=True)
    return run


class SimSession:
    def __init__(self, plan: DisplayPlan, *, image: Optional[str] = None, dev_src: Optional[Path] = None,
                 host_env: Optional[Dict[str, str]] = None, has_dxg: bool = False,
                 extra_env: Optional[Dict[str, str]] = None, runner: Optional[DockerRunner] = None,
                 opts: Optional[RunOptions] = None):
        self.plan = plan
        self.runner = runner or DockerRunner()
        base = opts or default_run_options(plan, image=image, project_src=dev_src,
                                           host_env=host_env, has_dxg=has_dxg)
        self.opts = dataclasses.replace(base, extra_env={**base.extra_env, **(extra_env or {})})
        self.api = ControlClient(f"http://127.0.0.1:{self.opts.api_port}", timeout=30)
        self.run_dir: Optional[Path] = None

    @property
    def view_hint(self) -> str:
        if self.plan.mode == "vnc":
            return f"open http://localhost:{self.opts.novnc_port} in a browser"
        if self.plan.mode == "headless":
            return f"connect Foxglove to ws://localhost:{self.opts.foxglove_port}"
        return "Gazebo and RViz open as windows on this desktop"

    async def start(self, pull: bool = True, wait_s: float = 120) -> dict:
        if pull:
            self.runner.ensure_image(self.opts.image)
        self.runner.start(self.opts)
        return await self.api.wait_until_ready(timeout=wait_s)

    async def open_run(self, run_dir: Path, *, gui: bool = True, rviz: bool = True,
                       timeout: float = 300, poll_s: float = 2.0,
                       on_progress: Optional[ProgressFn] = None) -> SimStatusResponse:
        """Start the run and wait until every readiness check passes."""
        run_dir = Path(run_dir).resolve()
        manifest = load_manifest(run_dir)
        req = to_start_request(run_dir, manifest, projects_dir=self.opts.projects_dir,
                               gui=gui and self.plan.has_gui, rviz=rviz and self.plan.has_gui)
        self.run_dir = run_dir
        await self.api.sim_start(req)
        t0 = time.monotonic()
        while True:
            st = await self.api.sim_status()
            if on_progress:
                on_progress(st, time.monotonic() - t0)
            if st.ready:
                return st
            if st.status in ("error", "stopped") or time.monotonic() - t0 > timeout:
                lines = await self.api.sim_logs(60)
                raise SimNotReady(st, "\n".join(lines))
            await asyncio.sleep(poll_s)

    async def run_mission(self, run_dir: Optional[Path] = None, *, on_progress: Optional[NavProgressFn] = None,
                          poll_s: float = 2.0) -> NavStatus:
        """Drive the run's spec.json mission (waypoints, then the goal) and wait for the result."""
        run_dir = Path(run_dir or self.run_dir or ".")
        spec = SimSpec.model_validate_json((run_dir / "spec.json").read_text())
        poses = [*spec.mission.waypoints, spec.mission.goal_pose]
        timeout = spec.mission.timeout_seconds
        await self.api.nav_waypoints(poses, timeout_seconds=timeout)
        last = None
        while True:
            st = await self.api.nav_status()
            key = (st.status, st.waypoint_index, round(st.distance_remaining))  # 1 m steps
            if on_progress and key != last:
                on_progress(st)
                last = key
            if not st.is_active:
                return st
            await asyncio.sleep(poll_s)

    # --- telemetry engine: recorded, scored missions -------------------------------------
    async def run_recorded_mission(self, run_dir: Optional[Path] = None, *, label: str = "nominal",
                                   triggers: Sequence[Mapping[str, Any]] = (), rate_hz: float = 20.0,
                                   on_progress: Optional[NavProgressFn] = None,
                                   on_trigger: Optional[TriggerFn] = None, poll_s: float = 0.5,
                                   params: Any = None) -> RecordedRun:
        """Drive spec.json's mission with telemetry recording, fire adversarial triggers on the way,
        and score the run (NQI). Recording always stops, even if the mission errors."""
        from lang2sim.telemetry.benchmark import score_recorded_run
        from lang2sim.telemetry.nqi_scorer import NQIParams

        run_dir = Path(run_dir or self.run_dir or ".")
        spec = SimSpec.model_validate_json((run_dir / "spec.json").read_text())
        poses = [*spec.mission.waypoints, spec.mission.goal_pose]
        triggers = [dict(t) for t in triggers]
        fired: Set[int] = set()
        fired_log: List[Dict[str, Any]] = []
        live = await self.api.record_start(label, goal=spec.mission.goal_pose, rate_hz=rate_hz)
        t0 = live.sim_time
        result: Optional[TelemetryRecordResult] = None
        try:
            await self.api.nav_waypoints(poses, timeout_seconds=spec.mission.timeout_seconds)
            last = None
            while True:
                st = await self.api.nav_status()
                key = (st.status, st.waypoint_index, round(st.distance_remaining))
                if on_progress and key != last:
                    on_progress(st)
                    last = key
                if triggers and len(fired) < len(triggers):
                    now = (await self.api.telemetry_live()).sim_time
                    for i in due_triggers(triggers, fired, now - t0, st.distance_remaining if st.is_active else None):
                        fired.add(i)
                        tr = triggers[i]
                        pose = tr.get("spawn_pose") or {}
                        err = None
                        try:
                            await self.api.world_spawn({"model_name": tr.get("model_name", "wooden_crate"),
                                                        "x": pose.get("x", 0.0), "y": pose.get("y", 0.0),
                                                        "yaw": pose.get("yaw", 0.0)})
                        except Exception as e:  # noqa: BLE001 - a failed spawn must not abort the run
                            err = str(e)
                        fired_log.append({**tr, "fired_at_s": round(now - t0, 2), "error": err})
                        if on_trigger:
                            on_trigger(tr, err)
                if not st.is_active:
                    break
                await asyncio.sleep(poll_s)
        finally:
            result = await self.api.record_stop()
        scored = score_recorded_run(result.trajectory, run_dir=run_dir, spec=spec, spawned=result.spawned_entities,
                                    nav_status=st.status, params=params or NQIParams())
        return RecordedRun(nav=st, result=result, scored=scored, triggers_fired=fired_log)

    async def return_to_start(self, run_dir: Optional[Path] = None, *, timeout_s: float = 180.0) -> NavStatus:
        """Drive back to the mission's initial pose (unrecorded), e.g. between benchmark runs."""
        run_dir = Path(run_dir or self.run_dir or ".")
        spec = SimSpec.model_validate_json((run_dir / "spec.json").read_text())
        await self.api.nav_goal(spec.mission.initial_pose, timeout_seconds=timeout_s)
        return await self.api.wait_nav_done(poll_s=1.0, timeout=timeout_s + 30)

    async def run_benchmark(self, run_dir: Optional[Path] = None, *, light: Optional[float] = None,
                            on_progress: Optional[NavProgressFn] = None, on_trigger: Optional[TriggerFn] = None,
                            on_phase: Optional[Callable[[str], None]] = None, params: Any = None,
                            poll_s: float = 0.5) -> Dict[str, Any]:
        """Nominal run, back to start, adversarial run; writes <run>/benchmark_report.json.

        The adversarial run uses spec.json's adversarial_triggers, or (none given) the
        Archetype 4 default: a wooden crate dropped onto the nominal path ahead of the
        robot. `light` dims the sun (0..1) for the adversarial run and restores it after.
        """
        from lang2sim.telemetry.benchmark import default_stress_trigger, write_benchmark
        from lang2sim.telemetry.nqi_scorer import NQIParams

        params = params or NQIParams()
        run_dir = Path(run_dir or self.run_dir or ".")
        spec = SimSpec.model_validate_json((run_dir / "spec.json").read_text())
        phase = on_phase or (lambda _m: None)

        phase("nominal run")
        nominal = await self.run_recorded_mission(run_dir, label="nominal", on_progress=on_progress, params=params,
                                                  poll_s=poll_s)
        phase("returning to start")
        await self.return_to_start(run_dir)
        triggers = ([t.model_dump(mode="json") for t in spec.adversarial_triggers] if spec.adversarial_triggers
                    else [default_stress_trigger(nominal.result.trajectory)])
        if light is not None:
            await self.api.world_light({"intensity": light})
        try:
            phase("adversarial run")
            adversarial = await self.run_recorded_mission(run_dir, label="adversarial", triggers=triggers,
                                                          on_progress=on_progress, on_trigger=on_trigger,
                                                          params=params, poll_s=poll_s)
        finally:
            if light is not None:
                try:
                    await self.api.world_light({"intensity": 1.0})
                except Exception:  # noqa: BLE001
                    pass
        path, report = write_benchmark(run_dir, nominal.scored, adversarial.scored, params, extra={
            "run_dir": str(run_dir),
            "stress": {"triggers": adversarial.triggers_fired, "light": light,
                       "source": "spec.adversarial_triggers" if spec.adversarial_triggers else "default_crate"},
            "trajectories": {"nominal": nominal.result.path, "adversarial": adversarial.result.path},
        })
        report["path"] = str(path)
        return report

    async def score_single_run(self, run_dir: Optional[Path] = None, **kw: Any) -> Dict[str, Any]:
        """Drive the mission once with recording and write a nominal-only benchmark_report.json."""
        from lang2sim.telemetry.benchmark import write_benchmark
        from lang2sim.telemetry.nqi_scorer import NQIParams

        params = kw.pop("params", None) or NQIParams()
        run_dir = Path(run_dir or self.run_dir or ".")
        run = await self.run_recorded_mission(run_dir, label="nominal", params=params, **kw)
        path, report = write_benchmark(run_dir, run.scored, None, params, extra={
            "run_dir": str(run_dir), "trajectories": {"nominal": run.result.path}})
        report["path"] = str(path)
        report["nav"] = run.nav.model_dump(mode="json")
        return report

    async def stop(self) -> None:
        try:
            await self.api.sim_stop()
        except Exception:  # noqa: BLE001 - container may already be gone
            pass
        await self.api.aclose()
        self.runner.remove(self.opts.name)

    async def __aenter__(self) -> "SimSession":
        await self.start()
        return self

    async def __aexit__(self, *exc) -> None:
        await self.stop()
