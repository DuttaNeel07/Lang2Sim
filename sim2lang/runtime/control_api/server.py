"""
Control API server. Runs INSIDE the sim container (uvicorn on :9090).
File: sim2lang/runtime/control_api/server.py

Started by docker/sim/entrypoint.sh (`serve`):
    uvicorn sim2lang.runtime.control_api.server:app --host 0.0.0.0 --port 9090

    GET  /v1/health        liveness + version
    GET  /v1/info          what the container resolved (display, gpu, ros, gz, arch)
    POST /v1/sim/start     SimStartRequest -> world pre-flight, sim_launch.py, launch
    GET  /v1/sim/status    SimStatusResponse incl. readiness checks and RTF
    POST /v1/sim/stop      SimStopRequest
    GET  /v1/sim/logs      tail of the sim process output (for the self-healer)

Navigation (/v1/nav/*) and telemetry are added by later modules; Part 4 mounts
routers/world.py under /v1/world.
"""

from __future__ import annotations

import collections
import contextlib
import os
import platform
import signal
import subprocess
import threading
import time
from pathlib import Path
from typing import Any, Deque, Dict, List, Optional

from fastapi import APIRouter, FastAPI, HTTPException, Query

import sim2lang
from sim2lang.core.models import SimStartRequest, SimStatusResponse, SimStopRequest
from sim2lang.runtime import launch_builder, world_check
from sim2lang.runtime.control_api import API_PREFIX
from sim2lang.runtime.ros_bridge import readiness

RUNTIME_ENV_FILE = Path(os.getenv("L2S_RUNTIME_ENV_FILE", "/tmp/l2s_runtime.env"))
LOG_RING_LINES = 2000
LAUNCH_GRACE_S = 3.0
_STARTED_AT = time.time()


def read_runtime_env(path: Path = RUNTIME_ENV_FILE) -> Dict[str, str]:
    """KEY=VALUE file written by the entrypoint."""
    out: Dict[str, str] = {}
    try:
        for line in path.read_text().splitlines():
            if "=" in line:
                k, v = line.split("=", 1)
                out[k.strip()] = v.strip()
    except OSError:
        pass
    return out


def model_search_paths(run_dir: Path) -> List[Path]:
    """Where model:// URIs may resolve: Gazebo's path, TurtleBot3 models, catalog, the run's own models/."""
    paths = world_check.default_model_paths()
    for p in (os.getenv("L2S_CATALOG_DIR", "/lang2sim/catalog") + "/models",
              str(run_dir / "models"),
              f"/opt/ros/{os.getenv('ROS_DISTRO', 'jazzy')}/share/turtlebot3_gazebo/models"):
        if Path(p).is_dir():
            paths.append(Path(p))
    return paths


class WorldRejected(Exception):
    def __init__(self, report: world_check.WorldReport):
        super().__init__(report.summary())
        self.report = report


class SimProcess:
    """One simulation (a `ros2 launch` process group), its log ring and lifecycle state."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self.proc: Optional[subprocess.Popen] = None
        self.request: Optional[SimStartRequest] = None
        self.started_at: Optional[float] = None
        self.stopped = False
        self.was_ready = False
        self.world_report: Optional[world_check.WorldReport] = None
        self.launch_file: Optional[Path] = None
        self.log: Deque[str] = collections.deque(maxlen=LOG_RING_LINES)
        self._log_file: Optional[Path] = None

    # --- preparation ---------------------------------------------------------
    def prepare(self, req: SimStartRequest) -> Path:
        """World pre-flight + sim_launch.py. Returns the launch file; raises on a bad run."""
        run_dir = Path(req.run_dir)
        world = Path(req.world_sdf_path)
        for p in (world, Path(req.map_yaml_path)):
            if not p.exists():
                raise FileNotFoundError(p)
        search = model_search_paths(run_dir)
        report, world_to_load = world_check.prepare_world(world, model_paths=search)
        self.world_report = report
        if world_to_load is None:
            raise WorldRejected(report)
        has_display = bool(os.getenv("DISPLAY"))
        cfg = launch_builder.LaunchConfig(
            run_dir=str(run_dir),
            world_sdf=str(world_to_load),
            world_name=req.world_name or report.world_name or "",
            map_yaml=req.map_yaml_path,
            x=req.initial_pose.x, y=req.initial_pose.y, yaw=req.initial_pose.yaw,
            gui=req.use_gui and has_display,
            rviz=req.use_rviz and has_display,
            camera=os.getenv("L2S_CAMERA", "1") != "0",
            nav2_params=req.nav2_params_path or "",
            resource_paths=[str(p) for p in search if "turtlebot3_gazebo" not in str(p)],
        )
        return launch_builder.write(cfg)

    # Overridden in tests.
    def build_command(self, req: SimStartRequest) -> List[str]:
        return ["ros2", "launch", str(self.launch_file)]

    # --- lifecycle -----------------------------------------------------------
    @property
    def alive(self) -> bool:
        return self.proc is not None and self.proc.poll() is None

    def status(self, ready: Optional[bool] = None) -> str:
        """idle | launching | running | degraded | stopped | error.

        `ready` comes from the ROS readiness checks (None = no ROS bridge here).
        """
        if self.proc is None:
            return "stopped" if self.stopped else "idle"
        rc = self.proc.poll()
        if rc is not None:
            return "stopped" if (self.stopped or rc == 0) else "error"
        if ready is None:
            age = time.time() - (self.started_at or 0)
            return "launching" if age < LAUNCH_GRACE_S else "running"
        if ready:
            self.was_ready = True
            return "running"
        return "degraded" if self.was_ready else "launching"

    def start(self, req: SimStartRequest) -> None:
        with self._lock:
            if self.alive:
                raise RuntimeError("simulation already running")
            run_dir = Path(req.run_dir)
            (run_dir / "logs").mkdir(parents=True, exist_ok=True)
            self.launch_file = self.prepare(req)
            (run_dir / "logs" / "world_check.txt").write_text(self.world_report.summary() + "\n")
            self._log_file = run_dir / "logs" / "sim.log"
            self.log.clear()
            self.request = req
            self.stopped = False
            self.was_ready = False
            self.started_at = time.time()
            self.proc = subprocess.Popen(
                self.build_command(req),
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1,
                cwd=str(run_dir), start_new_session=True,  # own process group for clean stop
            )
            threading.Thread(target=self._pump, args=(self.proc, self._log_file), daemon=True).start()

    def _pump(self, proc: subprocess.Popen, log_file: Optional[Path]) -> None:
        assert proc.stdout is not None
        fh = open(log_file, "a", buffering=1) if log_file else None
        try:
            for line in proc.stdout:
                self.log.append(line.rstrip("\n"))
                if fh:
                    fh.write(line)
        finally:
            if fh:
                fh.close()

    def stop(self, force: bool = False, grace_s: float = 15.0) -> None:
        with self._lock:
            proc = self.proc
            self.stopped = True
            if proc is None or proc.poll() is not None:
                return
            sig = signal.SIGKILL if force else signal.SIGINT
            try:
                os.killpg(proc.pid, sig)
                proc.wait(timeout=grace_s)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait(timeout=5)
            except ProcessLookupError:
                pass

    def tail(self, n: int) -> List[str]:
        return list(self.log)[-n:]


def create_app(sim: Optional[SimProcess] = None, use_ros: bool = True) -> FastAPI:
    """`use_ros=False` (tests) skips the rclpy bridge; it is also skipped where ROS is absent."""
    sim = sim or SimProcess()
    state: Dict[str, Any] = {"bridge": None}

    @contextlib.asynccontextmanager
    async def lifespan(_app: FastAPI):
        if use_ros:
            from sim2lang.runtime.ros_bridge import RosBridge
            state["bridge"] = RosBridge.try_create()
        yield
        sim.stop()  # don't leave gz/ros2 running when uvicorn exits
        if state["bridge"]:
            state["bridge"].stop()

    app = FastAPI(title="Lang2Sim control API", version=sim2lang.__version__, lifespan=lifespan)
    app.state.sim = sim
    app.state.ros = state
    v1 = APIRouter(prefix=API_PREFIX)

    def current_status() -> SimStatusResponse:
        bridge = state["bridge"]
        snap = bridge.snapshot() if bridge else None
        checks = readiness(snap) if snap else {}
        ready = all(checks.values()) if checks else None
        st = sim.status(ready if sim.alive else None)
        return SimStatusResponse(
            status=st,
            rtf=round(snap.rtf, 3) if snap and snap.rtf is not None else 0.0,
            sim_time=round(snap.sim_time, 3) if snap else 0.0,
            ready=bool(ready) and st == "running",
            readiness=checks,
            world_report=[f"[{i.level}] {i.code}: {i.message}" for i in sim.world_report.issues]
            if sim.world_report else [],
        )

    @v1.get("/health")
    def health() -> dict:
        return {"status": "ok", "version": sim2lang.__version__,
                "uptime_s": round(time.time() - _STARTED_AT, 1),
                "ros": state["bridge"] is not None}

    @v1.get("/info")
    def info() -> dict:
        env = read_runtime_env()
        return {
            "version": sim2lang.__version__,
            "display_mode": env.get("L2S_EFFECTIVE_DISPLAY_MODE"),
            "gpu": env.get("L2S_EFFECTIVE_GPU"),
            "display": env.get("DISPLAY") or None,
            "ros_distro": os.getenv("ROS_DISTRO"),
            "gz_version": os.getenv("GZ_VERSION"),
            "arch": platform.machine(),
            "projects_dir": os.getenv("L2S_PROJECTS_DIR"),
            "catalog_dir": os.getenv("L2S_CATALOG_DIR"),
            "foxglove": env.get("L2S_FOXGLOVE"),
        }

    @v1.get("/sim/status", response_model=SimStatusResponse)
    def sim_status() -> SimStatusResponse:
        return current_status()

    @v1.post("/sim/start", response_model=SimStatusResponse, status_code=202)
    def sim_start(req: SimStartRequest) -> SimStatusResponse:
        if state["bridge"]:
            state["bridge"].reset()
        try:
            sim.start(req)
        except RuntimeError as e:
            raise HTTPException(409, str(e))
        except FileNotFoundError as e:
            raise HTTPException(404, f"not found in container: {e}")
        except WorldRejected as e:
            raise HTTPException(422, {"message": "world rejected by pre-flight check",
                                      "issues": [vars(i) for i in e.report.issues]})
        return current_status()

    @v1.post("/sim/stop", response_model=SimStatusResponse)
    def sim_stop(req: Optional[SimStopRequest] = None) -> SimStatusResponse:
        sim.stop(force=bool(req and req.force))
        return current_status()

    @v1.get("/sim/logs")
    def sim_logs(tail: int = Query(200, ge=1, le=LOG_RING_LINES)) -> dict:
        return {"status": sim.status(None), "lines": sim.tail(tail)}

    app.include_router(v1)
    return app


app = create_app()
