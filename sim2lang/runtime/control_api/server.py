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
    GET  /v1/sim/health    healthy / degraded / crashed + the reasons (health_monitor)
    GET  /v1/heal/log      self-healing actions this run (also <run>/logs/heal_log.jsonl)

    POST /v1/nav/initial_pose  NavInitialPoseRequest   (409 until the sim is ready)
    POST /v1/nav/goal          NavGoalRequest          -> NavStatus (preempts a running task)
    POST /v1/nav/waypoints     NavWaypointsRequest     -> NavStatus
    POST /v1/nav/cancel                                -> NavStatus
    GET  /v1/nav/status                                -> NavStatus
    WS   /v1/nav/feedback?hz=5  {"nav": NavStatus, "telemetry": SimStatus, "sim": {...}}
    GET  /v1/telemetry         SimStatus: map pose, velocity, RTF, render mode, health

Part 4 mounts routers/world.py under /v1/world.
"""

from __future__ import annotations

import asyncio
import collections
import contextlib
import math
import os
import platform
import signal
import subprocess
import threading
import time
from pathlib import Path
from typing import Any, Callable, Deque, Dict, List, Optional

from fastapi import (
    APIRouter,
    FastAPI,
    HTTPException,
    Query,
    WebSocket,
    WebSocketDisconnect,
)

import sim2lang
from sim2lang.core.models import (
    NavGoalRequest,
    NavInitialPoseRequest,
    NavStatus,
    NavWaypointsRequest,
    Pose2D,
    SimStartRequest,
    SimStatus,
    SimStatusResponse,
    SimStopRequest,
)
from sim2lang.runtime import launch_builder, world_check
from sim2lang.runtime.control_api import API_PREFIX
from sim2lang.runtime.health_monitor import HealthMonitor
from sim2lang.runtime.ros_bridge import readiness
from sim2lang.runtime.self_healing import NodeControl, Observation, SelfHealer, classify

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
        self.lines_total = 0                      # monotonic, unlike the ring's length
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
                self.lines_total += 1
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

    def lines_since(self, seen: int) -> tuple:
        """(new lines, new total) given how many lines were already seen."""
        total = self.lines_total
        fresh = min(max(0, total - seen), len(self.log))
        return (list(self.log)[-fresh:] if fresh else []), total


def _pose(x: float, y: float, yaw: float) -> Pose2D:
    # Clamp after rounding: round(3.14159..., 4) == 3.1416 > pi would fail validation.
    return Pose2D(x=round(x, 3), y=round(y, 3), yaw=max(-math.pi, min(math.pi, round(yaw, 4))))


HEAL_PERIOD_S = 2.0


def create_app(sim: Optional[SimProcess] = None, use_ros: bool = True, nav: Any = None,
               node_control: Optional[Callable[[Path], Any]] = None) -> FastAPI:
    """`use_ros=False` (tests) skips the rclpy bridge and the healer thread; both are also
    skipped where ROS is absent.

    `nav` injects a NavDispatcher and `node_control` a NodeControl factory (tests).
    Self-healing is on unless L2S_SELF_HEAL=0.
    """
    sim = sim or SimProcess()
    state: Dict[str, Any] = {"bridge": None, "nav": nav, "healer": None, "nodes": None,
                             "seen": 0, "localized": False}
    monitor = HealthMonitor()
    make_nodes = node_control or (lambda run_dir: NodeControl(run_dir))
    heal_enabled = os.getenv("L2S_SELF_HEAL", "1") != "0"

    @contextlib.asynccontextmanager
    async def lifespan(_app: FastAPI):
        stop = threading.Event()
        if use_ros:
            from sim2lang.runtime.ros_bridge import RosBridge
            state["bridge"] = RosBridge.try_create()
            if state["bridge"] and heal_enabled:
                def loop() -> None:
                    while not stop.wait(HEAL_PERIOD_S):
                        try:
                            heal_tick()
                        except Exception:  # noqa: BLE001 - the healer must never kill the API
                            pass
                threading.Thread(target=loop, name="l2s-heal", daemon=True).start()
        yield
        stop.set()
        if state["nodes"]:
            state["nodes"].stop_all()
        if state["nav"]:
            state["nav"].shutdown()
        sim.stop()  # don't leave gz/ros2 running when uvicorn exits
        if state["bridge"] and hasattr(state["bridge"], "stop"):
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

    def telemetry_now() -> SimStatus:
        bridge = state["bridge"]
        snap = bridge.snapshot() if bridge else None
        st = current_status()
        env = read_runtime_env()
        if env.get("L2S_EFFECTIVE_DISPLAY_MODE") == "headless":
            render = "headless"
        else:
            render = {"nvidia": "gpu", "dri": "dri"}.get(env.get("L2S_EFFECTIVE_GPU", ""), "software")
        pose = (snap.map_pose or snap.odom_pose) if snap else None
        vel = snap.velocity if snap else None
        health = monitor.evaluate(st.status, snap).health
        return SimStatus(
            is_running=st.status in ("launching", "running", "degraded"),
            sim_time=st.sim_time,
            real_time_factor=st.rtf,
            robot_pose=_pose(*pose) if pose else Pose2D(x=0.0, y=0.0, yaw=0.0),
            robot_velocity=_pose(*vel) if vel else None,
            render_mode=render,
            health_status=health,
        )

    def dispatcher():
        if state["nav"] is None:
            from sim2lang.runtime.nav2_dispatcher import NavDispatcher
            state["nav"] = NavDispatcher()
        return state["nav"]

    def require_ready() -> None:
        st = current_status()
        if not st.ready:
            failed = [k for k, v in st.readiness.items() if not v]
            raise HTTPException(409, f"simulation is {st.status}, not ready"
                                     + (f" (waiting for: {', '.join(failed)})" if failed else ""))

    def heal_tick() -> List[dict]:
        """One supervisor pass: observe, classify, recover. Called every HEAL_PERIOD_S."""
        healer = state["healer"]
        if healer is None or not sim.alive:
            return []
        bridge = state["bridge"]
        snap = bridge.snapshot() if bridge else None
        if snap is not None and snap.map_pose is not None:
            state["localized"] = True
        lines, state["seen"] = sim.lines_since(state["seen"])
        obs = Observation(lines=lines, snap=snap, alive=state["nodes"].alive(),
                          nav=state["nav"].status() if state["nav"] else None,
                          sim_running=True, was_localized=state["localized"] or sim.was_ready)
        return healer.handle(classify(obs))

    def new_healer(req: SimStartRequest) -> None:
        if state["nodes"]:
            state["nodes"].stop_all()
        nodes = make_nodes(Path(req.run_dir))

        def current_goal():
            st = state["nav"].status() if state["nav"] else None
            return st.target_pose if st and (st.is_active or st.status == "failed") else None

        state.update(nodes=nodes, seen=0, localized=False, healer=SelfHealer(
            Path(req.run_dir), restart_node=nodes.restart, current_goal=current_goal,
            resend_goal=lambda p: bool(dispatcher().resume_with(p)),
            map_yaml=Path(req.map_yaml_path)))

    app.state.heal_tick = heal_tick

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
        monitor.reset()
        try:
            sim.start(req)
        except RuntimeError as e:
            raise HTTPException(409, str(e))
        except FileNotFoundError as e:
            raise HTTPException(404, f"not found in container: {e}")
        except WorldRejected as e:
            raise HTTPException(422, {"message": "world rejected by pre-flight check",
                                      "issues": [vars(i) for i in e.report.issues]})
        new_healer(req)
        return current_status()

    @v1.post("/sim/stop", response_model=SimStatusResponse)
    def sim_stop(req: Optional[SimStopRequest] = None) -> SimStatusResponse:
        sim.stop(force=bool(req and req.force))
        return current_status()

    @v1.get("/sim/health")
    def sim_health() -> dict:
        bridge = state["bridge"]
        r = monitor.evaluate(current_status().status, bridge.snapshot() if bridge else None)
        return {"health": r.health, "issues": r.issues, "rtf": r.rtf, "sim_time": r.sim_time,
                "readiness": r.readiness}

    @v1.get("/heal/log")
    def heal_log(tail: int = Query(100, ge=1, le=1000)) -> dict:
        healer = state["healer"]
        return {"enabled": heal_enabled, "events": healer.events[-tail:] if healer else []}

    @v1.get("/sim/logs")
    def sim_logs(tail: int = Query(200, ge=1, le=LOG_RING_LINES)) -> dict:
        return {"status": sim.status(None), "lines": sim.tail(tail)}

    # --- navigation ----------------------------------------------------------
    @v1.post("/nav/initial_pose", response_model=NavStatus, status_code=202)
    def nav_initial_pose(req: NavInitialPoseRequest) -> NavStatus:
        require_ready()
        dispatcher().set_initial_pose(req.pose)
        return dispatcher().status()

    @v1.post("/nav/goal", response_model=NavStatus, status_code=202)
    def nav_goal(req: NavGoalRequest) -> NavStatus:
        require_ready()
        return dispatcher().go_to(req.goal_pose, req.timeout_seconds)

    @v1.post("/nav/waypoints", response_model=NavStatus, status_code=202)
    def nav_waypoints(req: NavWaypointsRequest) -> NavStatus:
        require_ready()
        if not req.waypoints:
            raise HTTPException(422, "waypoints must not be empty")
        return dispatcher().follow(req.waypoints, req.timeout_seconds)

    @v1.post("/nav/cancel", response_model=NavStatus)
    def nav_cancel() -> NavStatus:
        return state["nav"].cancel() if state["nav"] else NavStatus()

    @v1.get("/nav/status", response_model=NavStatus)
    def nav_status() -> NavStatus:
        return state["nav"].status() if state["nav"] else NavStatus()

    @v1.get("/telemetry", response_model=SimStatus)
    def telemetry() -> SimStatus:
        return telemetry_now()

    @v1.websocket("/nav/feedback")
    async def nav_feedback(ws: WebSocket, hz: float = 5.0) -> None:
        await ws.accept()
        period = 1.0 / max(0.5, min(hz, 20.0))
        try:
            while True:
                st = current_status()
                await ws.send_json({
                    "nav": nav_status().model_dump(mode="json"),
                    "telemetry": telemetry_now().model_dump(mode="json"),
                    "sim": {"status": st.status, "ready": st.ready, "rtf": st.rtf},
                })
                await asyncio.sleep(period)
        except (WebSocketDisconnect, RuntimeError):
            pass

    app.include_router(v1)
    return app


app = create_app()
