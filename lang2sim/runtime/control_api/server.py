"""
Control API server. Runs INSIDE the sim container (uvicorn on :9090).
File: sim2lang/runtime/control_api/server.py

Started by docker/sim/entrypoint.sh (`serve`):
    uvicorn sim2lang.runtime.control_api.server:app --host 0.0.0.0 --port 9090

Step 2 scope:
    GET  /v1/health        liveness + version
    GET  /v1/info          what the container resolved (display, gpu, ros, gz, arch)
    GET  /v1/sim/status    SimStatusResponse
    POST /v1/sim/start     SimStartRequest -> launches the sim process
    POST /v1/sim/stop      SimStopRequest
    GET  /v1/sim/logs      tail of the sim process output (for the self-healer)

Step 3 swaps SimProcess's command for `ros2 launch <run_dir>/sim_launch.py` and
adds /v1/nav/*; Part 4 mounts routers/world.py under /v1/world.
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
from typing import Deque, Dict, List, Optional

from fastapi import APIRouter, FastAPI, HTTPException, Query

import lang2sim
from lang2sim.core.models import SimStartRequest, SimStatusResponse, SimStopRequest
from lang2sim.runtime.control_api import API_PREFIX

RUNTIME_ENV_FILE = Path(os.getenv("L2S_RUNTIME_ENV_FILE", "/tmp/l2s_runtime.env"))
LOG_RING_LINES = 2000
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


class SimProcess:
    """One simulation process group, its log ring and lifecycle state."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self.proc: Optional[subprocess.Popen] = None
        self.request: Optional[SimStartRequest] = None
        self.started_at: Optional[float] = None
        self.stopped = False
        self.log: Deque[str] = collections.deque(maxlen=LOG_RING_LINES)
        self._log_file: Optional[Path] = None

    # Overridden in Step 3 (sim_launch.py) and in tests.
    def build_command(self, req: SimStartRequest) -> List[str]:
        cmd = ["gz", "sim", "-r", "-v", "3"]
        if not req.use_gui or not os.getenv("DISPLAY"):
            cmd.append("-s")
        cmd.append(req.world_sdf_path)
        return cmd

    @property
    def status(self) -> str:
        if self.proc is None:
            return "stopped" if self.stopped else "idle"
        rc = self.proc.poll()
        if rc is None:
            # Treat the first few seconds as launching.
            return "launching" if self.started_at and time.time() - self.started_at < 3 else "running"
        if self.stopped:
            return "stopped"
        return "stopped" if rc == 0 else "error"

    def start(self, req: SimStartRequest) -> None:
        with self._lock:
            if self.proc is not None and self.proc.poll() is None:
                raise RuntimeError("simulation already running")
            for p in (req.world_sdf_path,):
                if not Path(p).exists():
                    raise FileNotFoundError(p)
            run_dir = Path(req.run_dir)
            (run_dir / "logs").mkdir(parents=True, exist_ok=True)
            self._log_file = run_dir / "logs" / "sim.log"
            self.log.clear()
            self.request = req
            self.stopped = False
            self.started_at = time.time()
            self.proc = subprocess.Popen(
                self.build_command(req),
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1,
                cwd=str(run_dir), start_new_session=True,  # own process group for clean stop
            )
            threading.Thread(target=self._pump, args=(self.proc,), daemon=True).start()

    def _pump(self, proc: subprocess.Popen) -> None:
        assert proc.stdout is not None
        with open(self._log_file, "a", buffering=1) if self._log_file else _NullFile() as fh:
            for line in proc.stdout:
                self.log.append(line.rstrip("\n"))
                fh.write(line)

    def stop(self, force: bool = False, grace_s: float = 10.0) -> None:
        with self._lock:
            proc = self.proc
            self.stopped = True
            if hasattr(os, "killpg"):
                sig_kill = getattr(signal, "SIGKILL", signal.SIGTERM)
                sig = sig_kill if force else signal.SIGINT
                try:
                    os.killpg(proc.pid, sig)
                    proc.wait(timeout=grace_s)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid, sig_kill)
                    proc.wait(timeout=5)
                except ProcessLookupError:
                    pass
            else:
                try:
                    if force:
                        proc.kill()
                    else:
                        proc.terminate()
                    proc.wait(timeout=grace_s)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    proc.wait(timeout=5)
                except ProcessLookupError:
                    pass

    def tail(self, n: int) -> List[str]:
        return list(self.log)[-n:]


class _NullFile:
    def __enter__(self):
        return self

    def __exit__(self, *a):
        return False

    def write(self, _s: str) -> None:
        pass


def create_app(sim: Optional[SimProcess] = None) -> FastAPI:
    sim = sim or SimProcess()

    @contextlib.asynccontextmanager
    async def lifespan(_app: FastAPI):
        yield
        sim.stop()  # don't leave gz/ros2 running when uvicorn exits

    app = FastAPI(title="Lang2Sim control API", version=lang2sim.__version__, lifespan=lifespan)
    app.state.sim = sim
    v1 = APIRouter(prefix=API_PREFIX)

    @v1.get("/health")
    def health() -> dict:
        return {"status": "ok", "version": lang2sim.__version__,
                "uptime_s": round(time.time() - _STARTED_AT, 1)}

    @v1.get("/info")
    def info() -> dict:
        env = read_runtime_env()
        return {
            "version": lang2sim.__version__,
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
        # rtf/sim_time come from health_monitor (Step 4); nodes/topics from Step 3.
        return SimStatusResponse(status=sim.status)

    @v1.post("/sim/start", response_model=SimStatusResponse, status_code=202)
    def sim_start(req: SimStartRequest) -> SimStatusResponse:
        try:
            sim.start(req)
        except RuntimeError as e:
            raise HTTPException(409, str(e))
        except FileNotFoundError as e:
            raise HTTPException(404, f"not found in container: {e}")
        return SimStatusResponse(status=sim.status)

    @v1.post("/sim/stop", response_model=SimStatusResponse)
    def sim_stop(req: Optional[SimStopRequest] = None) -> SimStatusResponse:
        sim.stop(force=bool(req and req.force))
        return SimStatusResponse(status=sim.status)

    @v1.get("/sim/logs")
    def sim_logs(tail: int = Query(200, ge=1, le=LOG_RING_LINES)) -> dict:
        return {"status": sim.status, "lines": sim.tail(tail)}

    try:
        from lang2sim.perturbation.routers.world import router as world_router
        v1.include_router(world_router)
    except ImportError:
        pass

    app.include_router(v1)
    return app


app = create_app()
