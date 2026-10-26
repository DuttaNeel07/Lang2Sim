"""
SimSession: host-side "open this run folder and show it" in one object.
File: sim2lang/runtime/session.py

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
from typing import Callable, Dict, Optional

from sim2lang.core.models import SimStatusResponse
from sim2lang.runtime.control_api.client import ControlClient
from sim2lang.runtime.display import DisplayPlan
from sim2lang.runtime.docker_runner import DockerRunner, RunOptions, default_run_options
from sim2lang.runtime.run_folder import load_manifest, to_start_request

cfg = importlib.import_module("sim2lang.core.config")

FIXTURES_DIR = Path(__file__).resolve().parents[2] / "catalog" / "fixtures"
ProgressFn = Callable[[SimStatusResponse, float], None]


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
