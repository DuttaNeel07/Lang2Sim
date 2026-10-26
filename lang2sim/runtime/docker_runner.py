"""
Starts/stops the sim container. The user never runs Docker themselves.
File: lang2sim/runtime/docker_runner.py

`build_run_args()` is pure (RunOptions -> argv) so every display/GPU/mount
combination is unit-tested without Docker. `DockerRunner` shells out to the
docker CLI (no SDK dependency) and takes an injectable `run` for tests.

Ports are published on 127.0.0.1 only (noVNC has no password):
    9090 control API, 8765 foxglove_bridge, 6080 noVNC.
"""

from __future__ import annotations

import importlib
import json
import os
import subprocess
from dataclasses import dataclass, field
from pathlib import Path, PurePosixPath
from typing import Callable, Dict, List, Optional, Sequence

from lang2sim.runtime.display import DisplayPlan

# NB: `from lang2sim.core import config` yields the Config *instance* (core/__init__
# re-exports it under the module's name), so load the module explicitly.
cfg = importlib.import_module("lang2sim.core.config")

CONTAINER_NAME = "lang2sim-sim"
LABEL = "dev.lang2sim.role=sim"
SIM_USER = "lang2sim"  # runtime user in docker/sim/Dockerfile

# In-container ports are fixed by the image; host ports are configurable.
CONTAINER_API_PORT = 9090
CONTAINER_FOXGLOVE_PORT = 8765
CONTAINER_NOVNC_PORT = 6080

RunFn = Callable[..., "subprocess.CompletedProcess[str]"]


class DockerError(RuntimeError):
    def __init__(self, message: str, stderr: str = ""):
        super().__init__(message if not stderr else f"{message}: {stderr.strip()}")
        self.stderr = stderr


@dataclass(frozen=True)
class RunOptions:
    image: str
    plan: DisplayPlan
    projects_dir: Path
    catalog_dir: Optional[Path] = None
    src_dir: Optional[Path] = None          # dev: bind-mount the repo over the baked-in package
    host_uid: Optional[int] = None
    host_gid: Optional[int] = None
    api_port: int = CONTAINER_API_PORT
    foxglove_port: int = CONTAINER_FOXGLOVE_PORT
    novnc_port: int = CONTAINER_NOVNC_PORT
    host_env: Dict[str, str] = field(default_factory=dict)  # DISPLAY, WAYLAND_DISPLAY, XAUTHORITY
    has_dxg: bool = False
    extra_env: Dict[str, str] = field(default_factory=dict)
    name: str = CONTAINER_NAME
    shm_size: str = "2g"
    command: Sequence[str] = ("serve",)


# ---------------------------------------------------------------------------
# Pure helpers
# ---------------------------------------------------------------------------

def image_for(base_image: str, plan: DisplayPlan) -> str:
    """Pick the `-vnc` tag variant when the plan needs it.

    ghcr.io/x/lang2sim-sim:staging  + vnc -> ghcr.io/x/lang2sim-sim:staging-vnc
    """
    if plan.image_variant != "sim-vnc":
        return base_image
    name, sep, tag = base_image.rpartition(":")
    if not sep or "/" in tag:  # no tag (the ':' was a registry port)
        return f"{base_image}:latest-vnc"
    return base_image if tag.endswith("-vnc") else f"{name}:{tag}-vnc"


def host_to_container_path(host_path: Path, projects_dir: Path) -> str:
    """Map a host path under projects_dir to its path inside the container."""
    rel = Path(host_path).resolve().relative_to(Path(projects_dir).resolve())
    return str(PurePosixPath(cfg.CONTAINER_PROJECTS_DIR, *rel.parts))


def build_run_args(opts: RunOptions) -> List[str]:
    """argv after `docker` for starting the sim container (detached)."""
    plan = opts.plan
    a: List[str] = ["run", "-d", "--name", opts.name, "--label", LABEL,
                    "--shm-size", opts.shm_size]

    def env(k: str, v: object) -> None:
        a.extend(["-e", f"{k}={v}"])

    def mount(src: object, dst: str, ro: bool = False) -> None:
        a.extend(["-v", f"{src}:{dst}{':ro' if ro else ''}"])

    # Ports (localhost only)
    a.extend(["-p", f"127.0.0.1:{opts.api_port}:{CONTAINER_API_PORT}"])
    if plan.foxglove:
        a.extend(["-p", f"127.0.0.1:{opts.foxglove_port}:{CONTAINER_FOXGLOVE_PORT}"])
    if plan.mode == "vnc":
        a.extend(["-p", f"127.0.0.1:{opts.novnc_port}:{CONTAINER_NOVNC_PORT}"])

    # Identity + plan (read by docker/sim/entrypoint.sh)
    if opts.host_uid is not None:
        env("L2S_HOST_UID", opts.host_uid)
    if opts.host_gid is not None:
        env("L2S_HOST_GID", opts.host_gid)
    env("L2S_DISPLAY_MODE", plan.mode)
    env("L2S_GPU", plan.gpu)
    env("L2S_FOXGLOVE", 1 if plan.foxglove else 0)

    # Data
    mount(opts.projects_dir, cfg.CONTAINER_PROJECTS_DIR)
    if opts.catalog_dir is not None:
        mount(opts.catalog_dir, cfg.CONTAINER_CATALOG_DIR, ro=True)
    if opts.src_dir is not None:
        mount(opts.src_dir, cfg.CONTAINER_SRC_DIR, ro=True)

    # Display
    if plan.mode == "x11":
        mount("/tmp/.X11-unix", "/tmp/.X11-unix")
        env("DISPLAY", opts.host_env.get("DISPLAY", ":0"))
        xauth = opts.host_env.get("XAUTHORITY")
        if xauth:
            mount(xauth, "/tmp/.Xauthority", ro=True)
            env("XAUTHORITY", "/tmp/.Xauthority")
    elif plan.mode == "wslg":
        mount("/tmp/.X11-unix", "/tmp/.X11-unix")
        mount("/mnt/wslg", "/mnt/wslg")
        env("DISPLAY", opts.host_env.get("DISPLAY", ":0"))
        env("WAYLAND_DISPLAY", opts.host_env.get("WAYLAND_DISPLAY", "wayland-0"))
        env("XDG_RUNTIME_DIR", "/mnt/wslg/runtime-dir")
        env("PULSE_SERVER", "/mnt/wslg/PulseServer")

    # GPU
    if plan.gpu == "nvidia":
        a.extend(["--gpus", "all"])
    elif plan.gpu == "dri":
        if plan.mode == "wslg" and opts.has_dxg:
            a.extend(["--device", "/dev/dxg"])
            mount("/usr/lib/wsl", "/usr/lib/wsl", ro=True)
        else:
            a.extend(["--device", "/dev/dri"])

    for k, v in sorted(opts.extra_env.items()):
        env(k, v)

    a.append(opts.image)
    a.extend(opts.command)
    return a


def default_run_options(plan: DisplayPlan, *, image: Optional[str] = None,
                        project_src: Optional[Path] = None,
                        host_env: Optional[Dict[str, str]] = None,
                        has_dxg: bool = False) -> RunOptions:
    """RunOptions from core.config for this user (paths, ports, uid)."""
    projects = cfg.get_projects_dir()
    catalog = cfg.get_catalog_dir()
    uid = os.getuid() if hasattr(os, "getuid") else None
    gid = os.getgid() if hasattr(os, "getgid") else None
    return RunOptions(
        image=image_for(image or cfg.DEFAULT_SIM_IMAGE, plan),
        plan=plan,
        projects_dir=projects,
        catalog_dir=catalog if catalog.is_dir() else None,
        src_dir=project_src,
        host_uid=uid,
        host_gid=gid,
        api_port=cfg.DEFAULT_CONTROL_PORT,
        foxglove_port=cfg.DEFAULT_FOXGLOVE_PORT,
        novnc_port=cfg.DEFAULT_NOVNC_PORT,
        host_env=dict(host_env or {}),
        has_dxg=has_dxg,
    )


def allow_local_x11_clients() -> bool:
    """`xhost +SI:localuser:<me>`: let containers running as our UID draw on X.

    Needed on Ubuntu desktops (X/XWayland reject unknown clients by default).
    No-op where xhost or DISPLAY is missing. Returns True if it ran.
    """
    import getpass
    import shutil

    if not os.environ.get("DISPLAY") or not shutil.which("xhost"):
        return False
    try:
        subprocess.run(["xhost", f"+SI:localuser:{getpass.getuser()}"],
                       capture_output=True, timeout=5, check=False)
    except (OSError, subprocess.SubprocessError):
        return False
    return True


# ---------------------------------------------------------------------------
# Docker CLI wrapper
# ---------------------------------------------------------------------------

class DockerRunner:
    def __init__(self, docker: str = "docker", run: RunFn = subprocess.run):
        self.docker = docker
        self._run = run

    def _call(self, args: Sequence[str], check: bool = True, timeout: Optional[float] = 60,
              ) -> "subprocess.CompletedProcess[str]":
        try:
            cp = self._run([self.docker, *args], capture_output=True, text=True,
                           timeout=timeout, check=False)
        except FileNotFoundError as e:
            raise DockerError("docker CLI not found") from e
        except subprocess.TimeoutExpired as e:
            raise DockerError(f"`docker {args[0]}` timed out") from e
        if check and cp.returncode != 0:
            raise DockerError(f"`docker {' '.join(args[:2])}` failed", cp.stderr)
        return cp

    # images
    def image_present(self, image: str) -> bool:
        return self._call(["image", "inspect", image], check=False).returncode == 0

    def pull(self, image: str) -> None:
        self._call(["pull", image], timeout=None)

    def ensure_image(self, image: str) -> bool:
        """Pull if missing. Returns True if a pull happened."""
        if self.image_present(image):
            return False
        self.pull(image)
        return True

    # containers
    def inspect(self, name: str = CONTAINER_NAME) -> Optional[dict]:
        cp = self._call(["container", "inspect", name], check=False)
        if cp.returncode != 0:
            return None
        data = json.loads(cp.stdout or "[]")
        return data[0] if data else None

    def state(self, name: str = CONTAINER_NAME) -> Optional[dict]:
        """Docker's .State (Status, Running, ExitCode, Health, ...) or None if absent."""
        info = self.inspect(name)
        return info.get("State") if info else None

    def is_running(self, name: str = CONTAINER_NAME) -> bool:
        st = self.state(name)
        return bool(st and st.get("Running"))

    def start(self, opts: RunOptions, replace: bool = True) -> str:
        """Start the container; returns its id. Replaces a stale one by default."""
        if self.inspect(opts.name) is not None:
            if not replace:
                raise DockerError(f"container {opts.name} already exists")
            self.remove(opts.name)
        if opts.plan.mode == "x11":
            allow_local_x11_clients()
        cp = self._call(build_run_args(opts), timeout=120)
        return cp.stdout.strip()

    def stop(self, name: str = CONTAINER_NAME, timeout: int = 10) -> None:
        self._call(["stop", "-t", str(timeout), name], check=False, timeout=timeout + 30)

    def remove(self, name: str = CONTAINER_NAME) -> None:
        self._call(["rm", "-f", name], check=False)

    def logs(self, name: str = CONTAINER_NAME, tail: int = 200) -> str:
        cp = self._call(["logs", "--tail", str(tail), name], check=False)
        return (cp.stdout or "") + (cp.stderr or "")

    def exec(self, cmd: Sequence[str], name: str = CONTAINER_NAME,
             timeout: Optional[float] = 60, user: str = SIM_USER,
             ) -> "subprocess.CompletedProcess[str]":
        """Run a command in the container as the sim user.

        Must match the user the sim runs as: ROS 2's default (Fast DDS) shared
        memory transport is per-user, so a root `ros2 topic echo` discovers
        topics but never receives data.
        """
        return self._call(["exec", "-u", user, name, *cmd], check=False, timeout=timeout)
