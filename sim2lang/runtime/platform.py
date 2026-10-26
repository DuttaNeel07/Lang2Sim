"""
Host platform + Docker detection.
File: sim2lang/runtime/platform.py

Answers: which OS family are we on (linux / wsl / macos / windows), is there a
display, and can we reach a Docker daemon (and does it have the NVIDIA runtime).
"""

from __future__ import annotations

import json
import os
import platform as _platform
import shutil
import subprocess
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Literal, Optional

HostOS = Literal["linux", "wsl", "macos", "windows"]
Arch = Literal["amd64", "arm64", "unknown"]

_ARCH_ALIASES = {"x86_64": "amd64", "amd64": "amd64", "aarch64": "arm64", "arm64": "arm64"}


@dataclass(frozen=True)
class DockerInfo:
    cli_path: Optional[str] = None
    server_reachable: bool = False
    server_version: Optional[str] = None
    operating_system: Optional[str] = None  # "Docker Desktop", "Ubuntu 24.04 LTS", ...
    arch: Arch = "unknown"
    runtimes: List[str] = field(default_factory=list)
    error: Optional[str] = None

    @property
    def installed(self) -> bool:
        return self.cli_path is not None

    @property
    def is_desktop(self) -> bool:
        return bool(self.operating_system and "Docker Desktop" in self.operating_system)

    @property
    def has_nvidia_runtime(self) -> bool:
        return "nvidia" in self.runtimes


@dataclass(frozen=True)
class HostPlatform:
    os: HostOS
    arch: Arch
    env: Dict[str, str] = field(default_factory=dict)  # DISPLAY / WAYLAND_DISPLAY / XAUTHORITY
    has_wslg: bool = False
    has_x11_socket_dir: bool = False
    docker: DockerInfo = field(default_factory=DockerInfo)

    @property
    def display(self) -> Optional[str]:
        return self.env.get("DISPLAY") or None


# ---------------------------------------------------------------------------
# Pure helpers
# ---------------------------------------------------------------------------

def normalize_arch(machine: str) -> Arch:
    return _ARCH_ALIASES.get(machine.lower(), "unknown")  # type: ignore[return-value]


def classify_os(system: str, release: str, wsl_interop: bool = False) -> HostOS:
    s = system.lower()
    if s == "darwin":
        return "macos"
    if s == "windows":
        return "windows"
    if s == "linux" and (wsl_interop or "microsoft" in release.lower()):
        return "wsl"
    return "linux"


def parse_docker_info(info_json: str, cli_path: Optional[str] = None) -> DockerInfo:
    """Parse `docker info --format '{{json .}}'`."""
    try:
        d = json.loads(info_json)
    except json.JSONDecodeError as e:
        return DockerInfo(cli_path=cli_path, error=f"bad docker info output: {e}")
    if d.get("ServerErrors"):
        return DockerInfo(cli_path=cli_path, error="; ".join(d["ServerErrors"]))
    return DockerInfo(
        cli_path=cli_path,
        server_reachable=True,
        server_version=d.get("ServerVersion"),
        operating_system=d.get("OperatingSystem"),
        arch=normalize_arch(d.get("Architecture", "")),
        runtimes=sorted((d.get("Runtimes") or {}).keys()),
    )


# ---------------------------------------------------------------------------
# Probes
# ---------------------------------------------------------------------------

def detect_docker(docker: str = "docker", timeout: float = 10.0) -> DockerInfo:
    cli = shutil.which(docker)
    if not cli:
        return DockerInfo(error="docker CLI not found on PATH")
    try:
        out = subprocess.run([cli, "info", "--format", "{{json .}}"],
                             capture_output=True, text=True, timeout=timeout, check=False)
    except subprocess.TimeoutExpired:
        return DockerInfo(cli_path=cli, error="`docker info` timed out (daemon starting?)")
    except OSError as e:
        return DockerInfo(cli_path=cli, error=str(e))
    if out.returncode != 0 or not out.stdout.strip().startswith("{"):
        msg = (out.stderr or out.stdout).strip().splitlines()
        return DockerInfo(cli_path=cli, error=msg[-1] if msg else "docker daemon not reachable")
    return parse_docker_info(out.stdout, cli)


def detect(docker: str = "docker", probe_docker: bool = True) -> HostPlatform:
    host_os = classify_os(_platform.system(), _platform.release(),
                          wsl_interop=os.path.exists("/proc/sys/fs/binfmt_misc/WSLInterop"))
    env = {k: os.environ[k] for k in ("DISPLAY", "WAYLAND_DISPLAY", "XAUTHORITY", "XDG_RUNTIME_DIR")
           if os.environ.get(k)}
    return HostPlatform(
        os=host_os,
        arch=normalize_arch(_platform.machine()),
        env=env,
        has_wslg=Path("/mnt/wslg").is_dir(),
        has_x11_socket_dir=Path("/tmp/.X11-unix").is_dir(),
        docker=detect_docker(docker) if probe_docker else DockerInfo(),
    )
