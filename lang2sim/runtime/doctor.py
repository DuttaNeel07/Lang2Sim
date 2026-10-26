"""
`lang2sim doctor`: can this machine run the simulation, and how will it look?
File: sim2lang/runtime/doctor.py

Part 2 wires the CLI subcommand to `run_checks()` / `format_report()`.
Standalone:  python -m sim2lang.runtime.doctor [--json] [--display vnc] [--api]
"""

from __future__ import annotations

import argparse
import asyncio
import importlib
import json
import shutil
import socket
import sys
from dataclasses import asdict, dataclass
from typing import List, Literal, Optional

from lang2sim.runtime import hardware_probe
from lang2sim.runtime import platform as host_platform
from lang2sim.runtime.display import DISPLAY_MODES, DisplayPlan, choose_display
from lang2sim.runtime.docker_runner import CONTAINER_NAME, DockerRunner, image_for

# NB: `from lang2sim.core import config` yields the Config *instance* (core/__init__
# re-exports it under the module's name), so load the module explicitly.
cfg = importlib.import_module("lang2sim.core.config")

Status = Literal["ok", "warn", "fail"]

MIN_DISK_GB = 10
MIN_RAM_MB = 8 * 1024


@dataclass(frozen=True)
class CheckResult:
    name: str
    status: Status
    detail: str
    fix: Optional[str] = None


def _port_free(port: int) -> bool:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            s.bind(("127.0.0.1", port))
        except OSError:
            return False
    return True


def run_checks(plat: Optional[host_platform.HostPlatform] = None,
               hw: Optional[hardware_probe.HardwareInfo] = None,
               prefer_display: Optional[str] = None,
               image: Optional[str] = None,
               runner: Optional[DockerRunner] = None,
               check_api: bool = False) -> List[CheckResult]:
    plat = plat or host_platform.detect()
    hw = hw or hardware_probe.probe()
    runner = runner or DockerRunner()
    r: List[CheckResult] = []
    d = plat.docker

    # Docker
    if not d.installed:
        fix = {"macos": "Install Docker Desktop: https://docs.docker.com/desktop/setup/install/mac-install/",
               "windows": "Install Docker Desktop with the WSL 2 backend",
               "wsl": "Install Docker Desktop on Windows and enable WSL integration for this distro",
               }.get(plat.os, "Install Docker Engine: https://docs.docker.com/engine/install/ubuntu/")
        r.append(CheckResult("docker", "fail", "docker CLI not found", fix))
    elif not d.server_reachable:
        r.append(CheckResult("docker", "fail", d.error or "daemon not reachable",
                             "Start Docker Desktop" if plat.os in ("macos", "windows", "wsl")
                             else "sudo systemctl start docker && sudo usermod -aG docker $USER"))
    else:
        r.append(CheckResult("docker", "ok", f"{d.operating_system or 'docker'} {d.server_version} ({d.arch})"))
        if plat.arch != "unknown" and d.arch != "unknown" and plat.arch != d.arch:
            r.append(CheckResult("docker arch", "warn",
                                 f"host is {plat.arch} but daemon is {d.arch}; image runs emulated (slow)"))

    # Display plan
    plan: Optional[DisplayPlan] = None
    try:
        plan = choose_display(plat, hw, prefer=prefer_display)  # type: ignore[arg-type]
        r.append(CheckResult("display", "ok", plan.describe() + " — " + "; ".join(plan.reasons)))
    except ValueError as e:
        r.append(CheckResult("display", "fail", str(e), f"use one of {', '.join(DISPLAY_MODES)}"))

    if plan and plan.mode == "x11" and plat.display:
        r.append(CheckResult("x11", "ok", f"DISPLAY={plat.display}",
                             None))
        if not shutil.which("xhost"):
            r.append(CheckResult("xhost", "warn", "xhost not found; container may be refused by X",
                                 "sudo apt install x11-xserver-utils"))
    if plan and plan.mode == "wslg" and not hw.has_dxg:
        r.append(CheckResult("wsl gpu", "warn", "/dev/dxg missing: software rendering",
                             "Update the Windows GPU driver and `wsl --update`"))
    nv = hw.nvidia
    if nv and d.server_reachable and not d.has_nvidia_runtime:
        r.append(CheckResult("nvidia toolkit", "warn", "NVIDIA GPU but Docker has no nvidia runtime",
                             "Install nvidia-container-toolkit, then `sudo nvidia-ctk runtime configure "
                             "--runtime=docker && sudo systemctl restart docker`"))

    # Image
    if plan and d.server_reachable:
        img = image_for(image or cfg.DEFAULT_SIM_IMAGE, plan)
        try:
            present = runner.image_present(img)
        except Exception as e:  # noqa: BLE001 - doctor must not crash
            present, err = False, str(e)
        else:
            err = ""
        if present:
            r.append(CheckResult("image", "ok", img))
        else:
            r.append(CheckResult("image", "warn", f"{img} not pulled yet {err}".strip(),
                                 f"docker pull {img}  (~1.5 GB; first `lang2sim` run does this)"))

    # Ports
    ours = False
    try:
        ours = d.server_reachable and runner.is_running(CONTAINER_NAME)
    except Exception:  # noqa: BLE001
        pass
    ports = {"control API": cfg.DEFAULT_CONTROL_PORT}
    if plan and plan.foxglove:
        ports["foxglove"] = cfg.DEFAULT_FOXGLOVE_PORT
    if plan and plan.mode == "vnc":
        ports["noVNC"] = cfg.DEFAULT_NOVNC_PORT
    for label, port in ports.items():
        if _port_free(port):
            r.append(CheckResult(f"port {port}", "ok", f"{label} port free"))
        elif ours:
            r.append(CheckResult(f"port {port}", "ok", f"{label} served by running {CONTAINER_NAME}"))
        else:
            r.append(CheckResult(f"port {port}", "fail", f"{label} port {port} in use",
                                 f"free it or set L2S_{'API' if label == 'control API' else label.upper()}_PORT"))

    # Resources
    try:
        free_gb = shutil.disk_usage(cfg.get_projects_dir()).free / 1e9
        r.append(CheckResult("disk", "ok" if free_gb >= MIN_DISK_GB else "warn",
                             f"{free_gb:.0f} GB free in {cfg.get_projects_dir()}"))
    except OSError as e:
        r.append(CheckResult("disk", "warn", str(e)))
    if hw.mem_total_mb:
        r.append(CheckResult("memory", "ok" if hw.mem_total_mb >= MIN_RAM_MB else "warn",
                             f"{hw.mem_total_mb // 1024} GB RAM"
                             + ("" if hw.mem_total_mb >= MIN_RAM_MB else " (Gazebo + Nav2 want 8 GB+)")))

    # Live API (optional)
    if check_api:
        r.append(asyncio.run(_check_api()))
    return r


async def _check_api() -> CheckResult:
    from lang2sim.runtime.control_api.client import ControlAPIError, ControlClient

    try:
        async with ControlClient(timeout=3) as api:
            h = await api.health()
            i = await api.info()
        return CheckResult("control API", "ok",
                           f"v{h.get('version')} display={i.get('display_mode')} gpu={i.get('gpu')}")
    except ControlAPIError as e:
        return CheckResult("control API", "warn", str(e), "Start the sim (it boots the container)")


def format_report(results: List[CheckResult]) -> str:
    icon = {"ok": "✓", "warn": "!", "fail": "✗"}
    lines = []
    for c in results:
        lines.append(f" {icon[c.status]} {c.name:<16} {c.detail}")
        if c.fix and c.status != "ok":
            lines.append(f"   {'':<16} fix: {c.fix}")
    worst = "fail" if any(c.status == "fail" for c in results) else \
            "warn" if any(c.status == "warn" for c in results) else "ok"
    lines.append({"ok": "\nReady to simulate.", "warn": "\nUsable, with warnings.",
                  "fail": "\nNot ready: fix the ✗ items above."}[worst])
    return "\n".join(lines)


def exit_code(results: List[CheckResult]) -> int:
    return 1 if any(c.status == "fail" for c in results) else 0


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(prog="lang2sim doctor")
    ap.add_argument("--json", action="store_true")
    ap.add_argument("--display", choices=DISPLAY_MODES)
    ap.add_argument("--image")
    ap.add_argument("--api", action="store_true", help="also query a running control API")
    a = ap.parse_args(argv)
    res = run_checks(prefer_display=a.display, image=a.image, check_api=a.api)
    print(json.dumps([asdict(c) for c in res], indent=2) if a.json else format_report(res))
    return exit_code(res)


if __name__ == "__main__":
    sys.exit(main())
