"""
Display + GPU plan: how the user will see the simulation on this host.
File: lang2sim/runtime/display.py

    Ubuntu        -> x11       (nvidia | dri | software)
    Windows 11    -> wslg      (run from inside WSL2; d3d12 via /dev/dxg)
    macOS         -> headless + Foxglove   (decision 3; prefer="vnc" for real GUI)
    Windows native-> vnc
    no display    -> headless + Foxglove   (decision 4: headless only when no display)

Low VRAM (< 2 GB) means software rendering with the GUI still on, never `-s`.
The plan is pure data; docker_runner turns it into `docker run` flags and the
entrypoint reads the same values from L2S_DISPLAY_MODE / L2S_GPU / L2S_FOXGLOVE.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Literal, Optional, Tuple

from lang2sim.runtime.hardware_probe import LOW_VRAM_MB, HardwareInfo
from lang2sim.runtime.platform import HostPlatform

DisplayMode = Literal["x11", "wslg", "vnc", "headless"]
GpuMode = Literal["nvidia", "dri", "software"]
ImageVariant = Literal["sim", "sim-vnc"]

DISPLAY_MODES: Tuple[str, ...] = ("x11", "wslg", "vnc", "headless")


@dataclass(frozen=True)
class DisplayPlan:
    mode: DisplayMode
    gpu: GpuMode
    foxglove: bool
    reasons: Tuple[str, ...] = field(default_factory=tuple)

    @property
    def image_variant(self) -> ImageVariant:
        return "sim-vnc" if self.mode == "vnc" else "sim"

    @property
    def has_gui(self) -> bool:
        return self.mode != "headless"

    @property
    def render_mode(self) -> str:
        """Value for core.models.SimStatus.render_mode."""
        if self.mode == "headless":
            return "headless"
        return {"nvidia": "gpu", "dri": "dri", "software": "software"}[self.gpu]

    def describe(self) -> str:
        view = {
            "x11": "Gazebo/RViz windows on your desktop (X11)",
            "wslg": "Gazebo/RViz windows via WSLg",
            "vnc": "Gazebo/RViz in your browser via noVNC",
            "headless": "no GUI; view in Foxglove",
        }[self.mode]
        extra = " + Foxglove" if self.foxglove and self.mode != "headless" else ""
        return f"{view}{extra} [render: {self.gpu}]"


def _gpu_for_linux_host(platform: HostPlatform, hw: HardwareInfo,
                        reasons: List[str]) -> GpuMode:
    nv = hw.nvidia
    if nv is not None:
        if not platform.docker.has_nvidia_runtime:
            reasons.append("NVIDIA GPU found but Docker has no nvidia runtime "
                           "(install nvidia-container-toolkit)")
        elif nv.vram_total_mb is not None and nv.vram_total_mb < LOW_VRAM_MB:
            reasons.append(f"NVIDIA VRAM {nv.vram_total_mb} MB < {LOW_VRAM_MB} MB: software rendering")
            return "software"
        else:
            reasons.append(f"NVIDIA GPU: {nv.name or 'detected'}")
            return "nvidia"
    if hw.dri_render_nodes:
        reasons.append(f"DRI render node {hw.dri_render_nodes[0]}")
        return "dri"
    reasons.append("no usable GPU: software rendering (llvmpipe)")
    return "software"


def choose_display(platform: HostPlatform, hw: HardwareInfo,
                   prefer: Optional[DisplayMode] = None,
                   force_software: bool = False,
                   foxglove: Optional[bool] = None) -> DisplayPlan:
    """Pick the display/GPU plan for this host. `prefer` overrides the mode."""
    reasons: List[str] = []
    if prefer is not None and prefer not in DISPLAY_MODES:
        raise ValueError(f"unknown display mode {prefer!r}; expected one of {DISPLAY_MODES}")

    mode: DisplayMode
    gpu: GpuMode = "software"

    if prefer:
        mode = prefer
        reasons.append(f"display mode forced to {prefer}")
    elif platform.os == "wsl" and platform.has_wslg:
        mode = "wslg"
        reasons.append("WSL2 with WSLg")
    elif platform.os == "linux" and platform.display and platform.has_x11_socket_dir:
        mode = "x11"
        reasons.append(f"X11 display {platform.display}")
    elif platform.os == "macos":
        mode = "headless"
        reasons.append("macOS: Docker has no GPU/X11 passthrough; Foxglove by default "
                       "(use --display vnc for the real Gazebo GUI)")
    elif platform.os == "windows":
        mode = "vnc"
        reasons.append("Windows outside WSL2: noVNC (run from WSL2 for WSLg)")
    else:
        mode = "headless"
        reasons.append("no display found")

    if mode == "x11":
        gpu = _gpu_for_linux_host(platform, hw, reasons)
    elif mode == "wslg":
        gpu = "dri" if hw.has_dxg else "software"
        reasons.append("WSL GPU via /dev/dxg (d3d12)" if hw.has_dxg else "no /dev/dxg: software rendering")
    else:
        gpu = "software"

    if force_software and gpu != "software":
        reasons.append("software rendering forced")
        gpu = "software"

    if foxglove is None:
        foxglove = mode == "headless" or platform.os == "macos"

    return DisplayPlan(mode=mode, gpu=gpu, foxglove=foxglove, reasons=tuple(reasons))
