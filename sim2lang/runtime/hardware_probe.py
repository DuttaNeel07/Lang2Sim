"""
Host hardware introspection: GPUs, VRAM, RAM, render nodes.
File: sim2lang/runtime/hardware_probe.py

Parsing is split from probing so the parsers are unit-testable without the
hardware. `probe()` never raises; missing tools just mean less information.
"""

from __future__ import annotations

import glob
import os
import platform as _platform
import shutil
import subprocess
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Literal, Optional

GpuVendor = Literal["nvidia", "intel", "amd", "apple", "unknown"]

# Decision 4: below this much VRAM we render in software (GUI stays on).
LOW_VRAM_MB = 2048

_PCI_VENDORS = {"0x10de": "nvidia", "0x8086": "intel", "0x1002": "amd"}


@dataclass(frozen=True)
class GpuInfo:
    vendor: GpuVendor
    name: str = ""
    vram_total_mb: Optional[int] = None
    vram_free_mb: Optional[int] = None


@dataclass(frozen=True)
class HardwareInfo:
    cpu_count: int = 1
    mem_total_mb: Optional[int] = None
    gpus: List[GpuInfo] = field(default_factory=list)
    dri_render_nodes: List[str] = field(default_factory=list)
    has_dxg: bool = False  # WSL2 GPU paravirtualisation (/dev/dxg)

    @property
    def nvidia(self) -> Optional[GpuInfo]:
        return next((g for g in self.gpus if g.vendor == "nvidia"), None)

    @property
    def max_vram_mb(self) -> Optional[int]:
        vals = [g.vram_total_mb for g in self.gpus if g.vram_total_mb is not None]
        return max(vals) if vals else None


# ---------------------------------------------------------------------------
# Parsers (pure)
# ---------------------------------------------------------------------------

def parse_nvidia_smi(csv_text: str) -> List[GpuInfo]:
    """Parse `nvidia-smi --query-gpu=name,memory.total,memory.free --format=csv,noheader,nounits`."""
    gpus: List[GpuInfo] = []
    for line in csv_text.strip().splitlines():
        parts = [p.strip() for p in line.split(",")]
        if len(parts) < 3:
            continue
        name, total, free = parts[0], parts[1], parts[2]
        gpus.append(GpuInfo("nvidia", name, _to_int(total), _to_int(free)))
    return gpus


def parse_meminfo(text: str) -> Optional[int]:
    """MemTotal from /proc/meminfo, in MB."""
    for line in text.splitlines():
        if line.startswith("MemTotal:"):
            kb = _to_int(line.split()[1])
            return kb // 1024 if kb is not None else None
    return None


def vendor_from_pci_id(vendor_id: str) -> GpuVendor:
    return _PCI_VENDORS.get(vendor_id.strip().lower(), "unknown")  # type: ignore[return-value]


def _to_int(s: str) -> Optional[int]:
    try:
        return int(float(s))
    except (TypeError, ValueError):
        return None


# ---------------------------------------------------------------------------
# Probes
# ---------------------------------------------------------------------------

def _run(cmd: List[str], timeout: float = 5.0) -> Optional[str]:
    if not shutil.which(cmd[0]):
        return None
    try:
        out = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout, check=False)
    except (OSError, subprocess.SubprocessError):
        return None
    return out.stdout if out.returncode == 0 else None


def _probe_nvidia() -> List[GpuInfo]:
    out = _run(["nvidia-smi", "--query-gpu=name,memory.total,memory.free",
                "--format=csv,noheader,nounits"])
    return parse_nvidia_smi(out) if out else []


def _probe_dri() -> List[GpuInfo]:
    gpus: List[GpuInfo] = []
    for node in sorted(glob.glob("/sys/class/drm/renderD*/device/vendor")):
        try:
            vendor = vendor_from_pci_id(Path(node).read_text())
        except OSError:
            continue
        if vendor != "nvidia":  # NVIDIA is covered (with VRAM) by nvidia-smi
            gpus.append(GpuInfo(vendor))
    return gpus


def _probe_mem_mb() -> Optional[int]:
    try:
        return parse_meminfo(Path("/proc/meminfo").read_text())
    except OSError:
        pass
    out = _run(["sysctl", "-n", "hw.memsize"])  # macOS
    if out and out.strip().isdigit():
        return int(out.strip()) // (1024 * 1024)
    return None


def probe() -> HardwareInfo:
    """Best-effort snapshot of this host. Never raises."""
    gpus = _probe_nvidia() + _probe_dri()
    if _platform.system() == "Darwin" and _platform.machine() == "arm64":
        gpus.append(GpuInfo("apple", "Apple Silicon (not passed through to Docker)"))
    return HardwareInfo(
        cpu_count=os.cpu_count() or 1,
        mem_total_mb=_probe_mem_mb(),
        gpus=gpus,
        dri_render_nodes=sorted(glob.glob("/dev/dri/renderD*")),
        has_dxg=os.path.exists("/dev/dxg"),
    )
