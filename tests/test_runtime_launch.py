"""
Part 3 runtime unit tests (no Docker, no ROS). Must stay 100% green in CI.
File: tests/test_runtime_launch.py

Docker integration (real container) lives in tests/integration/ and runs locally.
"""

from __future__ import annotations

import json
import subprocess
from pathlib import Path

import pytest

from sim2lang.runtime import doctor
from sim2lang.runtime.display import DisplayPlan, choose_display
from sim2lang.runtime.docker_runner import (
    DockerRunner,
    RunOptions,
    build_run_args,
    host_to_container_path,
    image_for,
)
from sim2lang.runtime.hardware_probe import (
    GpuInfo,
    HardwareInfo,
    parse_meminfo,
    parse_nvidia_smi,
    vendor_from_pci_id,
)
from sim2lang.runtime.platform import (
    DockerInfo,
    HostPlatform,
    classify_os,
    normalize_arch,
    parse_docker_info,
)

IMG = "ghcr.io/duttaneel07/lang2sim-sim:staging"


# ---------------------------------------------------------------------------
# fixtures
# ---------------------------------------------------------------------------

def docker_ok(nvidia: bool = False, desktop: bool = False, arch: str = "amd64") -> DockerInfo:
    return DockerInfo(cli_path="/usr/bin/docker", server_reachable=True, server_version="27.0",
                      operating_system="Docker Desktop" if desktop else "Ubuntu 24.04 LTS",
                      arch=arch, runtimes=["nvidia", "runc"] if nvidia else ["runc"])  # type: ignore[arg-type]


def ubuntu(nvidia_runtime: bool = False, display: bool = True) -> HostPlatform:
    return HostPlatform(os="linux", arch="amd64",
                        env={"DISPLAY": ":0"} if display else {},
                        has_x11_socket_dir=display, docker=docker_ok(nvidia_runtime))


def mac() -> HostPlatform:
    return HostPlatform(os="macos", arch="arm64", docker=docker_ok(desktop=True, arch="arm64"))


def wsl() -> HostPlatform:
    return HostPlatform(os="wsl", arch="amd64", env={"DISPLAY": ":0", "WAYLAND_DISPLAY": "wayland-0"},
                        has_wslg=True, has_x11_socket_dir=True, docker=docker_ok(desktop=True))


NV_8G = HardwareInfo(gpus=[GpuInfo("nvidia", "RTX 3070", 8192, 7000)], dri_render_nodes=["/dev/dri/renderD128"])
NV_1G = HardwareInfo(gpus=[GpuInfo("nvidia", "MX150", 1024, 900)])
INTEL = HardwareInfo(gpus=[GpuInfo("intel")], dri_render_nodes=["/dev/dri/renderD128"])
NONE = HardwareInfo()


# ---------------------------------------------------------------------------
# hardware_probe / platform parsers
# ---------------------------------------------------------------------------

def test_parse_nvidia_smi():
    gpus = parse_nvidia_smi("NVIDIA GeForce RTX 3070, 8192, 7400\nTesla T4, 15360, 15000\n")
    assert [g.name for g in gpus] == ["NVIDIA GeForce RTX 3070", "Tesla T4"]
    assert gpus[0].vram_total_mb == 8192 and gpus[1].vram_free_mb == 15000
    assert parse_nvidia_smi("") == []


def test_parse_meminfo_and_vendor():
    assert parse_meminfo("MemTotal:       16303428 kB\nMemFree: 1 kB\n") == 15921
    assert parse_meminfo("nothing") is None
    assert vendor_from_pci_id("0x8086\n") == "intel"
    assert vendor_from_pci_id("0x1002") == "amd"
    assert vendor_from_pci_id("0xffff") == "unknown"


def test_classify_os_and_arch():
    assert classify_os("Darwin", "24.0") == "macos"
    assert classify_os("Linux", "5.15.153.1-microsoft-standard-WSL2") == "wsl"
    assert classify_os("Linux", "6.8.0-generic") == "linux"
    assert classify_os("Windows", "11") == "windows"
    assert normalize_arch("x86_64") == "amd64" and normalize_arch("aarch64") == "arm64"


def test_parse_docker_info():
    info = parse_docker_info(json.dumps({"ServerVersion": "27.3.1", "OperatingSystem": "Docker Desktop",
                                         "Architecture": "aarch64",
                                         "Runtimes": {"runc": {}, "nvidia": {}}}), "/x/docker")
    assert info.server_reachable and info.is_desktop and info.has_nvidia_runtime
    assert info.arch == "arm64"
    bad = parse_docker_info(json.dumps({"ServerErrors": ["Cannot connect to the Docker daemon"]}))
    assert not bad.server_reachable and "Cannot connect" in (bad.error or "")


# ---------------------------------------------------------------------------
# display plan (decisions 3 and 4)
# ---------------------------------------------------------------------------

def test_ubuntu_nvidia_with_toolkit():
    p = choose_display(ubuntu(nvidia_runtime=True), NV_8G)
    assert (p.mode, p.gpu, p.foxglove) == ("x11", "nvidia", False)
    assert p.render_mode == "gpu" and p.image_variant == "sim"


def test_ubuntu_nvidia_without_toolkit_falls_back_to_dri():
    p = choose_display(ubuntu(nvidia_runtime=False), NV_8G)
    assert p.gpu == "dri"
    assert any("nvidia-container-toolkit" in r for r in p.reasons)


def test_low_vram_is_software_with_gui_on():
    p = choose_display(ubuntu(nvidia_runtime=True), NV_1G)
    assert p.mode == "x11" and p.gpu == "software" and p.has_gui


def test_ubuntu_intel_dri_and_no_gpu():
    assert choose_display(ubuntu(), INTEL).gpu == "dri"
    assert choose_display(ubuntu(), NONE).gpu == "software"


def test_macos_defaults_to_foxglove_and_vnc_on_request():
    p = choose_display(mac(), NONE)
    assert (p.mode, p.gpu, p.foxglove) == ("headless", "software", True)
    v = choose_display(mac(), NONE, prefer="vnc")
    assert v.mode == "vnc" and v.image_variant == "sim-vnc" and v.foxglove


def test_wsl_uses_wslg_and_dxg():
    hw = HardwareInfo(has_dxg=True)
    p = choose_display(wsl(), hw)
    assert (p.mode, p.gpu) == ("wslg", "dri")
    assert choose_display(wsl(), NONE).gpu == "software"


def test_headless_only_when_no_display():
    p = choose_display(ubuntu(display=False), NV_8G)
    assert p.mode == "headless" and p.foxglove and p.render_mode == "headless"


def test_force_software_and_bad_mode():
    assert choose_display(ubuntu(True), NV_8G, force_software=True).gpu == "software"
    with pytest.raises(ValueError):
        choose_display(ubuntu(), NONE, prefer="vr")  # type: ignore[arg-type]


# ---------------------------------------------------------------------------
# docker_runner.build_run_args (pure)
# ---------------------------------------------------------------------------

def _opts(plan: DisplayPlan, tmp_path: Path, **kw) -> RunOptions:
    base = dict(image=IMG, plan=plan, projects_dir=tmp_path / "projects", catalog_dir=tmp_path / "catalog",
                host_uid=1000, host_gid=1000, host_env={"DISPLAY": ":1", "XAUTHORITY": "/home/u/.Xauthority"})
    base.update(kw)
    return RunOptions(**base)


def _pairs(args, flag):
    return [args[i + 1] for i, a in enumerate(args) if a == flag]


def test_image_for():
    vnc = DisplayPlan("vnc", "software", False)
    x11 = DisplayPlan("x11", "dri", False)
    assert image_for(IMG, vnc) == IMG + "-vnc"
    assert image_for(IMG + "-vnc", vnc) == IMG + "-vnc"
    assert image_for(IMG, x11) == IMG
    assert image_for("localhost:5000/lang2sim-sim", vnc) == "localhost:5000/lang2sim-sim:latest-vnc"


def test_run_args_x11_nvidia(tmp_path):
    args = build_run_args(_opts(DisplayPlan("x11", "nvidia", False), tmp_path))
    assert args[:2] == ["run", "-d"]
    assert args[-2:] == [IMG, "serve"]
    assert "--gpus" in args and "--device" not in args
    assert _pairs(args, "-p") == ["127.0.0.1:9090:9090"]  # no foxglove / vnc ports
    env = _pairs(args, "-e")
    for e in ("L2S_HOST_UID=1000", "L2S_DISPLAY_MODE=x11", "L2S_GPU=nvidia", "L2S_FOXGLOVE=0",
              "DISPLAY=:1", "XAUTHORITY=/tmp/.Xauthority"):
        assert e in env
    vols = _pairs(args, "-v")
    assert "/tmp/.X11-unix:/tmp/.X11-unix" in vols
    assert f"{tmp_path / 'projects'}:/lang2sim/projects" in vols
    assert f"{tmp_path / 'catalog'}:/lang2sim/catalog:ro" in vols


def test_run_args_dri_and_dev_src(tmp_path):
    args = build_run_args(_opts(DisplayPlan("x11", "dri", False), tmp_path, src_dir=tmp_path / "repo"))
    assert _pairs(args, "--device") == ["/dev/dri"]
    assert f"{tmp_path / 'repo'}:/opt/lang2sim/src:ro" in _pairs(args, "-v")


def test_run_args_vnc_and_headless_ports(tmp_path):
    vnc = build_run_args(_opts(DisplayPlan("vnc", "software", True), tmp_path))
    assert set(_pairs(vnc, "-p")) == {"127.0.0.1:9090:9090", "127.0.0.1:8765:8765", "127.0.0.1:6080:6080"}
    assert not any(v.startswith("/tmp/.X11-unix") for v in _pairs(vnc, "-v"))
    hl = build_run_args(_opts(DisplayPlan("headless", "software", True), tmp_path, api_port=19090))
    assert set(_pairs(hl, "-p")) == {"127.0.0.1:19090:9090", "127.0.0.1:8765:8765"}
    assert "L2S_FOXGLOVE=1" in _pairs(hl, "-e")


def test_run_args_wslg_dxg(tmp_path):
    args = build_run_args(_opts(DisplayPlan("wslg", "dri", False), tmp_path, has_dxg=True))
    assert _pairs(args, "--device") == ["/dev/dxg"]
    vols = _pairs(args, "-v")
    assert "/mnt/wslg:/mnt/wslg" in vols and "/usr/lib/wsl:/usr/lib/wsl:ro" in vols
    assert "XDG_RUNTIME_DIR=/mnt/wslg/runtime-dir" in _pairs(args, "-e")


def test_host_to_container_path(tmp_path):
    proj = tmp_path / "projects"
    run = proj / "demo" / "runs" / "20261026_150000"
    run.mkdir(parents=True)
    assert host_to_container_path(run, proj) == "/lang2sim/projects/demo/runs/20261026_150000"
    with pytest.raises(ValueError):
        host_to_container_path(tmp_path / "elsewhere", proj)


# ---------------------------------------------------------------------------
# DockerRunner with a fake CLI
# ---------------------------------------------------------------------------

class FakeDocker:
    def __init__(self, existing: bool = True):
        self.calls = []
        self.existing = existing

    def __call__(self, argv, **kw):
        self.calls.append(argv[1:])
        sub = argv[1]
        if sub == "container" and argv[2] == "inspect":
            if self.existing:
                return subprocess.CompletedProcess(argv, 0, json.dumps([{"State": {"Running": True}}]), "")
            return subprocess.CompletedProcess(argv, 1, "", "No such container")
        if sub == "run":
            return subprocess.CompletedProcess(argv, 0, "abc123\n", "")
        return subprocess.CompletedProcess(argv, 0, "", "")


def test_runner_start_replaces_stale_container(tmp_path):
    fake = FakeDocker(existing=True)
    cid = DockerRunner(run=fake).start(_opts(DisplayPlan("headless", "software", True), tmp_path))
    assert cid == "abc123"
    subs = [c[0] for c in fake.calls]
    assert subs == ["container", "rm", "run"]
    assert DockerRunner(run=fake).is_running()


# ---------------------------------------------------------------------------
# doctor
# ---------------------------------------------------------------------------

class StubRunner:
    def __init__(self, present=True, running=False):
        self.present, self.running = present, running

    def image_present(self, image):
        return self.present

    def is_running(self, name="lang2sim-sim"):
        return self.running


def test_doctor_no_docker_fails():
    plat = HostPlatform(os="macos", arch="arm64", docker=DockerInfo(error="docker CLI not found"))
    res = doctor.run_checks(plat, NONE, runner=StubRunner())
    assert doctor.exit_code(res) == 1
    d = next(c for c in res if c.name == "docker")
    assert d.status == "fail" and "Docker Desktop" in (d.fix or "")


def test_doctor_happy_path_reports_plan():
    res = doctor.run_checks(ubuntu(nvidia_runtime=True), NV_8G, runner=StubRunner(present=True, running=True))
    disp = next(c for c in res if c.name == "display")
    assert disp.status == "ok" and "nvidia" in disp.detail
    assert next(c for c in res if c.name == "image").status == "ok"
    assert "Ready" in doctor.format_report([c for c in res if c.status == "ok"])


def test_doctor_missing_image_is_warning_with_pull_hint():
    res = doctor.run_checks(mac(), NONE, runner=StubRunner(present=False))
    img = next(c for c in res if c.name == "image")
    assert img.status == "warn" and "docker pull" in (img.fix or "")
