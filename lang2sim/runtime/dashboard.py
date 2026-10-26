"""
Live benchmark dashboard: data endpoints + the page, for the control API and the replay server.
File: lang2sim/runtime/dashboard.py

Architecture §6 Phase 4 ("FastAPI WebSocket streaming endpoints ... dashboard
featuring live performance charts") and §5.2 (dual-mode demo safety). One
router, mounted by:

  * the control API in the sim container (mode "live"): the run being simulated;
  * telemetry/replay.py on the host (mode "replay"): a recorded run folder.

Routes (same in both modes, so the page and the TUI never need to know which):

  GET /dashboard                 the page (self-contained HTML, no CDN: works offline)
  GET /v1/run                    run folder, world, mission, mode, spawned entities
  GET /v1/run/map                the zero-SLAM occupancy grid (map.yaml + map.pgm)
  GET /v1/run/trajectories       recorded runs (downsampled), crates, contact events
  GET /v1/run/report             benchmark_report.json (404 until a run was scored)

Live robot state comes from the existing WS /v1/nav/feedback ("live" key) and
GET /v1/telemetry/live. Stdlib only: imported inside the sim container.
"""

from __future__ import annotations

import base64
import json
import os
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional

from lang2sim.runtime.telemetry_recorder import load_recorded_runs

STATIC_DIR = Path(__file__).resolve().parent / "static"
DASHBOARD_HTML = STATIC_DIR / "dashboard.html"
REPORT_NAME = "benchmark_report.json"
MAX_POINTS = 1500


class MapError(ValueError):
    pass


def _yaml_kv(path: Path) -> Dict[str, str]:
    """Flat `key: value` reader for Nav2 map.yaml (no PyYAML in the image's venv guarantee)."""
    out: Dict[str, str] = {}
    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.split("#", 1)[0].strip()
        if ":" in line:
            k, v = line.split(":", 1)
            out[k.strip()] = v.strip().strip("'\"")
    return out


def read_pgm(path: Path) -> tuple:
    """(width, height, maxval, pixel bytes) of a binary (P5) or ASCII (P2) PGM, comments allowed."""
    data = Path(path).read_bytes()
    tokens: List[bytes] = []
    i = 0
    while len(tokens) < 4:
        while i < len(data) and data[i:i + 1].isspace():
            i += 1
        if i < len(data) and data[i:i + 1] == b"#":
            while i < len(data) and data[i:i + 1] not in (b"\n", b"\r"):
                i += 1
            continue
        j = i
        while j < len(data) and not data[j:j + 1].isspace():
            j += 1
        if j == i:
            raise MapError(f"{path}: truncated PGM header")
        tokens.append(data[i:j])
        i = j
    magic, w, h, maxval = tokens[0], int(tokens[1]), int(tokens[2]), int(tokens[3])
    if magic == b"P5":
        if maxval > 255:
            raise MapError(f"{path}: 16-bit PGM not supported")
        pixels = data[i + 1:i + 1 + w * h]
    elif magic == b"P2":
        pixels = bytes(int(v) * 255 // maxval for v in data[i:].split()[: w * h])
        maxval = 255
    else:
        raise MapError(f"{path}: not a PGM (magic {magic!r})")
    if len(pixels) != w * h:
        raise MapError(f"{path}: expected {w * h} pixels, got {len(pixels)}")
    return w, h, maxval, pixels


def read_map(run_dir: Path, map_name: str = "map.yaml") -> Dict[str, Any]:
    """Occupancy grid for drawing: cells 0 free, 1 occupied, 2 unknown, row 0 = top (image order)."""
    yaml_path = Path(run_dir) / map_name
    meta = _yaml_kv(yaml_path)
    image = Path(meta.get("image", "map.pgm"))
    if not image.is_absolute():
        image = yaml_path.parent / image
    w, h, maxval, pixels = read_pgm(image)
    negate = int(float(meta.get("negate", "0")))
    occ_t, free_t = float(meta.get("occupied_thresh", "0.65")), float(meta.get("free_thresh", "0.25"))
    cells = bytearray(w * h)
    for k, px in enumerate(pixels):
        p = px / maxval if negate else (maxval - px) / maxval      # occupancy probability
        cells[k] = 1 if p > occ_t else (0 if p < free_t else 2)
    origin = [float(v) for v in meta.get("origin", "[0, 0, 0]").strip("[]").split(",")]
    return {"width": w, "height": h, "resolution": float(meta["resolution"]),
            "origin": origin[:2], "cells_b64": base64.b64encode(bytes(cells)).decode("ascii")}


def read_mission(run_dir: Path) -> Optional[Dict[str, Any]]:
    try:
        spec = json.loads((Path(run_dir) / "spec.json").read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return None
    m = spec.get("mission") or {}
    pose = lambda p: {"x": p.get("x", 0.0), "y": p.get("y", 0.0), "yaw": p.get("yaw", 0.0)}  # noqa: E731
    return {"initial_pose": pose(m.get("initial_pose") or {}), "goal_pose": pose(m.get("goal_pose") or {}),
            "waypoints": [pose(p) for p in m.get("waypoints") or []],
            "timeout_seconds": m.get("timeout_seconds")}


def downsample(samples: List[Dict[str, float]], max_points: int = MAX_POINTS) -> List[List[float]]:
    """[[t, x, y, yaw, v], ...] with at most max_points rows (keeps first and last)."""
    n = len(samples)
    step = max(1, -(-n // max_points))
    idx = list(range(0, n, step))
    if n and idx[-1] != n - 1:
        idx.append(n - 1)
    return [[round(samples[i]["t"], 3), round(samples[i]["x"], 4), round(samples[i]["y"], 4),
             round(samples[i].get("yaw", 0.0), 4), round(samples[i].get("v", 0.0), 4)] for i in idx]


def trajectories_payload(run_dir: Path, max_points: int = MAX_POINTS) -> Dict[str, Any]:
    runs = load_recorded_runs(run_dir)
    out = {}
    for label, data in runs.items():
        meta = data.get("meta") or {}
        out[label] = {"label": label, "partial": bool(meta.get("partial")),
                      "pose_source": meta.get("pose_source"),
                      "samples": downsample(data.get("samples") or [], max_points),
                      "spawned_entities": meta.get("spawned_entities") or [],
                      "contact_events": meta.get("contact_events") or []}
    return {"runs": out}


def read_report(run_dir: Path) -> Optional[Dict[str, Any]]:
    try:
        return json.loads((Path(run_dir) / REPORT_NAME).read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return None


def project_path(run_dir: Path, projects_dir: Optional[str] = None) -> Optional[str]:
    """run_dir relative to the projects root (same relative path on host and in the container)."""
    root = projects_dir or os.getenv("L2S_PROJECTS_DIR")
    if not root:
        return None
    try:
        return str(Path(run_dir).resolve().relative_to(Path(root).resolve()))
    except ValueError:
        return None


def make_router(get_run_dir: Callable[[], Optional[Path]], mode: str,
                get_world: Callable[[], str] = lambda: "",
                get_spawned: Callable[[], List[Dict[str, Any]]] = lambda: []):
    """FastAPI router with the page and the /v1/run data routes for whatever run get_run_dir() names."""
    from fastapi import APIRouter, HTTPException
    from fastapi.responses import HTMLResponse

    router = APIRouter(tags=["dashboard"])

    def run_dir_or_404() -> Path:
        d = get_run_dir()
        if d is None:
            raise HTTPException(404, "no run loaded (start a simulation or a replay)")
        return Path(d)

    @router.get("/dashboard", response_class=HTMLResponse, include_in_schema=False)
    def dashboard() -> HTMLResponse:
        return HTMLResponse(DASHBOARD_HTML.read_text(encoding="utf-8"),
                            headers={"Cache-Control": "no-store"})

    @router.get("/v1/run")
    def run_info() -> Dict[str, Any]:
        d = get_run_dir()
        if d is None:
            return {"loaded": False, "mode": mode}
        report = read_report(d)
        return {"loaded": True, "mode": mode, "run_dir": str(d), "project_path": project_path(d),
                "world_name": get_world(), "mission": read_mission(d),
                "spawned_entities": get_spawned(), "has_report": report is not None,
                "report_generated_at": (report or {}).get("generated_at")}

    @router.get("/v1/run/map")
    def run_map() -> Dict[str, Any]:
        try:
            return read_map(run_dir_or_404())
        except (OSError, MapError, KeyError, ValueError) as e:
            raise HTTPException(404, f"map not readable: {e}")

    @router.get("/v1/run/trajectories")
    def run_trajectories() -> Dict[str, Any]:
        return trajectories_payload(run_dir_or_404())

    @router.get("/v1/run/report")
    def run_report() -> Dict[str, Any]:
        report = read_report(run_dir_or_404())
        if report is None:
            raise HTTPException(404, "no benchmark_report.json yet (run --mission or --benchmark)")
        return report

    return router
