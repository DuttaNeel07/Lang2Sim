"""
Host-side access to live telemetry from the control API (GET /v1/telemetry/live).
File: lang2sim/telemetry/live.py

One small synchronous fetch (stdlib only, short timeout) shared by the three
consumers the architecture names: the TUI TelemetrySidebar, the agent's
query_robot_telemetry() tool, and TelemetryCollector's control-API backend.
Each gets the same TelemetryLive fields, mapped to the shape it needs.
"""

from __future__ import annotations

import json
import os
import urllib.error
import urllib.request
from typing import Any, Dict, Optional

DEFAULT_API = "http://127.0.0.1:9090"


def api_base_url(base_url: Optional[str] = None) -> str:
    return (base_url or os.environ.get("LANG2SIM_CONTROL_API") or DEFAULT_API).rstrip("/")


def _get_json(url: str, timeout: float) -> Optional[Dict[str, Any]]:
    req = urllib.request.Request(url, headers={"User-Agent": "Lang2Sim-Telemetry"})
    try:
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            if resp.status != 200:
                return None
            data = json.loads(resp.read().decode("utf-8"))
            return data if isinstance(data, dict) else None
    except (urllib.error.URLError, OSError, ValueError):
        return None


def fetch_live(base_url: Optional[str] = None, timeout: float = 0.8) -> Optional[Dict[str, Any]]:
    """TelemetryLive as a dict, or None when no sim/control API is reachable.

    Falls back to the older GET /v1/telemetry (SimStatus) on images without the
    telemetry engine, converted to TelemetryLive field names (pose_source "map").
    """
    base = api_base_url(base_url)
    live = _get_json(f"{base}/v1/telemetry/live", timeout)
    if live is not None:
        return live
    st = _get_json(f"{base}/v1/telemetry", timeout)
    if st is None:
        return None
    vel = st.get("robot_velocity") or {}
    return {
        "sim_time": float(st.get("sim_time") or 0.0),
        "real_time_factor": float(st.get("real_time_factor") or 0.0),
        "pose": st.get("robot_pose") if st.get("is_running") else None,
        "pose_source": "map" if st.get("is_running") else "none",
        "linear_speed": float(vel.get("x", 0.0)) if isinstance(vel, dict) else 0.0,
        "angular_speed": float(vel.get("yaw", 0.0)) if isinstance(vel, dict) else 0.0,
        "collision_count": 0,
        "last_contact_force_n": 0.0,
        "legacy": True,
    }


def _fmt_sim_time(t: float) -> str:
    m, s = divmod(max(0.0, t), 60.0)
    return f"{int(m):02d}:{s:04.1f}"


def sidebar_fields(live: Optional[Dict[str, Any]]) -> Dict[str, Any]:
    """TelemetrySidebar.update_telemetry(**fields) arguments."""
    if live is None:
        return {"sim_state": "No simulation"}
    pose = live.get("pose")
    if pose is None:
        return {"sim_state": "Starting", "sim_time": _fmt_sim_time(float(live.get("sim_time") or 0.0))}
    clearance = live.get("obstacle_clearance_m")
    return {
        "sim_state": "Active (recording)" if live.get("recording") else "Active",
        "sim_time": _fmt_sim_time(float(live.get("sim_time") or 0.0)),
        "rtf": f"{float(live.get('real_time_factor') or 0.0):.2f}x",
        "robot_x": float(pose["x"]),
        "robot_y": float(pose["y"]),
        "speed": abs(float(live.get("linear_speed") or 0.0)),
        "clearance": None if clearance is None else float(clearance),
        "bumper_force": float(live.get("last_contact_force_n") or 0.0),
        "collision_count": int(live.get("collision_count") or 0),
    }


def agent_view(live: Optional[Dict[str, Any]]) -> Dict[str, Any]:
    """query_robot_telemetry() result (architecture §3.5: speed, distance to goal, bumper forces)."""
    if live is None:
        return {"status": "unavailable", "reason": "no simulation running (control API not reachable)"}
    pose = live.get("pose") or {}
    return {
        "status": "ok",
        "x": pose.get("x"),
        "y": pose.get("y"),
        "yaw": pose.get("yaw"),
        "pose_source": live.get("pose_source"),
        "speed": live.get("linear_speed"),
        "angular_speed": live.get("angular_speed"),
        "distance_to_goal": live.get("distance_to_goal_m"),
        "obstacle_clearance": live.get("obstacle_clearance_m"),
        "bumper_force_newtons": live.get("last_contact_force_n", 0.0),
        "collision_count": live.get("collision_count", 0),
        "localization_error": live.get("localization_error_m"),
        "sim_time": live.get("sim_time"),
        "real_time_factor": live.get("real_time_factor"),
        "recording": live.get("recording", False),
    }
