"""
Ground the chat in the running simulation: live telemetry + the latest NQI report as LLM context.
File: lang2sim/agent/grounding.py

Before each chat turn the workspace asks the control API (live sim or replay) for the
robot's state and the latest benchmark_report.json and adds them as a system message,
so questions like "why did the NQI drop?" or "how close did it get to the crate?" are
answered from measured numbers instead of the model's imagination. Stdlib only; every
fetch is short and failure just means "no simulation context".
"""

from __future__ import annotations

from typing import Any, Dict, List, Optional

from lang2sim.telemetry.live import _get_json, api_base_url

NQI_FORMULA = ("NQI = 100*[0.35*E_p + 0.25/(1 + 0.05*S_tau) + 0.40*min(1, C_min/0.5 m)] - 50*C_force; "
               "E_p = optimal/actual path length (A* through the mission waypoints), S_tau = integrated "
               "squared jerk + angular acceleration (sim time), C_min = closest chassis-to-obstacle distance, "
               "C_force = chassis bumper collisions > 5 N.")


def _n(v: Any, d: int = 2, unit: str = "") -> str:
    return "n/a" if v is None else f"{float(v):.{d}f}{unit}"


def _run_line(name: str, r: Dict[str, Any]) -> str:
    mission = {1: "succeeded", 0: "failed"}.get(r.get("S_mission"), r.get("nav_status") or "n/a")
    terms = r.get("terms") or {}
    return (f"- {name}: NQI {_n(r.get('nqi'), 1)}/100, mission {mission}, duration {_n(r.get('duration_s'), 1, ' s')}, "
            f"path {_n(r.get('path_length_m'), 2, ' m')} (optimal {_n(r.get('optimal_length_m'), 2, ' m')}), "
            f"E_p {_n(r.get('E_p'))}, S_tau {_n(r.get('S_tau'), 1)}, C_min {_n(r.get('C_min_m'), 2, ' m')}, "
            f"collisions {r.get('C_force', 'n/a')}, localization error {_n(r.get('localization_error_mean_m'), 2, ' m')}; "
            f"NQI terms: efficiency {_n(terms.get('efficiency'), 1)}, smoothness {_n(terms.get('smoothness'), 1)}, "
            f"clearance {_n(terms.get('clearance'), 1)}, collision penalty {_n(terms.get('collision_penalty'), 1)}")


def format_context(health: Optional[Dict[str, Any]], live: Optional[Dict[str, Any]], nav: Optional[Dict[str, Any]],
                   run: Optional[Dict[str, Any]], report: Optional[Dict[str, Any]]) -> Optional[str]:
    """The system message text, or None when no simulation answered."""
    if health is None and live is None:
        return None
    mode = "REPLAY of a recorded run" if (health or {}).get("mode") == "replay" or (run or {}).get("mode") == "replay" \
        else "LIVE simulation"
    lines: List[str] = [f"Lang2Sim simulation context ({mode}; measured data from the control API, not guesses)."]
    if run and run.get("loaded"):
        m = run.get("mission") or {}
        wps = ", ".join(f"({p['x']:.1f}, {p['y']:.1f})" for p in m.get("waypoints") or [])
        g = m.get("goal_pose") or {}
        lines.append(f"World: {run.get('world_name') or 'unknown'}; mission from ({(m.get('initial_pose') or {}).get('x', 0):.1f}, "
                     f"{(m.get('initial_pose') or {}).get('y', 0):.1f}) via [{wps}] to goal ({g.get('x', 0):.1f}, {g.get('y', 0):.1f}).")
        crates = run.get("spawned_entities") or []
        if crates:
            lines.append("Obstacles spawned this run: " + ", ".join(
                f"{c.get('model')} at ({c.get('x', 0):.2f}, {c.get('y', 0):.2f})" for c in crates) + ".")
    if live and live.get("pose"):
        p = live["pose"]
        lines.append(
            f"Robot now (sim time {_n(live.get('sim_time'), 1, ' s')}, RTF {_n(live.get('real_time_factor'))}): "
            f"pose ({p['x']:.2f}, {p['y']:.2f}, yaw {p.get('yaw', 0):.2f} rad) from {live.get('pose_source')}, "
            f"speed {_n(abs(live.get('linear_speed') or 0))} m/s, obstacle clearance {_n(live.get('obstacle_clearance_m'), 2, ' m')}, "
            f"distance to goal {_n(live.get('distance_to_goal_m'), 2, ' m')}, collisions {live.get('collision_count', 0)}, "
            f"localization error {_n(live.get('localization_error_m'), 2, ' m')}"
            + (f", recording the '{live.get('recording_label')}' run" if live.get("recording") else "") + ".")
    if nav:
        lines.append(f"Nav2: {nav.get('status')}" + (
            f", waypoint {nav.get('waypoint_index', 0) + 1}/{nav.get('waypoints_total')}, "
            f"{_n(nav.get('distance_remaining'), 1, ' m')} to go, {nav.get('recoveries', 0)} recoveries"
            if nav.get("is_active") else "") + ".")
    if report and report.get("runs"):
        runs = report["runs"]
        lines.append(f"Latest benchmark (benchmark_report.json, scored {report.get('generated_at', 'n/a')}):")
        for key, name in (("nominal", "nominal run"), ("adversarial", "stress run")):
            if key in runs:
                lines.append(_run_line(name, runs[key]))
        d = report.get("deltas") or {}
        if "adversarial" in runs:
            lines.append(f"Stress vs nominal: NQI {_n(d.get('nqi_pct'), 1)}%, duration {_n(d.get('duration_pct'), 1)}%, "
                         f"path length {_n(d.get('path_length_pct'), 1)}%, S_tau {_n(d.get('S_tau_pct'), 1)}%, "
                         f"C_min {_n(d.get('C_min_pct'), 1)}%.")
        stress = report.get("stress") or {}
        for tr in stress.get("triggers") or []:
            p = tr.get("spawn_pose") or {}
            lines.append(f"Stress trigger: {tr.get('model_name')} dropped at ({p.get('x')}, {p.get('y')}) "
                         f"{_n(tr.get('fired_at_s'), 1, ' s')} into the stress run"
                         + (f"; sun dimmed to {stress['light']:.0%}" if stress.get("light") is not None else "") + ".")
        sc = report.get("scoring") or {}
        if sc:
            lines.append(f"Mission success tolerance {sc.get('goal_tolerance_m')} m ({sc.get('goal_tolerance_source')}); "
                         "clearance measured from the TurtleBot3 Waffle chassis rectangle.")
        lines.append(NQI_FORMULA)
    lines.append("Answer with these numbers when relevant and say when something was not measured. To change the "
                 "simulation the user types /stress, /mission [stress], /benchmark or /report (or a plain request "
                 "like 'drop the lights to 10% and spawn a crate ahead'); you cannot act on the simulation yourself.")
    return "\n".join(lines)


def fetch_context(base_url: Optional[str] = None, timeout: float = 0.6) -> Optional[str]:
    """Blocking (call from a thread): gather the context from the control API, or None."""
    base = api_base_url(base_url)
    health = _get_json(f"{base}/v1/health", timeout)
    if health is None:
        return None
    live = _get_json(f"{base}/v1/telemetry/live", timeout)
    nav = _get_json(f"{base}/v1/nav/status", timeout)
    run = _get_json(f"{base}/v1/run", timeout)
    report = _get_json(f"{base}/v1/run/report", timeout) if run and run.get("has_report") else None
    return format_context(health, live, nav, run, report)


def with_context(history: List[Dict[str, str]], context: Optional[str]) -> List[Dict[str, str]]:
    """A copy of the chat history with the context as a system message just before the latest user turn."""
    if not context:
        return list(history)
    msgs = list(history)
    idx = len(msgs) - 1 if msgs and msgs[-1].get("role") == "user" else len(msgs)
    return msgs[:idx] + [{"role": "system", "content": context}] + msgs[idx:]
