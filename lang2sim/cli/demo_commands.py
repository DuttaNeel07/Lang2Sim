"""
Demo commands for the TUI: drive, stress and score the running simulation from the chat.
File: lang2sim/cli/demo_commands.py

The workspace chat talks to the LLM; these commands talk to the control API
(the live sim container, or the replay fallback on the same port), so the pitch
beats are deterministic and never depend on a model choosing the right tool:

  /mission [stress]        drive spec.json's mission with recording, score it (NQI table)
  /stress [light=0.1] [model=wooden_crate] [ahead=1.5]
                           while the robot drives: dim the sun and drop a crate ahead of it
  /benchmark [light=0.1]   nominal run, back to start, stress run -> nominal vs stress table
  /report                  the latest benchmark_report.json as the NQI table
  /status                  sim, robot and recording state
  /dashboard               open http://localhost:9090/dashboard
  /replay [run_dir|demo]   serve a recorded run on the API port (when the live sim is down)
  /help

A natural-language stress request ("drop the lights to 10% and spawn a crate in front
of the robot") is recognised by parse_stress_intent and executed as /stress; the reply
says exactly what was run.

Pure helpers (parsing, crate placement) are unit-tested; DemoRunner does the I/O and
reports through callbacks: say(rich markup), show(rich renderable) and progress(rich
markup, or None to finish) for one status line that updates in place.
"""

from __future__ import annotations

import base64
import math
import re
import shlex
import threading
import webbrowser
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Awaitable, Callable, Dict, List, Optional, Sequence, Tuple

COMMANDS = ("mission", "stress", "benchmark", "report", "status", "dashboard", "replay", "help")
DEMO_RUN = Path(__file__).resolve().parents[2] / "catalog" / "demo_runs" / "empty_room_benchmark"
CRATE_HALF = {"wooden_crate": (0.3, 0.3), "cargo_box": (0.5, 0.3), "barrel": (0.3, 0.3)}
KEEP_CLEAR_OF_TARGETS_M = 1.2

HELP = """[bold #38bdf8]Demo commands[/] (they act on the running simulation, or a replay):
  [bold]/mission[/] [#8b98a8]\\[stress][/]        drive the mission with telemetry recording and score it
  [bold]/stress[/] [#8b98a8]\\[light=0.1] \\[model=wooden_crate|cargo_box|barrel] \\[ahead=1.5][/]
                           dim the lights and drop an obstacle in front of the moving robot
  [bold]/benchmark[/] [#8b98a8]\\[light=0.1][/]   nominal run, back to start, stress run, NQI comparison
  [bold]/report[/]                 show the latest NQI table
  [bold]/status[/]                 simulation, robot and recording state
  [bold]/dashboard[/]              open the live dashboard in the browser
  [bold]/replay[/] [#8b98a8]\\[run_dir|demo][/]  serve a recorded run if the live sim is down
You can also just say it: [italic]"drop the lights to 10% and spawn a crate in front of the robot"[/]."""


# --------------------------------------------------------------------------- parsing
@dataclass
class Command:
    name: str
    args: List[str] = field(default_factory=list)
    opts: Dict[str, str] = field(default_factory=dict)
    note: str = ""  # shown to the user (e.g. how a sentence was interpreted)
    natural: bool = False  # parsed from a sentence: do only what it asked for


def parse_command(text: str) -> Optional[Command]:
    """'/stress light=0.2 model=barrel' -> Command; None if not a known slash command."""
    text = text.strip()
    if not text.startswith("/"):
        return None
    try:
        parts = shlex.split(text[1:])
    except ValueError:
        parts = text[1:].split()
    if not parts or parts[0].lower() not in COMMANDS:
        return None
    cmd = Command(parts[0].lower())
    for p in parts[1:]:
        if "=" in p:
            k, v = p.split("=", 1)
            cmd.opts[k.strip().lower()] = v.strip()
        else:
            cmd.args.append(p)
    return cmd


_LIGHT_WORDS = r"(?:light|lights|lighting|luminance|sun|brightness)"
_OBSTACLE_WORDS = {
    "barrel": "barrel",
    "cargo": "cargo_box",
    "box": "cargo_box",
    "crate": "wooden_crate",
    "obstacle": "wooden_crate",
    "pallet": "wooden_crate",
}


def parse_stress_intent(text: str) -> Optional[Command]:
    """Recognise a natural-language stress request; None unless it clearly asks to change the sim.

    Needs an action ("drop", "spawn", "dim", "inject", ...) and a target (lights, or an
    obstacle). Questions ("why did the robot slow down?") never match.
    """
    t = text.lower().strip()
    if (
        not t
        or t.startswith("/")
        or t.endswith("?")
        or re.match(r"^(why|what|how|when|where|is|are|did|does|can)\b", t)
    ):
        return None
    action = re.search(
        r"\b(drop|spawn|place|put|add|throw|inject|dim|lower|reduce|darken|set|stress[- ]?test)\b",
        t,
    )
    if not action:
        return None
    opts: Dict[str, str] = {}
    notes: List[str] = []
    light = None
    m = re.search(_LIGHT_WORDS + r"[^0-9%]{0,30}?(\d+(?:\.\d+)?)\s*(%|percent)", t) or re.search(
        r"(\d+(?:\.\d+)?)\s*(%|percent)[^.]{0,20}?" + _LIGHT_WORDS, t
    )
    if m:
        light = float(m.group(1)) / 100.0
    else:
        m = re.search(_LIGHT_WORDS + r"[^0-9]{0,20}?\b(0?\.\d+|1(?:\.0+)?)\b", t)
        if m:
            light = float(m.group(1))
        elif re.search(
            r"\b(dim|darken|lower|reduce|night|dark)\b[^.]{0,25}" + _LIGHT_WORDS, t
        ) or re.search(_LIGHT_WORDS + r"[^.]{0,15}\b(dim|low|dark|off)\b", t):
            light = 0.15
    if light is not None:
        opts["light"] = f"{max(0.0, min(1.0, light)):.2f}"
    obstacle = None
    for word, model in _OBSTACLE_WORDS.items():
        if re.search(rf"\b{word}e?s?\b", t):
            obstacle = model
            break
    if obstacle:
        opts["model"] = obstacle
    if not opts:
        return None
    if re.search(r"\b(blur|noise|fog)\b", t):
        notes.append("camera blur/noise injection is not available yet, skipped")
    said = []
    if "light" in opts:
        said.append(f"light={opts['light']}")
    if "model" in opts:
        said.append(f"model={opts['model']}")
    note = f"Interpreted as /stress {' '.join(said)}" + (f" ({'; '.join(notes)})" if notes else "")
    return Command("stress", opts=opts, note=note, natural=True)


def stress_targets(cmd: Command) -> Tuple[Optional[float], Optional[str]]:
    """(light 0..1 or None, obstacle model or None) for a /stress command.

    /stress alone dims to 10% and drops a wooden crate; light=off / model=none skip one.
    A sentence does only what it mentioned."""
    off = ("off", "none", "no", "skip")
    raw_light = cmd.opts.get("light", None if cmd.natural else "0.1")
    raw_model = cmd.opts.get("model", None if cmd.natural else "wooden_crate")
    light = (
        None
        if raw_light is None or raw_light.lower() in off
        else max(
            0.0,
            min(1.0, float(raw_light.rstrip("%")) / (100.0 if raw_light.endswith("%") else 1.0)),
        )
    )
    model = None if raw_model is None or raw_model.lower() in off else raw_model
    if model is not None and model not in CRATE_HALF:
        raise ValueError(f"unknown model '{model}' (use {', '.join(CRATE_HALF)})")
    return light, model


# --------------------------------------------------------------------------- crate placement
def _free(
    map_info: Optional[Dict[str, Any]], x: float, y: float, half: Tuple[float, float]
) -> bool:
    """True when the crate's footprint (plus 5 cm) is free space on the map (or no map is known)."""
    if not map_info:
        return True
    w, h, res = map_info["width"], map_info["height"], map_info["resolution"]
    ox, oy = map_info["origin"]
    cells = map_info.get("_cells")
    if cells is None:
        cells = map_info["_cells"] = base64.b64decode(map_info["cells_b64"])
    hx, hy = half[0] + 0.05, half[1] + 0.05
    steps = 5
    for i in range(steps + 1):
        for j in range(steps + 1):
            px, py = x - hx + 2 * hx * i / steps, y - hy + 2 * hy * j / steps
            col, row_b = int((px - ox) / res), int((py - oy) / res)
            if not (0 <= col < w and 0 <= row_b < h):
                return False
            if cells[(h - 1 - row_b) * w + col] != 0:
                return False
    return True


def pick_crate_spot(
    pose: Dict[str, float],
    heading_to: Optional[Tuple[float, float]],
    avoid: Sequence[Tuple[float, float]],
    map_info: Optional[Dict[str, Any]] = None,
    model: str = "wooden_crate",
    ahead: float = 1.5,
) -> Optional[Tuple[float, float, float]]:
    """(x, y, yaw) for a crate in front of the robot, across its way, or None if nowhere fits.

    'In front' is toward the robot's current target when known (its planned direction),
    else along its heading. Candidates from `ahead` outward and back in are tried; each
    must keep KEEP_CLEAR_OF_TARGETS_M from every waypoint/goal (a blocked goal is not a
    detour) and sit on free map cells.
    """
    x, y = pose["x"], pose["y"]
    if heading_to is not None and math.hypot(heading_to[0] - x, heading_to[1] - y) > 0.3:
        yaw = math.atan2(heading_to[1] - y, heading_to[0] - x)
    else:
        yaw = pose.get("yaw", 0.0)
    half = CRATE_HALF.get(model, CRATE_HALF["wooden_crate"])
    distances = [ahead + d for d in (0.0, 0.25, -0.25, 0.5, -0.5, 0.75, 1.0, 1.25)]
    for d in distances:
        if d < 0.8:  # never on top of the robot
            continue
        cx, cy = x + d * math.cos(yaw), y + d * math.sin(yaw)
        if any(math.hypot(cx - a, cy - b) < KEEP_CLEAR_OF_TARGETS_M for a, b in avoid):
            continue
        if _free(map_info, cx, cy, half):
            return round(cx, 3), round(cy, 3), round(yaw, 4)
    return None


# --------------------------------------------------------------------------- runner
Say = Callable[[str], None]
Show = Callable[[Any], None]


def _f(v: Any, d: int = 2, unit: str = "") -> str:
    return "–" if v is None else f"{float(v):.{d}f}{unit}"


class DemoRunner:
    """Executes demo commands against the control API and reports through say/show."""

    def __init__(
        self,
        base_url: Optional[str] = None,
        say: Say = print,
        show: Show = print,
        progress: Optional[Callable[[Optional[str]], None]] = None,
    ) -> None:
        from lang2sim.telemetry.live import api_base_url

        self.base_url = api_base_url(base_url)
        self.say, self.show = say, show
        self.progress = progress or (lambda m: say(m) if m else None)
        self.mission_lock = threading.Lock()
        self._replay_server = None

    # --- plumbing ----------------------------------------------------------
    def _session(self, run_dir: Optional[Path] = None):
        from lang2sim.runtime.session import SimSession

        return SimSession.attach(self.base_url, run_dir)

    async def _run_info(self, session) -> Dict[str, Any]:
        info = await session.api._request("GET", "/run")
        if not info.get("loaded"):
            raise RuntimeError(
                "no run is loaded in the simulation (start one with scripts/run_sim.py)"
            )
        return info

    async def _host_run_dir(self, session) -> Path:
        from lang2sim.runtime.session import host_run_dir

        info = await self._run_info(session)
        d = host_run_dir(info)
        if d is None:
            raise RuntimeError(
                f"run folder {info.get('project_path') or info.get('run_dir')} is not on this machine"
            )
        return d

    async def run(self, cmd: Command) -> None:
        handler: Callable[[Command], Awaitable[None]] = getattr(self, f"cmd_{cmd.name}")
        if cmd.note:
            self.say(f"[#8b98a8]{cmd.note}[/]")
        try:
            await handler(cmd)
        except Exception as e:  # noqa: BLE001 - report every failure in the chat, never crash the TUI
            from lang2sim.runtime.control_api.client import ControlAPIError, ControlAPIUnavailable

            if isinstance(e, ControlAPIUnavailable):
                self.say(
                    f"[#ef4444]✗ No simulation reachable at {self.base_url}.[/] Start one with "
                    "[bold]python scripts/run_sim.py --fixture empty_room[/], or [bold]/replay demo[/]."
                )
            elif isinstance(e, ControlAPIError) and e.status_code == 409 and "replay" in str(e):
                self.say(
                    "[#f39c12]This is a replay of a recorded run: it can't be driven or changed.[/] "
                    "Use [bold]/report[/] or the dashboard."
                )
            else:
                self.say(f"[#ef4444]✗ /{cmd.name} failed:[/] {e}")

    # --- commands ----------------------------------------------------------
    async def cmd_help(self, cmd: Command) -> None:
        self.say(HELP)

    async def cmd_status(self, cmd: Command) -> None:
        s = self._session()
        try:
            health = await s.api.health()
            st = await s.api.sim_status()
            live = await s.api.telemetry_live()
            nav = await s.api.nav_status()
            info = await s.api._request("GET", "/run")
        finally:
            await s.stop()
        mode = "REPLAY" if health.get("mode") == "replay" else "LIVE"
        lines = [
            f"[bold]{mode}[/] · sim [bold]{st.status}[/]{' · ready' if st.ready else ''} · "
            f"sim time {st.sim_time:.1f}s · RTF {st.rtf:.2f}"
        ]
        if info.get("loaded"):
            lines.append(
                f"Run: {info.get('world_name') or ''} [#8b98a8]{info.get('project_path') or info.get('run_dir')}[/]"
            )
        if live.pose:
            lines.append(
                f"Robot ({live.pose.x:.2f}, {live.pose.y:.2f}) · {abs(live.linear_speed):.2f} m/s · "
                f"clearance {_f(live.obstacle_clearance_m, 2, ' m')} · collisions {live.collision_count} · "
                f"pose from {live.pose_source.replace('_', ' ')}"
            )
        lines.append(
            f"Nav: {nav.status}"
            + (
                f" · waypoint {nav.waypoint_index + 1}/{nav.waypoints_total} · "
                f"{nav.distance_remaining:.1f} m to go"
                if nav.is_active
                else ""
            )
        )
        if live.recording:
            lines.append(
                f"[#ef4444]● recording[/] {live.recording_label} ({live.samples_recorded} samples)"
            )
        self.say("\n".join(lines))

    async def cmd_dashboard(self, cmd: Command) -> None:
        url = f"{self.base_url}/dashboard"
        try:
            opened = webbrowser.open(url)
        except Exception:  # noqa: BLE001
            opened = False
        self.say(
            f"Dashboard: [bold #38bdf8]{url}[/]" + ("" if opened else "  (open it in a browser)")
        )

    async def cmd_report(self, cmd: Command) -> None:
        s = self._session()
        try:
            report = await s.api._request("GET", "/run/report")
        finally:
            await s.stop()
        self.show_report(report)

    def show_report(self, report: Dict[str, Any]) -> None:
        from lang2sim.telemetry.nqi_scorer import render_comparison_table

        self.show(render_comparison_table(report))
        runs = report.get("runs") or {}
        nom, adv = runs.get("nominal") or {}, runs.get("adversarial")
        sc = report.get("scoring") or {}
        if adv:
            self.say(
                f"NQI [bold]{_f(nom.get('nqi'), 1)}[/] → [bold #ef4444]{_f(adv.get('nqi'), 1)}[/]  "
                f"({_f((report.get('deltas') or {}).get('nqi_pct'), 1)}%) · min clearance "
                f"{_f(adv.get('C_min_m'), 2, ' m')} · collisions {adv.get('C_force')}"
                + (
                    f" · goal tolerance {sc.get('goal_tolerance_m')} m ({sc.get('goal_tolerance_source', '').replace('_', ' ')})"
                    if sc
                    else ""
                )
            )
        elif nom:
            self.say(
                f"NQI [bold]{_f(nom.get('nqi'), 1)}[/] / 100 · mission "
                f"{'succeeded' if nom.get('S_mission') == 1 else 'failed'} · collisions {nom.get('C_force')}"
            )

    async def cmd_stress(self, cmd: Command) -> None:
        light, model = stress_targets(cmd)
        ahead = float(cmd.opts.get("ahead", 1.5))
        s = self._session()
        try:
            if light is not None:
                await s.api.world_light({"intensity": max(0.0, min(1.0, light))})
                self.say(f"[#f39c12]☾[/] Lights dimmed to [bold]{light:.0%}[/]")
            if model:
                live = await s.api.telemetry_live()
                if live.pose is None:
                    raise RuntimeError("robot pose unknown (is the simulation ready?)")
                nav = await s.api.nav_status()
                info = await s.api._request("GET", "/run")
                map_info = await s.api._request("GET", "/run/map")
                mission = info.get("mission") or {}
                avoid = [
                    (p["x"], p["y"])
                    for p in [*mission.get("waypoints", []), mission.get("goal_pose")]
                    if p
                ]
                target = (
                    (nav.target_pose.x, nav.target_pose.y)
                    if nav.is_active and nav.target_pose
                    else None
                )
                spot = pick_crate_spot(
                    live.pose.model_dump(), target, avoid, map_info, model, ahead
                )
                if spot is None:
                    raise RuntimeError(
                        "no free spot ahead of the robot that keeps waypoints reachable; "
                        "try again once it is mid-leg"
                    )
                res = await s.api.world_spawn(
                    {"model_name": model, "x": spot[0], "y": spot[1], "yaw": spot[2]}
                )
                name = (res.get("data") or {}).get("entity_name") or model
                d = math.hypot(spot[0] - live.pose.x, spot[1] - live.pose.y)
                self.say(
                    f"[#ef4444]■[/] Spawned [bold]{name}[/] at ({spot[0]:.2f}, {spot[1]:.2f}), "
                    f"{d:.1f} m ahead of the robot"
                    + (
                        ""
                        if nav.is_active
                        else " [#8b98a8](robot not navigating: placed along its heading)[/]"
                    )
                )
                if not live.recording:
                    self.say(
                        "[#8b98a8]Not recording: start [bold]/mission stress[/] first to score this run.[/]"
                    )
        finally:
            await s.stop()

    async def cmd_mission(self, cmd: Command) -> None:
        label = (
            "adversarial"
            if any(a.lower() in ("stress", "adversarial") for a in cmd.args)
            else "nominal"
        )
        if not self.mission_lock.acquire(blocking=False):
            self.say(
                "[#f39c12]A mission is already running.[/] Use [bold]/stress[/] to perturb it."
            )
            return
        s = self._session()
        try:
            run_dir = await self._host_run_dir(s)
            s.run_dir = run_dir
            if label == "adversarial":
                await self._back_to_start(s, run_dir)
                self.say(
                    "[bold]Stress run[/] recording. Type [bold]/stress[/] (or say it) while the robot drives."
                )
            else:
                self.say("[bold]Nominal run[/] recording: driving the mission from spec.json…")
            run = await s.run_recorded_mission(run_dir, label=label, on_progress=self._nav_progress)
            self.progress(None)
            self.say(
                f"Mission [bold]{run.nav.status}[/] · {run.result.samples} samples in "
                f"{run.result.duration_s:.1f} sim s · {run.result.collisions} collisions"
            )
            from lang2sim.telemetry.benchmark import score_run_folder

            path, report = score_run_folder(run_dir)
            self.show_report(report)
            self.say(f"[#8b98a8]Report: {path}[/]")
        finally:
            await s.stop()
            self.mission_lock.release()

    async def cmd_benchmark(self, cmd: Command) -> None:
        light = float(cmd.opts.get("light", 0.1))
        if not self.mission_lock.acquire(blocking=False):
            self.say("[#f39c12]A mission is already running.[/]")
            return
        s = self._session()
        try:
            run_dir = await self._host_run_dir(s)
            s.run_dir = run_dir
            await self._back_to_start(s, run_dir)

            def trig(tr: Dict[str, Any], err: Optional[str]) -> None:
                p = tr.get("spawn_pose") or {}
                self.say(
                    f"[#ef4444]■[/] Stress: {tr.get('model_name')} dropped at ({p.get('x')}, {p.get('y')})"
                    + (f" [#ef4444]failed: {err}[/]" if err else "")
                )

            def phase(m: str) -> None:
                self.progress(None)
                self.say(f"[bold #38bdf8]▸ {m}[/]")

            report = await s.run_benchmark(
                run_dir,
                light=light,
                on_progress=self._nav_progress,
                on_trigger=trig,
                on_phase=phase,
            )
            self.progress(None)
            self.show_report(report)
            self.say(f"[#8b98a8]Report: {report.get('path')}[/]")
        finally:
            await s.stop()
            self.mission_lock.release()

    async def cmd_replay(self, cmd: Command) -> None:
        target = Path(cmd.args[0]).expanduser() if cmd.args and cmd.args[0] != "demo" else DEMO_RUN
        port = (
            int(self.base_url.rsplit(":", 1)[-1])
            if self.base_url.rsplit(":", 1)[-1].isdigit()
            else 9090
        )
        try:
            from lang2sim.telemetry.live import _get_json

            if _get_json(f"{self.base_url}/v1/health", 0.5) is not None:
                self.say(
                    f"[#f39c12]Something already answers on {self.base_url}.[/] Stop the live sim "
                    "(or the other replay) first."
                )
                return
            import uvicorn

            from lang2sim.telemetry.replay import create_replay_app
        except ImportError as e:
            self.say(f"[#ef4444]✗ Replay needs the [sim] extra:[/] pip install -e '.[sim]' ({e})")
            return
        app = create_replay_app(target, loop=True)
        server = uvicorn.Server(
            uvicorn.Config(app, host="127.0.0.1", port=port, log_level="warning")
        )
        threading.Thread(target=server.run, name="l2s-replay", daemon=True).start()
        self._replay_server = server
        runs = ", ".join(f"{r.label} ({r.duration:.0f} s)" for r in app.state.clock.runs)
        self.say(
            f"[#f39c12]▶ Replay[/] of [bold]{target.name}[/] on {self.base_url}: {runs}, looping. "
            "The sidebar, dashboard and /report now use it."
        )

    # --- helpers -------------------------------------------------------------
    async def _back_to_start(self, s, run_dir: Path) -> None:
        from lang2sim.core.models import SimSpec

        spec = SimSpec.model_validate_json((run_dir / "spec.json").read_text())
        live = await s.api.telemetry_live()
        p0 = spec.mission.initial_pose
        if live.pose is not None and math.hypot(live.pose.x - p0.x, live.pose.y - p0.y) > 0.5:
            self.say("Returning to the start pose first (not recorded)…")
            await s.return_to_start(run_dir)

    def _nav_progress(self, st) -> None:
        if st.is_active:
            self.progress(
                f"[#8b98a8]  ▸ driving · waypoint {st.waypoint_index + 1}/{st.waypoints_total} · "
                f"{st.distance_remaining:.1f} m to go[/]"
            )
        else:
            self.progress(None)
