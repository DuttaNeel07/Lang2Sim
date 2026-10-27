"""
Demo commands for the TUI: drive, stress and score the running simulation from the chat.
File: lang2sim/cli/demo_commands.py

Users never type these: every sentence goes through cli/intent.py (the LLM, or the
offline parser), which produces one of these Commands. DemoRunner runs them against
the control API (the live sim container, or the replay fallback on the same port).
The slash forms below remain as hidden shortcuts:

  /mission [stress]        drive spec.json's mission with recording, score it (NQI table)
  /stress [light=0.1] [model=wooden_crate] [ahead=1.5]
                           while the robot drives: dim the sun and drop a crate ahead of it
  /benchmark [light=0.1]   nominal run, back to start, stress run -> nominal vs stress table
  /report                  the latest benchmark_report.json as the NQI table
  /status                  sim, robot and recording state
  /dashboard               open http://localhost:9090/dashboard
  /replay [run_dir|demo]   serve a recorded run on the API port (when the live sim is down)
  /help

parse_stress_intent and the other is_* helpers are the offline parser's building blocks.

Pure helpers (parsing, crate placement) are unit-tested; DemoRunner does the I/O and
reports through callbacks: say(rich markup), show(rich renderable) and progress(rich
markup, or None to finish) for one status line that updates in place.
"""

from __future__ import annotations

import asyncio
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
# Half extents (along the model's x, along its y) of each spawnable model (perturbation/sdf_snippets.py).
CRATE_HALF = {"wooden_crate": (0.3, 0.3), "cargo_box": (0.5, 0.3), "barrel": (0.3, 0.3), "rack": (1.4, 0.6)}
KEEP_CLEAR_OF_TARGETS_M = 1.2

HELP = """[bold #38bdf8]Just say what you want[/] — the copilot works out the action:
  [italic]"build a warehouse with 3 racks and 2 crates"[/]          a new world, driven and scored
  [italic]"start a stress run"[/]                                  record a run you will perturb
  [italic]"drop the lights to 10% and spawn a crate in its path"[/]  stress the robot while it drives
  [italic]"put a rack across the aisle"[/], [italic]"drop a barrel at 1.5, 0"[/]
  [italic]"stop the robot"[/], [italic]"resume"[/], [italic]"detour and reach rack 3"[/]
  [italic]"run the full benchmark"[/]                              normal vs stress, automatically
  [italic]"show me the score"[/], [italic]"where is the robot?"[/], [italic]"open the dashboard"[/]
  [italic]"why did the score drop?"[/]                             answered from the measured numbers
  [italic]"stop the simulation"[/]                                 shut it down"""


# --------------------------------------------------------------------------- parsing
@dataclass
class Command:
    name: str
    args: List[str] = field(default_factory=list)
    opts: Dict[str, str] = field(default_factory=dict)
    note: str = ""                     # shown to the user (e.g. how a sentence was interpreted)
    natural: bool = False              # parsed from a sentence: do only what it asked for
    then: Optional["Command"] = None   # mission: stress to apply once the robot is under way


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
_OBSTACLE_WORDS = {"barrel": "barrel", "drum": "barrel", "rack": "rack", "shelf": "rack", "shelve": "rack",
                   "shelving": "rack", "cargo": "cargo_box", "box": "cargo_box", "crate": "wooden_crate",
                   "obstacle": "wooden_crate", "pallet": "wooden_crate"}
_QUESTION = re.compile(r"^(why|what|how|when|where|who|which|is|are|was|were|did|does|do|can|could|should|would|will|explain|tell me)\b")
_COORDS = re.compile(r"\bat\s*\(?\s*(?:x\s*=\s*)?(-?\d+(?:\.\d+)?)\s*(?:,|\s)\s*(?:y\s*=\s*)?(-?\d+(?:\.\d+)?)\s*\)?")
_ACROSS = re.compile(r"\b(horizontal(?:ly)?|across|perpendicular|sideways|crosswise)\b")


def is_question(text: str) -> bool:
    t = text.lower().strip()
    return t.endswith("?") or bool(_QUESTION.match(t))


def parse_stress_intent(text: str) -> Optional[Command]:
    """Recognise a natural-language stress request; None unless it clearly asks to change the sim.

    Needs an action ("drop", "spawn", "dim", "inject", ...) and a target (lights, or an
    obstacle). Questions ("why did the robot slow down?") never match.
    """
    t = text.lower().strip()
    if not t or t.startswith("/") or is_question(t):
        return None
    action = re.search(r"\b(drop|spawn|place|put|add|throw|inject|dim|lower|reduce|darken|set|stress[- ]?test)\b", t)
    if not action:
        return None
    opts: Dict[str, str] = {}
    notes: List[str] = []
    light = None
    m = re.search(_LIGHT_WORDS + r"[^0-9%]{0,30}?(\d+(?:\.\d+)?)\s*(%|percent)", t) or \
        re.search(r"(\d+(?:\.\d+)?)\s*(%|percent)[^.]{0,20}?" + _LIGHT_WORDS, t)
    if m:
        light = float(m.group(1)) / 100.0
    else:
        m = re.search(_LIGHT_WORDS + r"[^0-9]{0,20}?\b(0?\.\d+|1(?:\.0+)?)\b", t)
        if m:
            light = float(m.group(1))
        elif re.search(r"\b(dim|darken|lower|reduce|night|dark)\b[^.]{0,25}" + _LIGHT_WORDS, t) or \
                re.search(_LIGHT_WORDS + r"[^.]{0,15}\b(dim|low|dark|off)\b", t):
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
        m = _COORDS.search(t)
        if m:
            opts["x"], opts["y"] = m.group(1), m.group(2)
        if _ACROSS.search(t):
            opts["across"] = "1"
    if not opts:
        return None
    if re.search(r"\b(blur|noise|fog)\b", t):
        notes.append("camera blur/noise injection is not available yet, skipped")
    from .intent import describe_stress

    note = f"Understood: {describe_stress(opts)}" + (f" ({'; '.join(notes)})" if notes else "")
    return Command("stress", opts=opts, note=note, natural=True)


def stress_targets(cmd: Command) -> Tuple[Optional[float], Optional[str]]:
    """(light 0..1 or None, obstacle model or None) for a /stress command.

    /stress alone dims to 10% and drops a wooden crate; light=off / model=none skip one.
    A sentence does only what it mentioned."""
    off = ("off", "none", "no", "skip")
    raw_light = cmd.opts.get("light", None if cmd.natural else "0.1")
    raw_model = cmd.opts.get("model", None if cmd.natural else "wooden_crate")
    light = None if raw_light is None or raw_light.lower() in off else max(0.0, min(1.0, float(raw_light.rstrip("%")) / (100.0 if raw_light.endswith("%") else 1.0)))
    model = None if raw_model is None or raw_model.lower() in off else raw_model
    if model is not None and model not in CRATE_HALF:
        raise ValueError(f"unknown model '{model}' (use {', '.join(CRATE_HALF)})")
    return light, model


# --------------------------------------------------------------------------- crate placement
def _free(map_info: Optional[Dict[str, Any]], x: float, y: float, half: Tuple[float, float],
          yaw: float = 0.0) -> bool:
    """True when the obstacle's footprint (rotated by yaw, plus 5 cm) is free map space (or no map)."""
    if not map_info:
        return True
    w, h, res = map_info["width"], map_info["height"], map_info["resolution"]
    ox, oy = map_info["origin"]
    cells = map_info.get("_cells")
    if cells is None:
        cells = map_info["_cells"] = base64.b64decode(map_info["cells_b64"])
    hx, hy = half[0] + 0.05, half[1] + 0.05
    c, s_ = math.cos(yaw), math.sin(yaw)
    nx, ny = max(2, int(2 * hx / res / 2)), max(2, int(2 * hy / res / 2))
    for i in range(nx + 1):
        for j in range(ny + 1):
            u, v = -hx + 2 * hx * i / nx, -hy + 2 * hy * j / ny
            px, py = x + c * u - s_ * v, y + s_ * u + c * v
            col, row_b = int(math.floor((px - ox) / res)), int(math.floor((py - oy) / res))
            if not (0 <= col < w and 0 <= row_b < h):
                return False
            if cells[(h - 1 - row_b) * w + col] != 0:
                return False
    return True


def pick_crate_spot(pose: Dict[str, float], heading_to: Optional[Tuple[float, float]],
                    avoid: Sequence[Tuple[float, float]], map_info: Optional[Dict[str, Any]] = None,
                    model: str = "wooden_crate", ahead: float = 1.5,
                    across: bool = False) -> Optional[Tuple[float, float, float]]:
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
    obj_yaw = yaw + (math.pi / 2 if across else 0.0)
    # Distance from the obstacle's centre to its edge facing the robot, so it never lands on the robot.
    reach = abs(half[0] * math.cos(obj_yaw - yaw)) + abs(half[1] * math.sin(obj_yaw - yaw))
    base = max(ahead, reach + 0.6)
    distances = [base + d for d in (0.0, 0.25, -0.25, 0.5, -0.5, 0.75, 1.0, 1.25)]
    for d in distances:
        if d - reach < 0.45:             # never on top of the robot
            continue
        cx, cy = x + d * math.cos(yaw), y + d * math.sin(yaw)
        if any(math.hypot(cx - a, cy - b) < KEEP_CLEAR_OF_TARGETS_M + max(half) - 0.3 for a, b in avoid):
            continue
        if _free(map_info, cx, cy, half, obj_yaw):
            return round(cx, 3), round(cy, 3), round(math.atan2(math.sin(obj_yaw), math.cos(obj_yaw)), 4)
    return None


def pick_spot_on_path(pose: Dict[str, float], path: Sequence[Tuple[float, float]],
                      avoid: Sequence[Tuple[float, float]], map_info: Optional[Dict[str, Any]] = None,
                      model: str = "wooden_crate", across: bool = False,
                      ahead: Tuple[float, float] = (1.2, 3.2)) -> Optional[Tuple[float, float, float]]:
    """(x, y, yaw) on a recorded path (the robot's real lane), `ahead` metres of path beyond the
    robot, or None. Same rules as pick_crate_spot: free map cells, clear of waypoints."""
    if len(path) < 2:
        return None
    x, y = pose["x"], pose["y"]
    i0 = min(range(len(path)), key=lambda i: (path[i][0] - x) ** 2 + (path[i][1] - y) ** 2)
    if math.hypot(path[i0][0] - x, path[i0][1] - y) > 1.0:
        return None                        # robot is not on this path (e.g. it already detoured)
    half = CRATE_HALF.get(model, CRATE_HALF["wooden_crate"])
    keep = KEEP_CLEAR_OF_TARGETS_M + max(half) - 0.3
    acc, candidates = 0.0, []
    for i in range(i0, len(path) - 1):
        (ax, ay), (bx, by) = path[i], path[i + 1]
        acc += math.hypot(bx - ax, by - ay)
        if acc > ahead[1]:
            break
        if acc >= ahead[0]:
            candidates.append((acc, bx, by, math.atan2(by - ay, bx - ax)))
    # middle of the window first (enough time for the robot to see it and re-plan)
    mid = sum(ahead) / 2
    for _, cx, cy, tangent in sorted(candidates, key=lambda c: abs(c[0] - mid)):
        yaw = tangent + (math.pi / 2 if across else 0.0)
        if math.hypot(cx - x, cy - y) < 0.8 + max(half):
            continue
        if any(math.hypot(cx - a, cy - b) < keep for a, b in avoid):
            continue
        if _free(map_info, cx, cy, half, yaw):
            return round(cx, 3), round(cy, 3), round(math.atan2(math.sin(yaw), math.cos(yaw)), 4)
    return None


def pick_spot_fan(pose: Dict[str, float], heading_to: Optional[Tuple[float, float]],
                  avoid: Sequence[Tuple[float, float]], map_info: Optional[Dict[str, Any]] = None,
                  model: str = "wooden_crate", ahead: float = 1.5,
                  across: bool = False) -> Optional[Tuple[float, float, float]]:
    """pick_crate_spot, widened to ±35° around the heading when straight ahead is blocked
    (the straight line to a waypoint often cuts a shelf corner the robot drives around)."""
    spot = pick_crate_spot(pose, heading_to, avoid, map_info, model, ahead, across)
    if spot is not None:
        return spot
    x, y = pose["x"], pose["y"]
    base = math.atan2(heading_to[1] - y, heading_to[0] - x) if heading_to is not None and \
        math.hypot(heading_to[0] - x, heading_to[1] - y) > 0.3 else pose.get("yaw", 0.0)
    for deg in (10, -10, 20, -20, 35, -35):
        turned = dict(pose, yaw=base + math.radians(deg))
        spot = pick_crate_spot(turned, None, avoid, map_info, model, ahead, across)
        if spot is not None:
            return spot
    return None


# --------------------------------------------------------------------------- other intents
_STOP_SIM = re.compile(r"^(/stop|stop|stop (the )?sim(ulation)?|kill (the )?sim(ulation)?|exit simulation|shut ?down( the sim(ulation)?)?)\.?$")
_HALT = re.compile(r"\b(stop (the )?robot|halt|freeze|stop moving|cancel (the )?nav(igation)?|pause (the )?robot)\b")
_GOTO = re.compile(r"\b(detour|divert|re-?route|re-?plan|reach|resume|continue|navigate|go to|drive to|head to|move to)\b")
_SIM_VERB = re.compile(r"\b(simulate|build|create|generate|make|set ?up|spawn|load|launch|design)\b")
_SIM_NOUN = re.compile(r"\b(warehouse|room|world|scene|lab|field|office|factory|workshop|layout|environment|simulation|map)\b")
_ORDINALS = {"first": 1, "1st": 1, "second": 2, "2nd": 2, "third": 3, "3rd": 3, "fourth": 4, "4th": 4,
             "fifth": 5, "5th": 5, "sixth": 6, "6th": 6}


def is_stop_sim(text: str) -> bool:
    return bool(_STOP_SIM.match(text.lower().strip()))


def is_halt(text: str) -> bool:
    t = text.lower().strip()
    return not is_question(t) and bool(_HALT.search(t))


def is_goto(text: str) -> bool:
    t = text.lower().strip()
    return not is_question(t) and bool(_GOTO.search(t))


def is_sim_request(text: str) -> bool:
    """'build a dark warehouse with 3 racks' -> True; questions and chit-chat -> False."""
    t = text.lower().strip()
    if t.startswith("/sim"):
        return True
    return not is_question(t) and bool(_SIM_VERB.search(t)) and bool(_SIM_NOUN.search(t))


def rack_number(text: str) -> Optional[int]:
    t = text.lower()
    for word, n in _ORDINALS.items():
        if re.search(rf"\b{word}\b", t):
            return n
    m = re.search(r"\b(?:rack|shelf|aisle)\s*#?\s*(\d+)", t)
    return int(m.group(1)) if m else None


def resolve_goal(text: str, objects: Sequence[Dict[str, Any]], mission_goal: Optional[Dict[str, float]],
                 map_info: Optional[Dict[str, Any]] = None) -> Tuple[Optional[Tuple[float, float, float]], str]:
    """Where "detour and reach rack 3" / "resume" should go: (x, y, yaw) and a description.

    A numbered rack -> the free aisle 1.5 m in front of it (on the side facing the room
    centre), checked against the map; otherwise the mission goal.
    """
    n = rack_number(text)
    if n is not None:
        rack = next((o for o in objects if o.get("id") == f"rack_{n}"), None)
        if rack is None:
            return None, f"there is no rack_{n} in this world"
        rx, ry = rack["pose"]["x"], rack["pose"]["y"]
        depth = (rack.get("dimensions") or {}).get("y", 1.0) / 2
        side = -1.0 if ry > 0 else 1.0
        for off in (0.9, 1.2, 1.5, 0.6, 1.8):
            gx, gy = rx, ry + side * (depth + off)
            if _free(map_info, gx, gy, (0.2, 0.2)):
                yaw = math.atan2(ry - gy, 0.0)
                return (round(gx, 3), round(gy, 3), round(yaw, 4)), f"the aisle in front of rack_{n}"
        return None, f"no free space in front of rack_{n} on the map"
    if mission_goal is not None:
        return (mission_goal["x"], mission_goal["y"], mission_goal.get("yaw", 0.0)), "the mission goal"
    return None, "no target in the request and no mission goal"


# --------------------------------------------------------------------------- runner
Say = Callable[[str], None]
Show = Callable[[Any], None]


def _describe_stress(opts: Dict[str, str]) -> str:
    from .intent import describe_stress

    return describe_stress(opts)


def _f(v: Any, d: int = 2, unit: str = "") -> str:
    return "–" if v is None else f"{float(v):.{d}f}{unit}"


class DemoRunner:
    """Executes demo commands against the control API and reports through say/show."""

    def __init__(self, base_url: Optional[str] = None, say: Say = print, show: Show = print,
                 progress: Optional[Callable[[Optional[str]], None]] = None) -> None:
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
            raise RuntimeError("no run is loaded in the simulation (start one with scripts/run_sim.py)")
        return info

    async def _host_run_dir(self, session) -> Path:
        from lang2sim.runtime.session import host_run_dir

        info = await self._run_info(session)
        d = host_run_dir(info)
        if d is None:
            raise RuntimeError(f"run folder {info.get('project_path') or info.get('run_dir')} is not on this machine")
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
                self.say(f"[#ef4444]✗ No simulation is running yet[/] ({self.base_url}). Describe a world to build, "
                         "e.g. [italic]\"build a warehouse with 3 racks and 2 crates\"[/], or say "
                         "[italic]\"replay the demo run\"[/].")
            elif isinstance(e, ControlAPIError) and e.status_code == 409 and "replay" in str(e):
                self.say("[#f39c12]This is a replay of a recorded run: it can't be driven or changed.[/] "
                         "Ask for the score, or open the dashboard.")
            else:
                what = {"stress": "change the world", "mission": "run the mission", "benchmark": "run the benchmark",
                        "goto": "send the robot there", "halt": "stop the robot", "report": "show the score",
                        "status": "read the status"}.get(cmd.name, cmd.name)
                self.say(f"[#ef4444]✗ Couldn't {what}:[/] {e}")

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
        lines = [f"[bold]{mode}[/] · sim [bold]{st.status}[/]{' · ready' if st.ready else ''} · "
                 f"sim time {st.sim_time:.1f}s · RTF {st.rtf:.2f}"]
        if info.get("loaded"):
            lines.append(f"Run: {info.get('world_name') or ''} [#8b98a8]{info.get('project_path') or info.get('run_dir')}[/]")
        if live.pose:
            lines.append(f"Robot ({live.pose.x:.2f}, {live.pose.y:.2f}) · {abs(live.linear_speed):.2f} m/s · "
                         f"clearance {_f(live.obstacle_clearance_m, 2, ' m')} · collisions {live.collision_count} · "
                         f"pose from {live.pose_source.replace('_', ' ')}")
        lines.append(f"Nav: {nav.status}" + (f" · waypoint {nav.waypoint_index + 1}/{nav.waypoints_total} · "
                                              f"{nav.distance_remaining:.1f} m to go" if nav.is_active else ""))
        if live.recording:
            lines.append(f"[#ef4444]● recording[/] {live.recording_label} ({live.samples_recorded} samples)")
        self.say("\n".join(lines))

    async def cmd_dashboard(self, cmd: Command) -> None:
        url = f"{self.base_url}/dashboard"
        try:
            opened = webbrowser.open(url)
        except Exception:  # noqa: BLE001
            opened = False
        self.say(f"Dashboard: [bold #38bdf8]{url}[/]" + ("" if opened else "  (open it in a browser)"))

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
            self.say(f"NQI [bold]{_f(nom.get('nqi'), 1)}[/] → [bold #ef4444]{_f(adv.get('nqi'), 1)}[/]  "
                     f"({_f((report.get('deltas') or {}).get('nqi_pct'), 1)}%) · min clearance "
                     f"{_f(adv.get('C_min_m'), 2, ' m')} · collisions {adv.get('C_force')}"
                     + (f" · goal tolerance {sc.get('goal_tolerance_m')} m ({sc.get('goal_tolerance_source', '').replace('_', ' ')})" if sc else ""))
        elif nom:
            self.say(f"NQI [bold]{_f(nom.get('nqi'), 1)}[/] / 100 · mission "
                     f"{'succeeded' if nom.get('S_mission') == 1 else 'failed'} · collisions {nom.get('C_force')}")

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
                avoid = [(p["x"], p["y"]) for p in [*mission.get("waypoints", []), mission.get("goal_pose")] if p]
                target = (nav.target_pose.x, nav.target_pose.y) if nav.is_active and nav.target_pose else None
                across = cmd.opts.get("across", "0").lower() in ("1", "true", "yes")
                if "x" in cmd.opts and "y" in cmd.opts:          # explicit coordinates win
                    px, py = float(cmd.opts["x"]), float(cmd.opts["y"])
                    heading = math.atan2(py - live.pose.y, px - live.pose.x)
                    yaw = heading + (math.pi / 2 if across else 0.0)
                    spot = (px, py, round(math.atan2(math.sin(yaw), math.cos(yaw)), 4))
                    if not _free(map_info, px, py, CRATE_HALF[model], spot[2]):
                        self.say("[#f39c12]That spot overlaps a wall or another object on the map; spawning anyway.[/]")
                    where = "at the requested position"
                else:
                    path = await self._recorded_path(s)
                    pose = live.pose.model_dump()

                    def place(acr: bool):
                        on_path = pick_spot_on_path(pose, path, avoid, map_info, model, acr) if path else None
                        return on_path or pick_spot_fan(pose, target, avoid, map_info, model, ahead, acr)

                    spot = place(across)
                    if spot is None and across:
                        # A rack laid across a narrow aisle overlaps the shelving or walls: try along the path.
                        spot = place(False)
                        if spot is not None:
                            across = False
                            self.say(f"[#f39c12]A {model.replace('_', ' ')} doesn't fit across the aisle here; "
                                     "placing it along the robot's path instead.[/]")
                    if spot is None:
                        raise RuntimeError(f"no free spot ahead of the robot for a {model.replace('_', ' ')} that keeps "
                                           "waypoints reachable; try again in a few seconds, or a smaller obstacle "
                                           "(\"drop a crate in front of the robot\")")
                    d = math.hypot(spot[0] - live.pose.x, spot[1] - live.pose.y)
                    where = f"{d:.1f} m ahead of the robot" + ("" if nav.is_active else
                                                                " [#8b98a8](robot not navigating: placed along its heading)[/]")
                res = await s.api.world_spawn({"model_name": model, "x": spot[0], "y": spot[1], "yaw": spot[2]})
                name = (res.get("data") or {}).get("entity_name") or model
                self.say(f"[#ef4444]■[/] Spawned [bold]{name}[/]{' across the path' if across else ''} at "
                         f"({spot[0]:.2f}, {spot[1]:.2f}), {where}")
                if not live.recording:
                    self.say("[#8b98a8]Not recording, so this won't be scored. Say [italic]\"start a stress run\"[/] "
                             "first, then the change while the robot drives (or give both in one sentence).[/]")
        finally:
            await s.stop()

    async def cmd_halt(self, cmd: Command) -> None:
        """Stop the robot where it is (cancel the Nav2 task); the simulation keeps running."""
        s = self._session()
        try:
            st = await s.api.nav_cancel()
            for _ in range(15):                      # Nav2 confirms the cancel asynchronously
                if not st.is_active:
                    break
                await asyncio.sleep(0.2)
                st = await s.api.nav_status()
            live = await s.api.telemetry_live()
        finally:
            await s.stop()
        where = f" at ({live.pose.x:.2f}, {live.pose.y:.2f})" if live.pose else ""
        self.say(f"[#f39c12]■ Robot stopped[/]{where}. Navigation {st.status}; the simulation keeps running. "
                 "Say [italic]resume[/] or [italic]go to rack 2[/] to move again.")

    async def cmd_goto(self, cmd: Command) -> None:
        """Re-plan to a target from the sentence (a numbered rack, or the mission goal), clearing costmaps."""
        import json

        from lang2sim.core.models import Pose2D

        text = cmd.opts.get("text", "")
        s = self._session()
        try:
            info = await self._run_info(s)
            map_info = await s.api._request("GET", "/run/map")
            objects: List[Dict[str, Any]] = []
            try:
                run_dir = await self._host_run_dir(s)
                objects = json.loads((run_dir / "spec.json").read_text()).get("objects") or []
            except Exception:  # noqa: BLE001 - racks are optional (the mission goal still works)
                pass
            if "x" in cmd.opts and "y" in cmd.opts:
                gx, gy = float(cmd.opts["x"]), float(cmd.opts["y"])
                live0 = await s.api.telemetry_live()
                yaw = math.atan2(gy - live0.pose.y, gx - live0.pose.x) if live0.pose else 0.0
                goal, desc = (gx, gy, round(yaw, 4)), f"({gx:.1f}, {gy:.1f})"
                if not _free(map_info, gx, gy, (0.2, 0.2)):
                    goal, desc = None, f"({gx:.1f}, {gy:.1f}) is inside a wall or an object on the map"
            else:
                if cmd.opts.get("rack"):
                    text = f"rack {cmd.opts['rack']}"
                goal, desc = resolve_goal(text, objects, (info.get("mission") or {}).get("goal_pose"), map_info)
            if goal is None:
                self.say(f"[#f39c12]Can't navigate: {desc}.[/]")
                return
            await s.api.nav_goal(Pose2D(x=goal[0], y=goal[1], yaw=goal[2]), timeout_seconds=180.0, clear_costmaps=True)
            self.say(f"[#38bdf8]➜ Re-planning to {desc}[/] ({goal[0]:.2f}, {goal[1]:.2f}); costmaps cleared, "
                     "Nav2 plans around everything it sees now.")
            last = None
            while True:
                await asyncio.sleep(1.0)
                st = await s.api.nav_status()
                if st.is_active:
                    key = round(st.distance_remaining, 1)
                    if key != last:
                        self.progress(f"[#8b98a8]  ▸ navigating to {desc} · {st.distance_remaining:.1f} m to go"
                                      + (f" · {st.recoveries} recoveries" if st.recoveries else "") + "[/]")
                        last = key
                    continue
                self.progress(None)
                live = await s.api.telemetry_live()
                pose = f" Robot at ({live.pose.x:.2f}, {live.pose.y:.2f})." if live.pose else ""
                if st.status == "succeeded":
                    self.say(f"[#2ed573]✓ Reached {desc}.[/]{pose}")
                elif st.status in ("canceled", "cancelled"):
                    self.say(f"[#8b98a8]Stopped on the way to {desc}, as asked.[/]{pose}")
                else:
                    self.say(f"[#ef4444]✗ Navigation {st.status}[/]{': ' + st.message if st.message else ''}.{pose}")
                return
        finally:
            await s.stop()

    async def cmd_mission(self, cmd: Command) -> None:
        label = "adversarial" if any(a.lower() in ("stress", "adversarial") for a in cmd.args) else "nominal"
        if not self.mission_lock.acquire(blocking=False):
            self.say("[#f39c12]A mission is already running.[/] Tell me what to change while it drives, e.g. "
                     "[italic]\"drop a crate in front of the robot\"[/].")
            return
        s = self._session()
        try:
            run_dir = await self._host_run_dir(s)
            s.run_dir = run_dir
            if label == "adversarial":
                await self._back_to_start(s, run_dir)
                self.say("[bold]Stress run[/] recording." + (
                    f" I'll {_describe_stress(cmd.then.opts)} once the robot is under way." if cmd.then else
                    " Say what to change while the robot drives, e.g. [italic]\"dim the lights and drop a crate "
                    "in its path\"[/]."))
            else:
                self.say("[bold]Nominal run[/] recording: driving the mission from spec.json…")
            pending = asyncio.ensure_future(self._stress_when_moving(cmd.then)) if cmd.then else None
            try:
                run = await s.run_recorded_mission(run_dir, label=label, on_progress=self._nav_progress)
            finally:
                if pending is not None and not pending.done():
                    pending.cancel()
            self.progress(None)
            if label == "adversarial" and not any(e.get("during_run", True) for e in run.result.spawned_entities):
                self.say("[#f39c12]No obstacle was dropped during this stress run, so the comparison is "
                         "lap-to-lap variation, not stress. Next time say the change while the robot drives, "
                         "or ask for the full benchmark.[/]")
            self.say(f"Mission [bold]{run.nav.status}[/] · {run.result.samples} samples in "
                     f"{run.result.duration_s:.1f} sim s · {run.result.collisions} collisions")
            import json

            from lang2sim.telemetry.benchmark import score_run_folder

            traj_file = run_dir / "logs" / f"trajectory_{label}.json"
            if not traj_file.is_file():      # the container could not write into the run folder
                traj_file.parent.mkdir(parents=True, exist_ok=True)
                traj_file.write_text(json.dumps(run.result.trajectory), encoding="utf-8")
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
                self.say(f"[#ef4444]■[/] Stress: {tr.get('model_name')} dropped at ({p.get('x')}, {p.get('y')})"
                         + (f" [#ef4444]failed: {err}[/]" if err else ""))

            def phase(m: str) -> None:
                self.progress(None)
                self.say(f"[bold #38bdf8]▸ {m}[/]")

            report = await s.run_benchmark(run_dir, light=light, on_progress=self._nav_progress, on_trigger=trig,
                                           on_phase=phase)
            self.progress(None)
            self.show_report(report)
            self.say(f"[#8b98a8]Report: {report.get('path')}[/]")
        finally:
            await s.stop()
            self.mission_lock.release()

    async def cmd_replay(self, cmd: Command) -> None:
        target = Path(cmd.args[0]).expanduser() if cmd.args and cmd.args[0] != "demo" else DEMO_RUN
        port = int(self.base_url.rsplit(":", 1)[-1]) if self.base_url.rsplit(":", 1)[-1].isdigit() else 9090
        try:
            from lang2sim.telemetry.live import _get_json

            if _get_json(f"{self.base_url}/v1/health", 0.5) is not None:
                self.say(f"[#f39c12]Something already answers on {self.base_url}.[/] Stop the live sim "
                         "(or the other replay) first.")
                return
            import uvicorn

            from lang2sim.telemetry.replay import create_replay_app
        except ImportError as e:
            self.say(f"[#ef4444]✗ Replay needs the [sim] extra:[/] pip install -e '.[sim]' ({e})")
            return
        app = create_replay_app(target, loop=True)
        server = uvicorn.Server(uvicorn.Config(app, host="127.0.0.1", port=port, log_level="warning"))
        threading.Thread(target=server.run, name="l2s-replay", daemon=True).start()
        self._replay_server = server
        runs = ", ".join(f"{r.label} ({r.duration:.0f} s)" for r in app.state.clock.runs)
        self.say(f"[#f39c12]▶ Replay[/] of [bold]{target.name}[/] on {self.base_url}: {runs}, looping. "
                 "The sidebar, dashboard and score table now use it.")

    # --- helpers -------------------------------------------------------------
    async def _stress_when_moving(self, stress: "Command", min_travel_m: float = 1.5,
                                  max_wait_s: float = 120.0) -> None:
        """Apply a stress request once the recorded robot has driven min_travel_m (it is mid-leg)."""
        s = self._session()
        try:
            start = None
            loop = asyncio.get_running_loop()
            t_end = loop.time() + max_wait_s
            while loop.time() < t_end:
                await asyncio.sleep(0.5)
                try:
                    live = await s.api.telemetry_live()
                except Exception:  # noqa: BLE001 - transient
                    continue
                if not live.recording or live.pose is None:
                    continue
                if start is None:
                    start = (live.pose.x, live.pose.y)
                elif math.hypot(live.pose.x - start[0], live.pose.y - start[1]) >= min_travel_m:
                    break
        finally:
            await s.stop()
        await self.run(stress)

    async def _recorded_path(self, s) -> List[Tuple[float, float]]:
        """The nominal run's path in this run folder (the robot's real lane), or []."""
        import json

        try:
            run_dir = await self._host_run_dir(s)
            data = json.loads((run_dir / "logs" / "trajectory_nominal.json").read_text())
        except Exception:  # noqa: BLE001 - no nominal run yet: fall back to the heading
            return []
        return [(p["x"], p["y"]) for p in data.get("samples") or []]

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
            self.progress(f"[#8b98a8]  ▸ driving · waypoint {st.waypoint_index + 1}/{st.waypoints_total} · "
                          f"{st.distance_remaining:.1f} m to go[/]")
        else:
            self.progress(None)
