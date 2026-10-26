"""
Self-healing supervisor: classify -> plan_recovery -> rate-limited executor.
File: lang2sim/runtime/self_healing.py

Runs inside the container, in the API server, every few seconds while a sim is
up. Inputs are what the server already has: new sim log lines, the ROS
snapshot/readiness, the nav status and which node processes are alive.

Four rules (issue #3, plan v2):

| Fault            | Seen as                                                  | Recovery                          |
|------------------|----------------------------------------------------------|-----------------------------------|
| missing_package  | "Package 'x' not found" / ModuleNotFoundError in the log | report only: never install at runtime |
| missing_tf       | robot_state_publisher dead (process gone, "process has   | restart robot_state_publisher     |
|                  | died", or TF errors while it is not running)             | (TF errors alone: reported once)  |
| unlinked_scan    | ros_gz_bridge dead, or no /scan while the clock runs     | restart the bridge                |
| goal_in_collision| planner says the goal is occupied / no valid path        | move the goal to the nearest free |
|                  |                                                          | map cell and resend it            |

Each action is rate-limited per fault (cooldown + max attempts in a window) and
written to <run>/logs/heal_log.jsonl as a HealEvent plus details.
"""

from __future__ import annotations

import json
import math
import os
import re
import subprocess
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Dict, Iterable, List, Optional, Tuple

from lang2sim.core.models import HealEvent, NavStatus, Pose2D
from lang2sim.runtime.ros_bridge import RosSnapshot

# ---------------------------------------------------------------------------
# Faults
# ---------------------------------------------------------------------------

PKG_PATTERNS = (
    re.compile(r"[Pp]ackage '([\w\-]+)' not found"),
    re.compile(r"PackageNotFoundError: .*?\"?'?([\w\-]+)'?\"?$"),
    re.compile(r"No module named '([\w\.]+)'"),
)
TF_PATTERNS = (
    re.compile(r"Could not find a connection between '(\w+)' and '(\w+)'"),
    re.compile(r"Timed out waiting for transform from (\w+) to (\w+)"),
    re.compile(r'"(\w+)" passed to lookupTransform argument \w+ does not exist'),
)
GOAL_PATTERNS = (
    re.compile(r"[Gg]oal (?:coordinates|pose)?.*(?:occupied|in collision|lethal)"),
    re.compile(r"(?:GridBased|NavFn|Smac).*[Ff]ailed to create (?:a )?plan"),
    re.compile(r"[Ff]ailed to create plan with tolerance"),
)
DIED = re.compile(r"\[(robot_state_publisher|parameter_bridge|ros_gz_bridge)[\w\-]*\]: process has died")

# healer name -> (ROS package, executable); processes are matched on "lib/<pkg>/<exe>"
NODES = {"robot_state_publisher": ("robot_state_publisher", "robot_state_publisher"),
         "bridge": ("ros_gz_bridge", "parameter_bridge")}
SCAN_GRACE_S = 5.0
TF_GRACE_S = 10.0


@dataclass(frozen=True)
class Fault:
    kind: str          # missing_package | missing_tf | unlinked_scan | goal_in_collision
    signature: str     # stable text used for rate limiting and the log
    evidence: str


@dataclass
class Observation:
    lines: List[str]                            # new log lines since the last check
    snap: Optional[RosSnapshot]
    alive: Dict[str, bool]                      # node process name -> running
    nav: Optional[NavStatus] = None
    sim_running: bool = True
    was_localized: bool = False                 # robot was localized earlier in this run


def classify(obs: Observation) -> List[Fault]:
    faults: Dict[str, Fault] = {}

    def add(kind: str, sig: str, ev: str) -> None:
        faults.setdefault(f"{kind}:{sig}", Fault(kind, sig, ev))

    for line in obs.lines:
        for p in PKG_PATTERNS:
            m = p.search(line)
            if m:
                add("missing_package", m.group(1), line.strip()[:240])
        m = DIED.search(line)
        if m:
            kind = "missing_tf" if m.group(1) == "robot_state_publisher" else "unlinked_scan"
            add(kind, f"{m.group(1)} died", line.strip()[:240])
        if obs.nav is not None and (obs.nav.is_active or obs.nav.status == "failed"):
            for p in GOAL_PATTERNS:
                if p.search(line):
                    add("goal_in_collision", "goal not plannable", line.strip()[:240])
        for p in TF_PATTERNS:
            if p.search(line) and obs.was_localized:
                # Nav2 keeps logging these for a while after robot_state_publisher is
                # restarted; only a dead publisher justifies restarting it. Otherwise
                # report once, never kill a healthy node over stale warnings.
                if obs.alive.get("robot_state_publisher") is False:
                    add("missing_tf", "tf lookup failing", line.strip()[:240])
                else:
                    add("tf_warning", "tf lookups failing", line.strip()[:240])

    if obs.sim_running:
        if obs.alive.get("robot_state_publisher") is False:
            add("missing_tf", "robot_state_publisher not running", "process not found")
        if obs.alive.get("bridge") is False:
            add("unlinked_scan", "ros_gz_bridge not running", "process not found")
        s = obs.snap
        if s is not None and s.fresh(s.clock_wall):
            if s.scan_wall is None or s.now - s.scan_wall > SCAN_GRACE_S:
                if obs.was_localized:  # scan worked before in this run
                    add("unlinked_scan", "no /scan while the clock runs",
                        f"last scan {'never' if s.scan_wall is None else f'{s.now - s.scan_wall:.0f} s ago'}")
    return list(faults.values())


# ---------------------------------------------------------------------------
# Recovery plans
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class Action:
    kind: str                       # report | restart_node | snap_goal
    target: str = ""
    detail: Dict[str, object] = field(default_factory=dict)


def plan_recovery(f: Fault) -> Action:
    if f.kind == "missing_package":
        return Action("report", f.signature, {"hint": "the sim image is missing this; rebuild it - "
                                                       "nothing is installed at runtime"})
    if f.kind == "missing_tf":
        return Action("restart_node", "robot_state_publisher")
    if f.kind == "unlinked_scan":
        return Action("restart_node", "bridge")
    if f.kind == "goal_in_collision":
        return Action("snap_goal")
    if f.kind == "tf_warning":
        return Action("report", f.signature, {"hint": "TF lookups failed while robot_state_publisher "
                                                       "was running; no restart (often transient)"})
    return Action("report", f.signature)


# ---------------------------------------------------------------------------
# Map helpers (goal snapping)
# ---------------------------------------------------------------------------

def read_map(map_yaml: Path) -> Tuple[List[bytes], int, int, float, Tuple[float, float], float]:
    """Parse a Nav2 map.yaml + binary PGM. Returns rows (top first), w, h, resolution, origin, free_thresh."""
    meta: Dict[str, str] = {}
    for line in Path(map_yaml).read_text().splitlines():
        if ":" in line:
            k, v = line.split(":", 1)
            meta[k.strip()] = v.strip()
    res = float(meta["resolution"])
    ox, oy = [float(x) for x in meta["origin"].strip("[]").split(",")[:2]]
    negate = meta.get("negate", "0").strip() == "1"
    free_thresh = float(meta.get("free_thresh", "0.196"))
    data = (Path(map_yaml).parent / meta["image"]).read_bytes()
    tokens, i = [], 0
    while len(tokens) < 4:                       # P5 header: magic, w, h, maxval (skip comments)
        while data[i:i + 1].isspace():
            i += 1
        if data[i:i + 1] == b"#":
            while data[i:i + 1] not in (b"\n", b""):
                i += 1
            continue
        j = i
        while not data[j:j + 1].isspace():
            j += 1
        tokens.append(data[i:j])
        i = j
    if tokens[0] != b"P5":
        raise ValueError("only binary PGM (P5) maps are supported")
    w, h = int(tokens[1]), int(tokens[2])
    pix = data[i + 1:i + 1 + w * h]
    if negate:
        pix = bytes(255 - b for b in pix)
    rows = [pix[r * w:(r + 1) * w] for r in range(h)]
    return rows, w, h, res, (ox, oy), free_thresh


def nearest_free(map_yaml: Path, x: float, y: float, clearance: float = 0.45,
                 max_radius: float = 3.0) -> Optional[Pose2D]:
    """Closest map point to (x, y) with every cell within `clearance` free. None if none nearby."""
    rows, w, h, res, (ox, oy), free_thresh = read_map(map_yaml)
    free_min = int((1 - free_thresh) * 255)      # brighter than this = free

    def free(c: int, r: int) -> bool:
        return 0 <= c < w and 0 <= r < h and rows[r][c] >= free_min

    k = int(math.ceil(clearance / res))
    disk = [(dc, dr) for dc in range(-k, k + 1) for dr in range(-k, k + 1) if dc * dc + dr * dr <= k * k]

    def clear(c: int, r: int) -> bool:
        return all(free(c + dc, r + dr) for dc, dr in disk)

    c0, r0 = int((x - ox) / res), h - 1 - int((y - oy) / res)
    for ring in range(0, int(max_radius / res) + 1):
        best = None
        for dc in range(-ring, ring + 1):
            for dr in (-ring, ring) if abs(dc) != ring else range(-ring, ring + 1):
                c, r = c0 + dc, r0 + dr
                if clear(c, r):
                    d = dc * dc + dr * dr
                    if best is None or d < best[0]:
                        best = (d, c, r)
        if best:
            _, c, r = best
            return Pose2D(x=round(ox + (c + 0.5) * res, 3), y=round(oy + (h - 1 - r + 0.5) * res, 3))
    return None


# ---------------------------------------------------------------------------
# Executor
# ---------------------------------------------------------------------------

@dataclass
class RateLimit:
    cooldown_s: float = 15.0
    max_attempts: int = 3
    window_s: float = 300.0


class SelfHealer:
    """Plans and executes recoveries. Side effects are injected so tests can fake them."""

    def __init__(self, run_dir: Path, *, restart_node: Callable[[str], bool],
                 resend_goal: Optional[Callable[[Pose2D], bool]] = None,
                 current_goal: Optional[Callable[[], Optional[Pose2D]]] = None,
                 map_yaml: Optional[Path] = None, limit: Optional[RateLimit] = None,
                 clock: Callable[[], float] = time.time) -> None:
        self.run_dir = Path(run_dir)
        self.log_path = self.run_dir / "logs" / "heal_log.jsonl"
        self._restart = restart_node
        self._resend = resend_goal
        self._goal = current_goal
        self._map = map_yaml
        self.limit = limit or RateLimit()
        self._clock = clock
        self._history: Dict[str, List[float]] = {}
        self._reported: set = set()
        self.events: List[dict] = []

    def _allowed(self, sig: str) -> Tuple[bool, str]:
        now = self._clock()
        hist = [t for t in self._history.get(sig, []) if now - t <= self.limit.window_s]
        self._history[sig] = hist
        if hist and now - hist[-1] < self.limit.cooldown_s:
            return False, "cooldown"
        if len(hist) >= self.limit.max_attempts:
            return False, "attempt limit reached"
        return True, ""

    def handle(self, faults: Iterable[Fault]) -> List[dict]:
        """Plan and run recoveries. Rate limits are per recovery action (e.g. 'restart_node bridge'),
        so several symptoms of one failure trigger one restart, at most once per pass."""
        out: List[dict] = []
        done_this_pass: set = set()
        for f in faults:
            action = plan_recovery(f)
            if action.kind == "report":
                key = f"{f.kind}:{f.signature}"
                if key in self._reported:
                    continue
                self._reported.add(key)
                out.append(self._log(f, action, success=False, note=str(action.detail.get("hint", ""))))
                continue
            key = f"{action.kind}:{action.target}"
            if key in done_this_pass:
                continue
            done_this_pass.add(key)
            ok, why = self._allowed(key)
            if not ok:
                if why == "attempt limit reached" and f"{key}:gaveup" not in self._reported:
                    self._reported.add(f"{key}:gaveup")
                    out.append(self._log(f, Action("give_up", action.target), success=False,
                                         note=f"{self.limit.max_attempts} attempts in "
                                              f"{self.limit.window_s:.0f} s; needs a person"))
                continue
            self._history.setdefault(key, []).append(self._clock())
            success, note = self._execute(action)
            out.append(self._log(f, action, success, note))
        return out

    def _execute(self, a: Action) -> Tuple[bool, str]:
        try:
            if a.kind == "restart_node":
                return bool(self._restart(a.target)), f"restarted {a.target}"
            if a.kind == "snap_goal":
                goal = self._goal() if self._goal else None
                if goal is None or self._map is None or self._resend is None:
                    return False, "no active goal or map to snap against"
                free = nearest_free(self._map, goal.x, goal.y)
                if free is None:
                    return False, f"no free cell within 3 m of ({goal.x}, {goal.y})"
                free = free.model_copy(update={"yaw": goal.yaw})
                return bool(self._resend(free)), f"goal moved ({goal.x}, {goal.y}) -> ({free.x}, {free.y})"
        except Exception as e:  # noqa: BLE001 - a failed recovery must never take the server down
            return False, f"{type(e).__name__}: {e}"
        return False, f"unknown action {a.kind}"

    def _log(self, f: Fault, a: Action, success: bool, note: str = "") -> dict:
        ev = HealEvent(timestamp=round(self._clock(), 3), error_signature=f"{f.kind}: {f.signature}",
                       action_taken=f"{a.kind} {a.target}".strip(), success=success)
        rec = {**ev.model_dump(), "evidence": f.evidence, "note": note}
        self.events.append(rec)
        try:
            self.log_path.parent.mkdir(parents=True, exist_ok=True)
            with open(self.log_path, "a") as fh:
                fh.write(json.dumps(rec) + "\n")
        except OSError:
            pass
        return rec


# ---------------------------------------------------------------------------
# Container-side node control (used by the API server)
# ---------------------------------------------------------------------------

class NodeControl:
    """Find and restart the nodes the healer knows how to replace."""

    def __init__(self, run_dir: Path, robot: str = "waffle") -> None:
        self.run_dir = Path(run_dir)
        self.robot = robot
        self._procs: List[subprocess.Popen] = []
        self._lock = threading.Lock()

    @staticmethod
    def _pgrep(pattern: str) -> bool:
        return subprocess.run(["pgrep", "-f", pattern], capture_output=True).returncode == 0

    def alive(self) -> Dict[str, bool]:
        return {name: self._pgrep(f"lib/{pkg}/{exe}") for name, (pkg, exe) in NODES.items()}

    def _command(self, name: str) -> List[str]:
        share = f"/opt/ros/{os.getenv('ROS_DISTRO', 'jazzy')}/share/turtlebot3_gazebo"
        model = f"turtlebot3_{self.robot}"
        if name == "robot_state_publisher":
            import yaml  # in the sim image (ROS dependency)

            urdf = Path(f"{share}/urdf/{model}.urdf").read_text()
            params = self.run_dir / "logs" / "rsp_params.l2s.yaml"
            params.write_text(yaml.safe_dump({"robot_state_publisher": {"ros__parameters": {
                "use_sim_time": True, "robot_description": urdf}}}))
            return ["ros2", "run", "robot_state_publisher", "robot_state_publisher",
                    "--ros-args", "--params-file", str(params)]
        if name == "bridge":
            return ["ros2", "run", "ros_gz_bridge", "parameter_bridge", "--ros-args",
                    "-p", f"config_file:={share}/params/{model}_bridge.yaml", "-p", "use_sim_time:=true"]
        raise ValueError(f"no restart recipe for {name}")

    def restart(self, name: str) -> bool:
        pkg, exe = NODES[name]
        subprocess.run(["pkill", "-f", f"lib/{pkg}/{exe}"], capture_output=True)
        with self._lock:
            log = open(self.run_dir / "logs" / f"healed_{name}.log", "a")
            self._procs.append(subprocess.Popen(self._command(name), stdout=log, stderr=subprocess.STDOUT,
                                                start_new_session=True))
        for _ in range(50):
            if self.alive().get(name):
                return True
            time.sleep(0.1)
        return False

    def stop_all(self) -> None:
        with self._lock:
            for p in self._procs:
                if p.poll() is None:
                    try:
                        os.killpg(p.pid, 2)
                        p.wait(timeout=5)
                    except Exception:  # noqa: BLE001
                        p.kill()
            self._procs.clear()
