"""
In-container telemetry recorder: kinematics + collision events for NQI scoring.
File: lang2sim/runtime/telemetry_recorder.py

The Telemetry Engine (architecture §4.1) consumes /odom, /tf and the chassis
contact topic and emits live metrics plus the trajectories the NQI scorer
(lang2sim.telemetry.nqi_scorer) turns into benchmark_report.json.

Why this lives in runtime/ and not telemetry/: it runs inside the sim container,
fed by ros_bridge.RosBridge, and must import with the standard library only
(lang2sim.telemetry pulls in numpy, Pillow and mss, which the sim image may not
ship). Its output is exactly Trajectory.to_dict(), so the host side loads it with
Trajectory.from_dict() and scores it unchanged.

Three rules keep the numbers comparable across machines and runs:
  * time is sim time (/clock, message stamps), never wall time: S_tau is a jerk
    integral, so wall-clock stamps would make a slow Mac (RTF 0.3) score far
    worse than a GPU box for the same motion;
  * poses are world-frame (== Nav2 map frame). Preference order: Gazebo ground
    truth > AMCL map pose > /odom shifted by the spawn pose (odom's origin is the
    spawn point, so raw /odom is wrong whenever the robot does not start at 0,0);
  * collisions are rising-edge contact events on the chassis, counted in sim
    time, ignoring the ground and the robot itself.

Crash safety: with a `stream_path`, every kept sample (and every collision) is
also appended to a JSON-lines file as it happens, so a run whose sim or API dies
mid-mission can still be scored (load_partial / telemetry.benchmark CLI).

Everything here is pure and unit-tested on the host.
"""

from __future__ import annotations

import json
import math
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

Pose = Tuple[float, float, float]  # x, y, yaw

FORCE_THRESHOLD_N = 5.0  # architecture §7.4 / Issue #4: force events > 5 N
CONTACT_COOLDOWN_S = 0.5  # sim seconds between two counted collisions
CONTACT_RELEASE_S = 0.25  # no contact message for this long -> contact ended
LIDAR_TO_CHASSIS_EDGE_M = (
    0.13  # TB3 Waffle: lidar sits over the base box centre (half-width 0.1325)
)
IGNORED_CONTACT_ENTITIES = ("ground_plane", "ground", "floor", "world")


def wrap_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def world_from_odom(odom: Pose, spawn: Pose) -> Pose:
    """Odom frame starts at the spawn pose; rotate+translate an odom pose into the world frame."""
    sx, sy, syaw = spawn
    x, y, yaw = odom
    c, s = math.cos(syaw), math.sin(syaw)
    return (sx + c * x - s * y, sy + s * x + c * y, wrap_angle(syaw + yaw))


def best_pose(
    ground_truth: Optional[Pose],
    map_pose: Optional[Pose],
    odom_pose: Optional[Pose],
    spawn: Optional[Pose],
) -> Tuple[Optional[Pose], str]:
    """(world pose, source) from the most trustworthy source available."""
    if ground_truth is not None:
        return ground_truth, "ground_truth"
    if map_pose is not None:
        return map_pose, "map"
    if odom_pose is not None:
        return world_from_odom(odom_pose, spawn or (0.0, 0.0, 0.0)), "odom"
    return None, "none"


def min_valid_range(ranges: Iterable[float], range_min: float, range_max: float) -> Optional[float]:
    """Closest finite lidar return inside [range_min, range_max], or None."""
    best = math.inf
    for r in ranges:
        if math.isfinite(r) and range_min <= r <= range_max and r < best:
            best = r
    return None if math.isinf(best) else best


# --------------------------------------------------------------------------- contacts
@dataclass
class ContactEvent:
    t: float
    entity: str
    force_n: Optional[float]
    source: str  # "wrench" | "geometric"

    def to_dict(self) -> Dict[str, Any]:
        return {
            "t": round(self.t, 3),
            "entity": self.entity,
            "force_n": None if self.force_n is None else round(self.force_n, 2),
            "source": self.source,
        }


PARTIAL_SUFFIX = ".partial.jsonl"


class ContactMonitor:
    """Counts chassis collisions (C_force) from contact messages, in sim time.

    Each message is a list of (other_entity_name, force_newtons_or_None). Gazebo
    Harmonic's contact sensor often reports contacts without wrench data; with
    `geometric_fallback` such a contact still counts (force unknown), otherwise it
    is ignored. A contact counts once on its rising edge; it ends after
    CONTACT_RELEASE_S without contact messages (the sensor only publishes while
    touching). Self-contacts and the ground are ignored.
    """

    def __init__(
        self,
        robot_name: str = "",
        threshold_n: float = FORCE_THRESHOLD_N,
        cooldown_s: float = CONTACT_COOLDOWN_S,
        release_s: float = CONTACT_RELEASE_S,
        geometric_fallback: bool = True,
        ignored: Sequence[str] = IGNORED_CONTACT_ENTITIES,
    ) -> None:
        self.on_event = None  # callable(ContactEvent), e.g. the recorder's stream
        self.robot_name = robot_name
        self.threshold_n = threshold_n
        self.cooldown_s = cooldown_s
        self.release_s = release_s
        self.geometric_fallback = geometric_fallback
        self.ignored = tuple(i.lower() for i in ignored)
        self.reset()

    def reset(self) -> None:
        self.count = 0
        self.events: List[ContactEvent] = []
        self.in_contact = False
        self.last_contact_t: Optional[float] = None
        self.last_counted_t: Optional[float] = None
        self.last_force_n = 0.0
        self.last_entity = ""
        self.force_source = "none"

    def _relevant(self, entity: str) -> bool:
        top = entity.lower().split("::")[0]  # scoped names: model::link::collision
        if top in self.ignored:
            return False
        if self.robot_name and top == self.robot_name.lower():
            return False
        return True

    def tick(self, t: float) -> None:
        """Call regularly (each sample) so a contact that stopped publishing is released."""
        if (
            self.in_contact
            and self.last_contact_t is not None
            and t - self.last_contact_t > self.release_s
        ):
            self.in_contact = False

    def process(self, contacts: Sequence[Tuple[str, Optional[float]]], t: float) -> bool:
        """Feed one contact message; returns True when it registers a new collision."""
        self.tick(t)
        hits: List[Tuple[str, Optional[float]]] = []
        for entity, force in contacts:
            if not self._relevant(entity):
                continue
            if force is not None and force > 0.0:
                self.force_source = "wrench"
                if force > self.threshold_n:
                    hits.append((entity, force))
            elif self.geometric_fallback:
                if self.force_source != "wrench":
                    self.force_source = "geometric"
                hits.append((entity, None))
        if not hits:
            return False
        entity, force = max(hits, key=lambda h: -1.0 if h[1] is None else h[1])
        self.last_force_n = 0.0 if force is None else force
        self.last_entity = entity
        self.last_contact_t = t
        new = not self.in_contact and (
            self.last_counted_t is None or t - self.last_counted_t >= self.cooldown_s
        )
        self.in_contact = True
        if new:
            self.count += 1
            self.last_counted_t = t
            ev = ContactEvent(t, entity, force, "wrench" if force is not None else "geometric")
            self.events.append(ev)
            if self.on_event is not None:
                try:
                    self.on_event(ev)
                except Exception:  # noqa: BLE001 - a logging failure must not lose the count
                    pass
        return new


# --------------------------------------------------------------------------- recorder
@dataclass
class _Run:
    label: str
    goal: Optional[Pose]
    rate_hz: float
    spawn: Optional[Pose]
    start_count: int
    samples: List[Dict[str, float]] = field(default_factory=list)
    sources: Dict[str, int] = field(default_factory=dict)
    min_scan: Optional[float] = None
    last_t: Optional[float] = None
    stream: Any = None  # open JSON-lines file, or None
    stream_path: Optional[Path] = None


class TelemetryRecorder:
    """Thread-safe trajectory recorder fed by the ROS bridge's callbacks.

    `offer()` is called for every pose update (30 Hz ground truth, or /odom);
    samples are kept at `rate_hz` in sim time. `stop()` returns a dict in
    lang2sim.telemetry.Trajectory.to_dict() format.
    """

    def __init__(self, contacts: Optional[ContactMonitor] = None) -> None:
        self._lock = threading.Lock()
        self.contacts = contacts or ContactMonitor()
        self._run: Optional[_Run] = None

    @property
    def recording(self) -> bool:
        return self._run is not None

    @property
    def label(self) -> Optional[str]:
        run = self._run
        return run.label if run else None

    @property
    def goal(self) -> Optional[Pose]:
        run = self._run
        return run.goal if run else None

    @property
    def sample_count(self) -> int:
        run = self._run
        return len(run.samples) if run else 0

    def collisions(self) -> int:
        run = self._run
        return self.contacts.count - (run.start_count if run else 0)

    def start(
        self,
        label: str,
        goal: Optional[Pose] = None,
        rate_hz: float = 20.0,
        spawn: Optional[Pose] = None,
        stream_path: Optional[Path] = None,
    ) -> None:
        """Begin a recording (replacing any in progress). Collisions are counted from here.

        With `stream_path`, samples and collisions are also appended there as JSON lines."""
        self.stop()
        stream = None
        if stream_path is not None:
            try:
                Path(stream_path).parent.mkdir(parents=True, exist_ok=True)
                stream = open(stream_path, "w", buffering=1, encoding="utf-8")  # line-buffered
                stream.write(
                    json.dumps(
                        {
                            "type": "header",
                            "label": label,
                            "rate_hz": max(1.0, rate_hz),
                            "goal": list(goal) if goal else None,
                            "spawn": list(spawn) if spawn else None,
                            "wall_time": time.time(),
                        }
                    )
                    + "\n"
                )
            except OSError:
                stream = None
        with self._lock:
            self._run = _Run(
                label=label,
                goal=goal,
                rate_hz=max(1.0, rate_hz),
                spawn=spawn,
                start_count=self.contacts.count,
                stream=stream,
                stream_path=Path(stream_path) if stream is not None else None,
            )
        self.contacts.on_event = self._stream_event if stream is not None else None

    def _write(self, run: _Run, record: Dict[str, Any]) -> None:
        if run.stream is None:
            return
        try:
            run.stream.write(json.dumps(record) + "\n")
        except (OSError, ValueError):
            run.stream = None

    def _stream_event(self, ev: ContactEvent) -> None:
        run = self._run
        if run is not None:
            self._write(run, {"type": "contact", **ev.to_dict()})

    def offer(
        self,
        t: float,
        pose: Optional[Pose],
        source: str,
        v: float,
        w: float,
        estimate: Optional[Pose] = None,
        scan_min: Optional[float] = None,
    ) -> bool:
        """One pose update. `estimate` is AMCL's map pose (kept only when `pose` is ground truth,
        so the trajectory's slam_x/slam_y/ate columns measure localization error).
        Returns True if the sample was kept."""
        with self._lock:
            run = self._run
            if run is None or pose is None:
                return False
            if scan_min is not None:
                run.min_scan = scan_min if run.min_scan is None else min(run.min_scan, scan_min)
            if run.last_t is not None:
                if t < run.last_t - 1e-6:  # sim time went backwards: world reset mid-run
                    return False
                if t - run.last_t < 1.0 / run.rate_hz - 1e-6:
                    return False
            s: Dict[str, float] = {
                "t": float(t),
                "x": float(pose[0]),
                "y": float(pose[1]),
                "yaw": float(pose[2]),
                "v": float(v),
                "w": float(w),
            }
            if source == "ground_truth" and estimate is not None:
                s["slam_x"], s["slam_y"] = float(estimate[0]), float(estimate[1])
            run.samples.append(s)
            run.sources[source] = run.sources.get(source, 0) + 1
            run.last_t = t
            self._write(run, {"type": "sample", "source": source, "scan_min": scan_min, **s})
            return True

    def stop(self, keep_partial: bool = False) -> Optional[Dict[str, Any]]:
        """End the recording; None if nothing was recording.

        The streamed partial file is removed unless `keep_partial` (the caller writes the
        final trajectory JSON next to it)."""
        with self._lock:
            run, self._run = self._run, None
        if run is None:
            return None
        self.contacts.on_event = None
        if run.stream is not None:
            try:
                run.stream.close()
            except OSError:
                pass
            if not keep_partial and run.stream_path is not None:
                try:
                    run.stream_path.unlink()
                except OSError:
                    pass
        events = [e.to_dict() for e in self.contacts.events[run.start_count :]]
        return assemble_trajectory(
            label=run.label,
            goal=run.goal,
            rate_hz=run.rate_hz,
            spawn=run.spawn,
            samples=run.samples,
            sources=run.sources,
            min_scan=run.min_scan,
            contact_force_source=self.contacts.force_source,
            events=events,
        )


def assemble_trajectory(
    *,
    label: str,
    goal: Optional[Sequence[float]],
    rate_hz: float,
    spawn: Optional[Sequence[float]],
    samples: List[Dict[str, float]],
    sources: Dict[str, int],
    min_scan: Optional[float],
    contact_force_source: str,
    events: List[Dict[str, Any]],
    partial: bool = False,
) -> Dict[str, Any]:
    """Trajectory.to_dict() format + telemetry meta, from recorded samples."""
    # slam_x/slam_y must be on every sample for Trajectory to keep the column.
    if samples and not all("slam_x" in s for s in samples):
        samples = [{k: v for k, v in s.items() if k not in ("slam_x", "slam_y")} for s in samples]
    main_source = max(sources, key=sources.get) if sources else "none"
    return {
        "label": label,
        "goal": [goal[0], goal[1]] if goal else None,
        "l_optimal": None,
        "collisions": len(events),
        "meta": {
            "time_base": "sim",
            "frame": "world",
            "rate_hz": rate_hz,
            "pose_source": main_source,
            "pose_sources": dict(sources),
            "spawn": list(spawn) if spawn else None,
            "goal_yaw": goal[2] if goal and len(goal) > 2 else None,
            "min_scan_range_m": None if min_scan is None else round(min_scan, 4),
            "contact_force_source": contact_force_source,
            "contact_events": events,
            "localization_error": "slam_x/slam_y hold AMCL's estimate vs Gazebo ground truth"
            if samples and "slam_x" in samples[0]
            else None,
            "partial": partial,
        },
        "samples": samples,
    }


def partial_path(logs_dir: Path, label: str) -> Path:
    return Path(logs_dir) / f"trajectory_{label}{PARTIAL_SUFFIX}"


def load_partial(path: Path) -> Optional[Dict[str, Any]]:
    """Rebuild a trajectory dict from a streamed .partial.jsonl (a run that never reached
    record/stop). Tolerates a torn last line. None if it holds fewer than 2 samples."""
    header: Dict[str, Any] = {}
    samples: List[Dict[str, float]] = []
    sources: Dict[str, int] = {}
    events: List[Dict[str, Any]] = []
    min_scan: Optional[float] = None
    force_source = "none"
    try:
        lines = Path(path).read_text(encoding="utf-8").splitlines()
    except OSError:
        return None
    for line in lines:
        try:
            rec = json.loads(line)
        except ValueError:
            continue  # torn write at the crash point
        kind = rec.pop("type", None)
        if kind == "header":
            header = rec
        elif kind == "sample":
            src = rec.pop("source", "none")
            sm = rec.pop("scan_min", None)
            if sm is not None:
                min_scan = sm if min_scan is None else min(min_scan, sm)
            sources[src] = sources.get(src, 0) + 1
            samples.append(rec)
        elif kind == "contact":
            events.append(rec)
            if rec.get("source") == "wrench" or force_source == "none":
                force_source = rec.get("source", force_source)
    if len(samples) < 2:
        return None
    data = assemble_trajectory(
        label=header.get("label", Path(path).name),
        goal=header.get("goal"),
        rate_hz=float(header.get("rate_hz", 20.0)),
        spawn=header.get("spawn"),
        samples=samples,
        sources=sources,
        min_scan=min_scan,
        contact_force_source=force_source,
        events=events,
        partial=True,
    )
    data["meta"]["recording_wall_time"] = header.get("wall_time")
    return data


def build_live(
    *,
    sim_time: float,
    rtf: Optional[float],
    pose: Optional[Pose],
    source: str,
    velocity: Optional[Tuple[float, float, float]],
    ground_truth: Optional[Pose],
    map_pose: Optional[Pose],
    scan_min: Optional[float],
    recorder: TelemetryRecorder,
    goal: Optional[Pose] = None,
) -> Dict[str, Any]:
    """Fields of core.models.TelemetryLive from the bridge state (kept as a dict: no pydantic here)."""
    goal = goal or recorder.goal
    loc_err = (
        math.hypot(map_pose[0] - ground_truth[0], map_pose[1] - ground_truth[1])
        if map_pose is not None and ground_truth is not None
        else None
    )
    c = recorder.contacts
    return {
        "sim_time": round(sim_time, 3),
        "real_time_factor": round(rtf, 3) if rtf is not None else 0.0,
        "pose": None
        if pose is None
        else {
            "x": round(pose[0], 3),
            "y": round(pose[1], 3),
            "yaw": max(-math.pi, min(math.pi, round(pose[2], 4))),
        },
        "pose_source": source,
        "linear_speed": round(velocity[0], 3) if velocity else 0.0,
        "angular_speed": round(velocity[2], 3) if velocity else 0.0,
        "localization_error_m": None if loc_err is None else round(loc_err, 3),
        "min_scan_range_m": None if scan_min is None else round(scan_min, 3),
        "obstacle_clearance_m": None
        if scan_min is None
        else round(max(0.0, scan_min - LIDAR_TO_CHASSIS_EDGE_M), 3),
        "distance_to_goal_m": (
            round(math.hypot(goal[0] - pose[0], goal[1] - pose[1]), 3)
            if goal is not None and pose is not None
            else None
        ),
        "collision_count": recorder.collisions() if recorder.recording else c.count,
        "last_contact_force_n": round(c.last_force_n, 2),
        "last_contact_entity": c.last_entity,
        "contact_force_source": c.force_source,
        "recording": recorder.recording,
        "recording_label": recorder.label,
        "samples_recorded": recorder.sample_count,
    }


# --------------------------------------------------------------------------- run folders
LABELS = ("nominal", "adversarial")


def read_jsonl(path: Path) -> List[Dict[str, Any]]:
    out: List[Dict[str, Any]] = []
    try:
        for line in Path(path).read_text(encoding="utf-8").splitlines():
            try:
                out.append(json.loads(line))
            except ValueError:
                continue
    except OSError:
        pass
    return out


def load_recorded_runs(run_dir: Path) -> Dict[str, Dict[str, Any]]:
    """Recorded trajectories of a run folder, by label (nominal first).

    Uses logs/trajectory_<label>.json when the recording finished, else the streamed
    logs/trajectory_<label>.partial.jsonl of a run that was cut short (meta.partial = True).
    A partial run's obstacles come from logs/spawns.jsonl: everything spawned in its sim
    session (after the last session_start marker) before the next recording started.
    """
    logs = Path(run_dir) / "logs"
    found: Dict[str, Dict[str, Any]] = {}
    labels = list(LABELS) + sorted(
        {p.name[len("trajectory_") :].split(".")[0] for p in logs.glob("trajectory_*")}
        - set(LABELS)
    )
    for label in labels:
        final, partial = (
            logs / f"trajectory_{label}.json",
            logs / f"trajectory_{label}{PARTIAL_SUFFIX}",
        )
        data = None
        if final.is_file():
            try:
                data = json.loads(final.read_text(encoding="utf-8"))
            except ValueError:
                data = None
        if data is None and partial.is_file():
            data = load_partial(partial)
        if data is not None:
            found[label] = data
    records = read_jsonl(logs / "spawns.jsonl")
    sessions = sorted(r["wall_time"] for r in records if r.get("type") == "session_start")
    spawns = [r for r in records if r.get("type") != "session_start" and "x" in r]
    starts = sorted(
        (d["meta"].get("recording_wall_time") or 0.0, k)
        for k, d in found.items()
        if d["meta"].get("partial")
    )
    for i, (t0, label) in enumerate(starts):
        t1 = starts[i + 1][0] if i + 1 < len(starts) else float("inf")
        s0 = max([s for s in sessions if s <= t0], default=0.0)
        found[label]["meta"]["spawned_entities"] = [
            {**e, "during_run": e.get("wall_time", 0) >= t0}
            for e in spawns
            if s0 <= e.get("wall_time", 0) < t1
        ]
    return found
