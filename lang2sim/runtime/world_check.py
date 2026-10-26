"""
World pre-flight check and normalizer (decision 2).
File: lang2sim/runtime/world_check.py

Runs before every launch, on the host or in the container (stdlib only):

* errors   -> the run is refused with a readable report for Part 1
             (unparseable SDF, malformed <pose>, robot already in the world,
              unresolvable model:// URIs when search paths are given)
* warnings -> launch continues (old SDF version, Fuel models not cached)
* fixes    -> missing Gazebo systems the robot needs (Sensors with ogre2 for the
             lidar/camera, Imu, Physics, UserCommands, SceneBroadcaster) are
             added to a normalized copy; the original file is never modified.

Gazebo loads its default systems only when a world declares none, so a world
that declares some but not Sensors silently gets no lidar and Nav2 never works.
"""

from __future__ import annotations

import os
import re
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable, List, Literal, Optional, Sequence, Tuple

Level = Literal["error", "warning", "fixed"]

# (short name, plugin filename, plugin class, extra XML inside <plugin>)
REQUIRED_SYSTEMS: Tuple[Tuple[str, str, str, str], ...] = (
    ("physics", "gz-sim-physics-system", "gz::sim::systems::Physics", ""),
    ("user-commands", "gz-sim-user-commands-system", "gz::sim::systems::UserCommands", ""),
    ("scene-broadcaster", "gz-sim-scene-broadcaster-system", "gz::sim::systems::SceneBroadcaster", ""),
    ("sensors", "gz-sim-sensors-system", "gz::sim::systems::Sensors", "<render_engine>ogre2</render_engine>"),
    ("imu", "gz-sim-imu-system", "gz::sim::systems::Imu", ""),
)
ROBOT_HINTS = ("turtlebot", "waffle", "burger")
NORMALIZED_NAME = "world.l2s.sdf"


@dataclass(frozen=True)
class Issue:
    level: Level
    code: str
    message: str


@dataclass
class WorldReport:
    world_name: Optional[str] = None
    sdf_version: Optional[str] = None
    issues: List[Issue] = field(default_factory=list)
    missing_systems: List[str] = field(default_factory=list)
    model_uris: List[str] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return not any(i.level == "error" for i in self.issues)

    @property
    def errors(self) -> List[Issue]:
        return [i for i in self.issues if i.level == "error"]

    def summary(self) -> str:
        if not self.issues:
            return f"world '{self.world_name}': ok"
        lines = [f"world '{self.world_name}': {'ok' if self.ok else 'REJECTED'}"]
        lines += [f"  [{i.level}] {i.code}: {i.message}" for i in self.issues]
        return "\n".join(lines)


def _system_key(plugin: ET.Element) -> str:
    s = f"{plugin.get('filename', '')} {plugin.get('name', '')}".lower()
    for key, fname, cls, _ in REQUIRED_SYSTEMS:
        if key in s.replace("_", "-") or fname in s or cls.lower() in s:
            return key
    return ""


def _pose_ok(el: ET.Element) -> bool:
    text = (el.text or "").strip()
    if not text:
        return True
    parts = text.split()
    want = 7 if el.get("rotation_format", "").startswith("quat") else 6
    if len(parts) != want:
        return False
    try:
        [float(p) for p in parts]
    except ValueError:
        return False
    return True


def default_model_paths() -> List[Path]:
    paths = [Path(p) for p in os.environ.get("GZ_SIM_RESOURCE_PATH", "").split(":") if p]
    return [p for p in paths if p.is_dir()]


def check_world(sdf_text: str, model_paths: Optional[Sequence[Path]] = None) -> WorldReport:
    """Inspect a world. `model_paths` given -> model:// URIs must resolve there."""
    r = WorldReport()
    try:
        root = ET.fromstring(sdf_text)
    except ET.ParseError as e:
        r.issues.append(Issue("error", "xml", f"world.sdf is not valid XML: {e}"))
        return r
    if root.tag != "sdf":
        r.issues.append(Issue("error", "root", f"root element is <{root.tag}>, expected <sdf>"))
        return r
    r.sdf_version = root.get("version")
    try:
        if r.sdf_version and tuple(int(x) for x in r.sdf_version.split(".")[:2]) < (1, 9):
            r.issues.append(Issue("warning", "sdf-version",
                                  f"SDF {r.sdf_version}; Gazebo Harmonic expects 1.9+"))
    except ValueError:
        r.issues.append(Issue("warning", "sdf-version", f"unreadable SDF version '{r.sdf_version}'"))

    worlds = root.findall("world")
    if len(worlds) != 1:
        r.issues.append(Issue("error", "world-count", f"expected exactly one <world>, found {len(worlds)}"))
        return r
    world = worlds[0]
    r.world_name = world.get("name")
    if not r.world_name:
        r.issues.append(Issue("error", "world-name", "<world> has no name attribute"))

    # Malformed poses (e.g. '04.0 1.25 0 0 0' from a template typo)
    for model in world.iter("model"):
        for pose in model.findall("pose"):
            if not _pose_ok(pose):
                r.issues.append(Issue("error", "pose",
                                      f"model '{model.get('name')}' has a malformed pose '{(pose.text or '').strip()}'"))

    # Robot must not be part of the world (Part 3 spawns it)
    names = [m.get("name", "") for m in world.findall("model")]
    uris = [(u.text or "").strip() for u in world.iter("uri")]
    for n in names + uris:
        if any(h in n.lower() for h in ROBOT_HINTS) and "turtlebot3_world" not in n.lower():
            r.issues.append(Issue("error", "robot-in-world",
                                  f"'{n}' looks like a robot; the world must not contain one (Part 3 spawns it)"))
            break

    # Required systems
    present = {_system_key(p) for p in world.findall("plugin")}
    r.missing_systems = [k for k, *_ in REQUIRED_SYSTEMS if k not in present]
    if r.missing_systems:
        r.issues.append(Issue("fixed", "systems",
                              "added missing Gazebo systems: " + ", ".join(r.missing_systems)))

    # model:// resolution
    r.model_uris = sorted({u for u in uris if u.startswith("model://")})
    if model_paths is not None:
        for uri in r.model_uris:
            name = uri[len("model://"):].split("/")[0]
            if not any((Path(p) / name).is_dir() for p in model_paths):
                r.issues.append(Issue("error", "model-missing",
                                      f"{uri} not found in {', '.join(str(p) for p in model_paths) or 'any model path'}"))
    if any(u.startswith(("http://", "https://")) for u in uris):
        r.issues.append(Issue("warning", "fuel",
                              "world downloads models from Gazebo Fuel; it fails offline unless they are cached"))
    return r


def normalize_world(sdf_text: str, missing: Iterable[str]) -> str:
    """Insert the missing system plugins right after <world ...>; everything else untouched."""
    missing = set(missing)
    plugins = "".join(
        f'\n    <plugin filename="{fname}" name="{cls}">{extra}</plugin>'
        for key, fname, cls, extra in REQUIRED_SYSTEMS if key in missing
    )
    if not plugins:
        return sdf_text
    m = re.search(r"<world\b[^>]*>", sdf_text)
    if not m:
        return sdf_text
    return sdf_text[:m.end()] + "\n    <!-- added by lang2sim world_check -->" + plugins + sdf_text[m.end():]


def prepare_world(world_path: Path, out_dir: Optional[Path] = None,
                  model_paths: Optional[Sequence[Path]] = None) -> Tuple[WorldReport, Optional[Path]]:
    """Check a world file; on success return the path Gazebo should load.

    That is the original file when nothing was missing, else a normalized copy
    written next to it (or in out_dir).
    """
    world_path = Path(world_path)
    text = world_path.read_text(encoding="utf-8")
    report = check_world(text, model_paths)
    if not report.ok:
        return report, None
    if not report.missing_systems:
        return report, world_path
    out = Path(out_dir or world_path.parent) / NORMALIZED_NAME
    out.write_text(normalize_world(text, report.missing_systems), encoding="utf-8")
    return report, out
