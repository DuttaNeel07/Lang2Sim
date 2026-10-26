"""
End-to-end pipeline (pitch build): prompt -> SimSpec -> Part 1 world + map -> run folder -> Part 3 sim.
File: sim2lang/pipeline.py

TEMPORARY wiring for the mid-review demo (branch demo/pitch-e2e). It connects the
layers that exist today so one prompt drives a real Gazebo + Nav2 run:

  1. prompt_to_spec(): the LLM (NemotronClient) is asked for a SimSpec JSON; if no
     API key is set or its answer does not validate, a keyword parser builds one
     from known-good layouts, so the demo never depends on the network.
  2. build_run(): Part 1's SIR generator/validator, Harmonic SDF compiler and map
     slicer write a complete run folder (world.sdf, map, spec.json, manifest.json).
  3. Part 3 (SimSession / scripts/run_sim.py) opens the run and drives the mission.
"""

from __future__ import annotations

import importlib
import json
import re
from pathlib import Path
from typing import List, Optional, Tuple

from sim2lang.core.models import (
    EnvironmentSpec,
    MissionSpec,
    ObjectSpec,
    Pose2D,
    RunManifest,
    SimSpec,
    SkyWeatherSpec,
    Vector3D,
)

cfg = importlib.import_module("sim2lang.core.config")

# --- known-good layouts (all verified to navigate end to end) ---------------------

RACK_SLOTS = [(-2.5, 2.5), (2.5, 2.5), (-2.5, -0.5), (2.5, -0.5)]
CRATE_SLOTS = [(4.5, -3.5), (-4.5, 0.9), (4.5, 4.2)]
WORDS = {"one": 1, "two": 2, "three": 3, "four": 4, "a": 1, "an": 1, "single": 1}

SPEC_SYSTEM_PROMPT = """You convert a robotics test scenario into JSON for a Gazebo simulation.
Reply with ONLY one JSON object, no prose, no code fences, in exactly this shape:
{"environment": {"environment_type": "indoor_warehouse"|"indoor_office", "room_dimensions": {"x": <8-14>, "y": <6-12>, "z": 3},
  "ambient_light_luminance": <0.15-1.0>, "surface_friction": <0.15-1.2>},
 "objects": [{"id": "rack_1", "model_name": "aws_warehouse_rack"|"wooden_crate"|"workbench", "dimensions": {"x":..,"y":..,"z":..},
              "pose": {"x":..,"y":..,"yaw": 0}}],
 "mission": {"initial_pose": {"x":..,"y":..,"yaw":0}, "waypoints": [{"x":..,"y":..}], "goal_pose": {"x":..,"y":..,"yaw":0},
             "timeout_seconds": 300}}
Rules: the room is centred on (0,0); keep every object and pose 0.8 m inside the walls; objects must not overlap;
leave aisles at least 2.2 m wide; racks are 3.0 x 0.8 x 2.0 m, crates 0.8 x 0.8 x 0.8 m; every waypoint and the goal
must be at least 0.6 m from any object; at most 4 waypoints."""


def _count(prompt: str, noun: str, default: int) -> int:
    m = re.search(rf"(\d+|one|two|three|four|a|an|single)\s+(?:\w+\s+)?(?:{noun})\b", prompt)
    if not m:
        return default
    tok = m.group(1)
    return int(tok) if tok.isdigit() else WORDS.get(tok, default)


def rule_spec(prompt: str) -> SimSpec:
    """Deterministic fallback: known-good layouts adjusted by keywords in the prompt."""
    p = prompt.lower()
    dim = any(w in p for w in ("dark", "dim", "night", "low light"))
    slippery = any(w in p for w in ("slippery", "wet", "ice", "oily"))
    env_kw = dict(ambient_light_luminance=0.25 if dim else 1.0, surface_friction=0.2 if slippery else 0.8,
                  sky_weather=SkyWeatherSpec(sky_enabled=False))

    if any(w in p for w in ("empty", "open room", "blank")) and "rack" not in p:
        return SimSpec(
            environment=EnvironmentSpec(environment_type="indoor_office",
                                        room_dimensions=Vector3D(x=8, y=6, z=3), **env_kw),
            mission=MissionSpec(initial_pose=Pose2D(x=-2.5, y=-1.5),
                                waypoints=[Pose2D(x=2.5, y=-1.5), Pose2D(x=2.5, y=1.5)],
                                goal_pose=Pose2D(x=-2.5, y=1.5, yaw=3.14159), timeout_seconds=300))

    racks = max(0, min(4, _count(p, "racks?", 4)))
    crates = max(0, min(3, _count(p, "crates?|boxes|box|pallets?", 1)))
    objects = [ObjectSpec(id=f"rack_{i + 1}", model_name="aws_warehouse_rack",
                          dimensions=Vector3D(x=3.0, y=0.8, z=2.0), pose=Pose2D(x=x, y=y))
               for i, (x, y) in enumerate(RACK_SLOTS[:racks])]
    objects += [ObjectSpec(id=f"crate_{i + 1}", model_name="wooden_crate",
                           dimensions=Vector3D(x=0.8, y=0.8, z=0.8), pose=Pose2D(x=x, y=y))
                for i, (x, y) in enumerate(CRATE_SLOTS[:crates])]
    return SimSpec(
        environment=EnvironmentSpec(environment_type="indoor_warehouse",
                                    room_dimensions=Vector3D(x=12, y=10, z=3), **env_kw),
        objects=objects,
        mission=MissionSpec(initial_pose=Pose2D(x=-4.5, y=-3.5),
                            waypoints=[Pose2D(x=0, y=1), Pose2D(x=4.5, y=1), Pose2D(x=0, y=-3)],
                            goal_pose=Pose2D(x=-4, y=3.5, yaw=1.5708), timeout_seconds=300))


def _extract_json(text: str) -> Optional[dict]:
    text = re.sub(r"```(?:json)?", "", text)
    start, end = text.find("{"), text.rfind("}")
    if start < 0 or end <= start:
        return None
    try:
        return json.loads(text[start:end + 1])
    except json.JSONDecodeError:
        return None


async def llm_spec(prompt: str, client=None) -> Tuple[Optional[SimSpec], str]:
    """Ask the LLM for a SimSpec. Returns (spec or None, note)."""
    try:
        if client is None:
            from sim2lang.agent.nemotron_client import NemotronClient
            client = NemotronClient()
        if not client.is_configured():
            return None, "no LLM API key set"
        chunks: List[str] = []
        async for c in client.stream_chat([{"role": "system", "content": SPEC_SYSTEM_PROMPT},
                                           {"role": "user", "content": prompt}], temperature=0.1):
            chunks.append(c)
        data = _extract_json("".join(chunks))
        if data is None:
            return None, "LLM reply was not JSON"
        return SimSpec.model_validate(data), "LLM"
    except Exception as e:  # noqa: BLE001 - any failure falls back to the parser
        return None, f"LLM failed: {type(e).__name__}: {str(e)[:120]}"


async def prompt_to_spec(prompt: str, client=None, use_llm: bool = True) -> Tuple[SimSpec, str]:
    """LLM first; validated by Part 1; keyword parser if the LLM is unavailable or invalid."""
    if use_llm:
        spec, note = await llm_spec(prompt, client)
        if spec is not None:
            from sim2lang.compiler import generate_simulation_ir

            ir = generate_simulation_ir(spec)
            if ir.is_valid:
                return ir.spec, "LLM (Nemotron), validated by the layout solver"
            note = "LLM layout rejected: " + "; ".join(ir.validation_messages[:2])
    else:
        note = "LLM disabled"
    return rule_spec(prompt), f"scenario parser ({note})"


def build_run(spec: SimSpec, project: str = "prompt_demo") -> Tuple[Path, List[str]]:
    """Part 1: validate + compile world + slice map, then write a complete run folder."""
    from sim2lang.compiler import compile_world_sdf, generate_simulation_ir
    from sim2lang.runtime.run_folder import write_manifest
    from sim2lang.world_gen import slice_map_from_layout

    ir = generate_simulation_ir(spec)
    if not ir.is_valid:
        raise ValueError("layout rejected: " + "; ".join(ir.validation_messages))
    run = cfg.get_run_dir(project)
    name = re.sub(r"[^a-z0-9_]", "_", project.lower())
    compile_world_sdf(ir.spec, run / "world.sdf", name)
    slice_map_from_layout(ir.spec.environment.room_dimensions, ir.spec.objects, run)
    (run / "spec.json").write_text(ir.spec.model_dump_json(indent=2))
    write_manifest(run, RunManifest(world_name=name, initial_pose=ir.spec.mission.initial_pose))
    steps = [f"layout valid: {len(ir.spec.objects)} objects, no overlaps, spawn/goal clear",
             f"world.sdf (Gazebo Harmonic) + map.pgm/map.yaml (0.05 m/px) -> {run}"]
    return run, steps


def describe(spec: SimSpec) -> str:
    env, m = spec.environment, spec.mission
    kinds = {}
    for o in spec.objects:
        kinds[o.model_name] = kinds.get(o.model_name, 0) + 1
    objs = ", ".join(f"{n} x {k}" for k, n in kinds.items()) or "no obstacles"
    extras = []
    if env.ambient_light_luminance < 0.5:
        extras.append("dim lighting")
    if env.surface_friction < 0.4:
        extras.append("slippery floor")
    return (f"{env.environment_type.replace('_', ' ')} {env.room_dimensions.x:g} x {env.room_dimensions.y:g} m, "
            f"{objs}{', ' + ', '.join(extras) if extras else ''}; mission: start ({m.initial_pose.x:g}, "
            f"{m.initial_pose.y:g}) -> {len(m.waypoints)} waypoints -> goal ({m.goal_pose.x:g}, {m.goal_pose.y:g})")
