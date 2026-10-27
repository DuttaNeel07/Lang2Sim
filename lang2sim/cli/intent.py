"""
Natural-language control of the running simulation.
File: lang2sim/cli/intent.py

Every sentence the user types goes through interpret(): the LLM (Nemotron) turns it
into one structured action (build a world, start a stress run, dim the lights and
drop a crate, stop the robot, go to rack 2, show the score, ...), which DemoRunner
executes against the control API. When no LLM key is set, the call fails or times
out, or the reply isn't a valid action, the offline rule parser below handles the
same sentences, so the demo never depends on the network. Slash commands still work
as hidden shortcuts but are never required or suggested.
"""

from __future__ import annotations

import asyncio
import json
import re
from dataclasses import dataclass
from typing import Any, Dict, List, Optional

from .demo_commands import (
    CRATE_HALF,
    Command,
    is_goto,
    is_halt,
    is_question,
    is_sim_request,
    is_stop_sim,
    parse_stress_intent,
    rack_number,
)

LLM_TIMEOUT_S = 8.0         # then the offline parser answers, so a slow model never stalls the demo
ACTIONS = ("new_sim", "stop_sim", "stress", "start_mission", "benchmark", "halt", "goto", "report", "status",
           "dashboard", "replay", "help", "chat")
SIM_ACTIONS = ("stress", "start_mission", "benchmark", "halt", "goto")      # need a running simulation

INTENT_SYSTEM_PROMPT = """/no_think
You are the command interpreter of Lang2Sim, a robot simulation copilot. A TurtleBot3
navigates a Gazebo world with Nav2. Turn the operator's sentence into exactly ONE JSON object, no prose:

{"action": "<one of the actions below>", ...fields}

Actions and fields:
- "new_sim": build and run a NEW world from a description. {"scenario": "<the world description>"}
  Only when they describe a world/scene to create (warehouse, room, racks, crates, lighting, floor).
- "stop_sim": shut the whole simulation down ("stop the simulation", "we're done", "shut it down").
- "stress": change the running world to stress the robot. {"light": 0.0-1.0 or null,
  "obstacle": "wooden_crate"|"cargo_box"|"barrel"|"rack"|null, "x": number|null, "y": number|null,
  "across": true|false}. light is the sun intensity (10% -> 0.1, "dim"/"dark" without a number -> 0.15,
  "lights off" -> 0.0). Obstacle words: crate/box/pallet/obstacle/something -> "wooden_crate",
  cargo box/carton -> "cargo_box", barrel/drum -> "barrel", rack/shelf/shelving/wall -> "rack".
  x/y only when they give coordinates. across=true for "across the aisle/path", "sideways", "blocking".
  Use this for "stress-test the robot", "make it harder", "throw something in its way", "drop a crate".
- "start_mission": drive the mission and score it. {"stress": true|false}. stress=true for a stress /
  adversarial run they will perturb ("start a stress run", "run it again under stress").
- "benchmark": the full automatic comparison: normal run, then a stress run with a crate and dim light,
  then the score table. {"light": 0.0-1.0 or null}
- "halt": stop the robot where it is; the simulation keeps running ("stop the robot", "freeze", "wait",
  and a bare "stop" — only stop_sim when they clearly mean the whole simulation).
- "goto": send the robot somewhere. {"rack": integer|null, "x": number|null, "y": number|null}.
  rack for "rack 3"/"the third rack"/"shelf 2"; x/y for coordinates; all null for "resume"/"continue"/
  "go to the goal"/"finish the route".
- "report": show the NQI score table / results.
- "status": where the robot is and what the simulation is doing right now.
- "dashboard": open the live dashboard.
- "replay": play back the recorded demo run (when the live simulation is down).
- "help": what they can say.
- "chat": a question, an explanation, or anything that is not an action above ("why did the score drop?",
  "what is NQI?", "explain the detour"). Questions about the run are "chat", not "report", unless they
  ask to see/show the table.

Polite or indirect requests are still actions: "can you drop a barrel ahead?" -> stress.
"""


@dataclass
class Interpretation:
    action: str
    command: Optional[Command] = None      # for actions DemoRunner executes
    scenario: str = ""                     # for new_sim
    source: str = "rules"                  # "Nemotron" (or the model name) / "rules"
    summary: str = ""                      # plain-English description of what will happen

    @property
    def needs_sim(self) -> bool:
        return self.action in SIM_ACTIONS


# --------------------------------------------------------------------------- descriptions
def _pct(v: str) -> str:
    return f"{float(v):.0%}"


def describe_stress(opts: Dict[str, str]) -> str:
    parts = []
    if "light" in opts:
        parts.append("turn the lights off" if float(opts["light"]) == 0 else f"dim the lights to {_pct(opts['light'])}")
    if "model" in opts:
        what = opts["model"].replace("_", " ")
        if "x" in opts and "y" in opts:
            parts.append(f"drop a {what} at ({float(opts['x']):.1f}, {float(opts['y']):.1f})")
        else:
            parts.append(f"drop a {what} {'across' if opts.get('across') == '1' else 'in'} the robot's path")
    return " and ".join(parts) or "stress the robot"


def describe(it: Interpretation) -> str:
    c = it.command
    if it.action == "new_sim":
        return f"build and run a new simulation: {it.scenario}"
    if it.action == "stop_sim":
        return "shut the simulation down"
    if it.action == "stress" and c:
        return describe_stress(c.opts)
    if it.action == "start_mission" and c:
        return "start a stress run (recorded and scored)" if c.args else "drive the mission and score it"
    if it.action == "benchmark":
        return "run the full benchmark: normal run, stress run, then the comparison"
    if it.action == "halt":
        return "stop the robot (the simulation keeps running)"
    if it.action == "goto" and c:
        if c.opts.get("rack"):
            return f"re-plan to rack {c.opts['rack']}"
        if "x" in c.opts and "y" in c.opts:
            return f"re-plan to ({float(c.opts['x']):.1f}, {float(c.opts['y']):.1f})"
        return "resume the route to the mission goal"
    return {"report": "show the score table", "status": "show the simulation status",
            "dashboard": "open the live dashboard", "replay": "play back the recorded demo run",
            "help": "list what you can ask for"}.get(it.action, "answer")


# --------------------------------------------------------------------------- building commands
def _num(v: Any) -> Optional[float]:
    try:
        return None if v is None or v == "" else float(v)
    except (TypeError, ValueError):
        return None


def from_action(data: Dict[str, Any], text: str) -> Optional[Interpretation]:
    """A validated Interpretation from the LLM's JSON, or None if it isn't a usable action."""
    action = str(data.get("action", "")).strip().lower()
    if action not in ACTIONS:
        return None
    it = Interpretation(action=action)
    if action == "new_sim":
        it.scenario = str(data.get("scenario") or text).strip()
    elif action == "stress":
        opts: Dict[str, str] = {}
        light = _num(data.get("light"))
        if light is not None:
            opts["light"] = f"{max(0.0, min(1.0, light / 100.0 if light > 1 else light)):.2f}"
        model = data.get("obstacle")
        if isinstance(model, str) and model.strip().lower() not in ("", "none", "null"):
            model = model.strip().lower().replace(" ", "_")
            if model not in CRATE_HALF:
                model = "wooden_crate"
            opts["model"] = model
            x, y = _num(data.get("x")), _num(data.get("y"))
            if x is not None and y is not None:
                opts["x"], opts["y"] = f"{x:g}", f"{y:g}"
            if data.get("across") in (True, "true", 1, "1"):
                opts["across"] = "1"
        if not opts:
            return None
        it.command = Command("stress", opts=opts, natural=True)
    elif action == "start_mission":
        it.command = Command("mission", args=["stress"] if data.get("stress") in (True, "true", 1) else [])
    elif action == "benchmark":
        light = _num(data.get("light"))
        it.command = Command("benchmark", opts={} if light is None else
                             {"light": f"{max(0.0, min(1.0, light / 100.0 if light > 1 else light)):.2f}"})
    elif action == "halt":
        it.command = Command("halt")
    elif action == "goto":
        opts = {"text": text}
        rack = data.get("rack")
        if isinstance(rack, (int, float)) or (isinstance(rack, str) and rack.isdigit()):
            opts["rack"] = str(int(rack))
            opts["text"] = f"rack {opts['rack']}"
        x, y = _num(data.get("x")), _num(data.get("y"))
        if x is not None and y is not None:
            opts["x"], opts["y"] = f"{x:g}", f"{y:g}"
        it.command = Command("goto", opts=opts)
    elif action in ("report", "status", "dashboard", "help"):
        it.command = Command(action)
    elif action == "replay":
        it.command = Command("replay", args=["demo"])
    it.summary = describe(it)
    return it


# --------------------------------------------------------------------------- offline rules
_POLITE = re.compile(r"^(please|kindly|can you|could you|would you|will you|can we|could we|let's|lets|i want (you )?to|"
                     r"i'd like (you )?to|go ahead and|now)\s+")
_HELP = re.compile(r"^(help|what can (you|i) (do|say|ask)|what are (the|my) options|options|commands)\b")
_BUILD = re.compile(r"\b(build|create|generate|simulate|set ?up|design)\b")
_NEW_WORLD = re.compile(r"\b(new|another|different|rebuild|instead|fresh)\b")
_BENCHMARK = re.compile(r"\bbenchmark\b|\bcompare\b.*\b(nominal|normal|stress)\b|\b(nominal|normal) (vs\.?|versus|and) stress\b|"
                        r"\bfull (test|evaluation|comparison)\b")
_MISSION = re.compile(r"\b(start|run|begin|launch|drive|do|kick off|repeat|redo|try)\b.*\b(mission|lap|run|test|tour|patrol|route)\b"
                      r"|^(stress|adversarial) (run|test|mission)\b|\bagain\b")
_STRESS_RUN = re.compile(r"\b(stress|adversarial|hard(er)?|perturb\w*|chaos)\b")
_REPORT = re.compile(r"\b(report|score|scores|nqi|results?|scorecard|table|comparison)\b")
_SHOW = re.compile(r"\b(show|display|print|give|see|open|get|pull up|let me see)\b")
_STATUS = re.compile(r"\b(status|where is the robot|where's the robot|state of the (sim|simulation|robot))\b")
_OBSTACLE_ANY = re.compile(r"\b(something|anything|an? object|an? thing)\b")


def rule_interpret(text: str, sim_active: bool, llm_available: bool = False) -> Interpretation:
    """The same sentences, without an LLM (no key, offline, or the model failed).

    llm_available: questions go to the chat model; without one, the measured data answers them."""
    raw = text.strip()
    t = raw.lower().rstrip(".!")
    t = _POLITE.sub("", t).rstrip("?").strip() if _POLITE.match(t) else t
    polite = t != raw.lower().rstrip(".!")

    def done(it: Interpretation) -> Interpretation:
        it.summary = describe(it)
        return it

    if re.fullmatch(r"(stop|wait|pause|freeze|hold on|hold)( (it|there|now|please|the robot))?", t) and sim_active:
        return done(Interpretation("halt", Command("halt")))    # a bare "stop" never kills the simulation
    if is_stop_sim(t) or re.search(r"\b(shut (it )?down|stop the (whole )?sim(ulation)?|end the sim(ulation)?|"
                                   r"we'?re done|close the sim(ulation)?)\b", t):
        return done(Interpretation("stop_sim"))
    if _HELP.match(t):
        return done(Interpretation("help", Command("help")))
    asked = is_question(t) and not polite
    if is_sim_request(t) and (not sim_active or _NEW_WORLD.search(t) or _BUILD.search(t)):
        return done(Interpretation("new_sim", scenario=raw))
    if not asked:
        if is_halt(t) or re.fullmatch(r"(stop|wait|pause|freeze|hold on|hold)( (it|there|now|please))?", t):
            return done(Interpretation("halt", Command("halt")))
        stress = parse_stress_intent(t)
        if stress is None and _OBSTACLE_ANY.search(t) and re.search(r"\b(drop|throw|put|place|spawn|add)\b", t):
            stress = parse_stress_intent(t + " crate")
        if stress is not None:
            stress.note = ""
            return done(Interpretation("stress", stress))
        if _BENCHMARK.search(t):
            m = re.search(r"(\d+(?:\.\d+)?)\s*(%|percent)", t)
            return done(Interpretation("benchmark", Command("benchmark", opts={"light": f"{float(m.group(1)) / 100:.2f}"}
                                                            if m else {})))
        if _MISSION.search(t) or re.match(r"^(stress[- ]?test|make it harder)\b", t):
            stress_run = bool(_STRESS_RUN.search(t))
            return done(Interpretation("start_mission", Command("mission", args=["stress"] if stress_run else [])))
        if is_goto(t):
            n = rack_number(t)
            opts = {"text": raw}
            if n is not None:
                opts["rack"] = str(n)
            m = re.search(r"(-?\d+(?:\.\d+)?)\s*,\s*(-?\d+(?:\.\d+)?)", t)
            if m and n is None:
                opts["x"], opts["y"] = m.group(1), m.group(2)
            return done(Interpretation("goto", Command("goto", opts=opts)))
        if re.search(r"\bdashboard\b", t):
            return done(Interpretation("dashboard", Command("dashboard")))
        if re.search(r"\breplay\b|\bplay back\b", t):
            return done(Interpretation("replay", Command("replay", args=["demo"])))
        if _REPORT.search(t) and (_SHOW.search(t) or len(t.split()) <= 3):
            return done(Interpretation("report", Command("report")))
        if _STATUS.search(t) and len(t.split()) <= 6:
            return done(Interpretation("status", Command("status")))
    if asked and not llm_available:
        # Offline there is no model to answer questions; the measured data is the next best answer.
        if _STATUS.search(t) or re.search(r"\b(where|how fast|how far)\b.*\brobot\b", t):
            return done(Interpretation("status", Command("status")))
        if _REPORT.search(t):
            return done(Interpretation("report", Command("report")))
    return done(Interpretation("chat"))


# --------------------------------------------------------------------------- LLM
def _extract_json(text: str) -> Optional[Dict[str, Any]]:
    text = re.sub(r"```(?:json)?", "", text)
    text = re.sub(r"<think>.*?</think>", "", text, flags=re.S)
    start, end = text.find("{"), text.rfind("}")
    if start < 0 or end <= start:
        return None
    try:
        data = json.loads(text[start:end + 1])
    except json.JSONDecodeError:
        return None
    return data if isinstance(data, dict) else None


def situation(sim_active: bool, mission_running: bool, racks: Optional[List[str]] = None,
              robot: Optional[Dict[str, float]] = None) -> str:
    lines = [f"Simulation running: {'yes' if sim_active else 'no'}.",
             f"Mission being driven right now: {'yes' if mission_running else 'no'}."]
    if racks:
        lines.append("Racks in the world: " + ", ".join(racks) + ".")
    if robot:
        lines.append(f"Robot at ({robot.get('x', 0):.1f}, {robot.get('y', 0):.1f}).")
    return " ".join(lines)


async def llm_interpret(text: str, client, context: str = "", timeout: Optional[float] = None) -> Optional[Interpretation]:
    """The LLM's reading of the sentence, or None (no key, error, timeout, not a valid action)."""
    if client is None or not client.is_configured():
        return None
    messages = [{"role": "system", "content": INTENT_SYSTEM_PROMPT}]
    if context:
        messages.append({"role": "system", "content": "Current state: " + context})
    messages.append({"role": "user", "content": text})

    async def collect() -> str:
        chunks: List[str] = []
        async for c in client.stream_chat(messages, temperature=0.0):
            chunks.append(c)
        return "".join(chunks)

    try:
        reply = await asyncio.wait_for(collect(), timeout=LLM_TIMEOUT_S if timeout is None else timeout)
    except Exception:  # noqa: BLE001 - timeout or transport error: the rules take over
        return None
    data = _extract_json(reply)
    if data is None:
        return None
    it = from_action(data, text)
    if it is not None:
        it.source = "Nemotron" if "nemotron" in str(getattr(client, "model", "")).lower() else \
            str(getattr(client, "model", "LLM"))
    return it


async def interpret(text: str, client=None, *, sim_active: bool = False, mission_running: bool = False,
                    context: str = "") -> Interpretation:
    """LLM first, offline rules as the fallback. Slash shortcuts never reach here."""
    it = await llm_interpret(text, client, context or situation(sim_active, mission_running))
    if it is None:
        llm = client is not None and client.is_configured()
        it = rule_interpret(text, sim_active, llm_available=llm)
        it.source = "offline parser (LLM unavailable)" if llm else "offline parser"
    return it
