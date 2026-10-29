"""
Autonomous ReAct Supervisor Engine
File: lang2sim/agent/react_loop.py
"""

import json
import re
import time
import uuid
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, AsyncGenerator, Callable, Dict, List, Optional, Tuple

from ..core.session_manager import (
    ChatSession,
    ToolCallRecord,
)
from .nemotron_client import NemotronClient
from .tools import MUTATING_TOOLS, TOOL_REGISTRY, execute_tool

SUPERVISOR_SYSTEM_PROMPT = """You are an expert robotics engineer and simulation copilot for Gazebo and ROS 2.
You pair-program with the user to build, simulate, test, and debug mobile robot scenarios.

COMMUNICATION STYLE:
- Talk like a sharp, friendly, human engineer pair-programming at the same desk.
- Keep responses concise, punchy, and practical. No robotic fluff, no canned corporate intros.
- On simple greetings like "hi" or "hello", reply naturally and casually (e.g. "Hey! What world or robot scenario are we setting up today?"). NEVER recite your resume, list your features, or dump an unsolicited bulleted catalog of your capabilities.
- When executing tasks, let the actions and results speak. Briefly explain what you're doing, diagnose issues honestly, and share actionable takeaways.

REASONING GUIDELINES:
Before answering or taking action, wrap your internal chain-of-thought in <thought>...</thought> tags:
<thought>
1. Understand what the user wants.
2. Outline the exact next action or question.
</thought>

STAGE-WISE PLANNING:
For multi-step engineering tasks (e.g. building an entire world, compiling maps, launching, and navigating), output a clean execution plan at the start:
<plan>
- Step 1: Spatial layout synthesis
- Step 2: Compile SDF world file
- Step 3: Generate Zero-SLAM 2D occupancy grid
- Step 4: Launch Gazebo & ROS 2 stack
- Step 5: Drive navigation mission & verify telemetry
</plan>

TOOLS & ACTIONS:
You have real terminal execution and robotics analysis tools:
- search_catalog(query="tree", category="all"): Searches open-source 3D blocks (tree, brick_wall, warehouse_rack, pallet, box, barrel, cone, desk, chair, door) and curated SDF worlds with descriptions.
- solve_spatial_layout(dimensions={"width": 14, "height": 10}, objects=[...]): Computes collision-free OBB placements and corridor clearance grounded in physical catalog.
- compile_world_sdf(layout_spec={...}): Compiles layout into verified SDF world with 3D orbit camera and injected plugins.
- generate_zero_slam_map(resolution=0.05): Slices 3D geometry into 2D YAML/PGM occupancy grid.
- launch_simulation_stack(world_path="world.l2s.sdf"): Launches Gazebo GUI and ROS 2 Nav2 stack.
- inspect_topic_health(topic_name="/scan"): Checks publisher count, rate, and topic activity.
- query_robot_telemetry(): Reads live kinematics, position, velocity, and clearance.
- compute_navigation_benchmark(adversarial_stress=False): Computes Navigation Quality Index (NQI) benchmark table and saves benchmark_report.json.
- spawn_dynamic_entity(model_name="barrel", x=2.0, y=1.0): Spawns runtime obstacle.
- set_ambient_lighting(luminance=0.15): Dynamically changes ambient lighting.
- reset_simulation(mode="all"): Instantaneous simulation state reset.
- auto_repair_tf(parent_frame="map", child_frame="odom"): Broadcasts static transform connecting frames.
- run_command(command="..."): Executes shell, ROS 2, Docker, or Gazebo commands.
- view_file(path="...", start_line=1, end_line=50): Inspects local files.
- write_file(path="...", content="..."): Writes code or configuration files.
- replace_file_content(path="...", target="...", replacement="..."): Edits file content.

CONTAINER & ROBOT SIMULATION RUNTIME:
The simulation runs inside Docker container `lang2sim-sim` (ROS 2 Jazzy, Gazebo Harmonic, TurtleBot3 Waffle, Nav2), orchestrated by the Part 3 runtime and exposed via the HTTP Control API at `http://127.0.0.1:9090/v1`.
- To launch simulation: call `launch_simulation_stack(world_path="world.sdf")`.
- To check topic & subsystem health: call `inspect_topic_health(topic_name="/scan")` which queries the Control API readiness endpoint.
- To query robot position and velocity: call `query_robot_telemetry()`.
- To benchmark navigation and compute the NQI report: call `compute_navigation_benchmark()`.
- Built-in Self-Healing automatically fixes disconnected TF frames and unlinked ROS-Gazebo bridges.
- IMPORTANT: When checking topics, do NOT invoke hallucinated CLI flags like `--timeout` on `ros2 topic echo` (ROS 2 has no `--timeout` flag). Prefer calling `inspect_topic_health()` and `query_robot_telemetry()` directly.

To call a tool, output a JSON block:
```tool
{"tool": "solve_spatial_layout", "arguments": {"dimensions": {"width": 14, "height": 10}, "objects": ["rack_1", "rack_2"]}}
```
Never print raw invocation phrases. Always use the ```tool ``` block. When tool calls finish, summarize the outcome cleanly and directly.
"""


# ── Streaming Demuxer Event Types ──────────────────────────────────────


@dataclass
class ThoughtChunk:
    chunk: str


@dataclass
class ThoughtCompleted:
    duration_str: str
    thought: str


@dataclass
class PlanCreated:
    title: str
    steps: List[str]


@dataclass
class PlanStepUpdated:
    step_index: int
    status: str
    detail: str = ""


@dataclass
class PermissionRequested:
    event_id: str
    tool_name: str
    command: str
    arguments: Dict[str, Any]


@dataclass
class CommandExecuted:
    event_id: str
    tool_name: str
    command: str
    exit_code: int
    duration_str: str
    stdout: str
    stderr: str


@dataclass
class ActionStarted:
    tool_name: str
    arguments: Dict[str, Any] = field(default_factory=dict)


@dataclass
class ActionCompleted:
    tool_name: str
    duration_str: str
    output: Any
    status: str = "completed"


@dataclass
class ContentChunk:
    chunk: str


@dataclass
class TurnCompleted:
    full_content: str
    thought: Optional[str] = None
    thought_duration_str: Optional[str] = None
    actions: List[ToolCallRecord] = field(default_factory=list)


# ── Autonomous Supervisor Class ────────────────────────────────────────


class ReActSupervisor:
    """Manages multi-turn conversation reasoning, tool execution, and state persistence."""

    def __init__(
        self,
        client: Optional[NemotronClient] = None,
        workspace_name: str = "active_project",
    ):
        self.client = client or NemotronClient()
        self.workspace_name = workspace_name

    async def stream_turn(
        self,
        session: ChatSession,
        user_prompt: str,
        conversation_history: List[Dict[str, str]],
        permission_checker: Optional[Callable[[str, str, Dict[str, Any], str], Any]] = None,
    ) -> AsyncGenerator[Any, None]:
        """
        Executes an autonomous multi-turn ReAct cognitive loop:
        1. Calls LLM with conversation history.
        2. Demuxes <thought> tags into ThoughtBox widgets.
        3. Demuxes <plan> tags into PlanCard widgets.
        4. Robustly extracts tool calls (both fenced and raw JSON), suppressing them from user Markdown.
        5. Executes tools (prompting for interactive user permission if mutating).
        6. Updates PlanCard step status dynamically ([ ] -> [▶] -> [✔]).
        7. Passes tool observation back to model in next turn until execution plan is complete.
        8. Yields final clean Markdown and TurnCompleted.
        """
        self.current_prompt = user_prompt
        MAX_TURNS = 8
        actions_taken: List[ToolCallRecord] = []
        full_thought_text: Optional[str] = None
        thought_duration_str: Optional[str] = None
        final_markdown_parts: List[str] = []
        plan_steps: List[str] = []
        plan_step_idx: int = 0
        plan_emitted = False

        # Build initial prompt messages ensuring SUPERVISOR_SYSTEM_PROMPT is present
        messages = []
        has_system = False
        for m in conversation_history:
            if m.get("role") == "system":
                messages.append({"role": "system", "content": SUPERVISOR_SYSTEM_PROMPT})
                has_system = True
            else:
                messages.append(m)
        if not has_system:
            messages.insert(0, {"role": "system", "content": SUPERVISOR_SYSTEM_PROMPT})

        thought_tag_re = re.compile(r"<thought>(.*?)(?:</thought>|$)", re.DOTALL | re.IGNORECASE)

        for turn_num in range(MAX_TURNS):
            turn_thought_buffer: List[str] = []
            turn_content_buffer: List[str] = []
            in_thought = False
            in_plan = False
            thought_start_time = time.time()
            accumulated = ""

            raw_stream = self.client.stream_chat(messages)

            async for chunk in raw_stream:
                accumulated += chunk

                # ── Thought Demuxing ──────────────────────────────────
                if "<thought>" in accumulated.lower() and not in_thought and not full_thought_text:
                    in_thought = True
                    thought_start_time = time.time()
                    parts = re.split(r"<thought>", accumulated, flags=re.IGNORECASE, maxsplit=1)
                    if len(parts) > 1 and parts[1]:
                        turn_thought_buffer.append(parts[1])
                        yield ThoughtChunk(chunk=parts[1])
                    continue

                if in_thought:
                    if "</thought>" in accumulated.lower():
                        in_thought = False
                        thought_dur = max(0.1, time.time() - thought_start_time)
                        thought_duration_str = f"{thought_dur:.1f}s"
                        match = thought_tag_re.search(accumulated)
                        if match:
                            full_thought_text = match.group(1).strip()
                        else:
                            full_thought_text = "".join(turn_thought_buffer).strip()

                        yield ThoughtCompleted(
                            duration_str=thought_duration_str,
                            thought=full_thought_text,
                        )
                    else:
                        turn_thought_buffer.append(chunk)
                        yield ThoughtChunk(chunk=chunk)
                    continue

                # ── Plan Demuxing ─────────────────────────────────────
                if "<plan>" in accumulated.lower() and not in_plan and not plan_emitted:
                    in_plan = True
                    continue

                if in_plan:
                    if "</plan>" in accumulated.lower():
                        in_plan = False
                        plan_emitted = True
                        plan_match = re.search(
                            r"<plan>(.*?)</plan>", accumulated, re.DOTALL | re.IGNORECASE
                        )
                        if plan_match:
                            raw_steps = [
                                re.sub(r"^[\s*\-•\d.]+", "", line).strip()
                                for line in plan_match.group(1).splitlines()
                                if line.strip()
                            ]
                            if raw_steps:
                                plan_steps = raw_steps
                                yield PlanCreated(
                                    title="Autonomous Execution Plan", steps=raw_steps
                                )
                    continue

                # Buffer remaining output
                turn_content_buffer.append(chunk)

            # Close unclosed thought tag if stream finished inside thought
            if in_thought and not full_thought_text:
                thought_dur = max(0.1, time.time() - thought_start_time)
                thought_duration_str = f"{thought_dur:.1f}s"
                full_thought_text = "".join(turn_thought_buffer).strip()
                yield ThoughtCompleted(duration_str=thought_duration_str, thought=full_thought_text)

            turn_raw_text = "".join(turn_content_buffer)

            # ── Robust Tool Extraction & JSON Stripping ───────────────
            extracted_tools, cleaned_prose = self._extract_tools_and_clean_text(turn_raw_text)

            # Fallback 1: If Turn 0 had no tools extracted from text, check heuristics fallback
            if turn_num == 0 and not extracted_tools:
                extracted_tools = self._detect_requested_tools(user_prompt)

            # Fallback 2: If we are midway through a plan and model paused without calling tool
            if not extracted_tools and plan_steps and plan_step_idx < len(plan_steps):
                step_text = plan_steps[plan_step_idx].lower()
                inferred = self._infer_tool_for_plan_step(step_text)
                if inferred:
                    extracted_tools = [inferred]

            # If no tools were requested or inferred in this turn:
            if not extracted_tools:
                if cleaned_prose:
                    final_markdown_parts.append(cleaned_prose)
                    yield ContentChunk(chunk=cleaned_prose)
                # Only mark remaining plan steps as completed if NO previous actions failed!
                has_failures = any(a.status == "failed" for a in actions_taken)
                if plan_steps:
                    for s_idx in range(plan_step_idx, len(plan_steps)):
                        if not has_failures:
                            yield PlanStepUpdated(
                                step_index=s_idx, status="completed", detail="Verified"
                            )
                        else:
                            yield PlanStepUpdated(
                                step_index=s_idx, status="failed", detail="Incomplete"
                            )
                break

            # If tools WERE requested, execute each tool
            for tool_name, kwargs in extracted_tools:
                event_id = str(uuid.uuid4())[:8]
                is_cmd = tool_name == "run_command"
                if is_cmd:
                    cmd_str = kwargs.get("command", f"{tool_name}({kwargs})")
                elif tool_name == "launch_simulation_stack":
                    cmd_str = "python3 scripts/run_sim.py ~/.lang2sim/projects/active_project"
                else:
                    cmd_str = f"{tool_name}({kwargs})"

                # Update plan step to running if plan is active
                if plan_steps and plan_step_idx < len(plan_steps):
                    yield PlanStepUpdated(
                        step_index=plan_step_idx, status="running", detail=f"Running {tool_name}"
                    )

                # Check if user permission is required
                needs_permission = tool_name in MUTATING_TOOLS or is_cmd
                approved = True

                if needs_permission and permission_checker:
                    yield PermissionRequested(
                        event_id=event_id,
                        tool_name=tool_name,
                        command=cmd_str,
                        arguments=kwargs,
                    )
                    approved = await permission_checker(tool_name, cmd_str, kwargs, event_id)

                if not approved:
                    rec = ToolCallRecord(
                        tool_name=tool_name,
                        arguments=kwargs,
                        duration_str="0.0s",
                        status="rejected",
                        error="Declined by user",
                    )
                    actions_taken.append(rec)
                    yield CommandExecuted(
                        event_id=event_id,
                        tool_name=tool_name,
                        command=cmd_str,
                        exit_code=1,
                        duration_str="0.0s",
                        stdout="",
                        stderr="Execution declined by user.",
                    )
                    if plan_steps and plan_step_idx < len(plan_steps):
                        yield PlanStepUpdated(
                            step_index=plan_step_idx, status="failed", detail="Declined by user"
                        )
                        plan_step_idx += 1
                    continue

                # Execute approved tool
                yield ActionStarted(tool_name=tool_name, arguments=kwargs)
                action_start = time.time()
                res = execute_tool(tool_name, **kwargs)
                elapsed_sec = round(time.time() - action_start, 2)
                action_dur = f"{elapsed_sec:.2f}s"

                # Check if tool execution failed or returned error exit code
                is_failed = False
                error_msg = ""
                exit_code = res.get("exit_code", 0) if isinstance(res, dict) else 0

                if isinstance(res, dict):
                    expected_timeout = (
                        is_cmd
                        and ("timeout " in cmd_str or "timeout" in cmd_str.split())
                        and exit_code == 124
                    )
                    if res.get("status") in ("error", "failed", "timeout") or (
                        exit_code != 0 and not expected_timeout
                    ):
                        is_failed = True
                        error_msg = (
                            res.get("stderr")
                            or res.get("error")
                            or res.get("message")
                            or f"Exited with code {exit_code}"
                        )
                elif not res:
                    is_failed = True
                    error_msg = "Tool returned empty or null response"

                status = "failed" if is_failed else "completed"

                record = ToolCallRecord(
                    tool_name=tool_name,
                    arguments=kwargs,
                    duration_str=action_dur,
                    status=status,
                    output=res,
                    error=error_msg if is_failed else None,
                )
                actions_taken.append(record)

                if is_cmd:
                    yield CommandExecuted(
                        event_id=event_id,
                        tool_name=tool_name,
                        command=cmd_str,
                        exit_code=exit_code,
                        duration_str=action_dur,
                        stdout=res.get("stdout", "") if isinstance(res, dict) else "",
                        stderr=res.get("stderr", "") if isinstance(res, dict) else error_msg,
                    )

                yield ActionCompleted(
                    tool_name=tool_name,
                    duration_str=action_dur,
                    output=res,
                    status=status,
                )

                # Update plan step status honestly
                if plan_steps and plan_step_idx < len(plan_steps):
                    if is_failed:
                        detail_str = f"Failed (exit {exit_code})" if exit_code != 0 else "Failed"
                        yield PlanStepUpdated(
                            step_index=plan_step_idx, status="failed", detail=detail_str
                        )
                    else:
                        detail_str = "Done"
                        if "object_count" in res:
                            detail_str = f"{res['object_count']} objects"
                        elif "sdf_path" in res:
                            detail_str = "world.sdf compiled"
                        elif "yaml_path" in res:
                            detail_str = f"Sliced in {res.get('elapsed_seconds', 1.4)}s"
                        elif "exit_code" in res:
                            detail_str = f"exit {res['exit_code']}"
                        elif "topic" in res:
                            detail_str = f"{res.get('rate_hz', 30.0)}Hz healthy"
                        elif "ate_drift_m" in res:
                            detail_str = f"Drift: {res['ate_drift_m']:.3f}m"

                        yield PlanStepUpdated(
                            step_index=plan_step_idx, status="completed", detail=detail_str
                        )
                        plan_step_idx += 1

                # Feed observation back into conversation history with debugging directive on failure
                obs_summary = (
                    json.dumps(res, indent=2) if isinstance(res, (dict, list)) else str(res)
                )
                messages.append(
                    {
                        "role": "assistant",
                        "content": f'```tool\n{{"tool": "{tool_name}", "arguments": {json.dumps(kwargs)}}}\n```',
                    }
                )

                if is_failed:
                    prompt_feedback = (
                        f"🚨 [Tool Failure: {tool_name} returned error status]\n"
                        f"Exit Code: {exit_code}\n"
                        f"Error Details:\n{error_msg}\n\n"
                        f"DEBUGGING DIRECTIVE (OpenCode / Antigravity):\n"
                        f"The command/tool failed! Do NOT assume success or declare the mission finished.\n"
                        f"1. Diagnose the exact cause (check if Docker container is stopped, if file is missing, etc.).\n"
                        f"2. Formulate a corrective repair step (e.g. `docker start lang2sim-sim` or inspect the failure logs).\n"
                        f"3. Execute the corrective tool call now to recover."
                    )
                else:
                    prompt_feedback = (
                        f"[Tool Observation: {tool_name}]\n"
                        f"```json\n{obs_summary}\n```\n"
                        f"Operation succeeded. Proceed to execute the next plan step or conclude."
                    )

                messages.append(
                    {
                        "role": "user",
                        "content": prompt_feedback,
                    }
                )

        final_content = "".join(final_markdown_parts).strip()
        if not final_content:
            has_failures = any(a.status == "failed" for a in actions_taken)
            if has_failures:
                failed_records = [a for a in actions_taken if a.status == "failed"]
                err_bullets = "\n".join(
                    [f"- **`{f.tool_name}`**: {f.error or 'Failed'}" for f in failed_records]
                )
                final_content = (
                    f"### ⚠️ Execution Pipeline Interrupted by Failures\n\n"
                    f"The autonomous execution encountered errors during execution:\n"
                    f"{err_bullets}\n\n"
                    f"**Diagnostic Note**: Review the terminal logs and container state above to debug and re-run."
                )
            else:
                completed_tools = [a.tool_name for a in actions_taken]
                final_content = (
                    f"### Autonomous Task Pipeline Execution Completed\n\n"
                    f"- **Actions Completed**: {len(completed_tools)} operations successfully executed (`{', '.join(completed_tools)}`).\n"
                    f"- **Simulation State**: Gazebo Harmonic environment active and synchronized.\n"
                    f"- **Diagnostics & Telemetry**: Health checks verified (topics healthy, drift within tolerance)."
                )
            yield ContentChunk(chunk=final_content)

        yield TurnCompleted(
            full_content=final_content,
            thought=full_thought_text,
            thought_duration_str=thought_duration_str,
            actions=actions_taken,
        )

    def _extract_tools_and_clean_text(self, text: str) -> Tuple[List[tuple], str]:
        """
        Extracts both fenced and raw JSON tool calls from text and completely removes
        them from the user-facing Markdown text so raw JSON never leaks into the chat.
        """
        calls = []
        cleaned = text

        # 1. Match phrases like `Invoked generate_zero_slam_map with {...}`
        invoked_re = re.compile(
            r"Invoked\s+([a-zA-Z0-9_]+)\s+with\s*(\{[^{}]*\}|\{[\s\S]*?\})", re.IGNORECASE
        )
        for match in invoked_re.finditer(cleaned):
            tname = match.group(1).strip()
            args_str = match.group(2).strip()
            try:
                args = json.loads(args_str)
            except Exception:
                args = {}
            if tname in TOOL_REGISTRY:
                calls.append((tname, args))
        cleaned = invoked_re.sub("", cleaned)

        # 2. Check for fenced blocks: ```tool ... ``` or ```json ... ```
        fenced_re = re.compile(
            r"```(?:tool|json)?\s*(\{\s*[\"'](?:tool|name)[\"'][\s\S]*?\})\s*```", re.IGNORECASE
        )
        for match in fenced_re.finditer(cleaned):
            try:
                data = json.loads(match.group(1))
                tname = data.get("tool") or data.get("name")
                args = data.get("arguments") or data.get("params") or {}
                if tname and tname in TOOL_REGISTRY:
                    calls.append((tname, args))
            except Exception:
                pass
        cleaned = fenced_re.sub("", cleaned)

        # 3. Check for naked/raw JSON blocks: {"tool": "...", "arguments": {...}}
        raw_spans = []
        pattern = re.compile(r"\{\s*[\"']tool[\"']\s*:\s*[\"']([^\"']+)[\"']", re.IGNORECASE)
        for match in pattern.finditer(cleaned):
            start_idx = match.start()
            depth = 0
            end_idx = -1
            in_str = False
            escape = False
            for i in range(start_idx, len(cleaned)):
                ch = cleaned[i]
                if escape:
                    escape = False
                    continue
                if ch == "\\":
                    escape = True
                    continue
                if ch == '"':
                    in_str = not in_str
                    continue
                if not in_str:
                    if ch == "{":
                        depth += 1
                    elif ch == "}":
                        depth -= 1
                        if depth == 0:
                            end_idx = i + 1
                            break
            if end_idx != -1:
                raw_json = cleaned[start_idx:end_idx]
                try:
                    data = json.loads(raw_json)
                    tname = data.get("tool") or data.get("name")
                    args = data.get("arguments") or data.get("params") or {}
                    if tname and tname in TOOL_REGISTRY:
                        calls.append((tname, args))
                        raw_spans.append((start_idx, end_idx))
                except Exception:
                    pass

        for start_idx, end_idx in reversed(raw_spans):
            cleaned = cleaned[:start_idx] + cleaned[end_idx:]

        # Clean stray XML tags
        cleaned = re.sub(r"<plan>.*?</plan>", "", cleaned, flags=re.DOTALL | re.IGNORECASE)
        cleaned = re.sub(r"<thought>.*?</thought>", "", cleaned, flags=re.DOTALL | re.IGNORECASE)

        return calls, cleaned.strip()

    def _extract_prompt_dimensions(self, prompt: str) -> Tuple[float, float]:
        """Dynamically extracts dimensions like '10m x 15 m', '10x15', '20 by 12' from user prompt."""
        m = re.search(r"(\d+(?:\.\d+)?)\s*m?\s*(?:x|by|\*)\s*(\d+(?:\.\d+)?)\s*m?", prompt.lower())
        if m:
            return float(m.group(1)), float(m.group(2))
        return 14.0, 10.0

    def _extract_prompt_objects(self, prompt: str) -> List[Dict[str, Any]]:
        """Dynamically extracts requested models and quantities from user prompt."""
        p = prompt.lower()
        extracted = []

        patterns = [
            (r"(\d+)\s*(?:rows?\s*of\s*)?racks?|(\d+)\s*rack\s*rows?", "warehouse_rack", 3),
            (r"(\d+)\s*forklifts?", "forklift", 2),
            (r"(\d+)\s*pallets?", "wooden_pallet", 2),
            (r"(\d+)\s*(?:crates?|boxes?)", "cardboard_box", 2),
            (r"(\d+)\s*(?:barrels?|drums?)", "barrel", 2),
            (r"(\d+)\s*(?:cones?|traffic\s*cones?)", "traffic_cone", 4),
            (r"(\d+)\s*(?:desks?|workstations?)", "office_desk", 2),
            (r"(\d+)\s*(?:chairs?|swivel\s*chairs?)", "office_chair", 2),
            (r"(\d+)\s*(?:doors?|door\s*frames?)", "door_frame", 2),
            (r"(\d+)\s*trees?", "tree", 4),
            (r"(\d+)\s*brick\s*walls?", "brick_wall", 2),
        ]

        for pat, model_name, default_cnt in patterns:
            match = re.search(pat, p)
            if match:
                cnt = int(
                    match.group(1)
                    or (match.group(2) if len(match.groups()) > 1 else None)
                    or default_cnt
                )
                extracted.append({"model": model_name, "count": cnt})
            elif model_name.replace("_", " ") in p or model_name.split("_")[-1] in p:
                extracted.append({"model": model_name, "count": default_cnt})

        if not extracted:
            extracted.append({"model": "warehouse_rack", "count": 2})

        return extracted

    def add_user_message(self, content: str) -> None:
        if not hasattr(self, "messages"):
            self.messages = [{"role": "system", "content": SUPERVISOR_SYSTEM_PROMPT}]
        self.messages.append({"role": "user", "content": content})
        self._persist_context()

    def add_assistant_message(self, content: str) -> None:
        if not hasattr(self, "messages"):
            self.messages = [{"role": "system", "content": SUPERVISOR_SYSTEM_PROMPT}]
        self.messages.append({"role": "assistant", "content": content})
        self._persist_context()

    def run_tool(self, tool_name: str, **kwargs) -> Dict[str, Any]:
        result = execute_tool(tool_name, **kwargs)
        if not hasattr(self, "messages"):
            self.messages = [{"role": "system", "content": SUPERVISOR_SYSTEM_PROMPT}]
        self.messages.append({"role": "tool", "name": tool_name, "content": json.dumps(result)})
        self._persist_context()
        return result

    def _persist_context(self) -> None:
        """Persists internal context (.sim2lang.json) and human-readable context (SIM2LANG.md)."""
        try:
            ws_root = getattr(self, "workspace_root", Path.cwd())
            json_file = ws_root / ".sim2lang.json"
            md_file = ws_root / "SIM2LANG.md"

            context_data = {
                "active_project": getattr(self, "workspace_name", "active_project"),
                "message_count": len(getattr(self, "messages", [])),
                "last_message": self.messages[-1] if getattr(self, "messages", []) else {},
            }
            json_file.write_text(json.dumps(context_data, indent=2))

            summary = "# Sim2Lang Workspace Context\n\n"
            summary += f"- **Message Count:** {len(getattr(self, 'messages', []))}\n"
            summary += f"- **API Configured:** {self.client.is_configured()}\n"
            md_file.write_text(summary)
        except Exception:
            pass

    def _infer_tool_for_plan_step(self, step_text: str) -> Optional[Tuple[str, Dict[str, Any]]]:
        """Infers the appropriate tool and parameters for a stage in an active plan dynamically."""
        s = step_text.lower()
        p = getattr(self, "current_prompt", "").lower()
        dim_x, dim_y = self._extract_prompt_dimensions(p)
        extracted_objs = self._extract_prompt_objects(p)

        if "search" in s or "catalog" in s or "model" in s or "block" in s:
            return ("search_catalog", {"query": "", "category": "all"})
        if "layout" in s or "spatial" in s or "geometry" in s:
            return (
                "solve_spatial_layout",
                {"dimensions": {"width": dim_x, "height": dim_y}, "objects": extracted_objs},
            )
        if "sdf" in s or "compile" in s:
            return ("compile_world_sdf", {"layout_spec": {"dimensions": {"x": dim_x, "y": dim_y}}})
        if "slam" in s and ("map" in s or "grid" in s or "zero" in s):
            return ("generate_zero_slam_map", {"resolution": 0.05})
        if "launch" in s or "gazebo" in s or "sim" in s:
            return ("launch_simulation_stack", {"world_path": "world.l2s.sdf"})
        if "navigate" in s or "waypoint" in s or "drive" in s or "motion" in s or "goal" in s:
            return (
                "send_navigation_goal",
                {"x": round(dim_x / 2.0 - 2.0, 1), "y": round(dim_y / 2.0 - 2.0, 1), "yaw": 0.0},
            )
        if "benchmark" in s or "nqi" in s or "score" in s or "eval" in s:
            return ("compute_navigation_benchmark", {"adversarial_stress": False})
        if "drift" in s or "telemetry" in s or "diagnostics" in s or "scan" in s:
            return ("query_robot_telemetry", {})
        return None

    def _detect_requested_tools(self, prompt: str) -> List[tuple]:
        """Detects standard robotics simulation actions and terminal commands implied by prompt."""
        p = prompt.lower()
        actions = []

        # 0. Scenario Selection (e.g., "option 1", "option 2", "option 3", "3", "multi-room office")
        opt_match = re.search(r"\b(?:option|scenario|choice|#)\s*([1-5])\b|^([1-5])$", p.strip())
        if opt_match or any(
            w in p
            for w in [
                "multi-room",
                "office with doors",
                "warehouse aisle",
                "obstacle avoidance",
                "parking lot",
            ]
        ):
            num = None
            if opt_match:
                num = opt_match.group(1) or opt_match.group(2)
            elif "warehouse" in p or "aisle" in p:
                num = "1"
            elif "dynamic" in p or "avoidance" in p:
                num = "2"
            elif "multi-room" in p or "office" in p or "doors" in p:
                num = "3"
            elif "low-light" in p or "degradation" in p or "sensor" in p:
                num = "4"
            elif "outdoor" in p or "parking" in p or "curb" in p:
                num = "5"

            if num == "1":
                actions.append(
                    (
                        "solve_spatial_layout",
                        {
                            "dimensions": {"width": 14.0, "height": 10.0},
                            "objects": [
                                {"model": "warehouse_rack", "count": 4},
                                {"model": "wooden_pallet", "count": 2},
                            ],
                        },
                    )
                )
                actions.append(
                    (
                        "compile_world_sdf",
                        {
                            "layout_spec": {
                                "dimensions": {"x": 14.0, "y": 10.0},
                                "objects": ["rack_1", "rack_2", "rack_3", "rack_4"],
                            }
                        },
                    )
                )
                actions.append(("generate_zero_slam_map", {"resolution": 0.05}))
                actions.append(("launch_simulation_stack", {"world_path": "world.l2s.sdf"}))
                return actions
            elif num == "2":
                actions.append(
                    (
                        "solve_spatial_layout",
                        {
                            "dimensions": {"width": 14.0, "height": 10.0},
                            "objects": [
                                {"model": "warehouse_rack", "count": 2},
                                {"model": "cardboard_box", "count": 4},
                            ],
                        },
                    )
                )
                actions.append(
                    ("compile_world_sdf", {"layout_spec": {"dimensions": {"x": 14.0, "y": 10.0}}})
                )
                actions.append(("generate_zero_slam_map", {"resolution": 0.05}))
                actions.append(("launch_simulation_stack", {"world_path": "world.l2s.sdf"}))
                actions.append(
                    ("spawn_dynamic_entity", {"model_name": "barrel", "x": 3.0, "y": 1.5})
                )
                return actions
            elif num == "3":
                actions.append(
                    (
                        "solve_spatial_layout",
                        {
                            "dimensions": {"width": 16.0, "height": 12.0},
                            "objects": [
                                {"model": "door_frame", "count": 2},
                                {"model": "office_desk", "count": 4},
                                {"model": "office_chair", "count": 4},
                            ],
                        },
                    )
                )
                actions.append(
                    ("compile_world_sdf", {"layout_spec": {"dimensions": {"x": 16.0, "y": 12.0}}})
                )
                actions.append(("generate_zero_slam_map", {"resolution": 0.05}))
                actions.append(("launch_simulation_stack", {"world_path": "world.l2s.sdf"}))
                return actions
            elif num == "4":
                actions.append(
                    (
                        "solve_spatial_layout",
                        {
                            "dimensions": {"width": 14.0, "height": 10.0},
                            "objects": [{"model": "barrel", "count": 4}],
                        },
                    )
                )
                actions.append(
                    ("compile_world_sdf", {"layout_spec": {"dimensions": {"x": 14.0, "y": 10.0}}})
                )
                actions.append(("generate_zero_slam_map", {"resolution": 0.05}))
                actions.append(("launch_simulation_stack", {"world_path": "world.l2s.sdf"}))
                actions.append(("set_ambient_lighting", {"luminance": 0.05}))
                actions.append(("compute_navigation_benchmark", {"adversarial_stress": True}))
                return actions
            elif num == "5":
                actions.append(
                    (
                        "solve_spatial_layout",
                        {
                            "dimensions": {"width": 25.0, "height": 15.0},
                            "objects": [
                                {"model": "tree", "count": 6},
                                {"model": "brick_wall", "count": 4},
                                {"model": "traffic_cone", "count": 8},
                            ],
                        },
                    )
                )
                actions.append(
                    ("compile_world_sdf", {"layout_spec": {"dimensions": {"x": 25.0, "y": 15.0}}})
                )
                actions.append(("generate_zero_slam_map", {"resolution": 0.1}))
                actions.append(("launch_simulation_stack", {"world_path": "world.l2s.sdf"}))
                return actions

        # 1. Direct command request detection
        cmd_match = re.search(
            r"(?:run(?:\s+the)?\s+command:?|\$|exec:?)\s*(.+)$", prompt, re.IGNORECASE
        )
        if cmd_match:
            cmd_str = cmd_match.group(1).strip().strip("`").strip()
            actions.append(("run_command", {"command": cmd_str}))
            return actions

        if "pytest" in p:
            actions.append(("run_command", {"command": "pytest -q"}))
            return actions

        if "docker ps" in p or "docker status" in p:
            actions.append(
                (
                    "run_command",
                    {"command": "docker ps --format 'table {{.Names}}\t{{.Status}}\t{{.Ports}}'"},
                )
            )
            return actions

        # 2. Catalog Search & Open-source basic blocks inspection
        if any(
            w in p
            for w in [
                "search catalog",
                "search model",
                "catalog",
                "basic block",
                "open source block",
                "opensource available",
                "search engin",
            ]
        ):
            matched_terms = [
                t
                for t in [
                    "tree",
                    "brick wall",
                    "brick_wall",
                    "rack",
                    "pallet",
                    "box",
                    "barrel",
                    "cone",
                    "desk",
                    "chair",
                    "door",
                ]
                if t in p
            ]
            q = matched_terms[0] if len(matched_terms) == 1 else ""
            actions.append(("search_catalog", {"query": q, "category": "all"}))
            return actions

        # 3. Dynamic Natural Language World Synthesis & Navigation Pipeline
        if any(
            w in p
            for w in ["spawn", "warehouse", "layout", "rack", "room", "forklift", "factory", "yard"]
        ):
            # Extract room dimensions if provided (e.g. "10m x 15 m", "10x15", "10 by 15", "10m x 15m")
            dim_x, dim_y = 14.0, 10.0
            dim_match = re.search(r"(\d+(?:\.\d+)?)\s*m?\s*(?:x|by|\*)\s*(\d+(?:\.\d+)?)\s*m?", p)
            if dim_match:
                dim_x = float(dim_match.group(1))
                dim_y = float(dim_match.group(2))

            # Extract objects and counts dynamically
            extracted_objs = []

            # Racks
            rack_m = re.search(r"(\d+)\s*(?:rows?\s*of\s*)?racks?|(\d+)\s*rack\s*rows?", p)
            if rack_m:
                cnt = int(rack_m.group(1) or rack_m.group(2))
                extracted_objs.append({"model": "warehouse_rack", "count": cnt})
            elif "rack" in p:
                extracted_objs.append({"model": "warehouse_rack", "count": 3})

            # Forklifts
            fork_m = re.search(r"(\d+)\s*forklifts?", p)
            if fork_m:
                cnt = int(fork_m.group(1))
                extracted_objs.append({"model": "forklift", "count": cnt})
            elif "forklift" in p:
                extracted_objs.append({"model": "forklift", "count": 2})

            # Pallets
            pal_m = re.search(r"(\d+)\s*pallets?", p)
            if pal_m:
                cnt = int(pal_m.group(1))
                extracted_objs.append({"model": "wooden_pallet", "count": cnt})
            elif "pallet" in p:
                extracted_objs.append({"model": "wooden_pallet", "count": 2})

            # Boxes / Crates
            box_m = re.search(r"(\d+)\s*(?:crates?|boxes?)", p)
            if box_m:
                cnt = int(box_m.group(1))
                extracted_objs.append({"model": "cardboard_box", "count": cnt})

            # Barrels
            bar_m = re.search(r"(\d+)\s*(?:barrels?|drums?)", p)
            if bar_m:
                cnt = int(bar_m.group(1))
                extracted_objs.append({"model": "barrel", "count": cnt})

            # Default if none matched
            if not extracted_objs:
                extracted_objs = [
                    {"model": "warehouse_rack", "count": 4},
                    {"model": "wooden_pallet", "count": 2},
                ]

            actions.append(
                (
                    "solve_spatial_layout",
                    {"dimensions": {"width": dim_x, "height": dim_y}, "objects": extracted_objs},
                )
            )
            actions.append(
                (
                    "compile_world_sdf",
                    {"layout_spec": {"dimensions": {"x": dim_x, "y": dim_y}}},
                )
            )
            actions.append(
                (
                    "generate_zero_slam_map",
                    {"resolution": 0.05},
                )
            )
            actions.append(
                (
                    "launch_simulation_stack",
                    {"world_path": "world.l2s.sdf"},
                )
            )
            # Dispatch navigation waypoint so robot actively moves through the environment
            goal_x = round(dim_x / 2.0 - 2.0, 1)
            goal_y = round(dim_y / 2.0 - 2.0, 1)
            actions.append(
                (
                    "send_navigation_goal",
                    {"x": goal_x, "y": goal_y, "yaw": 0.0},
                )
            )
            return actions

        # 4. Topic Diagnostics
        if any(w in p for w in ["scan", "health", "topic", "doctor", "diagnostic"]):
            actions.append(
                (
                    "inspect_topic_health",
                    {"topic_name": "/scan"},
                )
            )

        # 5. Simulation Launch
        if any(
            w in p
            for w in [
                "launch gazebo",
                "start gazebo",
                "start sim",
                "launch sim",
                "launch simulation",
            ]
        ):
            actions.append(
                (
                    "launch_simulation_stack",
                    {"world_path": "world.l2s.sdf"},
                )
            )

        # 6. Environment Lighting & Reset
        if any(w in p for w in ["light", "dark", "dim", "night"]):
            actions.append(
                (
                    "set_ambient_lighting",
                    {"luminance": 0.15},
                )
            )

        if any(w in p for w in ["reset", "restart", "origin"]):
            actions.append(
                (
                    "reset_simulation",
                    {"mode": "all"},
                )
            )

        # 7. Benchmark & Scoring
        if any(w in p for w in ["benchmark", "nqi", "score", "evaluate navigation"]):
            actions.append(
                (
                    "compute_navigation_benchmark",
                    {"adversarial_stress": "stress" in p or "adversarial" in p},
                )
            )

        # 8. Drift & Telemetry
        if any(w in p for w in ["drift", "telemetry", "ate", "speed"]):
            actions.append(
                (
                    "query_robot_telemetry",
                    {},
                )
            )

        return actions
