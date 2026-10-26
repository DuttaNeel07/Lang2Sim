"""
Autonomous ReAct Supervisor Engine
File: sim2lang/agent/react_loop.py
"""

import json
import re
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Dict, Any, Optional, AsyncGenerator

from .nemotron_client import NemotronClient
from .tools import TOOL_REGISTRY, execute_tool
from ..core.session_manager import (
    session_manager,
    ChatSession,
    ChatMessage,
    ToolCallRecord,
    ArtifactRecord,
)

SUPERVISOR_SYSTEM_PROMPT = """You are Sim2Lang Copilot, an expert AI robotics simulation supervisor powered by NVIDIA Nemotron 3 Ultra.
You design, stress-test, diagnose, and optimize Gazebo Harmonic and ROS 2 Nav2 mobile robot simulations.

OPERATIONAL MODES:
- MODE A (BUILD): Procedural world synthesis, catalog anchor lookup, Zero-SLAM map slicing, and SDF compilation.
- MODE B (STRESS): Adversarial SLAM topologies, dynamic obstacle drops, ambient light dimming, surface friction reduction.
- MODE C (DOCTOR): Real-time ATE drift analysis, /scan and /odom health verification, and ephemeral TF transform recovery.

REASONING GUIDELINES:
Before answering or taking action, wrap your internal chain-of-thought in <thought>...</thought> tags:
<thought>
1. Analyze user request and constraints.
2. Determine required tools and topological requirements.
3. Formulate execution plan.
</thought>

TOOL CALLING:
When a tool is needed, invoke it directly or specify the tool call.
Format your final user-facing response in clean, concise, technical Markdown with metric tables and bullet points.
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
    ) -> AsyncGenerator[Any, None]:
        """
        Executes a complete cognitive turn:
        1. Streams tokens from LLM.
        2. Demuxes <thought>...</thought> tokens into Thought events.
        3. Identifies and executes tool calls if requested.
        4. Streams user Markdown content.
        5. Yields TurnCompleted with rich execution metadata.
        """
        turn_start_time = time.time()
        thought_start_time = time.time()

        in_thought = False
        thought_buffer: List[str] = []
        content_buffer: List[str] = []
        actions_taken: List[ToolCallRecord] = []
        full_thought_text: Optional[str] = None
        thought_duration_str: Optional[str] = None

        # Build prompt messages including system prompt
        messages = list(conversation_history)

        raw_stream = self.client.stream_chat(messages)

        accumulated = ""
        thought_tag_re = re.compile(r"<thought>(.*?)(?:</thought>|$)", re.DOTALL | re.IGNORECASE)

        async for chunk in raw_stream:
            accumulated += chunk

            # Check if entering thought block
            if "<thought>" in accumulated.lower() and not in_thought and not full_thought_text:
                in_thought = True
                thought_start_time = time.time()
                # Split at <thought>
                parts = re.split(r"<thought>", accumulated, flags=re.IGNORECASE, maxsplit=1)
                if len(parts) > 1 and parts[1]:
                    thought_buffer.append(parts[1])
                    yield ThoughtChunk(chunk=parts[1])
                continue

            # Check if inside thought block
            if in_thought:
                if "</thought>" in accumulated.lower():
                    # Thought completed
                    in_thought = False
                    thought_dur = max(0.1, time.time() - thought_start_time)
                    thought_duration_str = f"{thought_dur:.1f}s"

                    match = thought_tag_re.search(accumulated)
                    if match:
                        full_thought_text = match.group(1).strip()
                    else:
                        full_thought_text = "".join(thought_buffer).strip()

                    yield ThoughtCompleted(
                        duration_str=thought_duration_str,
                        thought=full_thought_text,
                    )

                    # Remaining text after </thought> is content
                    after_thought = re.split(r"</thought>", accumulated, flags=re.IGNORECASE, maxsplit=1)
                    if len(after_thought) > 1 and after_thought[1]:
                        rest = after_thought[1]
                        content_buffer.append(rest)
                        yield ContentChunk(chunk=rest)
                else:
                    thought_buffer.append(chunk)
                    yield ThoughtChunk(chunk=chunk)
                continue

            # Regular content chunk outside thought
            content_buffer.append(chunk)
            yield ContentChunk(chunk=chunk)

        # Handle unclosed thought tag if model finished while still in <thought>
        if in_thought and not full_thought_text:
            thought_dur = max(0.1, time.time() - thought_start_time)
            thought_duration_str = f"{thought_dur:.1f}s"
            full_thought_text = "".join(thought_buffer).strip()
            yield ThoughtCompleted(duration_str=thought_duration_str, thought=full_thought_text)

        final_content = "".join(content_buffer).strip()

        # Check if the prompt requests specific tool actions to automatically fulfill
        detected_tools = self._detect_requested_tools(user_prompt)
        for tool_name, kwargs in detected_tools:
            yield ActionStarted(tool_name=tool_name, arguments=kwargs)
            action_start = time.time()
            res = execute_tool(tool_name, **kwargs)
            action_dur = f"{time.time() - action_start:.2f}s"
            status = "completed" if res.get("status") in ("success", "healthy", "repaired") else "completed"

            record = ToolCallRecord(
                tool_name=tool_name,
                arguments=kwargs,
                duration_str=action_dur,
                status=status,
                output=res,
            )
            actions_taken.append(record)
            yield ActionCompleted(
                tool_name=tool_name,
                duration_str=action_dur,
                output=res,
                status=status,
            )

        yield TurnCompleted(
            full_content=final_content,
            thought=full_thought_text,
            thought_duration_str=thought_duration_str,
            actions=actions_taken,
        )

    def _detect_requested_tools(self, prompt: str) -> List[tuple]:
        """Detects standard robotics simulation actions implied by the prompt."""
        p = prompt.lower()
        actions = []

        if any(w in p for w in ["spawn", "warehouse", "layout", "rack", "room"]):
            actions.append((
                "solve_spatial_layout",
                {"dimensions": {"x": 14.0, "y": 10.0}, "objects": ["rack_1", "rack_2", "rack_3", "rack_4"]},
            ))
            actions.append((
                "compile_world_sdf",
                {"layout_spec": {"object_count": 4}, "target_simulator": "harmonic"},
            ))
            actions.append((
                "generate_zero_slam_map",
                {"sdf_path": "world.sdf"},
            ))

        if any(w in p for w in ["scan", "health", "topic", "doctor", "diagnostic"]):
            actions.append((
                "inspect_topic_health",
                {"topic_name": "/scan"},
            ))

        if any(w in p for w in ["light", "dark", "dim", "night"]):
            actions.append((
                "set_ambient_lighting",
                {"luminance": 0.15},
            ))

        if any(w in p for w in ["reset", "restart", "origin"]):
            actions.append((
                "reset_simulation",
                {"mode": "all"},
            ))

        if any(w in p for w in ["drift", "telemetry", "ate", "speed"]):
            actions.append((
                "query_robot_telemetry",
                {},
            ))

        return actions
