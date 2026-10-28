"""
ReAct Perception-Reasoning-Action Supervisor Loop
File: lang2sim/agent/react_loop.py
"""

import json
from typing import List, Dict, Any, Optional
from pathlib import Path
from .nemotron_client import NemotronClient
from .tools import TOOL_REGISTRY, execute_tool

SYSTEM_PROMPT = """You are the Sim2Lang Autonomous Simulation Supervisor Agent.
Your role is to translate natural language robotics requests into validated, collision-free Gazebo simulations and Nav2 missions.
Follow the ReAct paradigm: Think carefully about constraints -> Call appropriate tools -> Observe outcomes -> Formulate next steps.
Available Tools:
- solve_spatial_layout(dimensions, objects)
- compile_world_sdf(layout_spec, target_simulator)
- generate_zero_slam_map(sdf_path)
- launch_simulation_stack(world_path, robot_model)
- spawn_dynamic_entity(model_name, x, y, z)
- set_ambient_lighting(luminance)
- reset_simulation(mode)
- query_robot_telemetry()
"""


class ReActSupervisor:
    """Manages multi-turn conversation memory and tool execution."""

    def __init__(
        self, client: Optional[NemotronClient] = None, workspace_root: Optional[Path] = None
    ):
        self.client = client or NemotronClient()
        self.workspace_root = workspace_root or Path.cwd()
        self.messages: List[Dict[str, Any]] = [{"role": "system", "content": SYSTEM_PROMPT}]

    def add_user_message(self, content: str) -> None:
        self.messages.append({"role": "user", "content": content})
        self._persist_context()

    def add_assistant_message(self, content: str) -> None:
        self.messages.append({"role": "assistant", "content": content})
        self._persist_context()

    def run_tool(self, tool_name: str, **kwargs) -> Dict[str, Any]:
        result = execute_tool(tool_name, **kwargs)
        self.messages.append({"role": "tool", "name": tool_name, "content": json.dumps(result)})
        self._persist_context()
        return result

    def _persist_context(self) -> None:
        """Persists internal context (.sim2lang.json) and human-readable context (SIM2LANG.md)."""
        try:
            json_file = self.workspace_root / ".sim2lang.json"
            md_file = self.workspace_root / "SIM2LANG.md"

            context_data = {
                "active_project": "active_project",
                "message_count": len(self.messages),
                "last_message": self.messages[-1] if self.messages else {},
            }
            json_file.write_text(json.dumps(context_data, indent=2))

            summary = "# Sim2Lang Workspace Context\n\n"
            summary += f"- **Message Count:** {len(self.messages)}\n"
            summary += f"- **API Configured:** {self.client.is_configured()}\n"
            md_file.write_text(summary)
        except Exception:
            pass
