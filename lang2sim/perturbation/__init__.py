"""Perturbation and FastMCP package for Lang2Sim."""

import sys

# Backward-compatibility alias
sys.modules.setdefault("sim2lang.perturbation", sys.modules[__name__])

from lang2sim.perturbation.gazebo_bridge import (
    BridgeResult,
    GazeboBridge,
    DryRunBridge,
    HarmonicBridge,
    ClassicBridge,
    get_bridge,
    reset_bridge_instance,
)
from lang2sim.perturbation.mcp_server import (
    mcp,
    call_tool_inprocess,
    spawn_dynamic_entity,
    set_ambient_lighting,
    reset_simulation,
    apply_chassis_perturbation,
    bridge_health,
    list_spawned_entities,
)

__all__ = [
    "BridgeResult",
    "GazeboBridge",
    "DryRunBridge",
    "HarmonicBridge",
    "ClassicBridge",
    "get_bridge",
    "reset_bridge_instance",
    "mcp",
    "call_tool_inprocess",
    "spawn_dynamic_entity",
    "set_ambient_lighting",
    "reset_simulation",
    "apply_chassis_perturbation",
    "bridge_health",
    "list_spawned_entities",
]
