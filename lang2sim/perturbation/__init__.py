"""Perturbation and FastMCP package for Lang2Sim."""

import sys

# Backward-compatibility alias
sys.modules.setdefault("sim2lang.perturbation", sys.modules[__name__])

from lang2sim.perturbation.gazebo_bridge import (
    BridgeResult,
    ClassicBridge,
    DryRunBridge,
    GazeboBridge,
    HarmonicBridge,
    get_bridge,
    reset_bridge_instance,
)

# The FastMCP tool server runs in-process in the host app (architecture §3.10). The sim
# container imports this package only for the /v1/world router and ships no fastmcp, so a
# hard import here would silently drop that router from the control API.
try:
    from lang2sim.perturbation.mcp_server import (
        apply_chassis_perturbation,
        bridge_health,
        call_tool_inprocess,
        list_spawned_entities,
        mcp,
        reset_simulation,
        set_ambient_lighting,
        spawn_dynamic_entity,
    )
except ImportError:  # no fastmcp (sim container)
    pass
from lang2sim.perturbation.routers.world import (
    mount_world_router,
)
from lang2sim.perturbation.routers.world import (
    router as world_router,
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
    "world_router",
    "mount_world_router",
]
