"""In-Process FastMCP Perturbation Server.

Exposes simulation stress-testing tools via FastMCP with 0ms in-memory IPC latency:
  - spawn_dynamic_entity(model_name, pose)
  - set_ambient_lighting(luminance)
  - reset_simulation(mode)
  - apply_chassis_perturbation(velocity_bias)
  - bridge_health()
  - list_spawned_entities()

Can also be launched standalone via HTTP for developer introspection with MCP Inspector:
  python -m lang2sim.perturbation.mcp_server --http --port 8765
"""

from __future__ import annotations

import argparse
import asyncio
import json
import sys
from typing import Any, Mapping, Optional

from fastmcp import FastMCP

from lang2sim.perturbation._compat import Pose2D
from lang2sim.perturbation.gazebo_bridge import BridgeResult, get_bridge

__all__ = [
    "mcp",
    "call_tool_inprocess",
    "spawn_dynamic_entity",
    "set_ambient_lighting",
    "reset_simulation",
    "apply_chassis_perturbation",
    "bridge_health",
    "list_spawned_entities",
]

# Create in-process FastMCP server
mcp = FastMCP("lang2sim-perturbation")


# --------------------------------------------------------------------------- Tool definitions
@mcp.tool
async def spawn_dynamic_entity(model_name: str, pose: Pose2D) -> dict[str, Any]:
    """Spawn an obstacle model (e.g. 'wooden_crate', 'barrel', 'cargo_box') at a given 2D pose."""
    bridge = get_bridge()
    res = await bridge.spawn_entity(model_name, pose)
    return res.to_dict()


@mcp.tool
async def set_ambient_lighting(luminance: float) -> dict[str, Any]:
    """Adjust ambient/sun simulation lighting luminance between 0.0 (dark) and 1.0 (full daylight)."""
    bridge = get_bridge()
    res = await bridge.set_light(luminance)
    return res.to_dict()


@mcp.tool
async def reset_simulation(mode: str = "world") -> dict[str, Any]:
    """Execute sub-second world/robot state reset without tearing down Gazebo or closing GUI windows."""
    bridge = get_bridge()
    res = await bridge.reset(mode)
    return res.to_dict()


@mcp.tool
async def apply_chassis_perturbation(velocity_bias: Pose2D, duration_s: float = 1.0) -> dict[str, Any]:
    """Inject traction loss or force impulse (linear x/y, angular yaw) onto the mobile robot."""
    bridge = get_bridge()
    res = await bridge.apply_perturbation(velocity_bias, duration_s)
    return res.to_dict()


@mcp.tool
def bridge_health() -> dict[str, Any]:
    """Query active bridge status, backend type, and connectivity."""
    return get_bridge().health()


@mcp.tool
def list_spawned_entities() -> list[str]:
    """List entity identifiers dynamically spawned in this simulation session."""
    return get_bridge().list_spawned_entities()


# --------------------------------------------------------------------------- In-process invocation helper
async def call_tool_inprocess(name: str, arguments: Mapping[str, Any]) -> dict[str, Any]:
    """Execute an MCP tool directly in-memory within the active asyncio loop with 0ms network latency."""
    res = await mcp.call_tool(name, dict(arguments))
    # Extract structured content or text content
    if res.structured_content is not None and "result" in res.structured_content:
        val = res.structured_content["result"]
        return val if isinstance(val, dict) else {"result": val}
    if res.content:
        for item in res.content:
            if getattr(item, "text", None):
                try:
                    return json.loads(item.text)
                except Exception:
                    return {"text": item.text}
    return {"ok": not res.is_error}


# --------------------------------------------------------------------------- CLI for developer inspection
def main() -> int:
    parser = argparse.ArgumentParser(description="Lang2Sim FastMCP Perturbation Server")
    parser.add_argument("--http", action="store_true", help="Run HTTP transport for MCP Inspector")
    parser.add_argument("--port", type=int, default=8765, help="HTTP server port (default: 8765)")
    parser.add_argument("--host", default="127.0.0.1", help="HTTP server bind host (default: 127.0.0.1)")
    parser.add_argument("--backend", choices=["auto", "dryrun", "harmonic", "classic"], default=None,
                        help="Override Gazebo bridge backend")
    args = parser.parse_args()

    if args.backend:
        get_bridge(args.backend)

    if args.http:
        print(f"Starting FastMCP HTTP server at http://{args.host}:{args.port}/mcp (Streamable HTTP)")
        print(f"Connect via MCP Inspector: npx @modelcontextprotocol/inspector")
        mcp.run(transport="http", host=args.host, port=args.port)
    else:
        mcp.run()
    return 0


if __name__ == "__main__":
    sys.exit(main())
