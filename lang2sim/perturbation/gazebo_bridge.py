"""Gazebo Dynamic Mutation Bridge.

Dispatches live simulation mutations (spawning models, dynamic lighting,
sub-second resets, chassis velocity impulses) across Gazebo backends:
  - Harmonic (GZ Sim): gz service CLI / gz transport / Docker pass-through
  - Classic (Gazebo 11 / ROS 2): ros2 service call / rclpy
  - DryRun: Deterministic simulation on non-Linux workstations or test suites

All subprocess executions are asynchronous with strict timeouts, ensuring zero
blocking of the terminal event loop.
"""

from __future__ import annotations

import abc
import asyncio
import math
import os
import shutil
import time
from dataclasses import dataclass, field
from typing import Any, Optional

from lang2sim.perturbation._compat import Pose2D
from lang2sim.perturbation.sdf_snippets import get_inline_sdf

__all__ = [
    "BridgeResult",
    "GazeboBridge",
    "DryRunBridge",
    "HarmonicBridge",
    "ClassicBridge",
    "get_bridge",
    "reset_bridge_instance",
]


@dataclass
class BridgeResult:
    """Result envelope returned by all mutation bridge operations."""
    ok: bool
    backend: str
    tool: str
    command: str = ""
    stdout: str = ""
    stderr: str = ""
    latency_ms: float = 0.0
    message: str = ""
    data: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return {
            "ok": self.ok,
            "backend": self.backend,
            "tool": self.tool,
            "command": self.command,
            "latency_ms": round(self.latency_ms, 2),
            "message": self.message,
            "data": self.data,
        }


class GazeboBridge(abc.ABC):
    """Abstract Base Class for simulator dynamic mutation dispatchers."""

    def __init__(self, world_name: Optional[str] = None, exec_prefix: Optional[str] = None) -> None:
        self.world_name = world_name or os.environ.get("LANG2SIM_WORLD_NAME") or "default"
        self.exec_prefix = (
            exec_prefix
            if exec_prefix is not None
            else (os.environ.get("LANG2SIM_EXEC_PREFIX") or "")
        ).strip()
        self.spawn_counter = 0
        self.spawned_entities: list[str] = []
        self.current_luminance: float = 1.0

    @abc.abstractmethod
    async def spawn_entity(
        self,
        model_name: str,
        pose: Pose2D,
        entity_name: Optional[str] = None,
    ) -> BridgeResult:
        """Spawn a dynamic obstacle into the live simulation."""
        ...

    @abc.abstractmethod
    async def set_light(self, luminance: float, light_name: Optional[str] = None) -> BridgeResult:
        """Dynamically alter ambient/sun lighting luminance (0.0 to 1.0)."""
        ...

    @abc.abstractmethod
    async def reset(self, mode: str = "world") -> BridgeResult:
        """Execute sub-second simulation reset without closing GUI windows."""
        ...

    @abc.abstractmethod
    async def apply_perturbation(self, velocity_bias: Pose2D, duration_s: float = 1.0) -> BridgeResult:
        """Apply force impulse or chassis velocity bias to the robot."""
        ...

    @abc.abstractmethod
    def health(self) -> dict[str, Any]:
        """Query bridge status and connectivity health."""
        ...

    def list_spawned_entities(self) -> list[str]:
        return list(self.spawned_entities)

    async def remove_entity(self, name: str) -> BridgeResult:
        """Delete a spawned entity from the live world (default: not supported by this backend)."""
        return BridgeResult(ok=False, backend=type(self).__name__, tool="remove_entity",
                            message="removing entities is not supported by this backend")

    def _next_entity_name(self, model_name: str, requested_name: Optional[str]) -> str:
        if requested_name:
            return requested_name
        self.spawn_counter += 1
        return f"{model_name}_{self.spawn_counter}"


# --------------------------------------------------------------------------- DryRun Bridge
class DryRunBridge(GazeboBridge):
    """Safe, zero-dependency bridge used on development machines and in unit tests."""

    async def remove_entity(self, name: str) -> BridgeResult:
        ok = name in self.spawned_entities
        if ok:
            self.spawned_entities.remove(name)
        return BridgeResult(ok=ok, backend="dryrun", tool="remove_entity",
                            message=f"Removed {name}" if ok else f"No spawned entity {name}",
                            data={"entity_name": name})

    def __init__(
        self,
        world_name: Optional[str] = None,
        exec_prefix: Optional[str] = None,
        simulated_target: str = "harmonic",
    ) -> None:
        super().__init__(world_name, exec_prefix)
        self.simulated_target = simulated_target

    async def spawn_entity(
        self,
        model_name: str,
        pose: Pose2D,
        entity_name: Optional[str] = None,
    ) -> BridgeResult:
        t0 = time.perf_counter()
        name = self._next_entity_name(model_name, entity_name)
        self.spawned_entities.append(name)
        # Formulate the command that would have run
        if self.simulated_target == "harmonic":
            cmd = (
                f"{self.exec_prefix} gz service -s /world/{self.world_name}/create "
                f"--reqtype gz.msgs.EntityFactory "
                f'--req "sdf_filename: \'model://{model_name}\', name: \'{name}\', '
                f'pose: {{position: {{x: {pose.x}, y: {pose.y}, z: 0.3}}, orientation: {{z: {pose.yaw}}}}}\"'
            ).strip()
        else:
            cmd = (
                f"{self.exec_prefix} ros2 service call /spawn_entity gazebo_msgs/srv/SpawnEntity "
                f'"{{name: \'{name}\', initial_pose: {{position: {{x: {pose.x}, y: {pose.y}, z: 0.3}}}}}}"'
            ).strip()

        latency = (time.perf_counter() - t0) * 1000.0 + 0.1
        return BridgeResult(
            ok=True,
            backend=f"dryrun ({self.simulated_target})",
            tool="spawn_dynamic_entity",
            command=cmd,
            stdout="[DryRun] Entity successfully registered in mock scene tree",
            latency_ms=latency,
            message=f"Spawned {name} at ({pose.x:.2f}, {pose.y:.2f})",
            data={"entity_name": name, "model_name": model_name, "pose": pose.model_dump()},
        )

    async def set_light(self, luminance: float, light_name: Optional[str] = None) -> BridgeResult:
        t0 = time.perf_counter()
        if not (0.0 <= luminance <= 1.0):
            return BridgeResult(
                ok=False,
                backend="dryrun",
                tool="set_ambient_lighting",
                message=f"Luminance must be between 0.0 and 1.0, got {luminance}",
            )
        self.current_luminance = float(luminance)
        light = light_name or os.environ.get("LANG2SIM_SUN_LIGHT") or "sun"
        if self.simulated_target == "harmonic":
            cmd = (
                f"{self.exec_prefix} gz service -s /world/{self.world_name}/light_config "
                f"--reqtype gz.msgs.Light "
                f'--req "name: \'{light}\', diffuse: {{r: {luminance:.2f}, g: {luminance:.2f}, b: {luminance:.2f}, a: 1.0}}"'
            ).strip()
        else:
            cmd = (
                f"{self.exec_prefix} ros2 service call /set_light_properties gazebo_msgs/srv/SetLightProperties "
                f'"{{light_name: \'{light}\', diffuse: {{r: {luminance:.2f}, g: {luminance:.2f}, b: {luminance:.2f}, a: 1.0}}}}"'
            ).strip()

        latency = (time.perf_counter() - t0) * 1000.0 + 0.1
        return BridgeResult(
            ok=True,
            backend=f"dryrun ({self.simulated_target})",
            tool="set_ambient_lighting",
            command=cmd,
            stdout="[DryRun] Lighting properties applied",
            latency_ms=latency,
            message=f"Ambient lighting adjusted to {luminance:.2f}",
            data={"luminance": luminance, "light_name": light},
        )

    async def reset(self, mode: str = "world") -> BridgeResult:
        t0 = time.perf_counter()
        if self.simulated_target == "harmonic":
            cmd = (
                f"{self.exec_prefix} gz service -s /world/{self.world_name}/control "
                f'--reqtype gz.msgs.WorldControl --req "reset: {{all: true}}"'
            ).strip()
        else:
            cmd = f"{self.exec_prefix} ros2 service call /reset_world std_srvs/srv/Empty '{{}}'".strip()

        latency = (time.perf_counter() - t0) * 1000.0 + 12.0  # simulates sub-300ms reset
        return BridgeResult(
            ok=True,
            backend=f"dryrun ({self.simulated_target})",
            tool="reset_simulation",
            command=cmd,
            stdout="[DryRun] Sub-second reset signal acknowledged",
            latency_ms=latency,
            message=f"Simulation reset completed ({mode}) in {latency:.1f}ms",
            data={"mode": mode, "reset_ms": latency},
        )

    async def apply_perturbation(self, velocity_bias: Pose2D, duration_s: float = 1.0) -> BridgeResult:
        t0 = time.perf_counter()
        robot = os.environ.get("LANG2SIM_ROBOT_NAME") or "turtlebot3_waffle"
        cmd = (
            f"{self.exec_prefix} ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "
            f'"{{linear: {{x: {velocity_bias.x}, y: {velocity_bias.y}}}, angular: {{z: {velocity_bias.yaw}}}}}"'
        ).strip()
        latency = (time.perf_counter() - t0) * 1000.0 + 0.1
        return BridgeResult(
            ok=True,
            backend=f"dryrun ({self.simulated_target})",
            tool="apply_chassis_perturbation",
            command=cmd,
            stdout="[DryRun] Chassis perturbation velocity injected",
            latency_ms=latency,
            message=f"Applied velocity impulse (vx={velocity_bias.x}, vy={velocity_bias.y}) for {duration_s}s",
            data={"velocity_bias": velocity_bias.model_dump(), "duration_s": duration_s, "robot": robot},
        )

    def health(self) -> dict[str, Any]:
        return {
            "status": "ready",
            "backend": f"dryrun ({self.simulated_target})",
            "world": self.world_name,
            "spawned_entities": len(self.spawned_entities),
            "current_luminance": self.current_luminance,
        }


# --------------------------------------------------------------------------- Subprocess execution helper
# `gz service` only sends a request when it knows the reply type, and exits 0 either way;
# success is Gazebo's "data: true" reply.
GZ_REPLY = "--reptype gz.msgs.Boolean --timeout 4000"
GZ_CMD_TIMEOUT_S = 8.0


def _gz_ok(rc: int, out: str) -> bool:
    return rc == 0 and "data: true" in out


async def _run_command(cmd_str: str, timeout_s: float = 3.0) -> tuple[int, str, str]:
    """Execute command asynchronously with timeout. Returns (returncode, stdout, stderr)."""
    try:
        proc = await asyncio.create_subprocess_shell(
            cmd_str,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.PIPE,
        )
        stdout, stderr = await asyncio.wait_for(proc.communicate(), timeout=timeout_s)
        return proc.returncode or 0, stdout.decode(errors="replace"), stderr.decode(errors="replace")
    except asyncio.TimeoutError:
        return -1, "", f"Command timed out after {timeout_s}s"
    except Exception as exc:
        return -2, "", str(exc)


# --------------------------------------------------------------------------- Harmonic Bridge
class HarmonicBridge(GazeboBridge):
    async def remove_entity(self, name: str) -> BridgeResult:
        t0 = time.perf_counter()
        req = f"name: '{name}', type: MODEL"
        cmd = f"{self.exec_prefix} gz service -s /world/{self.world_name}/remove --reqtype gz.msgs.Entity {GZ_REPLY} --req \"{req}\"".strip()
        rc, out, err = await _run_command(cmd, GZ_CMD_TIMEOUT_S)
        ok = _gz_ok(rc, out)
        if ok and name in self.spawned_entities:
            self.spawned_entities.remove(name)
        return BridgeResult(ok=ok, backend="harmonic", tool="remove_entity", command=cmd, stdout=out, stderr=err,
                            latency_ms=(time.perf_counter() - t0) * 1000.0,
                            message=f"Removed {name}" if ok else f"Failed to remove {name}: {err or out}",
                            data={"entity_name": name})

    """Bridge for Gazebo Harmonic (GZ Sim 8+) via `gz service` CLI / Docker pass-through."""

    async def spawn_entity(
        self,
        model_name: str,
        pose: Pose2D,
        entity_name: Optional[str] = None,
    ) -> BridgeResult:
        t0 = time.perf_counter()
        name = self._next_entity_name(model_name, entity_name)

        # Check if model exists in catalog or local file, else use inline SDF
        sdf_data = get_inline_sdf(model_name, name)
        # Using gz service /world/<name>/create
        req = (
            # Escape the SDF's double quotes: the request goes inside a double-quoted shell argument.
            f"sdf: '{sdf_data.replace(chr(10), ' ').replace(chr(34), chr(92) + chr(34))}', "
            f"name: '{name}', "
            f"pose: {{position: {{x: {pose.x}, y: {pose.y}, z: 0.35}}, "
            f"orientation: {{z: {math.sin(pose.yaw / 2):.6f}, w: {math.cos(pose.yaw / 2):.6f}}}}}"
        )
        cmd = f"{self.exec_prefix} gz service -s /world/{self.world_name}/create --reqtype gz.msgs.EntityFactory {GZ_REPLY} --req \"{req}\"".strip()

        rc, out, err = await _run_command(cmd, GZ_CMD_TIMEOUT_S)
        latency = (time.perf_counter() - t0) * 1000.0
        ok = _gz_ok(rc, out)
        if ok:
            self.spawned_entities.append(name)
        return BridgeResult(
            ok=ok,
            backend="harmonic",
            tool="spawn_dynamic_entity",
            command=cmd,
            stdout=out,
            stderr=err,
            latency_ms=latency,
            message=f"Spawned {name} into {self.world_name}" if rc == 0 else f"Failed to spawn: {err or out}",
            data={"entity_name": name, "model_name": model_name, "pose": pose.model_dump()},
        )

    async def set_light(self, luminance: float, light_name: Optional[str] = None) -> BridgeResult:
        t0 = time.perf_counter()
        if not (0.0 <= luminance <= 1.0):
            return BridgeResult(ok=False, backend="harmonic", tool="set_ambient_lighting", message="Luminance must be in [0, 1]")
        light = light_name or os.environ.get("LANG2SIM_SUN_LIGHT") or "sun"
        req = f"name: '{light}', diffuse: {{r: {luminance:.2f}, g: {luminance:.2f}, b: {luminance:.2f}, a: 1.0}}"
        cmd = f"{self.exec_prefix} gz service -s /world/{self.world_name}/light_config --reqtype gz.msgs.Light {GZ_REPLY} --req \"{req}\"".strip()

        rc, out, err = await _run_command(cmd, GZ_CMD_TIMEOUT_S)
        latency = (time.perf_counter() - t0) * 1000.0
        ok = _gz_ok(rc, out)
        if ok:
            self.current_luminance = luminance
        return BridgeResult(
            ok=ok,
            backend="harmonic",
            tool="set_ambient_lighting",
            command=cmd,
            stdout=out,
            stderr=err,
            latency_ms=latency,
            message=f"Light '{light}' luminance set to {luminance:.2f}",
            data={"luminance": luminance, "light_name": light},
        )

    async def reset(self, mode: str = "world") -> BridgeResult:
        t0 = time.perf_counter()
        req = "reset: {all: true}" if mode == "world" else "reset: {model_only: true}"
        cmd = f"{self.exec_prefix} gz service -s /world/{self.world_name}/control --reqtype gz.msgs.WorldControl {GZ_REPLY} --req \"{req}\"".strip()

        rc, out, err = await _run_command(cmd, GZ_CMD_TIMEOUT_S)
        latency = (time.perf_counter() - t0) * 1000.0
        return BridgeResult(
            ok=_gz_ok(rc, out),
            backend="harmonic",
            tool="reset_simulation",
            command=cmd,
            stdout=out,
            stderr=err,
            latency_ms=latency,
            message=f"Sub-second reset ({mode}) finished in {latency:.1f}ms",
            data={"mode": mode, "reset_ms": latency},
        )

    async def apply_perturbation(self, velocity_bias: Pose2D, duration_s: float = 1.0) -> BridgeResult:
        t0 = time.perf_counter()
        cmd = (
            f"{self.exec_prefix} ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "
            f'"{{linear: {{x: {velocity_bias.x}, y: {velocity_bias.y}}}, angular: {{z: {velocity_bias.yaw}}}}}"'
        ).strip()
        rc, out, err = await _run_command(cmd)
        latency = (time.perf_counter() - t0) * 1000.0
        return BridgeResult(
            ok=rc == 0,
            backend="harmonic",
            tool="apply_chassis_perturbation",
            command=cmd,
            stdout=out,
            stderr=err,
            latency_ms=latency,
            message=f"Injected velocity bias ({velocity_bias.x}, {velocity_bias.y})",
            data={"velocity_bias": velocity_bias.model_dump(), "duration_s": duration_s},
        )

    def health(self) -> dict[str, Any]:
        return {
            "status": "ready",
            "backend": "harmonic",
            "world": self.world_name,
            "exec_prefix": self.exec_prefix,
            "spawned_entities": len(self.spawned_entities),
            "current_luminance": self.current_luminance,
        }


# --------------------------------------------------------------------------- Classic Bridge
class ClassicBridge(GazeboBridge):
    """Bridge for Gazebo Classic (11) via ROS 2 Humble service calls."""

    async def spawn_entity(
        self,
        model_name: str,
        pose: Pose2D,
        entity_name: Optional[str] = None,
    ) -> BridgeResult:
        t0 = time.perf_counter()
        name = self._next_entity_name(model_name, entity_name)
        sdf_data = get_inline_sdf(model_name, name)
        # Using ros2 service call /spawn_entity
        req = (
            f"{{name: '{name}', xml: '{sdf_data.replace(chr(10), ' ')}', "
            f"initial_pose: {{position: {{x: {pose.x}, y: {pose.y}, z: 0.35}}}}}}"
        )
        cmd = f"{self.exec_prefix} ros2 service call /spawn_entity gazebo_msgs/srv/SpawnEntity \"{req}\"".strip()
        rc, out, err = await _run_command(cmd)
        latency = (time.perf_counter() - t0) * 1000.0
        if rc == 0:
            self.spawned_entities.append(name)
        return BridgeResult(
            ok=rc == 0,
            backend="classic",
            tool="spawn_dynamic_entity",
            command=cmd,
            stdout=out,
            stderr=err,
            latency_ms=latency,
            message=f"Spawned {name} in Gazebo Classic" if rc == 0 else f"Spawn failed: {err or out}",
            data={"entity_name": name, "model_name": model_name, "pose": pose.model_dump()},
        )

    async def set_light(self, luminance: float, light_name: Optional[str] = None) -> BridgeResult:
        t0 = time.perf_counter()
        if not (0.0 <= luminance <= 1.0):
            return BridgeResult(ok=False, backend="classic", tool="set_ambient_lighting", message="Luminance must be in [0, 1]")
        light = light_name or os.environ.get("LANG2SIM_SUN_LIGHT") or "sun"
        req = f"{{light_name: '{light}', diffuse: {{r: {luminance:.2f}, g: {luminance:.2f}, b: {luminance:.2f}, a: 1.0}}}}"
        cmd = f"{self.exec_prefix} ros2 service call /set_light_properties gazebo_msgs/srv/SetLightProperties \"{req}\"".strip()
        rc, out, err = await _run_command(cmd)
        latency = (time.perf_counter() - t0) * 1000.0
        self.current_luminance = luminance
        return BridgeResult(
            ok=rc == 0,
            backend="classic",
            tool="set_ambient_lighting",
            command=cmd,
            stdout=out,
            stderr=err,
            latency_ms=latency,
            message=f"Classic light '{light}' adjusted to {luminance:.2f}",
            data={"luminance": luminance, "light_name": light},
        )

    async def reset(self, mode: str = "world") -> BridgeResult:
        t0 = time.perf_counter()
        cmd = f"{self.exec_prefix} ros2 service call /reset_world std_srvs/srv/Empty '{{}}'".strip()
        rc, out, err = await _run_command(cmd)
        latency = (time.perf_counter() - t0) * 1000.0
        return BridgeResult(
            ok=rc == 0,
            backend="classic",
            tool="reset_simulation",
            command=cmd,
            stdout=out,
            stderr=err,
            latency_ms=latency,
            message=f"Gazebo Classic reset completed in {latency:.1f}ms",
            data={"mode": mode, "reset_ms": latency},
        )

    async def apply_perturbation(self, velocity_bias: Pose2D, duration_s: float = 1.0) -> BridgeResult:
        t0 = time.perf_counter()
        cmd = (
            f"{self.exec_prefix} ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "
            f'"{{linear: {{x: {velocity_bias.x}, y: {velocity_bias.y}}}, angular: {{z: {velocity_bias.yaw}}}}}"'
        ).strip()
        rc, out, err = await _run_command(cmd)
        latency = (time.perf_counter() - t0) * 1000.0
        return BridgeResult(
            ok=rc == 0,
            backend="classic",
            tool="apply_chassis_perturbation",
            command=cmd,
            stdout=out,
            stderr=err,
            latency_ms=latency,
            message=f"Classic velocity bias applied ({velocity_bias.x}, {velocity_bias.y})",
            data={"velocity_bias": velocity_bias.model_dump(), "duration_s": duration_s},
        )

    def health(self) -> dict[str, Any]:
        return {
            "status": "ready",
            "backend": "classic",
            "world": self.world_name,
            "spawned_entities": len(self.spawned_entities),
            "current_luminance": self.current_luminance,
        }


# --------------------------------------------------------------------------- Bridge Factory
_GLOBAL_BRIDGE: Optional[GazeboBridge] = None


def get_bridge(backend: Optional[str] = None) -> GazeboBridge:
    """Return cached singleton mutation bridge with automatic backend detection."""
    global _GLOBAL_BRIDGE
    if _GLOBAL_BRIDGE is not None and backend is None:
        return _GLOBAL_BRIDGE

    req = (backend or os.environ.get("LANG2SIM_SIM_BACKEND") or "auto").lower()
    exec_prefix = os.environ.get("LANG2SIM_EXEC_PREFIX")

    if req == "harmonic":
        bridge = HarmonicBridge(exec_prefix=exec_prefix)
    elif req == "classic":
        bridge = ClassicBridge(exec_prefix=exec_prefix)
    elif req == "dryrun":
        bridge = DryRunBridge(exec_prefix=exec_prefix)
    else:  # "auto"
        # Check if gz command exists on PATH or exec_prefix is supplied
        if exec_prefix or shutil.which("gz"):
            bridge = HarmonicBridge(exec_prefix=exec_prefix)
        elif shutil.which("ros2"):
            bridge = ClassicBridge(exec_prefix=exec_prefix)
        else:
            bridge = DryRunBridge(exec_prefix=exec_prefix)

    if backend is None:
        _GLOBAL_BRIDGE = bridge
    return bridge


def reset_bridge_instance() -> None:
    """Reset cached bridge instance (used by test suites)."""
    global _GLOBAL_BRIDGE
    _GLOBAL_BRIDGE = None
