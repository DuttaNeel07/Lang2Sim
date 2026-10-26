"""FastAPI router for dynamic simulation stress mutations (/v1/world).

Mounted inside the simulation container (Control API server, :9090)
or accessible via ControlClient on host.
"""

from __future__ import annotations

from typing import Any, List, Optional
from fastapi import APIRouter, FastAPI, HTTPException
from pydantic import BaseModel, Field

from lang2sim.perturbation._compat import Pose2D
from lang2sim.perturbation.gazebo_bridge import GazeboBridge, get_bridge

router = APIRouter(prefix="/world", tags=["world_mutation"])


class SpawnRequest(BaseModel):
    model_name: str = Field("wooden_crate", description="Asset type to spawn")
    x: float = Field(2.0, description="X coordinate in meters")
    y: float = Field(0.0, description="Y coordinate in meters")
    z: float = Field(0.5, description="Z coordinate in meters")
    yaw: float = Field(0.0, description="Heading rotation in radians")
    linear_velocity: Optional[List[float]] = Field(None, description="Initial [vx, vy, vz]")
    angular_velocity: Optional[List[float]] = Field(None, description="Initial [wx, wy, wz]")
    entity_name: Optional[str] = Field(None, description="Optional custom entity name")


class LightRequest(BaseModel):
    diffuse: float = Field(0.5, ge=0.0, le=1.0, description="Sun diffuse reflectance")
    specular: float = Field(0.1, ge=0.0, le=1.0, description="Sun specular reflectance")
    intensity: Optional[float] = Field(None, ge=0.0, le=1.0, description="Overall luminance multiplier")
    light_name: str = Field("sun", description="Target light entity")


class ResetRequest(BaseModel):
    mode: str = Field("world", description="Reset scope ('world' or 'robot')")


class ChassisPerturbationRequest(BaseModel):
    force_x: float = Field(10.0, description="X impulse force / velocity bias")
    force_y: float = Field(0.0, description="Y impulse force / velocity bias")
    force_z: float = Field(0.0, description="Z angular impulse / yaw velocity bias")
    duration_s: float = Field(0.2, ge=0.01, le=5.0, description="Impulse duration in seconds")
    robot_model: str = Field("turtlebot3_waffle", description="Target robot model name")


@router.post("/spawn")
async def spawn_entity(req: SpawnRequest) -> dict[str, Any]:
    bridge: GazeboBridge = get_bridge()
    res = await bridge.spawn_entity(
        req.model_name,
        Pose2D(x=req.x, y=req.y, yaw=req.yaw),
    )
    if not res.ok:
        raise HTTPException(status_code=500, detail=res.to_dict())
    return res.to_dict()


@router.post("/light")
async def set_light(req: LightRequest) -> dict[str, Any]:
    bridge: GazeboBridge = get_bridge()
    lum = req.intensity if req.intensity is not None else req.diffuse
    res = await bridge.set_light(lum)
    if not res.ok:
        raise HTTPException(status_code=500, detail=res.to_dict())
    return res.to_dict()


@router.post("/reset")
async def reset_world(req: Optional[ResetRequest] = None) -> dict[str, Any]:
    bridge: GazeboBridge = get_bridge()
    mode = req.mode if req and req.mode else "world"
    res = await bridge.reset(mode)
    if not res.ok:
        raise HTTPException(status_code=500, detail=res.to_dict())
    return res.to_dict()


@router.post("/chassis")
async def apply_chassis(req: ChassisPerturbationRequest) -> dict[str, Any]:
    bridge: GazeboBridge = get_bridge()
    res = await bridge.apply_perturbation(
        Pose2D(x=req.force_x, y=req.force_y, yaw=req.force_z),
        duration_s=req.duration_s,
    )
    if not res.ok:
        raise HTTPException(status_code=500, detail=res.to_dict())
    return res.to_dict()


@router.get("/health")
def world_health() -> dict[str, Any]:
    bridge: GazeboBridge = get_bridge()
    h = bridge.health()
    h["ok"] = h.get("status") in ("ready", "healthy", "ok")
    return h


@router.get("/entities")
def list_entities() -> dict[str, Any]:
    bridge: GazeboBridge = get_bridge()
    entities = bridge.list_spawned_entities()
    return {"backend": type(bridge).__name__, "count": len(entities), "entities": entities}


def mount_world_router(app: FastAPI, prefix: str = "/v1") -> None:
    """Convenience helper to mount the world router into any FastAPI app."""
    app.include_router(router, prefix=prefix)
