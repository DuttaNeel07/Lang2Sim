"""Compatibility shim for Pose2D schema until Day-0 core models are merged."""

from __future__ import annotations

try:
    from lang2sim.core.models import Pose2D
except ImportError:
    try:
        from sim2lang.core.models import Pose2D
    except ImportError:
        from pydantic import BaseModel, Field

        class Pose2D(BaseModel):
            """2D ground pose (x, y, yaw)."""
            x: float = Field(..., description="X coordinate in meters")
            y: float = Field(..., description="Y coordinate in meters")
            yaw: float = Field(0.0, ge=-3.14159, le=3.14159, description="Heading in radians")

__all__ = ["Pose2D"]
