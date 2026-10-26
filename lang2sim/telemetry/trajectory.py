"""Robot trajectory container shared by the telemetry collectors and the NQI scorer.

A :class:`Trajectory` stores time-stamped planar kinematics (``t, x, y, yaw, v, w``)
as NumPy arrays. It can be built from raw odometry samples, loaded from / saved to
JSON or CSV, and resampled onto a uniform time grid (required before numerically
differentiating velocity into acceleration and jerk).

This module is pure Python + NumPy so it runs anywhere (no ROS required).
"""

from __future__ import annotations

import csv
import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterable, Mapping, Optional, Sequence

import numpy as np

__all__ = ["Trajectory"]

_FIELDS = ("t", "x", "y", "yaw", "v", "w")


def _as_array(values: Optional[Iterable[float]], n: int) -> Optional[np.ndarray]:
    if values is None:
        return None
    arr = np.asarray(list(values) if not isinstance(values, np.ndarray) else values, dtype=float)
    if arr.shape != (n,):
        raise ValueError(f"expected array of length {n}, got shape {arr.shape}")
    return arr


@dataclass
class Trajectory:
    """Time-stamped planar robot trajectory.

    Attributes:
        t: Timestamps in seconds (strictly increasing after :meth:`__post_init__`).
        x, y: Position in meters (world / map frame).
        yaw: Heading in radians. Derived from motion direction if not supplied.
        v: Signed linear speed in m/s. Derived from positions if not supplied.
        w: Angular speed in rad/s. Derived from yaw if not supplied.
        goal: Optional ``(x, y)`` mission goal.
        l_optimal: Optional optimal (A*) path length in meters.
        collisions: Number of force-qualified collision events during the run.
        label: Free-form run label (e.g. ``"nominal"``).
    """

    t: np.ndarray
    x: np.ndarray
    y: np.ndarray
    yaw: Optional[np.ndarray] = None
    v: Optional[np.ndarray] = None
    w: Optional[np.ndarray] = None
    goal: Optional[tuple[float, float]] = None
    l_optimal: Optional[float] = None
    collisions: int = 0
    label: str = ""
    meta: dict[str, Any] = field(default_factory=dict)
    slam_x: Optional[np.ndarray] = None
    slam_y: Optional[np.ndarray] = None
    ate: Optional[np.ndarray] = None

    # ------------------------------------------------------------------ init
    def __post_init__(self) -> None:
        self.t = np.asarray(self.t, dtype=float)
        n = self.t.shape[0]
        if self.t.ndim != 1 or n < 2:
            raise ValueError("trajectory needs at least 2 samples")
        self.x = _as_array(self.x, n)  # type: ignore[assignment]
        self.y = _as_array(self.y, n)  # type: ignore[assignment]
        self.yaw = _as_array(self.yaw, n)
        self.v = _as_array(self.v, n)
        self.w = _as_array(self.w, n)
        self.slam_x = _as_array(self.slam_x, n)
        self.slam_y = _as_array(self.slam_y, n)
        self.ate = _as_array(self.ate, n)

        # Sort by time and drop duplicate timestamps (odom can repeat stamps).
        order = np.argsort(self.t, kind="stable")
        keep = np.concatenate(([True], np.diff(self.t[order]) > 1e-9))
        idx = order[keep]
        for name in _FIELDS + ("slam_x", "slam_y", "ate"):
            arr = getattr(self, name)
            if arr is not None:
                setattr(self, name, arr[idx])
        if self.t.shape[0] < 2:
            raise ValueError("trajectory needs at least 2 distinct timestamps")

        if self.goal is not None:
            self.goal = (float(self.goal[0]), float(self.goal[1]))
        self._derive_missing()

    def _derive_missing(self) -> None:
        dx = np.gradient(self.x, self.t)
        dy = np.gradient(self.y, self.t)
        if self.yaw is None:
            heading = np.arctan2(dy, dx)
            # Hold the last valid heading while (almost) stationary.
            moving = np.hypot(dx, dy) > 1e-6
            if moving.any():
                first = int(np.argmax(moving))
                heading[:first] = heading[first]
                for i in range(first + 1, heading.size):
                    if not moving[i]:
                        heading[i] = heading[i - 1]
            else:
                heading[:] = 0.0
            self.yaw = heading
        if self.v is None:
            speed = np.hypot(dx, dy)
            # Sign by projection on heading (reverse motion => negative).
            proj = dx * np.cos(self.yaw) + dy * np.sin(self.yaw)
            self.v = np.where(proj < 0, -speed, speed)
        if self.w is None:
            self.w = np.gradient(np.unwrap(self.yaw), self.t)
        if self.ate is None and self.slam_x is not None and self.slam_y is not None:
            self.ate = np.hypot(self.x - self.slam_x, self.y - self.slam_y)

    # ------------------------------------------------------------ properties
    @property
    def ate_mean(self) -> float:
        return float(np.mean(self.ate)) if self.ate is not None and len(self.ate) > 0 else 0.0

    @property
    def ate_max(self) -> float:
        return float(np.max(self.ate)) if self.ate is not None and len(self.ate) > 0 else 0.0

    @property
    def ate_rmse(self) -> float:
        return float(np.sqrt(np.mean(self.ate ** 2))) if self.ate is not None and len(self.ate) > 0 else 0.0

    @property
    def n(self) -> int:
        return int(self.t.shape[0])

    @property
    def duration(self) -> float:
        return float(self.t[-1] - self.t[0])

    @property
    def positions(self) -> np.ndarray:
        """``(N, 2)`` array of positions."""
        return np.column_stack((self.x, self.y))

    @property
    def start(self) -> tuple[float, float]:
        return float(self.x[0]), float(self.y[0])

    @property
    def end(self) -> tuple[float, float]:
        return float(self.x[-1]), float(self.y[-1])

    # ------------------------------------------------------------ transforms
    def resample(self, dt: float = 0.05) -> "Trajectory":
        """Return a copy linearly interpolated onto a uniform grid of step ``dt``."""
        if dt <= 0:
            raise ValueError("dt must be positive")
        t_new = np.arange(self.t[0], self.t[-1] + 1e-9, dt)
        if t_new.size < 2:
            t_new = np.array([self.t[0], self.t[-1]])
        yaw_u = np.unwrap(self.yaw)  # type: ignore[arg-type]
        return Trajectory(
            t=t_new,
            x=np.interp(t_new, self.t, self.x),
            y=np.interp(t_new, self.t, self.y),
            yaw=np.angle(np.exp(1j * np.interp(t_new, self.t, yaw_u))),
            v=np.interp(t_new, self.t, self.v),  # type: ignore[arg-type]
            w=np.interp(t_new, self.t, self.w),  # type: ignore[arg-type]
            goal=self.goal,
            l_optimal=self.l_optimal,
            collisions=self.collisions,
            label=self.label,
            meta=dict(self.meta),
        )

    # ------------------------------------------------------------ factories
    @classmethod
    def from_samples(
        cls,
        samples: Sequence[Mapping[str, float]],
        **kwargs: Any,
    ) -> "Trajectory":
        """Build from a list of dicts with keys ``t, x, y`` and optional ``yaw, v, w``."""
        if len(samples) < 2:
            raise ValueError("need at least 2 samples")
        cols: dict[str, Optional[list[float]]] = {}
        for name in _FIELDS + ("slam_x", "slam_y", "ate"):
            if all(name in s and s[name] is not None for s in samples):
                cols[name] = [float(s[name]) for s in samples]
            else:
                cols[name] = None
        for req in ("t", "x", "y"):
            if cols[req] is None:
                raise ValueError(f"every sample needs key '{req}'")
        return cls(**cols, **kwargs)  # type: ignore[arg-type]

    # Alias matching the plan's naming.
    from_odom_samples = from_samples

    def to_dict(self) -> dict[str, Any]:
        samples = []
        for i in range(self.n):
            s = {name: float(getattr(self, name)[i]) for name in _FIELDS}
            if self.slam_x is not None:
                s["slam_x"] = float(self.slam_x[i])
            if self.slam_y is not None:
                s["slam_y"] = float(self.slam_y[i])
            if self.ate is not None:
                s["ate"] = float(self.ate[i])
            samples.append(s)
        res = {
            "label": self.label,
            "goal": list(self.goal) if self.goal is not None else None,
            "l_optimal": self.l_optimal,
            "collisions": int(self.collisions),
            "meta": self.meta,
            "samples": samples,
        }
        if self.ate is not None:
            res["ate_mean"] = round(self.ate_mean, 4)
            res["ate_max"] = round(self.ate_max, 4)
            res["ate_rmse"] = round(self.ate_rmse, 4)
        return res

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "Trajectory":
        goal = data.get("goal")
        return cls.from_samples(
            data["samples"],
            goal=tuple(goal) if goal is not None else None,
            l_optimal=data.get("l_optimal"),
            collisions=int(data.get("collisions", 0)),
            label=str(data.get("label", "")),
            meta=dict(data.get("meta") or {}),
        )

    def to_json(self, path: str | Path) -> Path:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(self.to_dict(), indent=2), encoding="utf-8")
        return path

    @classmethod
    def from_json(cls, path: str | Path) -> "Trajectory":
        return cls.from_dict(json.loads(Path(path).read_text(encoding="utf-8")))

    @classmethod
    def from_csv(cls, path: str | Path, **kwargs: Any) -> "Trajectory":
        """Load a CSV with a header row containing at least ``t,x,y``."""
        with Path(path).open(newline="", encoding="utf-8") as fh:
            rows = [
                {k.strip(): float(v) for k, v in row.items() if v not in (None, "")}
                for row in csv.DictReader(fh)
            ]
        return cls.from_samples(rows, **kwargs)
