"""
Health monitor: healthy / degraded / crashed, with the reasons.
File: lang2sim/runtime/health_monitor.py

Pure function over what the API server already knows (process status, the ROS
snapshot, readiness), plus a little memory so a slow simulation is only
reported once RTF has stayed low for a while. Feeds /v1/telemetry's
health_status, /v1/sim/health and the self-healer.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Literal, Optional

from lang2sim.runtime.ros_bridge import RosSnapshot, readiness

Health = Literal["healthy", "degraded", "crashed"]

LOW_RTF = 0.2           # below this the sim is too slow to be useful
LOW_RTF_FOR_S = 15.0    # ...for this long before we call it degraded
STALL_S = 5.0           # clock not advancing this long while the process runs = stalled


@dataclass
class HealthReport:
    health: Health
    issues: List[str] = field(default_factory=list)
    rtf: Optional[float] = None
    sim_time: float = 0.0
    readiness: Dict[str, bool] = field(default_factory=dict)


class HealthMonitor:
    def __init__(self, low_rtf: float = LOW_RTF, low_rtf_for_s: float = LOW_RTF_FOR_S) -> None:
        self.low_rtf, self.low_rtf_for_s = low_rtf, low_rtf_for_s
        self._low_since: Optional[float] = None

    def reset(self) -> None:
        self._low_since = None

    def evaluate(self, process_status: str, snap: Optional[RosSnapshot]) -> HealthReport:
        """process_status: SimProcess.status() value (idle/launching/running/degraded/stopped/error)."""
        if process_status == "error":
            return HealthReport("crashed", ["simulation process exited with an error"])
        if process_status in ("idle", "stopped") or snap is None:
            self.reset()
            return HealthReport("healthy")

        checks = readiness(snap)
        issues: List[str] = []
        if process_status == "launching":
            return HealthReport("healthy", [], snap.rtf, snap.sim_time, checks)

        # Running (or degraded): every check should hold.
        issues += [f"{name} lost" for name, ok in checks.items() if not ok]
        if snap.clock_wall is not None and snap.now - snap.clock_wall > STALL_S:
            issues.append(f"simulation clock stalled for {snap.now - snap.clock_wall:.0f} s")
        if snap.rtf is not None and snap.rtf < self.low_rtf:
            self._low_since = self._low_since or snap.now
            if snap.now - self._low_since >= self.low_rtf_for_s:
                issues.append(f"real-time factor {snap.rtf:.2f} below {self.low_rtf} for "
                              f"{snap.now - self._low_since:.0f} s")
        else:
            self._low_since = None
        return HealthReport("degraded" if issues else "healthy", issues, snap.rtf, snap.sim_time, checks)
