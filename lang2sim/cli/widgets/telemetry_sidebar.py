"""
Telemetry Sidebar Widget (20% Workspace Right Pane)
File: lang2sim/cli/widgets/telemetry_sidebar.py
"""

from dataclasses import dataclass
from typing import Optional

from textual.containers import Vertical
from textual.widget import Widget
from textual.widgets import Static


@dataclass
class TelemetryData:
    # Filled from GET /v1/telemetry/live (lang2sim.telemetry.live.sidebar_fields).
    sim_state: str = "No simulation"
    sim_time: str = "--:--"
    rtf: str = "--"
    robot_x: float = 0.0
    robot_y: float = 0.0
    speed: float = 0.0
    clearance: Optional[float] = None
    bumper_force: float = 0.0
    collision_count: int = 0
    tokens_used: int = 1420
    context_capacity_pct: int = 1


class TelemetrySidebar(Widget):
    """Right-hand sidebar displaying live simulation, robot kinematics, bumper, and context stats."""

    DEFAULT_CSS = """
    TelemetrySidebar {
        width: 100%;
        height: 100%;
        background: #0e0e0e;
        padding: 1;
    }
    """

    def __init__(self, **kwargs):
        super().__init__(id="telemetry-sidebar", **kwargs)
        self.data = TelemetryData()

    def compose(self):
        with Vertical():
            yield Static("[bold #f1f2f6]SIMULATION[/]", classes="sidebar-section-title")
            yield Static(self._format_sim_state(), id="sim-state", classes="sidebar-metric")
            yield Static(self._format_sim_time(), id="sim-time", classes="sidebar-metric")
            yield Static(self._format_rtf(), id="sim-rtf", classes="sidebar-metric")

            yield Static("[bold #f1f2f6]ROBOT TELEMETRY[/]", classes="sidebar-section-title")
            yield Static(self._format_pose(), id="robot-pose", classes="sidebar-metric")
            yield Static(self._format_speed(), id="robot-speed", classes="sidebar-metric")
            yield Static(self._format_clearance(), id="robot-clearance", classes="sidebar-metric")

            yield Static("[bold #f1f2f6]BUMPER[/]", classes="sidebar-section-title")
            yield Static(self._format_bumper_force(), id="bumper-force", classes="sidebar-metric")
            yield Static(
                self._format_collision_count(), id="collision-count", classes="sidebar-metric"
            )

            yield Static("[bold #f1f2f6]CONTEXT[/]", classes="sidebar-section-title")
            yield Static(self._format_context(), id="token-context", classes="sidebar-metric")

    def _format_sim_state(self) -> str:
        color = "#2ed573" if "Active" in self.data.sim_state else "#ffa502"
        return f"State: [{color}]{self.data.sim_state}[/]"

    def _format_sim_time(self) -> str:
        return f"Sim Time: [#ced6e0]{self.data.sim_time}[/]"

    def _format_rtf(self) -> str:
        return f"RTF: [#ced6e0]{self.data.rtf}[/]"

    def _format_pose(self) -> str:
        return f"Pose: [#ced6e0]({self.data.robot_x:.2f}m, {self.data.robot_y:.2f}m)[/]"

    def _format_speed(self) -> str:
        return f"Speed: [#ced6e0]{self.data.speed:.2f}m/s[/]"

    def _format_clearance(self) -> str:
        if self.data.clearance is None:
            return "Clearance: [#57606f]--[/]"
        color = "#ff4757" if self.data.clearance < 0.25 else "#2ed573"
        return f"Clearance: [{color}]{self.data.clearance:.2f}m[/]"

    def _format_bumper_force(self) -> str:
        return f"Forces: [#ced6e0]{self.data.bumper_force:.1f} N[/]"

    def _format_collision_count(self) -> str:
        color = "#ff4757" if self.data.collision_count > 0 else "#ced6e0"
        return f"Hits: [{color}]{self.data.collision_count}[/]"

    def _format_context(self) -> str:
        return f"[#ced6e0]{self.data.tokens_used:,} tokens\n{self.data.context_capacity_pct}% used\n$0.00 spent[/]"

    def update_telemetry(self, **kwargs) -> None:
        """Update any metric dynamically and refresh corresponding label."""
        for key, value in kwargs.items():
            if hasattr(self.data, key):
                setattr(self.data, key, value)

        try:
            self.query_one("#sim-state", Static).update(self._format_sim_state())
            self.query_one("#sim-time", Static).update(self._format_sim_time())
            self.query_one("#sim-rtf", Static).update(self._format_rtf())
            self.query_one("#robot-pose", Static).update(self._format_pose())
            self.query_one("#robot-speed", Static).update(self._format_speed())
            self.query_one("#robot-clearance", Static).update(self._format_clearance())
            self.query_one("#bumper-force", Static).update(self._format_bumper_force())
            self.query_one("#collision-count", Static).update(self._format_collision_count())
            self.query_one("#token-context", Static).update(self._format_context())
        except Exception:
            pass
