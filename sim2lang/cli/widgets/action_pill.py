"""
Tool Action Status Pill Widget
File: sim2lang/cli/widgets/action_pill.py
"""

from textual.widget import Widget
from textual.widgets import Static
from textual.reactive import reactive

class ActionPill(Widget):
    """Tool invocation status badge pill (e.g. ■ solve_spatial_layout · 0.4s · completed)."""

    DEFAULT_CSS = """
    ActionPill {
        height: auto;
        background: #161616;
        border: solid #222222;
        padding: 0 1;
        margin-bottom: 1;
    }
    """

    tool_name = reactive("")
    duration_str = reactive("0.0s")
    status = reactive("running")  # running, completed, error

    def __init__(self, tool_name: str, duration_str: str = "0.4s", status: str = "completed", **kwargs):
        super().__init__(classes="action-pill", **kwargs)
        self.tool_name = tool_name
        self.duration_str = duration_str
        self.status = status

    def compose(self):
        yield Static(self._format_content(), id="pill-content")

    def _format_content(self) -> str:
        icon = "[#38bdf8]■[/]"
        if self.status == "completed":
            status_text = "[#2ed573]completed[/]"
        elif self.status == "running":
            status_text = "[#ffa502]running...[/]"
        else:
            status_text = "[#ff4757]failed[/]"
        return f"{icon}  [bold #f1f2f6]{self.tool_name}[/] · [#a4b0be]{self.duration_str}[/] · {status_text}"

    def set_running(self) -> None:
        self.status = "running"
        try:
            self.query_one("#pill-content", Static).update(self._format_content())
        except Exception:
            pass

    def set_completed(self, duration: str = "0.5s") -> None:
        self.status = "completed"
        self.duration_str = duration
        try:
            self.query_one("#pill-content", Static).update(self._format_content())
        except Exception:
            pass

    def set_failed(self, duration: str = "0.5s") -> None:
        self.status = "error"
        self.duration_str = duration
        try:
            self.query_one("#pill-content", Static).update(self._format_content())
        except Exception:
            pass
