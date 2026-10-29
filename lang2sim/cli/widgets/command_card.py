"""
Interactive Terminal Command & Permission Card Widget
File: lang2sim/cli/widgets/command_card.py
"""

from typing import Any, Dict, Optional

from textual.binding import Binding
from textual.containers import Horizontal
from textual.message import Message
from textual.widget import Widget
from textual.widgets import Button, Static


class CommandCard(Widget):
    """
    OpenCode/Antigravity-style Interactive Terminal Command Card.
    Presents proposed commands to the user, asks for approval, runs execution,
    and displays exit codes, execution duration, and terminal stdout/stderr.
    """

    DEFAULT_CSS = """
    CommandCard {
        height: auto;
        background: #111114;
        border: solid #27272a;
        border-left: thick #ffa502;
        padding: 1 1;
        margin-bottom: 1;
    }

    CommandCard.status-running {
        border-left: thick #38bdf8;
    }

    CommandCard.status-completed {
        border-left: thick #2ed573;
    }

    CommandCard.status-failed {
        border-left: thick #ff4757;
    }

    CommandCard.status-rejected {
        border-left: thick #747d8c;
    }

    .cmd-header {
        height: 1;
        margin-bottom: 1;
    }

    .cmd-title {
        color: #f1f2f6;
        text-style: bold;
    }

    .cmd-badge {
        text-align: right;
    }

    .cmd-snippet {
        background: #09090b;
        border: solid #1e1e24;
        color: #38bdf8;
        padding: 0 1;
        margin-bottom: 1;
        height: auto;
    }

    .cmd-actions {
        height: 3;
        margin-top: 1;
        align-horizontal: left;
    }

    .cmd-actions Button {
        margin-right: 2;
        min-width: 16;
        height: 3;
    }

    .cmd-output {
        background: #08080a;
        border: solid #1c1c22;
        color: #ced6e0;
        padding: 0 1;
        margin-top: 1;
        height: auto;
        max-height: 12;
        overflow-y: auto;
    }
    """

    BINDINGS = [
        Binding("y", "approve", "Approve", show=False),
        Binding("enter", "approve", "Approve", show=False),
        Binding("a", "always_allow", "Always Allow", show=False),
        Binding("n", "reject", "Reject", show=False),
        Binding("escape", "reject", "Reject", show=False),
    ]

    class Approved(Message):
        """User approved command execution."""

        def __init__(self, event_id: str, always_allow: bool = False):
            super().__init__()
            self.event_id = event_id
            self.always_allow = always_allow

    class Rejected(Message):
        """User rejected command execution."""

        def __init__(self, event_id: str):
            super().__init__()
            self.event_id = event_id

    def __init__(
        self,
        event_id: str,
        command: str,
        tool_name: str = "run_command",
        arguments: Optional[Dict[str, Any]] = None,
        permission_required: bool = True,
        **kwargs,
    ):
        super().__init__(**kwargs)
        self.event_id = event_id
        self.command = command
        self.tool_name = tool_name
        self.arguments = arguments or {}
        self.permission_required = permission_required
        self.status = "awaiting_permission" if permission_required else "running"
        self.exit_code: Optional[int] = None
        self.duration_str: str = "0.0s"
        self.stdout: str = ""
        self.stderr: str = ""

    def compose(self):
        # Header: Title + Status Badge
        with Horizontal(classes="cmd-header"):
            yield Static(
                f"⚡ [bold #f1f2f6]Terminal Command[/] [#747d8c]({self.tool_name})[/]",
                classes="cmd-title",
            )
            yield Static(self._format_badge(), id="cmd-badge", classes="cmd-badge")

        # Command Code Snippet
        yield Static(f"[bold #38bdf8]$[/] [bold #ffffff]{self.command}[/]", classes="cmd-snippet")

        # Interactive Approval Buttons
        if self.status == "awaiting_permission":
            with Horizontal(id="cmd-actions", classes="cmd-actions"):
                yield Button(
                    "Approve & Run (y)", id=f"btn-approve-{self.event_id}", variant="success"
                )
                yield Button(
                    "Always Allow (a)", id=f"btn-always-{self.event_id}", variant="default"
                )
                yield Button("Reject (n)", id=f"btn-reject-{self.event_id}", variant="error")

        # Output Terminal Container (hidden until execution completes)
        yield Static("", id="cmd-output", classes="cmd-output", expand=True)

    def on_mount(self) -> None:
        self._update_classes()
        out_widget = self.query_one("#cmd-output", Static)
        out_widget.display = False
        if self.status == "awaiting_permission":
            try:
                self.query_one(f"#btn-approve-{self.event_id}", Button).focus()
            except Exception:
                pass

    def _format_badge(self) -> str:
        if self.status == "awaiting_permission":
            return "[bold #ffa502]⏳ AWAITING PERMISSION[/]"
        elif self.status == "running":
            return "[bold #38bdf8]● RUNNING...[/]"
        elif self.status == "completed":
            code = self.exit_code if self.exit_code is not None else 0
            if code == 0:
                return f"[bold #2ed573]✔ EXIT 0[/] [#747d8c]· {self.duration_str}[/]"
            else:
                return f"[bold #ff4757]✖ EXIT {code}[/] [#747d8c]· {self.duration_str}[/]"
        elif self.status == "rejected":
            return "[bold #747d8c]⛔ REJECTED[/]"
        return f"[#747d8c]{self.status}[/]"

    def _update_classes(self) -> None:
        self.remove_class("status-running")
        self.remove_class("status-completed")
        self.remove_class("status-failed")
        self.remove_class("status-rejected")

        if self.status == "running":
            self.add_class("status-running")
        elif self.status == "completed":
            if (self.exit_code or 0) == 0:
                self.add_class("status-completed")
            else:
                self.add_class("status-failed")
        elif self.status == "rejected":
            self.add_class("status-rejected")

    def on_button_pressed(self, event: Button.Pressed) -> None:
        btn_id = event.button.id or ""
        if "btn-approve" in btn_id:
            self.action_approve()
        elif "btn-always" in btn_id:
            self.action_always_allow()
        elif "btn-reject" in btn_id:
            self.action_reject()

    def action_approve(self) -> None:
        if self.status != "awaiting_permission":
            return
        self.set_running()
        self.post_message(self.Approved(self.event_id, always_allow=False))

    def action_always_allow(self) -> None:
        if self.status != "awaiting_permission":
            return
        self.set_running()
        self.post_message(self.Approved(self.event_id, always_allow=True))

    def action_reject(self) -> None:
        if self.status != "awaiting_permission":
            return
        self.status = "rejected"
        self._update_classes()
        self._remove_actions_row()
        self.query_one("#cmd-badge", Static).update(self._format_badge())
        self.post_message(self.Rejected(self.event_id))

    def _remove_actions_row(self) -> None:
        try:
            actions_row = self.query_one("#cmd-actions")
            actions_row.remove()
        except Exception:
            pass

    def set_running(self) -> None:
        self.status = "running"
        self._update_classes()
        self._remove_actions_row()
        try:
            self.query_one("#cmd-badge", Static).update(self._format_badge())
        except Exception:
            pass

    def set_result(
        self, exit_code: int, duration_str: str, stdout: str = "", stderr: str = ""
    ) -> None:
        self.status = "completed"
        self.exit_code = exit_code
        self.duration_str = duration_str
        self.stdout = stdout
        self.stderr = stderr
        self._update_classes()
        self._remove_actions_row()

        try:
            self.query_one("#cmd-badge", Static).update(self._format_badge())
            out_widget = self.query_one("#cmd-output", Static)
            lines = []
            if stdout.strip():
                lines.append(f"[#a4b0be]{stdout.strip()}[/]")
            if stderr.strip():
                lines.append(f"[#ff6b81]{stderr.strip()}[/]")
            if not lines:
                lines.append("[italic #57606f](No output produced)[/]")

            out_widget.update("\n".join(lines))
            out_widget.display = True
        except Exception:
            pass
