"""
Stage-wise Execution Plan Card Widget
File: lang2sim/cli/widgets/plan_card.py
"""

from typing import Any, Dict, List

from textual.containers import Horizontal
from textual.widget import Widget
from textual.widgets import Static


class PlanCard(Widget):
    """
    Antigravity/OpenCode-style Stage-wise Execution Plan Card.
    Visualizes multi-step autonomous workflows ([ ] -> [▶] -> [✔]).
    """

    DEFAULT_CSS = """
    PlanCard {
        height: auto;
        background: #111114;
        border: solid #27272a;
        border-left: thick #a78bfa;
        padding: 0 1;
        margin-top: 1;
        margin-bottom: 1;
    }

    .plan-header {
        height: 1;
        margin-bottom: 1;
    }

    .plan-title {
        color: #f1f2f6;
        text-style: bold;
    }

    .plan-counter {
        text-align: right;
        color: #a78bfa;
    }

    .plan-step {
        height: auto;
        padding: 0 1;
        margin-bottom: 0;
    }
    """

    def __init__(
        self,
        steps: List[str],
        title: str = "Autonomous Execution Plan",
        **kwargs,
    ):
        super().__init__(**kwargs)
        self.plan_title = title
        # Each step: {"title": str, "status": "pending"|"running"|"completed"|"failed", "detail": ""}
        self.steps: List[Dict[str, Any]] = [
            {"title": s, "status": "pending", "detail": ""} for s in steps
        ]
        if self.steps:
            self.steps[0]["status"] = "running"

    def compose(self):
        with Horizontal(classes="plan-header"):
            yield Static(f"📋 [bold #f1f2f6]{self.plan_title}[/]", classes="plan-title")
            yield Static(self._format_counter(), id="plan-counter", classes="plan-counter")

        for i in range(len(self.steps)):
            yield Static(self._format_step(i), id=f"step-{i}", classes="plan-step")

    def _format_counter(self) -> str:
        completed = sum(1 for s in self.steps if s["status"] == "completed")
        total = len(self.steps)
        if completed == total and total > 0:
            return f"[bold #2ed573]All {total} Steps Completed ✔[/]"
        return f"[#ced6e0]{completed}/{total} Completed[/]"

    def _format_step(self, index: int) -> str:
        step = self.steps[index]
        status = step["status"]
        title = step["title"]
        detail = f" [#747d8c]({step['detail']})[/]" if step.get("detail") else ""

        if status == "completed":
            return f"  [bold #2ed573]✔[/]  [bold #ffffff]{title}[/]{detail}"
        elif status == "running":
            return f"  [bold #38bdf8]▶[/]  [bold #38bdf8]{title}[/]{detail}"
        elif status == "failed":
            return f"  [bold #ff4757]✖[/]  [bold #ff4757]{title}[/]{detail}"
        else:
            return f"  [#57606f][ ][/]  [#a4b0be]{title}[/]"

    def update_step(self, index: int, status: str, detail: str = "") -> None:
        """Updates the status and detail of a specific step."""
        if 0 <= index < len(self.steps):
            self.steps[index]["status"] = status
            if detail:
                self.steps[index]["detail"] = detail
            try:
                self.query_one(f"#step-{index}", Static).update(self._format_step(index))
                self.query_one("#plan-counter", Static).update(self._format_counter())
            except Exception:
                pass
