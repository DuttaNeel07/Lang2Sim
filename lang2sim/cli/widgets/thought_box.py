"""
Collapsible Thought Box Widget
File: lang2sim/cli/widgets/thought_box.py
"""

from textual.events import Click
from textual.reactive import reactive
from textual.widget import Widget
from textual.widgets import Static


class ThoughtBox(Widget):
    """Collapsible reasoning thought block matching OpenCode UX (+ Thought: 1.2s)."""

    DEFAULT_CSS = """
    ThoughtBox {
        height: auto;
        margin-bottom: 1;
    }
    """

    is_collapsed = reactive(True)
    duration_str = reactive("1.2s")
    thought_content = reactive("")

    def __init__(self, duration_str: str = "1.2s", content: str = "", **kwargs):
        super().__init__(classes="thought-box", **kwargs)
        self.duration_str = duration_str
        self.thought_content = (
            content or "Evaluating warehouse boundaries and spatial constraints..."
        )

    def compose(self):
        yield Static(self._format_header(), id="thought-header", classes="thought-header")
        yield Static(self.thought_content, id="thought-body", classes="thought-content")

    def _format_header(self) -> str:
        symbol = "+" if self.is_collapsed else "−"
        hint = "" if self.is_collapsed else " (click to collapse)"
        return f"{symbol} Thought: {self.duration_str}{hint}"

    def on_click(self, event: Click) -> None:
        self.toggle()

    def toggle(self) -> None:
        self.is_collapsed = not self.is_collapsed
        header = self.query_one("#thought-header", Static)
        body = self.query_one("#thought-body", Static)
        header.update(self._format_header())
        body.display = not self.is_collapsed

    def on_mount(self) -> None:
        body = self.query_one("#thought-body", Static)
        body.display = not self.is_collapsed
