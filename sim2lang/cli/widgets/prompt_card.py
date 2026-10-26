"""
Input Card Widget (Floating & Sticky Docked)
File: sim2lang/cli/widgets/prompt_card.py
"""

from textual.widget import Widget
from textual.widgets import Input, Static
from textual.containers import Vertical
from textual.message import Message

class PromptCard(Widget):
    """Input card with cyan vertical bar and model status badge row."""

    DEFAULT_CSS = """
    PromptCard {
        height: auto;
        max-height: 5;
        background: #161616;
        border-left: thick #00d2ff;
        padding: 0 1;
    }

    PromptCard > Vertical {
        height: auto;
    }

    PromptCard Input {
        background: transparent;
        border: none;
        height: 1;
        padding: 0;
        margin: 0;
        color: #ffffff;
    }

    PromptCard Input:focus {
        border: none;
        background: transparent;
    }

    PromptCard #badge-row {
        height: 1;
        margin-top: 1;
        color: #747d8c;
    }
    """

    class Submitted(Message):
        """Dispatched when the user presses Enter in the prompt card."""
        def __init__(self, value: str) -> None:
            self.value = value
            super().__init__()

    def __init__(self, placeholder: str = 'Ask anything.. "Spawn 14x10m warehouse with 8 racks, send AMR to rack 5"', card_id: str = "prompt-input", **kwargs):
        super().__init__(**kwargs)
        self.placeholder_text = placeholder
        self.card_id = card_id

    def compose(self):
        with Vertical():
            yield Input(placeholder=self.placeholder_text, id=self.card_id)
            yield Static(
                "[bold #38bdf8]■ Build[/]  ·  [#a4b0be]Nemotron 3 Ultra 550B A55B Nvidia[/]  ·  [#f39c12]medium[/]",
                id="badge-row",
            )

    def on_input_submitted(self, event: Input.Submitted) -> None:
        value = event.value.strip()
        if value:
            # Clear input
            event.input.value = ""
            self.post_message(self.Submitted(value))

    def focus_input(self) -> None:
        inp = self.query_one(f"#{self.card_id}", Input)
        inp.focus()
