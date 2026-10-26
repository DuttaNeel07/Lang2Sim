"""
Screen 1: Landing Screen
File: sim2lang/cli/screens/landing_screen.py
"""

from textual.screen import Screen
from textual.widgets import Static
from textual.containers import Vertical, Center, Horizontal
from ..widgets.prompt_card import PromptCard

# ── Logo: pixel-bitmap glyphs rendered with half-blocks (2 pixel rows per text row) ──
GLYPHS = {
    "l": ["##", "##", "##", "##", "##", "##", "..", ".."],
    "a": ["......", "......", "######", "....##", "######", "######", "......", "......"],
    "n": ["......", "......", "######", "##..##", "##..##", "##..##", "......", "......"],
    "g": ["......", "......", "######", "##..##", "######", "....##", "....##", "######"],
    "2": ["######", "....##", "######", "##....", "##....", "######", "......", "......"],
    "s": ["......", "......", "######", "##....", "....##", "######", "......", "......"],
    "i": ["##", "..", "##", "##", "##", "##", "..", ".."],
    "m": ["..........", "..........", "##########", "##..##..##",
          "##..##..##", "##..##..##", "..........", ".........."],
}

# Left-to-right fade: dark gray -> white (one shade per letter)
COLORS = {
    "l": "#6b7280",
    "a": "#80858f",
    "n": "#979ca6",
    "g": "#adb2ba",
    "2": "#c8ccd2",
    "s": "#dfe1e5",
    "i": "#f1f2f4",
    "m": "#ffffff",
}

# (top pixel, bottom pixel) -> half-block character
HALF = {(0, 0): " ", (1, 0): "▀", (0, 1): "▄", (1, 1): "█"}


def build_logo(word: str = "lang2sim", gap: int = 1) -> str:
    """Render glyph bitmaps as 4 text rows with per-letter Rich color tags."""
    rows = []
    for r in range(0, 8, 2):
        line = ""
        for idx, ch in enumerate(word):
            g = GLYPHS[ch]
            seg = "".join(
                HALF[(int(g[r][x] == "#"), int(g[r + 1][x] == "#"))]
                for x in range(len(g[0]))
            )
            line += f"[{COLORS[ch]}]{seg}[/]"
            if idx < len(word) - 1:
                line += " " * gap
        rows.append(line)
    return "\n".join(rows)


LOGO_TEXT = build_logo()


class LandingScreen(Screen):
    """The initial landing screen matching the reference image."""

    def compose(self):
        with Center(id="landing-container"):
            with Vertical(id="landing-inner"):
                yield Static(LOGO_TEXT, id="landing-logo")
                yield PromptCard(
                    placeholder="Ask anything.. 'Spawn 14x10m warehouse with 8 racks, send AMR to rack 5'",
                    card_id="landing-prompt-input",
                    id="landing-card",
                )
                with Horizontal(id="shortcuts-container"):
                    yield Static(
                        "[#6b7280]tab[/] [#9ca3af]agents[/]   [#6b7280]ctrl+p[/] [#9ca3af]commands[/]",
                        id="landing-shortcuts",
                    )
                with Center():
                    yield Static(
                        "[#f59e0b]●[/] [bold #f3f4f6]Tip[/] [#9ca3af]Press[/] "
                        "[bold #ffffff]shift+enter[/] [#9ca3af]to add newlines in your prompt[/]",
                        id="landing-tip",
                    )

        with Horizontal(id="workspace-footer"):
            yield Static("~", classes="footer-left")
            yield Static("", classes="footer-spacer")
            yield Static("[#6b7280]lang2sim 2.0.0[/]", classes="footer-right")

    def on_mount(self) -> None:
        self.query_one(PromptCard).focus_input()

    def on_prompt_card_submitted(self, message: PromptCard.Submitted) -> None:
        """When user submits a prompt, transition to WorkspaceScreen."""
        from .workspace_screen import WorkspaceScreen
        self.app.switch_screen(WorkspaceScreen(initial_prompt=message.value))
