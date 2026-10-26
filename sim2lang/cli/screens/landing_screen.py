"""
Screen 1: Landing Screen
File: sim2lang/cli/screens/landing_screen.py
"""

from textual.screen import Screen
from textual.widgets import Static
from textual.containers import Vertical, Center, Horizontal
from ..widgets.prompt_card import PromptCard

LOGO_TEXT = """[#9ca3af]█                     [#ffffff] ▀▀▀▀█          ▀▀        [/]
[#9ca3af]█  ▄▀▀▀█ █▀▀▀█ ▄▀▀▀█  [#ffffff]    ▄█ ▄▀▀▀▀   ██ █▀▀█▀▀█[/]
[#9ca3af]█  █ ▀▀█ █   █ █▄▄▄█  [#ffffff] ▄▀▀   ▀▀▀▀█   ██ █  █  █[/]
[#9ca3af]█  █▄▄▄█ █   █     █  [#ffffff] █▄▄▄█ ▄▄▄▄▀   ██ █  █  █[/]
[#9ca3af]               ▄▄▄▄█  [#ffffff]                            [/]"""

class LandingScreen(Screen):
    """The initial landing screen matching the reference image."""

    def compose(self):
        with Center(id="landing-container"):
            with Vertical(id="landing-inner"):
                yield Static(LOGO_TEXT, id="landing-logo")
                yield PromptCard(
                    placeholder="Ask anything.. 'Spawn 14x10m warehouse with 8 racks, send AMR to rack 5'",
                    card_id="landing-prompt-input",
                    id="landing-card"
                )
                with Horizontal(id="shortcuts-container"):
                    yield Static("[#6b7280]tab[/] [#9ca3af]agents[/]   [#6b7280]ctrl+p[/] [#9ca3af]commands[/]", id="landing-shortcuts")
                with Center():
                    yield Static("[#f59e0b]●[/] [bold #f3f4f6]Tip[/] [#9ca3af]Press[/] [bold #ffffff]shift+enter[/] [#9ca3af]to add newlines in your prompt[/]", id="landing-tip")

        with Horizontal(id="workspace-footer"):
            yield Static("~", classes="footer-left")
            yield Static(" " * 60)
            yield Static("[#6b7280]lang2sim 2.0.0[/]", classes="footer-right")

    def on_mount(self) -> None:
        self.query_one(PromptCard).focus_input()

    def on_prompt_card_submitted(self, message: PromptCard.Submitted) -> None:
        """When user submits a prompt, transition to WorkspaceScreen."""
        from .workspace_screen import WorkspaceScreen
        self.app.switch_screen(WorkspaceScreen(initial_prompt=message.value))
