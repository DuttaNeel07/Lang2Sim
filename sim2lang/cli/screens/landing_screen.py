"""
Screen 1: Landing Screen
File: sim2lang/cli/screens/landing_screen.py
"""

from textual.screen import Screen
from textual.widgets import Static
from textual.containers import Vertical, Center, Horizontal
from ..widgets.prompt_card import PromptCard

LOGO_TEXT = """
██╗      █████╗ ███╗   ██╗ ██████╗ ██████╗ ███████╗██╗███╗   ███╗
██║     ██╔══██╗████╗  ██║██╔════╝ ╚════██╗██╔════╝██║████╗ ████║
██║     ███████║██╔██╗ ██║██║  ███╗ █████╔╝███████╗██║██╔████╔██║
██║     ██╔══██║██║╚██╗██║██║   ██║██╔═══╝ ╚════██║██║██║╚██╔╝██║
███████╗██║  ██║██║ ╚████║╚██████╔╝███████╗███████║██║██║ ╚═╝ ██║
╚══════╝╚═╝  ╚═╝╚═╝  ╚═══╝ ╚═════╝ ╚══════╝╚══════╝╚═╝╚═╝     ╚═╝
"""

class LandingScreen(Screen):
    """The initial landing screen with centered logo, floating card, and keybind tips."""

    def compose(self):
        with Center(id="landing-container"):
            with Vertical():
                yield Static(f"[bold #f1f2f6]{LOGO_TEXT}[/]", id="landing-logo")
                with Center():
                    yield PromptCard(
                        placeholder='Ask anything.. "Spawn 14x10m warehouse with 8 racks, send AMR to rack 5"',
                        card_id="landing-prompt-input",
                        id="landing-card"
                    )
                with Center():
                    yield Static("[#57606f]tab[/] [#747d8c]agents[/]    [#57606f]ctrl+p[/] [#747d8c]commands[/]", id="landing-shortcuts")
                with Center():
                    yield Static("[#f39c12]●[/] [bold #f1f2f6]Tip[/]  [#747d8c]Press[/] [bold #ced6e0]shift+enter[/] [#747d8c]to add newlines in your prompt[/]", id="landing-tip")

        with Horizontal(id="workspace-footer"):
            yield Static("~", classes="footer-left")
            yield Static(" " * 50)
            yield Static("[#57606f]lang2sim 2.0.0[/]", classes="footer-right")

    def on_mount(self) -> None:
        # Automatically focus the prompt input field
        self.query_one(PromptCard).focus_input()

    def on_prompt_card_submitted(self, message: PromptCard.Submitted) -> None:
        """When user submits a prompt, transition to WorkspaceScreen and start execution."""
        from .workspace_screen import WorkspaceScreen
        self.app.switch_screen(WorkspaceScreen(initial_prompt=message.value))
