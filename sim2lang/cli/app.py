"""
Main Textual Application for Lang2Sim
File: sim2lang/cli/app.py
"""

from pathlib import Path
from textual.app import App
from textual.binding import Binding
from .screens.landing_screen import LandingScreen

CSS_PATH = Path(__file__).parent / "styles" / "app.tcss"

class Lang2SimApp(App):
    """Lang2Sim Terminal Copilot Application."""

    TITLE = "Sim2Lang — lang2sim"
    CSS_PATH = CSS_PATH

    BINDINGS = [
        Binding("ctrl+c", "quit", "Quit", show=False),
        Binding("ctrl+p", "command_palette", "Command Palette", show=False),
    ]

    def on_mount(self) -> None:
        """Mount initial landing screen."""
        self.push_screen(LandingScreen())

def main():
    """Console script entrypoint: lang2sim."""
    app = Lang2SimApp()
    app.run()

if __name__ == "__main__":
    main()
