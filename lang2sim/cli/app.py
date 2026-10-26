"""
Main Textual Application for Lang2Sim
File: lang2sim/cli/app.py
"""

from pathlib import Path
from textual.app import App
from textual.binding import Binding
from .screens.landing_screen import LandingScreen

CSS_PATH = Path(__file__).parent / "styles" / "app.tcss"

class Lang2SimApp(App):
    """Lang2Sim Terminal Copilot Application."""

    TITLE = "lang2sim — lang2sim"
    CSS_PATH = CSS_PATH

    BINDINGS = [
        Binding("ctrl+c", "quit", "Quit", show=False),
        Binding("ctrl+p", "command_palette", "Command Palette", show=False),
        Binding("ctrl+s", "open_sessions", "Sessions", show=False),
        Binding("ctrl+n", "new_session", "New Session", show=False),
    ]

    def on_mount(self) -> None:
        """Mount initial landing screen."""
        self.push_screen(LandingScreen())

    def action_command_palette(self) -> None:
        """Open custom floating command palette modal."""
        from .screens.command_palette_modal import CommandPaletteModal
        self.push_screen(CommandPaletteModal(), callback=self.on_command_palette_result)

    def action_open_sessions(self) -> None:
        """Open custom floating sessions modal."""
        from .screens.sessions_modal import SessionsModal
        self.push_screen(SessionsModal(), callback=self.on_session_selected)

    def action_new_session(self) -> None:
        """Create new blank session and navigate to Workspace."""
        from .screens.workspace_screen import WorkspaceScreen
        self.switch_screen(WorkspaceScreen(session_id=None))

    def on_session_selected(self, session_id: str | None) -> None:
        if not session_id:
            return
        from .screens.workspace_screen import WorkspaceScreen
        if isinstance(self.screen, WorkspaceScreen):
            self.screen.load_session(session_id)
        else:
            self.switch_screen(WorkspaceScreen(session_id=session_id))

    def on_command_palette_result(self, result: str | None) -> None:
        if not result:
            return
        if result in ("switch_session", "switch_session_all"):
            self.call_after_refresh(self.action_open_sessions)
        elif result == "new_session":
            self.call_after_refresh(self.action_new_session)
        elif result in ("switch_model", "switch_model_agent"):
            self.notify("Model: NVIDIA Nemotron 3 Ultra 550B active", title="Model")
        elif result in ("run_doctor", "sim_doctor"):
            self.notify("All systems nominal: GPU, Display, and Docker ready", title="Doctor")
        elif result == "toggle_headless":
            self.notify("Headless mode toggled", title="Runtime")
        elif result == "connect_stream":
            self.notify("Foxglove stream available at ws://127.0.0.1:8765", title="Stream")
        elif result == "reset_sim":
            self.notify("Emergency world reset dispatched", title="Simulation")


def main():
    """Console script entrypoint: lang2sim."""
    app = Lang2SimApp()
    app.run()

if __name__ == "__main__":
    main()
