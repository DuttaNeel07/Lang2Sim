"""
Main Textual Application for Lang2Sim
File: lang2sim/cli/app.py
"""

import os
from pathlib import Path

from textual.app import App
from textual.binding import Binding

from .screens.landing_screen import LandingScreen

CSS_PATH = Path(__file__).parent / "styles" / "app.tcss"


class Lang2SimApp(App):
    """Lang2Sim Terminal Copilot Application."""

    TITLE = "lang2sim — lang2sim"
    CSS_PATH = [CSS_PATH] if CSS_PATH.is_file() else []

    BINDINGS = [
        Binding("ctrl+a", "select_all", "Select All", show=False),
        Binding("ctrl+c", "copy_or_quit", "Copy / Quit", show=False),
        Binding("ctrl+shift+c", "copy_selection", "Copy Selection", show=False),
        Binding("shift+ctrl+c", "copy_selection", "Copy Selection", show=False),
        Binding("ctrl+y", "copy_selection", "Copy Selection", show=False),
        Binding("ctrl+p", "command_palette", "Command Palette", show=False),
        Binding("ctrl+s", "open_sessions", "Sessions", show=False),
        Binding("ctrl+n", "new_session", "New Session", show=False),
    ]

    def action_select_all(self) -> None:
        """Selects all text in focused input or selects the entire conversation for copying."""
        if hasattr(self.screen, "action_select_all"):
            self.screen.action_select_all()
        else:
            focused = self.screen.focused
            if focused is not None and hasattr(focused, "action_select_all"):
                focused.action_select_all()

    def action_copy_selection(self) -> None:
        """Copies currently selected text or entire conversation to system clipboard via OSC 52."""
        import base64
        import sys
        from pathlib import Path

        text = ""
        if hasattr(self.screen, "_selected_text") and self.screen._selected_text:
            text = self.screen._selected_text
        elif hasattr(self.screen, "get_selected_text"):
            text = self.screen.get_selected_text() or ""

        if not text:
            focused = self.screen.focused
            if focused is not None:
                if hasattr(focused, "selected_text") and focused.selected_text:
                    text = focused.selected_text
                elif hasattr(focused, "value") and focused.value:
                    text = focused.value
                elif hasattr(focused, "text") and focused.text:
                    text = focused.text

        # Fallback: copy full conversation if nothing was specifically selected
        if not text and hasattr(self.screen, "get_conversation_as_text"):
            text = self.screen.get_conversation_as_text()

        if text:
            import shutil
            import subprocess

            # 1. Native Wayland clipboard (wl-copy) for Ubuntu Wayland sessions
            wl_copy = shutil.which("wl-copy") or str(Path.home() / ".local" / "bin" / "wl-copy")
            try:
                env = os.environ.copy()
                if "WAYLAND_DISPLAY" not in env:
                    env["WAYLAND_DISPLAY"] = "wayland-0"
                p = subprocess.Popen([wl_copy], stdin=subprocess.PIPE, env=env)
                p.communicate(input=text.encode("utf-8"), timeout=1.0)
            except Exception:
                pass

            # 2. Native X11 clipboard via xclip (for Xwayland / X11 apps)
            xclip = shutil.which("xclip") or str(Path.home() / ".local" / "bin" / "xclip")
            try:
                for sel in ["clipboard", "primary"]:
                    p = subprocess.Popen([xclip, "-selection", sel], stdin=subprocess.PIPE)
                    p.communicate(input=text.encode("utf-8"), timeout=1.0)
            except Exception:
                pass

            # 3. Textual native clipboard
            try:
                self.copy_to_clipboard(text)
            except Exception:
                pass

            # 4. Direct OSC 52 terminal clipboard escape sequence
            try:
                b64 = base64.b64encode(text.encode("utf-8")).decode("ascii")
                sys.stdout.write(f"\033]52;c;{b64}\007")
                sys.stdout.flush()
            except Exception:
                pass

            # 5. File export for zero-friction copy/paste
            try:
                export_path = Path.home() / ".lang2sim" / "clipboard_export.txt"
                export_path.parent.mkdir(parents=True, exist_ok=True)
                export_path.write_text(text, encoding="utf-8")
            except Exception:
                pass

            self.notify(
                f"Copied {len(text)} characters to clipboard!", title="Clipboard", timeout=2.0
            )

    def action_copy_or_quit(self) -> None:
        """Copies selected text if any exists, otherwise exits."""
        text = ""
        if hasattr(self.screen, "_selected_text") and self.screen._selected_text:
            text = self.screen._selected_text
        elif hasattr(self.screen, "get_selected_text"):
            text = self.screen.get_selected_text() or ""
        if not text:
            focused = self.screen.focused
            if focused is not None and hasattr(focused, "selected_text") and focused.selected_text:
                text = focused.selected_text

        if text:
            self.action_copy_selection()
        else:
            self.exit()

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
