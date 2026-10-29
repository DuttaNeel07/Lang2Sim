"""
Sessions Modal (ctrl+s / Command Palette)
File: lang2sim/cli/screens/sessions_modal.py
"""

from typing import List, Optional

from textual.binding import Binding
from textual.containers import Horizontal, Vertical, VerticalScroll
from textual.screen import ModalScreen
from textual.widgets import Input, Static

from ...core.session_manager import ChatSession, session_manager


class RenameModal(ModalScreen[Optional[str]]):
    """Small popup modal to rename a session."""

    DEFAULT_CSS = """
    RenameModal {
        align: center middle;
        background: rgba(0, 0, 0, 0.75);
    }

    #rename-dialog {
        width: 50;
        height: auto;
        background: #161618;
        border: solid #27272a;
        padding: 1 2;
    }

    #rename-title {
        color: #ffffff;
        text-style: bold;
        margin-bottom: 1;
    }

    #rename-input {
        background: #09090b;
        border: solid #3f3f46;
        color: #ffffff;
    }
    """

    BINDINGS = [
        Binding("escape", "cancel", "Cancel"),
    ]

    def __init__(self, current_title: str, **kwargs):
        super().__init__(**kwargs)
        self.current_title = current_title

    def compose(self):
        with Vertical(id="rename-dialog"):
            yield Static("Rename Session", id="rename-title")
            yield Input(value=self.current_title, id="rename-input")

    def on_mount(self) -> None:
        inp = self.query_one("#rename-input", Input)
        inp.focus()

    def on_input_submitted(self, event: Input.Submitted) -> None:
        val = event.value.strip()
        self.dismiss(val if val else None)

    def action_cancel(self) -> None:
        self.dismiss(None)


class SessionsModal(ModalScreen[Optional[str]]):
    """Modal dialog displaying grouped chat sessions with search, pin, delete, and rename."""

    BINDINGS = [
        Binding("escape", "dismiss_modal", "Dismiss", show=False),
        Binding("up", "cursor_up", "Cursor Up", show=False),
        Binding("down", "cursor_down", "Cursor Down", show=False),
        Binding("enter", "submit_selected", "Open Session", show=False),
        Binding("ctrl+f", "toggle_pin", "Pin/Unpin", show=False),
        Binding("ctrl+d", "delete_selected", "Delete", show=False),
        Binding("ctrl+r", "rename_selected", "Rename", show=False),
    ]

    def __init__(self, workspace_name: Optional[str] = None, **kwargs):
        super().__init__(**kwargs)
        self.workspace_name = workspace_name
        self.all_sessions: List[ChatSession] = []
        self.filtered_sessions: List[ChatSession] = []
        self.selected_index: int = 0

    def compose(self):
        with Vertical(id="sessions-dialog"):
            # Header: Sessions | esc
            with Horizontal(id="sessions-header"):
                yield Static("Sessions", id="sessions-title")
                yield Static("", classes="header-spacer")
                yield Static("esc", id="sessions-esc")

            # Search bar
            yield Input(placeholder="Search", id="sessions-search")

            # Sessions list container
            with VerticalScroll(id="sessions-list"):
                yield Static("", id="sessions-list-anchor")

            # Footer actions bar
            with Horizontal(id="sessions-footer"):
                yield Static(
                    "pin/unpin [bold #9ca3af]ctrl+f[/]   delete [bold #9ca3af]ctrl+d[/]   rename [bold #9ca3af]ctrl+r[/]",
                    id="sessions-footer-actions",
                )

    async def on_mount(self) -> None:
        self.load_and_refresh()
        self.query_one("#sessions-search", Input).focus()

    def load_and_refresh(self) -> None:
        self.all_sessions = session_manager.list_sessions(self.workspace_name)
        search_query = ""
        try:
            search_query = self.query_one("#sessions-search", Input).value.strip().lower()
        except Exception:
            pass
        self.filter_sessions(search_query)

    def on_input_changed(self, event: Input.Changed) -> None:
        if event.input.id == "sessions-search":
            self.filter_sessions(event.value.strip().lower())

    def on_input_submitted(self, event: Input.Submitted) -> None:
        if event.input.id == "sessions-search":
            self.action_submit_selected()

    def filter_sessions(self, query: str) -> None:
        if not query:
            self.filtered_sessions = list(self.all_sessions)
        else:
            self.filtered_sessions = [s for s in self.all_sessions if query in s.title.lower()]

        if self.selected_index >= len(self.filtered_sessions):
            self.selected_index = max(0, len(self.filtered_sessions) - 1)

        self.run_worker(self.rebuild_list())

    async def rebuild_list(self) -> None:
        list_container = self.query_one("#sessions-list", VerticalScroll)
        await list_container.remove_children()

        if not self.filtered_sessions:
            row = Horizontal(
                Static("[#71717a]No matching sessions found[/]", classes="session-empty")
            )
            await list_container.mount(row)
            return

        grouped = session_manager.group_sessions_by_date(self.filtered_sessions)
        current_item_idx = 0

        for header, sessions in grouped.items():
            # Date group header (lavender)
            header_row = Horizontal(
                Static(f"[bold #a78bfa]{header}[/]", classes="session-group-header")
            )
            await list_container.mount(header_row)

            for s in sessions:
                is_selected = current_item_idx == self.selected_index
                item_classes = "session-item -selected" if is_selected else "session-item"
                bullet = "● " if is_selected else "  "
                title_text = s.title

                row = Horizontal(
                    Static(f"{bullet}{title_text}", classes="session-item-text"),
                    classes=item_classes,
                )
                await list_container.mount(row)
                current_item_idx += 1

    def update_selection(self) -> None:
        rows = list(self.query("#sessions-list .session-item"))
        for idx, row in enumerate(rows):
            is_selected = idx == self.selected_index
            row.set_class(is_selected, "-selected")
            try:
                txt_widget = row.query_one(".session-item-text", Static)
                raw_text = self.filtered_sessions[idx].title
                bullet = "● " if is_selected else "  "
                txt_widget.update(f"{bullet}{raw_text}")
            except Exception:
                pass
        if 0 <= self.selected_index < len(rows):
            rows[self.selected_index].scroll_visible(animate=False)

    def action_cursor_up(self) -> None:
        if self.filtered_sessions and self.selected_index > 0:
            self.selected_index -= 1
            self.update_selection()

    def action_cursor_down(self) -> None:
        if self.filtered_sessions and self.selected_index < len(self.filtered_sessions) - 1:
            self.selected_index += 1
            self.update_selection()

    def action_submit_selected(self) -> None:
        if self.filtered_sessions and 0 <= self.selected_index < len(self.filtered_sessions):
            chosen = self.filtered_sessions[self.selected_index]
            self.dismiss(chosen.id)
        else:
            self.dismiss(None)

    def action_toggle_pin(self) -> None:
        if self.filtered_sessions and 0 <= self.selected_index < len(self.filtered_sessions):
            target = self.filtered_sessions[self.selected_index]
            session_manager.pin_session(target.id)
            self.load_and_refresh()

    def action_delete_selected(self) -> None:
        if self.filtered_sessions and 0 <= self.selected_index < len(self.filtered_sessions):
            target = self.filtered_sessions[self.selected_index]
            session_manager.delete_session(target.id)
            self.load_and_refresh()

    def action_rename_selected(self) -> None:
        if self.filtered_sessions and 0 <= self.selected_index < len(self.filtered_sessions):
            target = self.filtered_sessions[self.selected_index]

            def on_renamed(new_name: Optional[str]) -> None:
                if new_name:
                    session_manager.rename_session(target.id, new_name)
                    self.load_and_refresh()

            self.app.push_screen(RenameModal(current_title=target.title), callback=on_renamed)

    def action_dismiss_modal(self) -> None:
        self.dismiss(None)
