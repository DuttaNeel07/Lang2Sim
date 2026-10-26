"""
Custom Command Palette Modal (ctrl+p)
File: lang2sim/cli/screens/command_palette_modal.py
"""

from typing import List, Dict, Optional
from textual.screen import ModalScreen
from textual.widgets import Input, Static
from textual.containers import Vertical, Horizontal, VerticalScroll
from textual.binding import Binding


COMMANDS: List[Dict[str, str]] = [
    # Suggested
    {"id": "switch_session", "category": "Suggested", "title": "Switch session", "shortcut": "ctrl+x l"},
    {"id": "switch_model", "category": "Suggested", "title": "Switch model", "shortcut": "ctrl+x m"},
    {"id": "run_doctor", "category": "Suggested", "title": "Run simulation doctor", "shortcut": "ctrl+x d"},

    # Session
    {"id": "switch_session_all", "category": "Session", "title": "Switch session", "shortcut": "ctrl+x l"},
    {"id": "new_session", "category": "Session", "title": "New session", "shortcut": "ctrl+x n"},
    {"id": "open_editor", "category": "Session", "title": "Open editor", "shortcut": "ctrl+x e"},
    {"id": "move_session", "category": "Session", "title": "Move session", "description": "Move to another project dir", "shortcut": ""},

    # Agent
    {"id": "switch_model_agent", "category": "Agent", "title": "Switch model", "shortcut": "ctrl+x m"},
    {"id": "switch_agent", "category": "Agent", "title": "Switch agent", "shortcut": "ctrl+x a"},
    {"id": "toggle_mcps", "category": "Agent", "title": "Toggle MCPs", "shortcut": ""},
    {"id": "variant_cycle", "category": "Agent", "title": "Variant cycle", "shortcut": "ctrl+t"},
    {"id": "switch_variant", "category": "Agent", "title": "Switch model variant", "shortcut": ""},

    # Runtime & Display
    {"id": "sim_doctor", "category": "Runtime", "title": "Run doctor diagnostic", "shortcut": "ctrl+x d"},
    {"id": "toggle_headless", "category": "Runtime", "title": "Toggle headless mode", "shortcut": "ctrl+x h"},
    {"id": "connect_stream", "category": "Runtime", "title": "Connect Foxglove / noVNC", "shortcut": "ctrl+x f"},
    {"id": "reset_sim", "category": "Runtime", "title": "Emergency reset world", "shortcut": "ctrl+x r"},

    # Provider
    {"id": "connect_provider", "category": "Provider", "title": "Connect provider", "shortcut": ""},
]


class CommandPaletteModal(ModalScreen[Optional[str]]):
    """Floating Command Palette Modal matching OpenCode UI."""

    BINDINGS = [
        Binding("escape", "dismiss_modal", "Close", show=False),
        Binding("up", "cursor_up", "Up", show=False),
        Binding("down", "cursor_down", "Down", show=False),
        Binding("enter", "submit_selected", "Select", show=False),
    ]

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.selected_index: int = 0
        self.filtered_commands: List[Dict[str, str]] = list(COMMANDS)

    def compose(self):
        with Vertical(id="command-palette-dialog"):
            with Horizontal(id="palette-header"):
                yield Static("Commands", id="palette-title")
                yield Static("esc", id="palette-esc")

            yield Input(placeholder="Search", id="palette-search")
            yield VerticalScroll(id="palette-list")

    async def on_mount(self) -> None:
        self.query_one(Input).focus()
        await self.rebuild_list()

    async def on_input_changed(self, event: Input.Changed) -> None:
        query = event.value.strip().lower()
        if not query:
            self.filtered_commands = list(COMMANDS)
        else:
            self.filtered_commands = [
                cmd for cmd in COMMANDS
                if query in cmd["title"].lower()
                or query in cmd["category"].lower()
                or query in cmd.get("description", "").lower()
            ]
        self.selected_index = 0
        await self.rebuild_list()

    def on_input_submitted(self, event: Input.Submitted) -> None:
        self.action_submit_selected()

    async def rebuild_list(self) -> None:
        list_container = self.query_one("#palette-list", VerticalScroll)
        await list_container.remove_children()

        if not self.filtered_commands:
            await list_container.mount(Static("[#6b7280]No matching commands[/]", classes="palette-empty"))
            return

        current_category = None
        current_item_idx = 0

        for cmd in self.filtered_commands:
            cat = cmd["category"]
            if cat != current_category:
                current_category = cat
                await list_container.mount(Static(cat, classes="palette-category"))

            is_selected = (current_item_idx == self.selected_index)
            item_classes = "palette-item -selected" if is_selected else "palette-item"

            title_text = cmd["title"]
            if cmd.get("description"):
                title_text += f" [#6b7280]{cmd['description']}[/]"

            children = [Static(title_text, classes="palette-item-title")]
            if cmd.get("shortcut"):
                children.append(Static(cmd["shortcut"], classes="palette-shortcut"))

            row = Horizontal(*children, classes=item_classes)
            await list_container.mount(row)

            current_item_idx += 1

    def update_selection(self) -> None:
        rows = list(self.query("#palette-list .palette-item"))
        for idx, row in enumerate(rows):
            row.set_class(idx == self.selected_index, "-selected")
        if 0 <= self.selected_index < len(rows):
            rows[self.selected_index].scroll_visible(animate=False)

    def action_cursor_up(self) -> None:
        if self.filtered_commands and self.selected_index > 0:
            self.selected_index -= 1
            self.update_selection()

    def action_cursor_down(self) -> None:
        if self.filtered_commands and self.selected_index < len(self.filtered_commands) - 1:
            self.selected_index += 1
            self.update_selection()

    def action_submit_selected(self) -> None:
        if self.filtered_commands and 0 <= self.selected_index < len(self.filtered_commands):
            cmd = self.filtered_commands[self.selected_index]
            self.dismiss(cmd["id"])
        else:
            self.dismiss(None)

    def action_dismiss_modal(self) -> None:
        self.dismiss(None)
