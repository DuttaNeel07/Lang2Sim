"""
Screen 2: 80/20 Workspace Screen
File: sim2lang/cli/screens/workspace_screen.py
"""

import asyncio
from textual.screen import Screen
from textual.widgets import Static, Markdown
from textual.containers import Horizontal, Vertical, VerticalScroll
from textual import work
from ..widgets.prompt_card import PromptCard
from ..widgets.thought_box import ThoughtBox
from ..widgets.action_pill import ActionPill
from ..widgets.telemetry_sidebar import TelemetrySidebar

class WorkspaceScreen(Screen):
    """The 80/20 split workspace screen featuring conversational execution and live telemetry."""

    def __init__(self, initial_prompt: str = "", **kwargs):
        super().__init__(**kwargs)
        self.initial_prompt = initial_prompt

    def compose(self):
        # Top title bar
        yield Static("S2L | Simulation Active — lang2sim", id="workspace-header")

        # 80/20 Horizontal body
        with Horizontal(id="workspace-body"):
            # Left Pane: 80% Width
            with Vertical(id="left-pane"):
                with VerticalScroll(id="chat-scroll"):
                    yield Static("", id="scroll-anchor")
                yield PromptCard(
                    placeholder='lang2sim > Type follow-up (e.g. "drop light to 20%, spawn crate")',
                    card_id="sticky-prompt-input",
                    id="sticky-input-card",
                )

            # Right Sidebar: 20% Width
            yield TelemetrySidebar()

        # Bottom footer bar
        with Horizontal(id="workspace-footer"):
            yield Static("~", classes="footer-left")
            yield Static(" " * 40)
            yield Static("[#ced6e0]1.4K (1%)[/]   [#57606f]ctrl+p commands[/]   [#2ed573]●[/] [#57606f]lang2sim 2.0.0[/]", classes="footer-right")

    def on_mount(self) -> None:
        # Focus sticky input card
        self.query_one("#sticky-input-card", PromptCard).focus_input()
        # If there was an initial prompt from landing screen, process it!
        if self.initial_prompt:
            self.handle_user_prompt(self.initial_prompt)

    def on_prompt_card_submitted(self, message: PromptCard.Submitted) -> None:
        """When user types in sticky input card, add prompt and run execution."""
        self.handle_user_prompt(message.value)

    def handle_user_prompt(self, prompt: str) -> None:
        """Post user message and run async task to process."""
        chat_scroll = self.query_one("#chat-scroll", VerticalScroll)

        # 1. Add user prompt card
        user_card = Static(f"[bold #ffffff]{prompt}[/]", classes="user-prompt-card")
        chat_scroll.mount(user_card)
        chat_scroll.scroll_end(animate=False)

        # 2. Trigger non-blocking worker for reasoning and execution
        self.run_execution_cycle(prompt)

    @work(exclusive=False, thread=True)
    def run_execution_cycle(self, prompt: str) -> None:
        """Simulate or execute non-blocking reasoning cycle with thoughts, tools, and response."""
        chat_scroll = self.query_one("#chat-scroll", VerticalScroll)
        sidebar = self.query_one(TelemetrySidebar)

        # Step 1: Mount thought block
        thought = ThoughtBox(
            duration_str="1.2s",
            content=f"Analyzing prompt '{prompt}'. Formulating spatial bounds, corridor clearance rules, and simulation parameters..."
        )
        self.app.call_from_thread(chat_scroll.mount, thought)
        self.app.call_from_thread(chat_scroll.scroll_end)

        # Simulate quick thinking pause
        asyncio.run(asyncio.sleep(0.4))

        # Step 2: Mount Tool Action Pills
        pill1 = ActionPill(tool_name="solve_spatial_layout", duration_str="0.4s", status="completed")
        self.app.call_from_thread(chat_scroll.mount, pill1)
        
        asyncio.run(asyncio.sleep(0.3))
        
        pill2 = ActionPill(tool_name="compile_world_sdf", duration_str="0.8s", status="completed")
        self.app.call_from_thread(chat_scroll.mount, pill2)
        
        asyncio.run(asyncio.sleep(0.3))

        # Step 3: Mount Markdown Response
        response_md = Markdown(
            f"""
* **Warehouse Area Defined:** 14x10m with non-intersecting outer boundary.
* **Corridor Safety Corridor:** Enforced $W_{{corridor}} \\ge 2.2\\text{{m}}$ and wall standoff $\\ge 0.5\\text{{m}}$.
* **Zero-SLAM Map Sliced:** Generated verified `map.yaml` and `map.pgm`.
* **Nav2 Simple Commander:** Dispatched mobile robot to target anchor pose.
* **Gazebo Status:** Physics running at Real-Time Factor 1.0x.
            """
        )
        self.app.call_from_thread(chat_scroll.mount, response_md)
        self.app.call_from_thread(chat_scroll.scroll_end)

        # Update telemetry sidebar live
        self.app.call_from_thread(
            sidebar.update_telemetry,
            sim_state="Active (GUI)",
            sim_time="00:12.6",
            rtf="1.0x",
            robot_x=3.85,
            robot_y=2.10,
            speed=0.35,
            clearance=0.48,
            tokens_used=1840,
        )
