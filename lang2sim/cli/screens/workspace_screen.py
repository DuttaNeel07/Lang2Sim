"""
Screen 2: 80/20 Workspace Screen
File: lang2sim/cli/screens/workspace_screen.py
"""

from typing import List, Dict, Any, Optional
from textual.screen import Screen
from textual.widgets import Static, Markdown
from textual.containers import Horizontal, Vertical, VerticalScroll
from textual.binding import Binding
from textual import work
from rich.markup import escape
from ..widgets.prompt_card import PromptCard
from ..widgets.telemetry_sidebar import TelemetrySidebar
from ...agent.nemotron_client import NemotronClient
from ...core.session_manager import session_manager, ChatSession, ChatMessage

SYSTEM_PROMPT = (
    "You are Sim2Lang Copilot, an expert AI robotics simulation assistant. "
    "You help roboticists design, configure, inspect, and reason about Gazebo Harmonic "
    "simulations, SDF scene layouts, Nav2 navigation stacks, and robot sensor telemetry. "
    "Be concise, technically precise, conversational, and direct. Format your answers with clean Markdown."
)


class WorkspaceScreen(Screen):
    """The 80/20 split workspace screen featuring conversational LLM execution and live telemetry."""

    BINDINGS = [
        Binding("ctrl+s", "open_sessions", "Sessions", show=False),
    ]

    def __init__(
        self,
        initial_prompt: str = "",
        session_id: Optional[str] = None,
        workspace_name: str = "active_project",
        **kwargs,
    ):
        super().__init__(**kwargs)
        self.initial_prompt = initial_prompt
        self.client = NemotronClient()
        self.workspace_name = workspace_name

        # Session initialization
        if session_id:
            loaded = session_manager.get_session(session_id)
            if loaded:
                self.session = loaded
            else:
                self.session = session_manager.create_session(
                    title=initial_prompt[:40] if initial_prompt else "New Session",
                    workspace_name=workspace_name,
                    system_prompt=SYSTEM_PROMPT,
                )
        else:
            self.session = session_manager.create_session(
                title=initial_prompt[:40] if initial_prompt else "New Session",
                workspace_name=workspace_name,
                system_prompt=SYSTEM_PROMPT,
            )

        self.conversation_history: List[Dict[str, str]] = [
            {"role": m.role, "content": m.content} for m in self.session.messages
        ]
        self.total_tokens_used: int = self.session.total_tokens
        self.is_streaming: bool = False
        from ..demo_commands import DemoRunner
        self.demo = DemoRunner(say=self._demo_say, show=self._demo_show)

    def compose(self):
        # Top title bar
        yield Static(f"S2L | {self.session.title} — lang2sim", id="workspace-header")

        # 80/20 Horizontal body
        with Horizontal(id="workspace-body"):
            # Left Pane: 80% Width
            with Vertical(id="left-pane"):
                with VerticalScroll(id="chat-scroll"):
                    yield Static("", id="scroll-anchor")
                yield PromptCard(
                    placeholder='lang2sim > Ask, or /mission · /stress · /benchmark · /report · /help',
                    card_id="sticky-prompt-input",
                    id="sticky-input-card",
                )

            # Right Sidebar: 20% Width
            yield TelemetrySidebar()

        # Bottom footer bar
        with Horizontal(id="workspace-footer"):
            yield Static("~", classes="footer-left")
            yield Static("", classes="footer-spacer")
            yield Static(
                f"[#ced6e0]{self.total_tokens_used:,} tokens (1%)[/]   "
                "[#57606f]ctrl+s sessions[/]   [#57606f]ctrl+p commands[/]   [#2ed573]●[/] [#57606f]lang2sim 2.0.0[/]",
                id="footer-status",
                classes="footer-right",
            )

    TELEMETRY_POLL_S = 1.0

    def on_mount(self) -> None:
        """Mount existing session messages, focus input, and process initial prompt."""
        self.set_interval(self.TELEMETRY_POLL_S, self.poll_telemetry)
        chat_scroll = self.query_one("#chat-scroll", VerticalScroll)

        # Restore past messages if loading an existing session
        for msg in self.session.messages:
            if msg.role == "user":
                user_card = Static(f"[bold #ffffff]{msg.content}[/]", classes="user-prompt-card")
                chat_scroll.mount(user_card)
            elif msg.role == "assistant":
                resp_md = Markdown(msg.content, classes="assistant-response")
                chat_scroll.mount(resp_md)

        sidebar = self.query_one(TelemetrySidebar)
        capacity_pct = min(100, max(1, self.total_tokens_used // 1000))
        sidebar.update_telemetry(
            tokens_used=self.total_tokens_used,
            context_capacity_pct=capacity_pct,
        )

        chat_scroll.scroll_end(animate=False)
        self.query_one("#sticky-input-card", PromptCard).focus_input()

        if self.initial_prompt:
            self.handle_user_prompt(self.initial_prompt)

    def action_open_sessions(self) -> None:
        """Open custom floating sessions modal."""
        from .sessions_modal import SessionsModal
        self.app.push_screen(
            SessionsModal(workspace_name=self.session.workspace_name),
            callback=self.on_session_switched,
        )

    def on_session_switched(self, session_id: Optional[str]) -> None:
        if session_id and session_id != self.session.id:
            self.load_session(session_id)

    def load_session(self, session_id: str) -> None:
        """Load and mount a selected session cleanly."""
        loaded = session_manager.get_session(session_id)
        if not loaded:
            return

        self.session = loaded
        self.conversation_history = [
            {"role": m.role, "content": m.content} for m in self.session.messages
        ]
        self.total_tokens_used = self.session.total_tokens

        chat_scroll = self.query_one("#chat-scroll", VerticalScroll)
        chat_scroll.remove_children()

        for msg in self.session.messages:
            if msg.role == "user":
                user_card = Static(f"[bold #ffffff]{msg.content}[/]", classes="user-prompt-card")
                chat_scroll.mount(user_card)
            elif msg.role == "assistant":
                resp_md = Markdown(msg.content, classes="assistant-response")
                chat_scroll.mount(resp_md)

        # Update header and sidebar
        self.query_one("#workspace-header", Static).update(
            f"S2L | {self.session.title} — lang2sim"
        )
        sidebar = self.query_one(TelemetrySidebar)
        capacity_pct = min(100, max(1, self.total_tokens_used // 1000))
        sidebar.update_telemetry(
            tokens_used=self.total_tokens_used,
            context_capacity_pct=capacity_pct,
        )

        footer_right = self.query_one("#footer-status", Static)
        if footer_right:
            footer_right.update(
                f"[#ced6e0]{self.total_tokens_used:,} tokens ({capacity_pct}%)[/]   "
                "[#57606f]ctrl+s sessions[/]   [#57606f]ctrl+p commands[/]   [#2ed573]●[/] [#57606f]lang2sim 2.0.0[/]"
            )

        chat_scroll.scroll_end(animate=False)
        self.query_one("#sticky-input-card", PromptCard).focus_input()

    def on_prompt_card_submitted(self, message: PromptCard.Submitted) -> None:
        """When user types in sticky input card, add prompt and run execution."""
        self.handle_user_prompt(message.value)

    def handle_user_prompt(self, prompt: str) -> None:
        """Mount user message and launch async LLM streaming worker."""
        cleaned_prompt = prompt.strip()
        if not cleaned_prompt:
            return

        # Demo commands (/mission, /stress, /benchmark, ...) and plain-language stress requests act on
        # the running simulation directly; they never wait for (or depend on) the LLM.
        from ..demo_commands import parse_command, parse_stress_intent
        command = parse_command(cleaned_prompt) or parse_stress_intent(cleaned_prompt)
        if command is not None:
            chat_scroll = self.query_one("#chat-scroll", VerticalScroll)
            chat_scroll.mount(Static(f"[bold #ffffff]{escape(cleaned_prompt)}[/]", classes="user-prompt-card"))
            chat_scroll.scroll_end(animate=False)
            self.session.messages.append(ChatMessage(role="user", content=cleaned_prompt,
                                                     tokens=max(1, len(cleaned_prompt) // 4)))
            session_manager.save_session(self.session)
            self.run_demo_command(command)
            return
        if self.is_streaming:
            return

        chat_scroll = self.query_one("#chat-scroll", VerticalScroll)

        # 1. Auto-name session if it is still "New Session"
        if self.session.title == "New Session":
            self.session.title = cleaned_prompt[:40]
            self.query_one("#workspace-header", Static).update(
                f"S2L | {self.session.title} — lang2sim"
            )

        # 2. Mount user prompt card
        user_card = Static(f"[bold #ffffff]{cleaned_prompt}[/]", classes="user-prompt-card")
        chat_scroll.mount(user_card)
        chat_scroll.scroll_end(animate=False)

        prompt_tokens = max(1, len(cleaned_prompt) // 4)
        self.session.messages.append(
            ChatMessage(role="user", content=cleaned_prompt, tokens=prompt_tokens)
        )
        self.conversation_history.append({"role": "user", "content": cleaned_prompt})
        session_manager.save_session(self.session)

        # 3. Mount streaming assistant Markdown widget
        response_widget = Markdown("● *Thinking...*", classes="assistant-response")
        chat_scroll.mount(response_widget)
        chat_scroll.scroll_end(animate=False)

        # 4. Stream response asynchronously with low latency
        self.stream_response(response_widget, cleaned_prompt)

    # --- demo commands ------------------------------------------------------------
    @work(exclusive=False, group="demo")
    async def run_demo_command(self, command) -> None:
        await self.demo.run(command)
        self.poll_telemetry()

    def _demo_say(self, markup: str) -> None:
        self._mount_chat(Static(markup, classes="demo-line"))

    def _demo_show(self, renderable) -> None:
        self._mount_chat(Static(renderable, classes="demo-table"))

    def _mount_chat(self, widget) -> None:
        try:
            chat_scroll = self.query_one("#chat-scroll", VerticalScroll)
        except Exception:  # noqa: BLE001 - screen gone
            return
        chat_scroll.mount(widget)
        chat_scroll.scroll_end(animate=False)

    @work(thread=True, exclusive=True, group="telemetry")
    def poll_telemetry(self) -> None:
        """Fetch live telemetry off the UI thread (GET /v1/telemetry/live) and refresh the sidebar."""
        from ...telemetry.live import fetch_live, sidebar_fields

        fields = sidebar_fields(fetch_live(timeout=0.5))
        try:
            self.app.call_from_thread(self._apply_telemetry, fields)
        except Exception:  # noqa: BLE001 - app shutting down
            pass

    def _apply_telemetry(self, fields: Dict[str, Any]) -> None:
        try:
            self.query_one(TelemetrySidebar).update_telemetry(**fields)
        except Exception:  # noqa: BLE001 - screen not mounted any more
            pass

    @work(exclusive=True)
    async def stream_response(self, response_widget: Markdown, prompt: str) -> None:
        """Stream conversational response directly from LLM with low latency."""
        self.is_streaming = True
        chat_scroll = self.query_one("#chat-scroll", VerticalScroll)
        sidebar = self.query_one(TelemetrySidebar)
        sticky_card = self.query_one("#sticky-input-card", PromptCard)

        if not self.client.is_configured():
            await response_widget.update(
                "**● API Key Required**\n\n"
                "Please configure your API key to chat with Sim2Lang:\n"
                "- Run: `export NVIDIA_API_KEY='nvapi-...'`\n"
                "- Or set `OPENAI_API_KEY='sk-...'`\n"
                "- Or add `NVIDIA_API_KEY=...` to `.env` in the project root."
            )
            self.is_streaming = False
            sticky_card.focus_input()
            return

        chunks: List[str] = []
        try:
            async for chunk in self.client.stream_chat(self.conversation_history):
                chunks.append(chunk)
                await response_widget.update("".join(chunks))
                chat_scroll.scroll_end(animate=False)

            full_text = "".join(chunks).strip()
            if full_text:
                self.conversation_history.append({"role": "assistant", "content": full_text})
                prompt_tokens = max(1, len(prompt) // 4)
                resp_tokens = max(1, len(full_text) // 4)
                self.total_tokens_used += prompt_tokens + resp_tokens
                capacity_pct = min(100, max(1, self.total_tokens_used // 1000))

                self.session.messages.append(
                    ChatMessage(role="assistant", content=full_text, tokens=resp_tokens)
                )
                self.session.total_tokens = self.total_tokens_used
                session_manager.save_session(self.session)

                sidebar.update_telemetry(
                    tokens_used=self.total_tokens_used,
                    context_capacity_pct=capacity_pct,
                )

                footer_right = self.query_one("#footer-status", Static)
                if footer_right:
                    footer_right.update(
                        f"[#ced6e0]{self.total_tokens_used:,} tokens ({capacity_pct}%)[/]   "
                        "[#57606f]ctrl+s sessions[/]   [#57606f]ctrl+p commands[/]   [#2ed573]●[/] [#57606f]lang2sim 2.0.0[/]"
                    )
            else:
                await response_widget.update("*(No response received from model)*")
        except Exception as e:
            await response_widget.update(f"[bold #ef4444]⚠ Connection Error:[/] {str(e)}")
        finally:
            self.is_streaming = False
            sticky_card.focus_input()
