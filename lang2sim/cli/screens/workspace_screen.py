"""
Screen 2: 80/20 Workspace Screen
File: lang2sim/cli/screens/workspace_screen.py
"""

import asyncio
from pathlib import Path
from typing import Any, List, Dict, Optional
from textual.screen import Screen
from textual.widgets import Static, Markdown
from textual.containers import Horizontal, Vertical, VerticalScroll
from textual.binding import Binding
from textual import work
from rich.markup import escape
from ..widgets.prompt_card import PromptCard
from ..widgets.telemetry_sidebar import TelemetrySidebar
from ...agent.nemotron_client import NemotronClient
from ...core.session_manager import session_manager, ChatMessage

SYSTEM_PROMPT = (
    "You are Sim2Lang Copilot, an expert AI robotics simulation assistant. "
    "You help roboticists design, configure, inspect, and reason about Gazebo Harmonic "
    "simulations, SDF scene layouts, Nav2 navigation stacks, and robot sensor telemetry. "
    "Be concise, technically precise, conversational, and direct. Format your answers with clean Markdown. "
    "When a system message carries Lang2Sim simulation context, base answers about the robot, the run and the "
    "NQI score on those measured numbers."
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
        self.sim_session = None
        self.workspace_name = workspace_name
        self._sim_reachable = False   # a control API answered the last telemetry poll
        self.current_spec = None

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
        self.demo = DemoRunner(say=self._demo_say, show=self._demo_show, progress=self._demo_progress)
        self._progress_line: Optional[Static] = None

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
        """Route a prompt: build/stop a simulation, act on the running one, or ask the LLM.

        Order: /sim and /stop (simulations this app starts) -> slash demo commands ->
        while a simulation is reachable: "stop the robot", stress requests (spawn/dim),
        "detour to rack 3" -> "build a warehouse with ..." starts a new simulation ->
        everything else (and every question) goes to the grounded LLM chat.
        """
        from ..demo_commands import (
            Command,
            is_goto,
            is_halt,
            is_sim_request,
            is_stop_sim,
            parse_command,
            parse_stress_intent,
        )

        cleaned_prompt = prompt.strip()
        if not cleaned_prompt:
            return
        lowered = cleaned_prompt.lower()

        if lowered.startswith("/sim") or (not self._sim_active() and is_sim_request(cleaned_prompt)):
            scenario = cleaned_prompt[4:].strip() if lowered.startswith("/sim") else cleaned_prompt
            widget = self._start_turn(cleaned_prompt, "● *Building your simulation...*")
            self.run_simulation(widget, scenario or "a warehouse with 4 racks and 1 crate")
            return
        if is_stop_sim(cleaned_prompt):
            self.stop_simulation(self._start_turn(cleaned_prompt, "● *Stopping...*"))
            return

        command = parse_command(cleaned_prompt)
        if command is None and self._sim_active():
            if is_halt(cleaned_prompt):
                command = Command("halt")
            else:
                command = parse_stress_intent(cleaned_prompt)
                if command is None and is_goto(cleaned_prompt):
                    command = Command("goto", opts={"text": cleaned_prompt})
        elif command is None:
            command = parse_stress_intent(cleaned_prompt)   # answers "no simulation running" clearly
        if command is not None:
            chat_scroll = self.query_one("#chat-scroll", VerticalScroll)
            chat_scroll.mount(Static(f"[bold #ffffff]{escape(cleaned_prompt)}[/]", classes="user-prompt-card"))
            chat_scroll.scroll_end(animate=False)
            self.session.messages.append(ChatMessage(role="user", content=cleaned_prompt,
                                                     tokens=max(1, len(cleaned_prompt) // 4)))
            session_manager.save_session(self.session)
            self.run_demo_command(command)
            return

        response_widget = self._start_turn(cleaned_prompt, "● *Thinking...*")
        self.conversation_history.append({"role": "user", "content": cleaned_prompt})
        self.stream_response(response_widget, cleaned_prompt)

    def _sim_active(self) -> bool:
        """A simulation this app started, or any control API answering (run_sim.py, replay)."""
        return self.sim_session is not None or self._sim_reachable

    def _start_turn(self, prompt: str, placeholder: str) -> Markdown:
        """Mount the user's prompt and an assistant reply widget; persist the user message."""
        chat_scroll = self.query_one("#chat-scroll", VerticalScroll)
        if self.session.title == "New Session":
            self.session.title = prompt[:40]
            self.query_one("#workspace-header", Static).update(f"S2L | {self.session.title} — lang2sim")
        chat_scroll.mount(Static(f"[bold #ffffff]{escape(prompt)}[/]", classes="user-prompt-card"))
        self.session.messages.append(ChatMessage(role="user", content=prompt, tokens=max(1, len(prompt) // 4)))
        session_manager.save_session(self.session)
        widget = Markdown(placeholder, classes="assistant-response")
        chat_scroll.mount(widget)
        chat_scroll.scroll_end(animate=False)
        return widget

    # ------------------------------------------------------------------ pitch wiring
    @work(exclusive=True, group="sim")
    async def run_simulation(self, widget: Markdown, prompt: str) -> None:
        """Prompt -> SimSpec (LLM or parser) -> Part 1 world + map -> Part 3 sim -> mission -> Part 4 NQI."""
        from ...pipeline import build_run, describe, launch_and_drive, make_session, prompt_to_spec

        self.is_streaming = True
        chat_scroll = self.query_one("#chat-scroll", VerticalScroll)
        sidebar = self.query_one(TelemetrySidebar)
        sticky_card = self.query_one("#sticky-input-card", PromptCard)
        lines: List[str] = ["**● Building your simulation**", ""]
        status: List[str] = [""]

        async def render() -> None:
            await widget.update("\n".join(lines + ([f"`{status[0]}`"] if status[0] else [])))
            chat_scroll.scroll_end(animate=False)

        async def add(line: str) -> None:
            lines.append(line)
            await render()

        def on_event(text: str) -> None:
            status[0] = text
            if text.startswith(("READY", "mission sent")) or ", 0." in text:
                lines.append(f"- {text}")
            asyncio.ensure_future(render())

        try:
            spec, source = await prompt_to_spec(prompt, client=self.client)
            self.current_spec = spec
            await add(f"- **1. Spec** ({source}): {describe(spec)}")
            run, steps = await asyncio.to_thread(build_run, spec)
            await add(f"- **2. World builder (Part 1):** {steps[0]}")
            await add(f"- **3. Files:** {steps[1]}")
            if self.sim_session is not None:
                await self.sim_session.stop()
                self.sim_session = None
            session, plan = make_session()
            self.sim_session = session
            await add(f"- **4. Runtime (Part 3):** {plan.describe()}, starting `{session.opts.image}`")
            sidebar.update_telemetry(sim_state="Launching")
            await session.start()
            await add(f"- {session.view_hint}")
            locked = self.demo.mission_lock.acquire(blocking=False)
            try:
                nav, report = await launch_and_drive(session, run, on_event=on_event)
            finally:
                if locked:
                    self.demo.mission_lock.release()
            status[0] = ""
            await add("")
            await add(f"**Mission {nav.status.upper()}**" + (f": {nav.message}" if nav.message else "") + "\n")
            if report and "nqi" in report:
                import json as _json
                self.demo.show_report(_json.loads(Path(report["report_path"]).read_text()))
            await add(f"\nSimulation is still running: `/mission stress` + say *\"drop a crate in front of the robot\"*, "
                      f"`/benchmark`, `/dashboard`, *\"go to rack 2\"*, *\"stop the robot\"*, or `/stop`. Logs: `{run / 'logs'}`")
        except Exception as e:  # noqa: BLE001 - show the failure in the chat, keep the app alive
            status[0] = ""
            await add(f"\n**⚠ Simulation error:** `{type(e).__name__}: {e}`")
        finally:
            self.is_streaming = False
            sticky_card.focus_input()

    @work(exclusive=True, group="sim")
    async def stop_simulation(self, widget: Markdown) -> None:
        if self.sim_session is None:
            await widget.update("No simulation is running.")
            return
        await widget.update("● Stopping simulation...")
        await self.sim_session.stop()
        self.sim_session = None
        self.query_one(TelemetrySidebar).update_telemetry(sim_state="Stopped")
        await widget.update("🛑 **Simulation and container stopped successfully.**")

    async def on_unmount(self) -> None:
        if getattr(self, "sim_session", None) is not None:
            await self.sim_session.stop()

    # --- demo commands ------------------------------------------------------------
    @work(exclusive=False, group="demo")
    async def run_demo_command(self, command) -> None:
        await self.demo.run(command)
        self.poll_telemetry()

    def _demo_say(self, markup: str) -> None:
        self._mount_chat(Static(markup, classes="demo-line"))

    def _demo_show(self, renderable) -> None:
        self._mount_chat(Static(renderable, classes="demo-table"))

    def _demo_progress(self, markup: Optional[str]) -> None:
        """One status line updated in place (None: leave it and start a new one next time)."""
        if markup is None:
            self._progress_line = None
            return
        if self._progress_line is None or not self._progress_line.is_attached:
            self._progress_line = Static(markup, classes="demo-line demo-progress")
            self._mount_chat(self._progress_line)
        else:
            self._progress_line.update(markup)

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
        self._sim_reachable = fields.get("sim_state") != "No simulation"
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
            # Ground the answer in the running simulation (live telemetry + latest NQI report).
            import asyncio
            from ...agent.grounding import fetch_context, with_context

            try:
                context = await asyncio.wait_for(asyncio.to_thread(fetch_context), timeout=3.0)
            except Exception:  # noqa: BLE001 - no sim context is fine
                context = None
            async for chunk in self.client.stream_chat(with_context(self.conversation_history, context)):
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
