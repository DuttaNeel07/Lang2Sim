"""
Screen 2: 80/20 Workspace Screen
File: lang2sim/cli/screens/workspace_screen.py
"""

import asyncio
import json
from typing import List, Dict, Any, Optional
from textual.screen import Screen
from textual.widgets import Static, Markdown
from textual.containers import Horizontal, Vertical, VerticalScroll
from textual.binding import Binding
from textual import work, events
from ..widgets.prompt_card import PromptCard
from ..widgets.telemetry_sidebar import TelemetrySidebar
from ..widgets.thought_box import ThoughtBox
from ..widgets.action_pill import ActionPill
from ..widgets.command_card import CommandCard
from ..widgets.plan_card import PlanCard
from ...agent.nemotron_client import NemotronClient
from ...agent.react_loop import (
    SUPERVISOR_SYSTEM_PROMPT,
    ReActSupervisor,
    ThoughtChunk,
    ThoughtCompleted,
    PlanCreated,
    PlanStepUpdated,
    PermissionRequested,
    CommandExecuted,
    ActionStarted,
    ActionCompleted,
    ContentChunk,
    TurnCompleted,
)
from ...core.session_manager import session_manager, ChatSession, ChatMessage

SYSTEM_PROMPT = SUPERVISOR_SYSTEM_PROMPT


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
        self.always_allow_commands: bool = False
        self._pending_permissions: Dict[str, asyncio.Future] = {}
        self._active_command_cards: Dict[str, CommandCard] = {}
        self._active_plan_card: Optional[PlanCard] = None
        self._selected_text: Optional[str] = None

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

        self.conversation_history: List[Dict[str, str]] = []
        for m in self.session.messages:
            if m.role == "system":
                self.conversation_history.append({"role": "system", "content": m.content})
            elif m.role == "user":
                self.conversation_history.append({"role": "user", "content": m.content})
            elif m.role == "assistant":
                self.conversation_history.append({"role": "assistant", "content": m.content})
        self.total_tokens_used: int = self.session.total_tokens
        self.is_streaming: bool = False


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
                    placeholder='lang2sim > Type follow-up (e.g. "drop light to 20%, spawn crate")',
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

    def on_mount(self) -> None:
        """Mount existing session messages, focus input, and process initial prompt."""
        chat_scroll = self.query_one("#chat-scroll", VerticalScroll)

        # Restore past messages if loading an existing session
        for msg in self.session.messages:
            if msg.role == "user":
                user_card = Static(f"[bold #ffffff]{msg.content}[/]", classes="user-prompt-card")
                chat_scroll.mount(user_card)
            elif msg.role == "assistant":
                if msg.thought:
                    dur = msg.thought_duration_str or "1.2s"
                    thought_box = ThoughtBox(duration_str=dur, content=msg.thought)
                    chat_scroll.mount(thought_box)
                if msg.actions:
                    for act in msg.actions:
                        if act.tool_name == "run_command" or (act.arguments and "command" in act.arguments):
                            cmd = act.arguments.get("command", "") if act.arguments else ""
                            out = act.output.get("stdout", "") if isinstance(act.output, dict) else ""
                            err = act.output.get("stderr", "") if isinstance(act.output, dict) else ""
                            rc = act.output.get("exit_code", 0) if isinstance(act.output, dict) else 0
                            card = CommandCard(
                                event_id="restored",
                                command=cmd,
                                tool_name=act.tool_name,
                                arguments=act.arguments,
                                permission_required=False,
                            )
                            chat_scroll.mount(card)
                            card.set_result(exit_code=rc, duration_str=act.duration_str, stdout=out, stderr=err)
                        else:
                            pill = ActionPill(
                                tool_name=act.tool_name,
                                duration_str=act.duration_str,
                                status=act.status,
                            )
                            chat_scroll.mount(pill)
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

        # Start periodic telemetry polling from active control server
        self.set_interval(1.5, self.poll_live_telemetry)

        if self.initial_prompt:
            self.handle_user_prompt(self.initial_prompt)

    @work(exclusive=True, group="telemetry")
    async def poll_live_telemetry(self) -> None:
        """Polls live Gazebo & ROS 2 telemetry from ControlClient or local container in real time."""
        # 1. Try ControlClient
        try:
            from lang2sim.runtime.control_api.client import ControlClient
            async with ControlClient(timeout=0.3) as client:
                status = await client.sim_status()
                if status.status in ("running", "launching"):
                    telem = await client.telemetry()
                    pose = telem.robot_pose
                    vel = telem.robot_velocity
                    rtf_str = f"{telem.real_time_factor:.2f}x" if telem.real_time_factor is not None else "1.0x"
                    sidebar = self.query_one(TelemetrySidebar)
                    sidebar.update_telemetry(
                        sim_state=f"Active ({status.status.capitalize()})",
                        sim_time=f"{int(telem.sim_time // 60):02d}:{int(telem.sim_time % 60):02d}",
                        rtf=rtf_str,
                        robot_x=pose.x if pose else 0.0,
                        robot_y=pose.y if pose else 0.0,
                        speed=vel.x if vel else 0.0,
                        collision_count=0,
                    )
                    return
        except Exception:
            pass

        # 2. Inspect Docker container lang2sim-sim or local ROS 2 processes
        try:
            import subprocess
            from datetime import datetime, timezone

            inspect_proc = subprocess.run(
                "docker inspect lang2sim-sim --format '{{json .State}}'",
                shell=True,
                capture_output=True,
                text=True,
                timeout=1,
            )
            sidebar = self.query_one(TelemetrySidebar)
            if inspect_proc.returncode == 0 and inspect_proc.stdout.strip():
                state_data = json.loads(inspect_proc.stdout)
                is_running = state_data.get("Running", False)
                if is_running:
                    started_at = state_data.get("StartedAt", "")
                    sim_time_str = "00:00.0"
                    if started_at:
                        try:
                            clean_ts = started_at[:26] + "Z" if "." in started_at else started_at
                            start_dt = datetime.fromisoformat(clean_ts.replace("Z", "+00:00"))
                            elapsed = (datetime.now(timezone.utc) - start_dt).total_seconds()
                            mins = int(elapsed // 60)
                            secs = elapsed % 60
                            sim_time_str = f"{mins:02d}:{secs:04.1f}"
                        except Exception:
                            pass

                    # Check if gazebo or rviz2 is running in container
                    ps_proc = subprocess.run(
                        "docker exec lang2sim-sim ps aux",
                        shell=True,
                        capture_output=True,
                        text=True,
                        timeout=1,
                    )
                    out = ps_proc.stdout.lower()
                    has_gui = "rviz2" in out or "gz sim" in out or "gz-sim" in out or "ign gazebo" in out
                    sim_state = "Active (GUI)" if has_gui else "Active (Docker)"
                    rtf_str = "1.0x" if has_gui else "0.0x"

                    sidebar.update_telemetry(
                        sim_state=sim_state,
                        sim_time=sim_time_str,
                        rtf=rtf_str,
                    )
                else:
                    sidebar.update_telemetry(
                        sim_state="Idle (Offline)",
                        sim_time="00:00.0",
                        rtf="0.0x",
                    )
            else:
                sidebar.update_telemetry(
                    sim_state="Idle (Offline)",
                    sim_time="00:00.0",
                    rtf="0.0x",
                )
        except Exception:
            pass

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
                if msg.thought:
                    dur = msg.thought_duration_str or "1.2s"
                    thought_box = ThoughtBox(duration_str=dur, content=msg.thought)
                    chat_scroll.mount(thought_box)
                if msg.actions:
                    for act in msg.actions:
                        if act.tool_name == "run_command" or (act.arguments and "command" in act.arguments):
                            cmd = act.arguments.get("command", "") if act.arguments else ""
                            out = act.output.get("stdout", "") if isinstance(act.output, dict) else ""
                            err = act.output.get("stderr", "") if isinstance(act.output, dict) else ""
                            rc = act.output.get("exit_code", 0) if isinstance(act.output, dict) else 0
                            card = CommandCard(
                                event_id="restored",
                                command=cmd,
                                tool_name=act.tool_name,
                                arguments=act.arguments,
                                permission_required=False,
                            )
                            chat_scroll.mount(card)
                            card.set_result(exit_code=rc, duration_str=act.duration_str, stdout=out, stderr=err)
                        else:
                            pill = ActionPill(
                                tool_name=act.tool_name,
                                duration_str=act.duration_str,
                                status=act.status,
                            )
                            chat_scroll.mount(pill)
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

    def _resolve_first_pending_permission(self, approved: bool, always: bool = False) -> bool:
        """Resolves the oldest pending command permission card."""
        if not self._pending_permissions:
            return False
        if always:
            self.always_allow_commands = True
        event_id, fut = next(iter(self._pending_permissions.items()))
        self._pending_permissions.pop(event_id, None)
        card = self._active_command_cards.get(event_id)
        if card:
            if approved:
                card.set_running()
            else:
                card.status = "rejected"
                card._update_classes()
                card._remove_actions_row()
                try:
                    card.query_one("#cmd-badge", Static).update(card._format_badge())
                except Exception:
                    pass
        if fut and not fut.done():
            fut.set_result(approved)
        return True

    def get_conversation_as_text(self) -> str:
        """Assembles entire active chat into clean plain text for copying."""
        lines = []
        for msg in self.session.messages:
            role_label = "USER" if msg.role == "user" else "ASSISTANT"
            lines.append(f"[{role_label}]:\n{msg.content}\n")
            if msg.thought:
                lines.append(f"[THOUGHT]:\n{msg.thought}\n")
            if msg.actions:
                for act in msg.actions:
                    lines.append(f"[TOOL: {act.tool_name}]: {act.duration_str} - status: {act.status}")
        return "\n".join(lines).strip()

    def action_select_all(self) -> None:
        """Select all text in current input or select entire conversation for copying."""
        try:
            from textual.widgets import Input
            inp = self.query_one("#sticky-prompt-input", Input)
            if inp.has_focus and inp.value:
                inp.action_select_all()
                self._selected_text = inp.value
                self.app.notify("Selected input text (Ctrl+Shift+C to copy)", timeout=1.5)
                return
        except Exception:
            pass

        full_text = self.get_conversation_as_text()
        self._selected_text = full_text
        self.app.notify("Selected all chat text! Press Ctrl+Shift+C to copy", title="Select All", timeout=2.0)

    def on_key(self, event: events.Key) -> None:
        """Global key interceptor for immediate command approval, Ctrl+A select all, and copy."""
        key = event.key.lower()

        # 1. Ctrl+A: Select All
        if key == "ctrl+a":
            event.prevent_default()
            event.stop()
            self.action_select_all()
            return

        # 2. Ctrl+Shift+C / Shift+Ctrl+C: Copy to clipboard
        if key in ("ctrl+shift+c", "shift+ctrl+c"):
            event.prevent_default()
            event.stop()
            self.app.action_copy_selection()
            return

        # 3. Pending permission card hotkeys
        if self._pending_permissions:
            try:
                from textual.widgets import Input
                inp = self.query_one("#sticky-prompt-input", Input)
                if not inp.value.strip():
                    if key in ("y", "enter"):
                        event.prevent_default()
                        event.stop()
                        self._resolve_first_pending_permission(approved=True, always=False)
                        return
                    elif key == "a":
                        event.prevent_default()
                        event.stop()
                        self._resolve_first_pending_permission(approved=True, always=True)
                        return
                    elif key in ("n", "escape"):
                        event.prevent_default()
                        event.stop()
                        self._resolve_first_pending_permission(approved=False, always=False)
                        return
            except Exception:
                pass

    def on_prompt_card_submitted(self, message: PromptCard.Submitted) -> None:
        """When user types in sticky input card, add prompt and run execution."""
        self.handle_user_prompt(message.value)

    def handle_user_prompt(self, prompt: str) -> None:
        """Mount user message and launch async LLM streaming worker."""
        cleaned_prompt = prompt.strip()
        if not cleaned_prompt:
            return

        # 1. Natural language permission approval routing
        if self._pending_permissions:
            lowered = cleaned_prompt.lower()
            if lowered in ("y", "yes", "run", "approve", "ok", "proceed", "go"):
                self._resolve_first_pending_permission(approved=True, always=False)
                return
            elif lowered in ("a", "always", "always allow", "always_allow"):
                self._resolve_first_pending_permission(approved=True, always=True)
                return
            elif lowered in ("n", "no", "reject", "cancel", "stop", "abort"):
                self._resolve_first_pending_permission(approved=False, always=False)
                return

        # 2. Recovery for "try again" / "retry" / "restart"
        if cleaned_prompt.lower() in ("try again", "retry", "restart"):
            for event_id, fut in list(self._pending_permissions.items()):
                if not fut.done():
                    fut.cancel()
            self._pending_permissions.clear()
            self.is_streaming = False

        if self.is_streaming:
            return

        chat_scroll = self.query_one("#chat-scroll", VerticalScroll)

        # Auto-name session if it is still "New Session"
        if self.session.title == "New Session":
            self.session.title = cleaned_prompt[:40]
            self.query_one("#workspace-header", Static).update(
                f"S2L | {self.session.title} — lang2sim"
            )

        # Mount user prompt card
        user_card = Static(f"[bold #ffffff]{cleaned_prompt}[/]", classes="user-prompt-card")
        chat_scroll.mount(user_card)
        chat_scroll.scroll_end(animate=False)

        prompt_tokens = max(1, len(cleaned_prompt) // 4)
        self.session.messages.append(
            ChatMessage(role="user", content=cleaned_prompt, tokens=prompt_tokens)
        )
        self.conversation_history.append({"role": "user", "content": cleaned_prompt})
        session_manager.save_session(self.session)

        # Mount streaming assistant Markdown widget
        response_widget = Markdown("● *Thinking...*", classes="assistant-response")
        chat_scroll.mount(response_widget)
        chat_scroll.scroll_end(animate=False)

        # Stream response asynchronously with low latency
        self.stream_response(response_widget, cleaned_prompt)

    def on_command_card_approved(self, message: CommandCard.Approved) -> None:
        """User approved command execution."""
        if message.always_allow:
            self.always_allow_commands = True
        fut = self._pending_permissions.pop(message.event_id, None)
        if fut and not fut.done():
            fut.set_result(True)

    def on_command_card_rejected(self, message: CommandCard.Rejected) -> None:
        """User rejected command execution."""
        fut = self._pending_permissions.pop(message.event_id, None)
        if fut and not fut.done():
            fut.set_result(False)

    @work(exclusive=True, group="chat_stream")
    async def stream_response(self, response_widget: Markdown, prompt: str) -> None:
        """Stream conversational response directly from LLM / ReAct supervisor with low latency."""
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

        supervisor = ReActSupervisor(client=self.client, workspace_name=self.workspace_name)
        thought_box: Optional[ThoughtBox] = None
        thought_text_parts: List[str] = []
        content_parts: List[str] = []
        active_pills: Dict[str, ActionPill] = {}

        async def permission_checker(tool_name: str, cmd_str: str, args: dict, event_id: str) -> bool:
            if self.always_allow_commands:
                card = CommandCard(
                    event_id=event_id,
                    command=cmd_str,
                    tool_name=tool_name,
                    arguments=args,
                    permission_required=False,
                )
                self._active_command_cards[event_id] = card
                await chat_scroll.mount(card, before=response_widget)
                chat_scroll.scroll_end(animate=False)
                return True

            card = CommandCard(
                event_id=event_id,
                command=cmd_str,
                tool_name=tool_name,
                arguments=args,
                permission_required=True,
            )
            self._active_command_cards[event_id] = card
            await chat_scroll.mount(card, before=response_widget)
            chat_scroll.scroll_end(animate=False)

            fut = asyncio.get_event_loop().create_future()
            self._pending_permissions[event_id] = fut
            approved = await fut
            return approved

        try:
            async for event in supervisor.stream_turn(
                self.session,
                prompt,
                self.conversation_history,
                permission_checker=permission_checker,
            ):
                if isinstance(event, ThoughtChunk):
                    if not thought_box:
                        thought_box = ThoughtBox(duration_str="...", content="")
                        await chat_scroll.mount(thought_box, before=response_widget)
                        chat_scroll.scroll_end(animate=False)
                    thought_text_parts.append(event.chunk)
                    thought_box.update_content("".join(thought_text_parts))

                elif isinstance(event, ThoughtCompleted):
                    if not thought_box:
                        thought_box = ThoughtBox(duration_str=event.duration_str, content=event.thought)
                        await chat_scroll.mount(thought_box, before=response_widget)
                    else:
                        thought_box.update_content(event.thought, duration_str=event.duration_str)
                    chat_scroll.scroll_end(animate=False)

                elif isinstance(event, PlanCreated):
                    self._active_plan_card = PlanCard(steps=event.steps, title=event.title)
                    await chat_scroll.mount(self._active_plan_card, before=response_widget)
                    chat_scroll.scroll_end(animate=False)

                elif isinstance(event, PlanStepUpdated):
                    if self._active_plan_card:
                        self._active_plan_card.update_step(event.step_index, event.status, event.detail)
                    chat_scroll.scroll_end(animate=False)

                elif isinstance(event, PermissionRequested):
                    chat_scroll.scroll_end(animate=False)

                elif isinstance(event, CommandExecuted):
                    card = self._active_command_cards.get(event.event_id)
                    if card:
                        card.set_result(
                            exit_code=event.exit_code,
                            duration_str=event.duration_str,
                            stdout=event.stdout,
                            stderr=event.stderr,
                        )
                    chat_scroll.scroll_end(animate=False)

                elif isinstance(event, ActionStarted):
                    if event.tool_name != "run_command" and self._active_plan_card is None:
                        pill = ActionPill(tool_name=event.tool_name, duration_str="0.0s", status="running")
                        active_pills[event.tool_name] = pill
                        await chat_scroll.mount(pill, before=response_widget)
                        chat_scroll.scroll_end(animate=False)

                elif isinstance(event, ActionCompleted):
                    pill = active_pills.get(event.tool_name)
                    if pill:
                        pill.set_completed(event.duration_str)
                    elif event.tool_name != "run_command" and self._active_plan_card is None:
                        pill = ActionPill(tool_name=event.tool_name, duration_str=event.duration_str, status=event.status)
                        await chat_scroll.mount(pill, before=response_widget)

                    # Dynamically sync live telemetry metrics into sidebar
                    if event.tool_name == "query_robot_telemetry" and isinstance(event.output, dict):
                        sidebar = self.query_one(TelemetrySidebar)
                        sidebar.update_telemetry(
                            robot_x=float(event.output.get("x", 0.0)),
                            robot_y=float(event.output.get("y", 0.0)),
                            speed=float(event.output.get("speed", 0.0)),
                            clearance=float(event.output.get("clearance", 0.0)),
                            bumper_force=float(event.output.get("bumper_force_newtons", 0.0)),
                            collision_count=int(event.output.get("collision_count", 0)),
                        )
                    chat_scroll.scroll_end(animate=False)

                elif isinstance(event, ContentChunk):
                    content_parts.append(event.chunk)
                    await response_widget.update("".join(content_parts))
                    chat_scroll.scroll_end(animate=False)

                elif isinstance(event, TurnCompleted):
                    full_text = event.full_content
                    if not full_text:
                        full_text = "".join(content_parts).strip()
                    if full_text:
                        await response_widget.update(full_text)
                        history_content = full_text
                        self.conversation_history.append({"role": "assistant", "content": full_text})
                        prompt_tokens = max(1, len(prompt) // 4)
                        resp_tokens = max(1, len(full_text) // 4)
                        self.total_tokens_used += prompt_tokens + resp_tokens
                        capacity_pct = min(100, max(1, self.total_tokens_used // 1000))

                        self.session.messages.append(
                            ChatMessage(
                                role="assistant",
                                content=full_text,
                                thought=event.thought,
                                thought_duration_str=event.thought_duration_str,
                                actions=event.actions,
                                tokens=resp_tokens,
                            )
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
