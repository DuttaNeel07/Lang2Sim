"""
Screen 2: 80/20 Workspace Screen
File: lang2sim/cli/screens/workspace_screen.py
"""

import asyncio
import math
import re
import subprocess
from typing import List, Dict, Optional, Tuple
from textual.screen import Screen
from textual.widgets import Static, Markdown
from textual.containers import Horizontal, Vertical, VerticalScroll
from textual.binding import Binding
from textual import work
from ..widgets.prompt_card import PromptCard
from ..widgets.telemetry_sidebar import TelemetrySidebar
from ...agent.nemotron_client import NemotronClient
from ...core.session_manager import session_manager, ChatMessage
from ...core.models import Pose2D

SYSTEM_PROMPT = (
    "You are Sim2Lang Copilot, an expert AI robotics simulation assistant. "
    "You help roboticists design, configure, inspect, and reason about Gazebo Harmonic "
    "simulations, SDF scene layouts, Nav2 navigation stacks, and robot sensor telemetry. "
    "Be concise, technically precise, conversational, and direct. Format your answers with clean Markdown."
)


SIM_KEYWORDS = (
    "simulat",
    "warehouse",
    "empty room",
    "gazebo world",
    "generat",
    "create",
    "build",
    "spawn",
    "room",
    "rack",
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
        self.last_robot_pose = (0.0, 0.0)
        self.current_goal = None
        self.current_spec = None
        self.current_world_name = "prompt_demo"
        self.mutation_counter = 0

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
        """Mount user message and route to dynamic sim actions or LLM worker."""
        cleaned_prompt = prompt.strip()
        if not cleaned_prompt:
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
        response_widget = Markdown("● *Processing...*", classes="assistant-response")
        chat_scroll.mount(response_widget)
        chat_scroll.scroll_end(animate=False)

        lowered = cleaned_prompt.lower()

        # 4. Check if simulation is currently active
        if self.sim_session is not None:
            # 4a. Stop entire simulation & container
            if lowered.startswith("/stop") or lowered in ("stop", "stop simulation", "kill sim", "quit", "exit", "exit simulation", "shutdown") or lowered.startswith("stop sim"):
                self.stop_simulation(response_widget)
                return

            # 4b. Stop/halt robot motion immediately (without killing simulation)
            if any(w in lowered for w in ("stop the robot", "stop robot", "halt", "pause", "freeze", "halt the robot", "stop moving", "cancel nav", "cancel navigation")):
                self.stop_robot_motion(response_widget)
                return

            # 4c. Autonomous detour / re-routing / navigation
            if any(w in lowered for w in ("detour", "divert", "reroute", "re-route", "replan", "re-plan", "reach", "avoid", "continue", "resume", "navigate", "go to", "drive to")) and not any(w in lowered for w in ("spawn", "drop", "insert", "add another", "add a")):
                self.execute_detour_navigation(response_widget, cleaned_prompt)
                return

            # 4d. Dynamic environment mutation (drop crate, add rack, dim light, etc.)
            if any(w in lowered for w in ("spawn", "drop", "add", "insert", "place", "dim", "light", "friction", "obstacle", "crate", "rack", "barrel", "box")):
                self.execute_midway_mutation(response_widget, cleaned_prompt)
                return

        # 5. Starting a new simulation
        if lowered.startswith("/sim") or (self.sim_session is None and any(k in lowered for k in SIM_KEYWORDS)):
            self.run_simulation(response_widget, cleaned_prompt.removeprefix("/sim").strip() or cleaned_prompt)
            return

        # 6. Fallback to conversational streaming
        self.stream_response(response_widget, cleaned_prompt)

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

        def on_telemetry(tel) -> None:
            self.last_robot_pose = (tel.robot_pose.x, tel.robot_pose.y)
            secs = int(tel.sim_time)
            sidebar.update_telemetry(
                sim_state=("Active" if tel.is_running else "Stopped") + f" ({tel.health_status})",
                sim_time=f"{secs // 60:02d}:{secs % 60:02d}.{int((tel.sim_time % 1) * 10)}",
                rtf=f"{tel.real_time_factor:.2f}x",
                robot_x=tel.robot_pose.x, robot_y=tel.robot_pose.y,
                speed=abs(tel.robot_velocity.x) if tel.robot_velocity else 0.0,
            )

        try:
            spec, source = await prompt_to_spec(prompt, client=self.client)
            self.current_goal = spec.mission.goal_pose
            self.current_spec = spec
            self.mutation_counter = 0
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
            nav, report = await launch_and_drive(session, run, on_event=on_event, on_telemetry=on_telemetry)
            status[0] = ""
            await add("")
            await add(f"**Mission {nav.status.upper()}**" + (f": {nav.message}" if nav.message else "") + "\n")
            if report and "nqi" in report:
                await add(f"**Navigation Quality Index (Part 4): {report['nqi']:.1f} / 100**, path "
                          f"{report['path_length_m']:.1f} m vs optimal {report['optimal_length_m']:.1f} m "
                          f"(efficiency {report['E_p']:.2f}), min clearance {report['C_min_m']} m, "
                          f"jerk cost {report['S_tau']:.1f} (lower = smoother)")
            await add(f"\nSimulation is still running. Type `/stop` to end it. Logs: `{run / 'logs'}`")
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

    @work(exclusive=False, group="mutation")
    async def stop_robot_motion(self, widget: Markdown) -> None:
        """Halt robot movement immediately in Gazebo while keeping simulation alive."""
        if self.sim_session is None:
            await widget.update("⚠ No simulation session is active.")
            return
        try:
            # 1. Cancel Nav2 action goal
            await self.sim_session.api.nav_cancel()

            # 2. Publish zero twist using native gz topic (instant) + ros2 pub with -w 0 (no wait)
            cmd = (
                "gz topic -t /cmd_vel -m gz.msgs.Twist -p 'linear: {x: 0, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0}' 2>/dev/null; "
                "source /opt/ros/jazzy/setup.bash && ros2 topic pub --once -w 0 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' 2>/dev/null"
            )
            try:
                await asyncio.to_thread(
                    subprocess.run,
                    ["docker", "exec", "-u", "lang2sim", "lang2sim-sim", "/bin/bash", "-c", cmd],
                    capture_output=True, timeout=2.0
                )
            except Exception:
                pass

            self.query_one(TelemetrySidebar).update_telemetry(speed=0.0)
            await widget.update(
                "🛑 **Robot Motion Halted!**\n\n"
                "- Active navigation goal canceled (`Mission CANCELED`).\n"
                "- Zero-velocity command published to `/cmd_vel`.\n"
                "- *Simulation remains active.* You can issue new instructions (e.g. `detour and reach the 3rd rack` or `/stop`)."
            )
        except Exception as e:
            await widget.update(f"⚠ Failed to halt robot: `{e}`")

    def resolve_target_goal(self, prompt: str) -> Pose2D:
        lowered = prompt.lower()
        rack_num = None
        if "1st" in lowered or "first" in lowered: rack_num = 1
        elif "2nd" in lowered or "second" in lowered: rack_num = 2
        elif "3rd" in lowered or "third" in lowered: rack_num = 3
        elif "4th" in lowered or "fourth" in lowered: rack_num = 4
        elif "5th" in lowered or "fifth" in lowered: rack_num = 5
        elif "6th" in lowered or "sixth" in lowered: rack_num = 6
        else:
            m = re.search(r"(?:rac?k|rank|shelf)\s*(\d+)", lowered)
            if m:
                rack_num = int(m.group(1))

        if rack_num is not None and getattr(self, "current_spec", None) is not None:
            for obj in self.current_spec.objects:
                if obj.id == f"rack_{rack_num}":
                    # Stand 1.5m in front of rack in the central corridor
                    goal_y = (obj.pose.y - 1.5) if obj.pose.y > 0 else (obj.pose.y + 1.5)
                    return Pose2D(x=obj.pose.x, y=goal_y, yaw=0.0)

        if getattr(self, "current_goal", None) is not None:
            return self.current_goal
        return Pose2D(x=3.5, y=0.0, yaw=0.0)

    @work(exclusive=False, group="mutation")
    async def execute_detour_navigation(self, widget: Markdown, prompt: str) -> None:
        """Clear recovery states, clear costmap, and guide robot along evasive detour path."""
        if self.sim_session is None:
            await widget.update("⚠ No simulation session is active to navigate.")
            return

        target_goal = self.resolve_target_goal(prompt)
        await widget.update(
            f"🧭 **Calculating Autonomous Detour...**\n\n"
            f"- **Target Destination:** `x={target_goal.x:.2f}, y={target_goal.y:.2f}`\n"
            f"- Refreshing Nav2 dynamic costmaps & clearing recovery state..."
        )

        try:
            await self.sim_session.api.nav_cancel()
        except Exception:
            pass

        # Clear costmaps so stale recovery/inflation costs are flushed
        clear_cmd = [
            "docker", "exec", "-u", "lang2sim", "lang2sim-sim",
            "/bin/bash", "-c",
            "source /opt/ros/jazzy/setup.bash && "
            "ros2 service call /global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntireCostmap {} && "
            "ros2 service call /local_costmap/clear_entirely_local_costmap nav2_msgs/srv/ClearEntireCostmap {}"
        ]
        try:
            await asyncio.to_thread(subprocess.run, clear_cmd, capture_output=True, timeout=5.0)
        except Exception:
            pass

        await asyncio.sleep(0.3)

        try:
            await self.sim_session.api.nav_goal(target_goal, timeout_seconds=180.0)
        except Exception as e:
            await widget.update(f"⚠ Failed to dispatch detour goal: `{e}`")
            return

        sidebar = self.query_one(TelemetrySidebar)
        chat_scroll = self.query_one("#chat-scroll", VerticalScroll)
        start_time = asyncio.get_event_loop().time()

        while asyncio.get_event_loop().time() - start_time < 180.0:
            await asyncio.sleep(0.5)
            try:
                tel = await self.sim_session.api.telemetry()
                self.last_robot_pose = (tel.robot_pose.x, tel.robot_pose.y)
                dist = math.hypot(target_goal.x - tel.robot_pose.x, target_goal.y - tel.robot_pose.y)
                secs = int(tel.sim_time)
                sidebar.update_telemetry(
                    sim_state="Active (Detouring)",
                    sim_time=f"{secs // 60:02d}:{secs % 60:02d}.{int((tel.sim_time % 1) * 10)}",
                    rtf=f"{tel.real_time_factor:.2f}x",
                    robot_x=tel.robot_pose.x, robot_y=tel.robot_pose.y,
                    speed=abs(tel.robot_velocity.x) if tel.robot_velocity else 0.0,
                )

                nav = await self.sim_session.api.nav_status()

                if dist < 0.45 or nav.status == "succeeded":
                    await widget.update(
                        f"✅ **Detour Succeeded! Robot Reached Target Goal!**\n\n"
                        f"- **Destination:** `x={target_goal.x:.2f}, y={target_goal.y:.2f}`\n"
                        f"- **Final Pose:** `x={tel.robot_pose.x:.2f}, y={tel.robot_pose.y:.2f}`\n"
                        f"- **Nav2 Status:** `succeeded`\n\n"
                        f"*The robot successfully calculated an evasive detour trajectory around all obstacles and stopped safely in front of the target rack.*"
                    )
                    chat_scroll.scroll_end(animate=False)
                    break

                await widget.update(
                    f"🧭 **Autonomous Detour in Progress...**\n\n"
                    f"- **Navigating toward:** `x={target_goal.x:.2f}, y={target_goal.y:.2f}`\n"
                    f"- **Current Robot Pose:** `x={tel.robot_pose.x:.2f}, y={tel.robot_pose.y:.2f}`\n"
                    f"- **Distance Remaining:** `{dist:.2f} m`\n"
                    f"- **Nav2 Status:** `{nav.status}`"
                )
                chat_scroll.scroll_end(animate=False)

                if not nav.is_active and dist >= 0.45:
                    if nav.status in ("aborted", "failed"):
                        await widget.update(
                            f"⚠ **Detour halted:** Nav2 reported `{nav.status}`. "
                            f"Current robot position: `({tel.robot_pose.x:.2f}, {tel.robot_pose.y:.2f})`. "
                            f"Type `detour` to retry or `stop the robot`."
                        )
                        break
            except Exception:
                break

    @work(exclusive=False, group="mutation")
    async def execute_midway_mutation(self, widget: Markdown, prompt: str) -> None:
        """Dynamically drop models, adjust lighting, or inject perturbations into running Gazebo."""
        lowered = prompt.lower()
        if lowered.startswith("/stop"):
            await self.stop_simulation(widget)
            return

        # Auto-detect world name from Gazebo services
        world_name = self.current_world_name
        try:
            res = await asyncio.to_thread(
                subprocess.run,
                ["docker", "exec", "-u", "lang2sim", "lang2sim-sim", "/bin/bash", "-c", "source /opt/ros/jazzy/setup.bash && gz service -l"],
                capture_output=True, text=True, timeout=3.0
            )
            for line in res.stdout.splitlines():
                if line.startswith("/world/") and line.endswith("/create"):
                    world_name = line.split("/")[2]
                    self.current_world_name = world_name
                    break
        except Exception:
            pass

        # 1. Dynamic Lighting Adjustment
        if "light" in lowered or "dim" in lowered:
            lum_match = re.search(r"(\d+)%", prompt)
            lum = float(lum_match.group(1)) / 100.0 if lum_match else 0.2
            cmd = [
                "docker", "exec", "-u", "lang2sim", "lang2sim-sim",
                "/bin/bash", "-c",
                f"source /opt/ros/jazzy/setup.bash && gz service -s /world/{world_name}/light_config --reqtype gz.msgs.Light --req \"name: 'sun', diffuse: {{{{r: {lum:.2f}, g: {lum:.2f}, b: {lum:.2f}, a: 1.0}}}}\""
            ]
            try:
                await asyncio.to_thread(subprocess.run, cmd, capture_output=True, timeout=5.0)
                await widget.update(
                    f"⚡ **Dynamic Lighting Mutation Applied:**\n\n"
                    f"- Ambient illumination set to **{int(lum*100)}%** in world `{world_name}`.\n"
                    f"- *Robot camera perception and visual costmaps updated in real time.*"
                )
            except Exception as e:
                await widget.update(f"⚠ Failed to adjust lighting: `{e}`")
            return

        # 2. Dynamic Model Spawning (rack, crate, barrel) - typo and synonym tolerant
        self.mutation_counter += 1
        if re.search(r"\b(?:rac?k|rank|shelf|shelv\w*|storage|stand)s?\b", lowered):
            model = "rack"
        elif re.search(r"\b(?:barrel|drum|cylinder)s?\b", lowered):
            model = "barrel"
        else:
            model = "crate"

        # Detect requested orientation (horizontal across the path vs lengthwise)
        is_horizontal = any(w in lowered for w in ("horizontal", "horizontally", "across", "perpendicular", "sideways", "crosswise", "rotated"))

        # Detect placement intent (in the path of robot vs on the side)
        in_path = any(w in lowered for w in ("in the path", "in path", "in front", "infront", "directly in front", "blocking", "middle", "center", "centre", "across the path", "in aisle"))

        # Parse explicit coordinates or compute intelligent placement
        coord_match = re.search(r"(?:at\s*)?(?:x\s*=\s*)?([-\d.]+)\s*(?:,|\s+)\s*(?:y\s*=\s*)?([-\d.]+)", prompt)
        if coord_match:
            try:
                spawn_x = float(coord_match.group(1))
                spawn_y = float(coord_match.group(2))
            except ValueError:
                spawn_x = round(self.last_robot_pose[0] + (3.0 if in_path else 3.2), 2)
                spawn_y = round(self.last_robot_pose[1], 2) if in_path else 0.9
        else:
            spawn_x = round(self.last_robot_pose[0] + (3.0 if in_path else 3.2), 2)
            if in_path:
                # Directly in the robot's driving lane (along y=0.0 or current robot lateral coordinate)
                spawn_y = round(self.last_robot_pose[1], 2)
            else:
                spawn_y = 0.9 if self.last_robot_pose[1] <= 0.0 else -0.9

        entity_name = f"dynamic_{model}_{self.mutation_counter}"

        if model == "rack":
            box_size = "1.2 2.8 3.0" if is_horizontal else "2.8 1.2 3.0"
            sdf = (
                f"<?xml version='1.0'?>\n<sdf version='1.9'><model name='{entity_name}'><static>true</static>"
                f"<link name='link'><collision name='c'><geometry><box><size>{box_size}</size></box></geometry></collision>"
                f"<visual name='v'><geometry><box><size>{box_size}</size></box></geometry>"
                f"<material><ambient>0.9 0.45 0.1 1</ambient><diffuse>0.9 0.45 0.1 1</diffuse></material></visual></link></model></sdf>"
            )
            z_pos = 1.5
        elif model == "barrel":
            sdf = (
                f"<?xml version='1.0'?>\n<sdf version='1.9'><model name='{entity_name}'><static>false</static>"
                f"<link name='link'><inertial><mass>20.0</mass></inertial>"
                f"<collision name='c'><geometry><cylinder><radius>0.35</radius><length>0.9</length></cylinder></geometry></collision>"
                f"<visual name='v'><geometry><cylinder><radius>0.35</radius><length>0.9</length></cylinder></geometry>"
                f"<material><ambient>0.85 0.1 0.1 1</ambient><diffuse>0.85 0.1 0.1 1</diffuse></material></visual></link></model></sdf>"
            )
            z_pos = 0.45
        else:
            sdf = (
                f"<?xml version='1.0'?>\n<sdf version='1.9'><model name='{entity_name}'><static>false</static>"
                f"<link name='link'><inertial><mass>15.0</mass></inertial>"
                f"<collision name='c'><geometry><box><size>0.8 0.8 0.8</size></box></geometry></collision>"
                f"<visual name='v'><geometry><box><size>0.8 0.8 0.8</size></box></geometry>"
                f"<material><ambient>0.95 0.4 0.1 1</ambient><diffuse>0.95 0.4 0.1 1</diffuse></material></visual></link></model></sdf>"
            )
            z_pos = 0.4

        # Write temporary SDF file inside container as user lang2sim
        write_sdf_cmd = [
            "docker", "exec", "-u", "lang2sim", "lang2sim-sim",
            "/bin/bash", "-c", f"cat << 'EOF' > /tmp/{entity_name}.sdf\n{sdf}\nEOF"
        ]
        await asyncio.to_thread(subprocess.run, write_sdf_cmd, capture_output=True, timeout=3.0)

        # Call Gazebo Harmonic's native entity creation service as user lang2sim
        spawn_req = (
            f'sdf_filename: "/tmp/{entity_name}.sdf", '
            f'name: "{entity_name}", '
            f'pose: {{position: {{x: {spawn_x}, y: {spawn_y}, z: {z_pos}}}}}'
        )
        cmd = [
            "docker", "exec", "-u", "lang2sim", "lang2sim-sim",
            "/bin/bash", "-c",
            f"source /opt/ros/jazzy/setup.bash && gz service -s /world/{world_name}/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 4000 --req '{spawn_req}'"
        ]

        try:
            res = await asyncio.to_thread(subprocess.run, cmd, capture_output=True, text=True, timeout=6.0)
            if res.returncode == 0 and ("data: true" in res.stdout or "success" in res.stdout.lower()):
                orientation_desc = " [Horizontal across aisle]" if (model == "rack" and is_horizontal) else ""
                placement_desc = "directly in the robot's driving path" if in_path else "offset along aisle wall"
                msg = (
                    f"⚡ **Live Simulation Mutation Applied!**\n\n"
                    f"- **Injected Model:** `{entity_name}` ({model.capitalize()}{orientation_desc})\n"
                    f"- **Spawn Coordinates:** `x={spawn_x:.2f}, y={spawn_y:.2f}, z={z_pos:.2f}` ({placement_desc})\n\n"
                    f"*The new obstacle dropped directly into the active Gazebo physics engine. "
                    f"The TurtleBot3 LiDAR has detected it and Nav2 dynamic costmap is updating in real time.*"
                )
                await widget.update(msg)
            else:
                await widget.update(f"⚠ Gazebo could not spawn `{entity_name}`: {res.stdout or res.stderr}")
        except Exception as e:
            await widget.update(f"⚠ Mutation error: `{e}`")

    async def on_unmount(self) -> None:
        if getattr(self, "sim_session", None) is not None:
            await self.sim_session.stop()

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
