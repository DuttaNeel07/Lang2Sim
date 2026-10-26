"""
Headless Automated Verification Suite for Lang2Sim TUI
File: tests/test_agent_tui.py
"""

import pytest
from lang2sim.cli.app import Lang2SimApp
from lang2sim.cli.screens.landing_screen import LandingScreen
from lang2sim.cli.screens.workspace_screen import WorkspaceScreen
from lang2sim.cli.widgets.thought_box import ThoughtBox
from lang2sim.cli.widgets.action_pill import ActionPill
from lang2sim.cli.widgets.prompt_card import PromptCard
from lang2sim.cli.widgets.telemetry_sidebar import TelemetrySidebar
from lang2sim.agent.tools import TOOL_REGISTRY, execute_tool

@pytest.mark.asyncio
async def test_landing_screen_renders_and_has_focus():
    """Verify Screen 1 launches with centered logo, prompt card, and input focus."""
    app = Lang2SimApp()
    async with app.run_test() as pilot:
        assert isinstance(app.screen, LandingScreen)
        prompt_card = app.screen.query_one(PromptCard)
        assert prompt_card is not None
        input_widget = app.screen.query_one("#landing-prompt-input")
        assert input_widget.has_focus

@pytest.mark.asyncio
async def test_submitting_prompt_switches_to_workspace():
    """Verify typing into PromptCard and hitting Enter transitions smoothly to WorkspaceScreen."""
    app = Lang2SimApp()
    async with app.run_test() as pilot:
        prompt_card = app.screen.query_one(PromptCard)
        prompt_card.post_message(PromptCard.Submitted("Spawn 14x10 warehouse, send AMR to rack 5"))
        await pilot.pause()

        # Should now be on WorkspaceScreen
        assert isinstance(app.screen, WorkspaceScreen)
        assert app.screen.query_one("#chat-scroll") is not None
        assert app.screen.query_one(TelemetrySidebar) is not None

@pytest.mark.asyncio
async def test_thought_box_collapsible():
    """Verify ThoughtBox expands and collapses properly."""
    app = Lang2SimApp()
    async with app.run_test() as pilot:
        thought = ThoughtBox(duration_str="1.2s", content="Evaluating spatial clearance...")
        await app.screen.mount(thought)
        await pilot.pause()

        assert thought.is_collapsed is True
        
        # Toggle to expand
        thought.toggle()
        assert thought.is_collapsed is False
        assert thought.query_one("#thought-body").display is True
        
        # Toggle to collapse
        thought.toggle()
        assert thought.is_collapsed is True
        assert thought.query_one("#thought-body").display is False

@pytest.mark.asyncio
async def test_action_pill_status():
    """Verify ActionPill renders status and allows completing."""
    app = Lang2SimApp()
    async with app.run_test() as pilot:
        pill = ActionPill(tool_name="solve_spatial_layout", duration_str="0.4s", status="running")
        await app.screen.mount(pill)
        await pilot.pause()

        assert pill.status == "running"
        pill.set_completed("0.5s")
        assert pill.status == "completed"
        assert pill.duration_str == "0.5s"

@pytest.mark.asyncio
async def test_telemetry_sidebar_updates():
    """Verify TelemetrySidebar reflects live coordinates, velocities, and collisions."""
    app = Lang2SimApp()
    async with app.run_test() as pilot:
        sidebar = TelemetrySidebar()
        await app.screen.mount(sidebar)
        await pilot.pause()

        sidebar.update_telemetry(robot_x=7.50, robot_y=3.00, speed=0.45, collision_count=1)
        assert sidebar.data.robot_x == 7.50
        assert sidebar.data.robot_y == 3.00
        assert sidebar.data.collision_count == 1

def test_tool_registry_has_all_14_tools():
    """Verify all 14 tools are registered and return valid typed payloads."""
    expected_tools = [
        "solve_spatial_layout",
        "compile_world_sdf",
        "generate_zero_slam_map",
        "launch_simulation_stack",
        "inspect_topic_health",
        "spawn_dynamic_entity",
        "set_ambient_lighting",
        "reset_simulation",
        "apply_chassis_perturbation",
        "capture_gazebo_window",
        "capture_rviz_window",
        "read_robot_camera",
        "query_robot_telemetry",
        "auto_repair_tf",
    ]
    for tool_name in expected_tools:
        assert tool_name in TOOL_REGISTRY, f"Missing tool: {tool_name}"
        res = execute_tool(
            tool_name,
            dimensions={"x": 10},
            objects=[],
            layout_spec={"object_count": 8},
            world_path="world.sdf",
            topic_name="/scan",
            model_name="box",
            x=1.0,
            y=1.0,
            luminance=0.5,
            velocity_bias={"x": 0.1},
            parent_frame="map",
            child_frame="odom",
        )
        assert isinstance(res, dict)
        # query_robot_telemetry reads the live control API: "unavailable" when no sim is running.
        assert res.get("status") in ["success", "healthy", "repaired", "ok", "unavailable"] or "x" in res

@pytest.mark.asyncio
async def test_command_palette_modal_open_and_filter():
    """Verify CommandPaletteModal opens on ctrl+p, filters dynamically, and dismisses on esc."""
    from lang2sim.cli.screens.command_palette_modal import CommandPaletteModal
    from textual.widgets import Input

    app = Lang2SimApp()
    async with app.run_test() as pilot:
        # Trigger command palette
        await pilot.press("ctrl+p")
        await pilot.pause()

        # Modal should now be top screen
        assert isinstance(app.screen, CommandPaletteModal)
        modal = app.screen
        assert modal.query_one("#command-palette-dialog") is not None
        assert len(modal.filtered_commands) > 10

        # Type search query "doctor"
        search_input = modal.query_one("#palette-search", Input)
        search_input.value = "doctor"
        await pilot.pause()

        # Should filter down to doctor commands
        assert len(modal.filtered_commands) >= 1
        assert any("doctor" in cmd["title"].lower() for cmd in modal.filtered_commands)

        # Press escape to dismiss
        await pilot.press("escape")
        await pilot.pause()

        # Should be back on LandingScreen
        assert isinstance(app.screen, LandingScreen)


@pytest.mark.asyncio
async def test_nemotron_client_config():
    """Verify NemotronClient loads config, detects key types, and initializes properly."""
    from lang2sim.agent.nemotron_client import NemotronClient

    # Test with custom key
    client = NemotronClient(api_key="nvapi-test-key")
    assert client.is_configured() is True
    assert "nvidia" in client.model

    openai_client = NemotronClient(api_key="sk-test-key")
    assert openai_client.base_url == "https://api.openai.com/v1"
    assert "gpt" in openai_client.model


@pytest.mark.asyncio
async def test_workspace_screen_conversation_flow():
    """Verify WorkspaceScreen accumulates multi-turn conversation and updates status."""
    from unittest.mock import AsyncMock, patch

    app = Lang2SimApp()
    async with app.run_test() as pilot:
        # Navigate to workspace
        workspace = WorkspaceScreen(initial_prompt="")
        app.switch_screen(workspace)
        await pilot.pause()

        assert len(workspace.conversation_history) == 1  # System prompt
        assert workspace.conversation_history[0]["role"] == "system"

        # Mock stream_chat to return chunks
        async def mock_stream(messages, **kwargs):
            for word in ["Gazebo ", "Harmonic ", "is ", "running."]:
                yield word

        with patch.object(workspace.client, "is_configured", return_value=True), \
             patch.object(workspace.client, "stream_chat", side_effect=mock_stream):
            workspace.handle_user_prompt("What is running?")
            await pilot.pause()
            for _ in range(10):
                if len(workspace.conversation_history) == 3:
                    break
                await pilot.pause(0.05)

            # Verify prompt and assistant response appended
            assert len(workspace.conversation_history) == 3
            assert workspace.conversation_history[1]["role"] == "user"
            assert workspace.conversation_history[1]["content"] == "What is running?"
            assert workspace.conversation_history[2]["role"] == "assistant"
            assert workspace.conversation_history[2]["content"] == "Gazebo Harmonic is running."
            assert workspace.total_tokens_used > 0


@pytest.mark.asyncio
async def test_session_manager_persistence_and_grouping(tmp_path):
    """Verify session manager creates, saves, lists, pins, and groups sessions."""
    from lang2sim.core.session_manager import SessionManager, ChatMessage

    mgr = SessionManager(sessions_dir=tmp_path)
    s1 = mgr.create_session(title="Review ROS 2 SLAM pipeline architecture")
    s1.messages.append(ChatMessage(role="user", content="Test prompt", tokens=5))
    mgr.save_session(s1)

    s2 = mgr.create_session(title="PipeGen SLAM roadmap")
    mgr.pin_session(s2.id, True)

    listed = mgr.list_sessions()
    assert len(listed) == 2
    assert listed[0].id == s2.id  # Pinned first
    assert listed[0].pinned is True

    grouped = mgr.group_sessions_by_date(listed)
    assert "Pinned" in grouped
    assert "Today" in grouped

    # Rename
    mgr.rename_session(s1.id, "Renamed SLAM session")
    loaded = mgr.get_session(s1.id)
    assert loaded.title == "Renamed SLAM session"

    # Delete
    assert mgr.delete_session(s1.id) is True
    assert mgr.get_session(s1.id) is None


@pytest.mark.asyncio
async def test_sessions_modal_flow():
    """Verify SessionsModal opens on ctrl+s, filters dynamically, and switches sessions."""
    from lang2sim.cli.screens.sessions_modal import SessionsModal
    from lang2sim.core.session_manager import session_manager
    from textual.widgets import Input

    import uuid
    unique_tag = str(uuid.uuid4())[:6]
    title1 = f"Test SLAM Exploration {unique_tag}"
    title2 = f"Investigate TF discontinuity {unique_tag}"

    # Ensure test sessions exist
    s1 = session_manager.create_session(title=title1)
    s2 = session_manager.create_session(title=title2)

    app = Lang2SimApp()
    async with app.run_test() as pilot:
        # Navigate to workspace
        ws = WorkspaceScreen(initial_prompt="")
        app.switch_screen(ws)
        await pilot.pause()

        # Trigger sessions modal via ctrl+s
        await pilot.press("ctrl+s")
        await pilot.pause()

        assert isinstance(app.screen, SessionsModal)
        modal = app.screen
        assert modal.query_one("#sessions-dialog") is not None
        assert len(modal.filtered_sessions) >= 2

        # Filter by search unique_tag
        search_inp = modal.query_one("#sessions-search", Input)
        search_inp.value = title1
        await pilot.pause()

        assert len(modal.filtered_sessions) == 1
        assert modal.filtered_sessions[0].title == title1


        # Select with enter
        await pilot.press("enter")
        await pilot.pause()

        # Should be back on workspace with that session loaded
        assert isinstance(app.screen, WorkspaceScreen)
        assert app.screen.session.id == s1.id


@pytest.mark.asyncio
async def test_command_palette_switch_session_enter_opens_sessions_modal():
    """Verify selecting 'Switch session' in Command Palette and pressing Enter transitions to SessionsModal."""
    from lang2sim.cli.screens.command_palette_modal import CommandPaletteModal
    from lang2sim.cli.screens.sessions_modal import SessionsModal

    app = Lang2SimApp()
    async with app.run_test() as pilot:
        await pilot.press("ctrl+p")
        await pilot.pause()

        assert isinstance(app.screen, CommandPaletteModal)
        modal = app.screen
        # First item is "Switch session"
        assert modal.filtered_commands[0]["id"] == "switch_session"
        assert modal.selected_index == 0

        # Press enter while focused in input
        await pilot.press("enter")
        await pilot.pause()
        await pilot.pause()

        # SessionsModal should now be displayed
        assert isinstance(app.screen, SessionsModal)



