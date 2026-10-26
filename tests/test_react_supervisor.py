"""
Unit & Integration Tests for ReAct Supervisor and Enriched Session Memory
File: tests/test_react_supervisor.py
"""

import pytest
from unittest.mock import AsyncMock, patch
from textual.widgets import Markdown, Static

from lang2sim.cli.app import Lang2SimApp
from lang2sim.cli.screens.workspace_screen import WorkspaceScreen
from lang2sim.cli.widgets.thought_box import ThoughtBox
from lang2sim.cli.widgets.action_pill import ActionPill
from lang2sim.agent.nemotron_client import NemotronClient
from lang2sim.agent.react_loop import (
    ReActSupervisor,
    ThoughtChunk,
    ThoughtCompleted,
    ActionStarted,
    ActionCompleted,
    ContentChunk,
    TurnCompleted,
)
from lang2sim.core.session_manager import (
    SessionManager,
    ChatMessage,
    ToolCallRecord,
    ArtifactRecord,
    SimulationStateSnapshot,
)


@pytest.mark.asyncio
async def test_react_demuxer_separates_thought_and_content():
    """Verify ReActSupervisor demuxes <thought>...</thought> blocks cleanly from user Markdown."""
    client = NemotronClient(api_key="nvapi-test")

    async def mock_stream(messages, **kwargs):
        tokens = [
            "<thought>\n",
            "1. Check warehouse bounds.\n",
            "2. Place obstacles.\n",
            "</thought>\n",
            "### Layout Generated\n",
            "- Obstacle 1 placed at (2.0, 0.0)",
        ]
        for t in tokens:
            yield t

    supervisor = ReActSupervisor(client=client)
    mgr = SessionManager()
    session = mgr.create_session(title="Test ReAct Stream")

    events = []
    with patch.object(client, "stream_chat", side_effect=mock_stream):
        async for ev in supervisor.stream_turn(session, "Build warehouse", []):
            events.append(ev)

    # Check for thought completion
    thought_completions = [e for e in events if isinstance(e, ThoughtCompleted)]
    assert len(thought_completions) == 1
    assert "1. Check warehouse bounds." in thought_completions[0].thought
    assert "2. Place obstacles." in thought_completions[0].thought
    assert thought_completions[0].duration_str is not None

    # Check for content chunks
    content_chunks = [e for e in events if isinstance(e, ContentChunk)]
    assert len(content_chunks) >= 2
    full_content = "".join(c.chunk for c in content_chunks)
    assert "### Layout Generated" in full_content
    assert "<thought>" not in full_content

    # Check turn completion
    turn_completions = [e for e in events if isinstance(e, TurnCompleted)]
    assert len(turn_completions) == 1
    assert turn_completions[0].thought == thought_completions[0].thought
    assert "### Layout Generated" in turn_completions[0].full_content


@pytest.mark.asyncio
async def test_react_supervisor_tool_execution_and_recording():
    """Verify that detected tools are executed, yield ActionStarted/Completed, and produce ToolCallRecords."""
    client = NemotronClient(api_key="nvapi-test")

    async def mock_stream(messages, **kwargs):
        yield "Creating the warehouse layout and compiling SDF."

    supervisor = ReActSupervisor(client=client)
    mgr = SessionManager()
    session = mgr.create_session(title="Test Tool Invocation")

    events = []
    with patch.object(client, "stream_chat", side_effect=mock_stream):
        async for ev in supervisor.stream_turn(session, "Spawn warehouse racks", []):
            events.append(ev)

    action_starts = [e for e in events if isinstance(e, ActionStarted)]
    action_dones = [e for e in events if isinstance(e, ActionCompleted)]

    assert len(action_starts) >= 1
    assert any(a.tool_name == "solve_spatial_layout" for a in action_starts)
    assert len(action_dones) == len(action_starts)
    assert all(a.status == "completed" for a in action_dones)

    # Verify TurnCompleted contains the actions list
    turn_completions = [e for e in events if isinstance(e, TurnCompleted)]
    assert len(turn_completions) == 1
    turn = turn_completions[0]
    assert len(turn.actions) >= 1
    assert any(act.tool_name == "solve_spatial_layout" for act in turn.actions)


@pytest.mark.asyncio
async def test_workspace_screen_restores_thought_box_and_action_pills(tmp_path):
    """Verify WorkspaceScreen reconstructs ThoughtBox and ActionPill widgets when loading session."""
    mgr = SessionManager(sessions_dir=tmp_path)
    session = mgr.create_session(title="Persisted Reasoning Test")

    # User message
    session.messages.append(ChatMessage(role="user", content="Analyze map and health"))

    # Assistant message with thinking and actions
    session.messages.append(
        ChatMessage(
            role="assistant",
            content="Map analyzed and /scan topic is healthy.",
            thought="Inspecting laser scan frequency and odometry TF tree.",
            thought_duration_str="1.5s",
            actions=[
                ToolCallRecord(
                    tool_name="inspect_topic_health",
                    arguments={"topic_name": "/scan"},
                    duration_str="0.32s",
                    status="completed",
                    output={"status": "healthy", "rate_hz": 30.0},
                )
            ],
            tokens=45,
        )
    )
    mgr.save_session(session)

    # Mount in textual app using the custom SessionManager
    app = Lang2SimApp()
    async with app.run_test() as pilot:
        with patch("lang2sim.cli.screens.workspace_screen.session_manager", mgr):
            ws = WorkspaceScreen(session_id=session.id)
            app.switch_screen(ws)
            await pilot.pause()

            # Verify ThoughtBox is mounted
            thought_boxes = ws.query(ThoughtBox)
            assert len(thought_boxes) == 1
            assert thought_boxes[0].duration_str == "1.5s"
            assert "Inspecting laser scan frequency" in thought_boxes[0].thought_content

            # Verify ActionPill is mounted
            action_pills = ws.query(ActionPill)
            assert len(action_pills) == 1
            assert action_pills[0].tool_name == "inspect_topic_health"
            assert action_pills[0].status == "completed"
            assert action_pills[0].duration_str == "0.32s"

            # Verify assistant markdown is mounted
            md_widgets = ws.query(".assistant-response")
            assert len(md_widgets) >= 1


def test_in_chat_memory_strict_session_isolation(tmp_path):
    """Verify strict session isolation: zero cross-session memory bleed."""
    mgr = SessionManager(sessions_dir=tmp_path)

    s1 = mgr.create_session(title="Session 1 - Warehouse Alpha", system_prompt="You are Copilot Alpha.")
    s2 = mgr.create_session(title="Session 2 - Hospital Beta", system_prompt="You are Copilot Beta.")

    # Add thought and action records only to Session 1
    s1.messages.append(
        ChatMessage(
            role="assistant",
            content="Alpha warehouse configured.",
            thought="Thinking strictly within Alpha workspace.",
            actions=[
                ToolCallRecord(
                    tool_name="compile_world_sdf",
                    duration_str="0.5s",
                    status="completed",
                )
            ],
        )
    )
    s1.artifacts.append(
        ArtifactRecord(type="sdf", path="/tmp/alpha.sdf", size_bytes=2048)
    )
    s1.simulation_state = SimulationStateSnapshot(
        world_sdf="alpha.sdf",
        map_yaml="alpha.yaml",
        active_robot="turtlebot3_waffle",
        ate_drift_m=0.03,
        status="running",
    )
    mgr.save_session(s1)

    # Session 2 should remain completely empty
    mgr.save_session(s2)

    loaded_s1 = mgr.get_session(s1.id)
    loaded_s2 = mgr.get_session(s2.id)

    assert loaded_s1 is not None
    assert loaded_s2 is not None

    # Verify s1 has enriched data
    assert len(loaded_s1.messages) == 2  # system + assistant
    assert loaded_s1.messages[1].thought == "Thinking strictly within Alpha workspace."
    assert len(loaded_s1.messages[1].actions) == 1
    assert len(loaded_s1.artifacts) == 1
    assert loaded_s1.simulation_state.ate_drift_m == 0.03

    # Verify s2 is 100% isolated: NO thought, NO actions, NO artifacts, NO bleed
    assert len(loaded_s2.messages) == 1  # only system prompt
    assert len(loaded_s2.artifacts) == 0
    assert loaded_s2.simulation_state.world_sdf is None
    assert loaded_s2.simulation_state.status == "idle"
    assert loaded_s2.simulation_state.ate_drift_m == 0.0
