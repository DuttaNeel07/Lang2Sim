"""
Unit & Integration Tests for ReAct Supervisor and Enriched Session Memory
File: tests/test_react_supervisor.py
"""

from unittest.mock import patch

import pytest

from lang2sim.agent.nemotron_client import NemotronClient
from lang2sim.agent.react_loop import (
    ActionCompleted,
    ActionStarted,
    ContentChunk,
    ReActSupervisor,
    ThoughtCompleted,
    TurnCompleted,
)
from lang2sim.cli.app import Lang2SimApp
from lang2sim.cli.screens.workspace_screen import WorkspaceScreen
from lang2sim.cli.widgets.action_pill import ActionPill
from lang2sim.cli.widgets.thought_box import ThoughtBox
from lang2sim.core.session_manager import (
    ArtifactRecord,
    ChatMessage,
    SessionManager,
    SimulationStateSnapshot,
    ToolCallRecord,
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
    assert len(content_chunks) >= 1
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

    s1 = mgr.create_session(
        title="Session 1 - Warehouse Alpha", system_prompt="You are Copilot Alpha."
    )
    s2 = mgr.create_session(
        title="Session 2 - Hospital Beta", system_prompt="You are Copilot Beta."
    )

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
    s1.artifacts.append(ArtifactRecord(type="sdf", path="/tmp/alpha.sdf", size_bytes=2048))
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


@pytest.mark.asyncio
async def test_workspace_full_turn_react_execution_and_persistence(tmp_path):
    """End-to-end test: user prompt -> ReAct demuxing -> live tool execution -> UI mounting -> session persistence."""
    mgr = SessionManager(sessions_dir=tmp_path)

    app = Lang2SimApp()
    async with app.run_test() as pilot:
        with patch("lang2sim.cli.screens.workspace_screen.session_manager", mgr):
            ws = WorkspaceScreen(initial_prompt="")
            ws.always_allow_commands = True
            app.switch_screen(ws)
            await pilot.pause()

            # Mock stream_chat emitting thoughts and text
            async def mock_stream(messages, **kwargs):
                tokens = [
                    "<thought>\n",
                    "Evaluating SLAM drift parameters and room geometry.\n",
                    "</thought>\n",
                    "### Warehouse Ready\n",
                    "- Collision-free placement verified.\n",
                    "- Zero-SLAM map sliced.",
                ]
                for t in tokens:
                    yield t

            with (
                patch.object(ws.client, "is_configured", return_value=True),
                patch.object(ws.client, "stream_chat", side_effect=mock_stream),
                patch(
                    "lang2sim.agent.react_loop.execute_tool",
                    return_value={"status": "success", "object_count": 4},
                ),
            ):
                ws.handle_user_prompt("Spawn warehouse and test SLAM drift")
                await pilot.pause()

                for _ in range(30):
                    if len(ws.conversation_history) == 3:
                        break
                    await pilot.pause(0.1)

                # 1. Verify conversation history updated
                assert len(ws.conversation_history) == 3
                assert ws.conversation_history[1]["role"] == "user"
                assert ws.conversation_history[2]["role"] == "assistant"
                assert "Warehouse Ready" in ws.conversation_history[2]["content"]

                # 2. Verify UI widgets mounted in chat-scroll
                thought_boxes = ws.query(ThoughtBox)
                assert len(thought_boxes) == 1
                assert "Evaluating SLAM drift parameters" in thought_boxes[0].thought_content

                action_pills = ws.query(ActionPill)
                assert len(action_pills) >= 1
                assert all(p.status == "completed" for p in action_pills)

                # 3. Verify session was saved to disk with enriched fields
                saved = mgr.get_session(ws.session.id)
                assert saved is not None
                assert len(saved.messages) == 3  # system + user + assistant
                asst_msg = saved.messages[2]
                assert asst_msg.role == "assistant"
                assert "Warehouse Ready" in asst_msg.content
                assert "Evaluating SLAM drift parameters" in asst_msg.thought
                assert len(asst_msg.actions) >= 1


def test_os_tools_execution_and_file_ops(tmp_path):
    """Verify run_command, view_file, write_file, replace_file_content work reliably."""
    from lang2sim.agent.tools import replace_file_content, run_command, view_file, write_file

    # 1. Test run_command
    res = run_command("echo 'Hello Lang2Sim'")
    assert res["status"] == "success"
    assert res["exit_code"] == 0
    assert "Hello Lang2Sim" in res["stdout"]

    # 2. Test write_file & view_file
    test_file = tmp_path / "sub" / "test.txt"
    w_res = write_file(str(test_file), "Line 1\nLine 2\nLine 3\n")
    assert w_res["status"] == "success"
    assert test_file.exists()

    v_res = view_file(str(test_file), start_line=2, end_line=3)
    assert v_res["status"] == "success"
    assert v_res["content"] == "Line 2\nLine 3"

    # 3. Test replace_file_content
    r_res = replace_file_content(str(test_file), "Line 2", "Modified Line")
    assert r_res["status"] == "success"
    v_res2 = view_file(str(test_file))
    assert "Modified Line" in v_res2["content"]


@pytest.mark.asyncio
async def test_react_supervisor_planning_demuxing():
    """Verify ReActSupervisor demuxes <plan>...</plan> blocks into PlanCreated events."""
    client = NemotronClient(api_key="nvapi-test")

    async def mock_stream(messages, **kwargs):
        tokens = [
            "<plan>\n",
            "- Step 1: Procedural geometry synthesis\n",
            "- Step 2: SDF compilation & map slicing\n",
            "- Step 3: Run diagnostic verification\n",
            "</plan>\n",
            "Plan defined. Ready to execute.",
        ]
        for t in tokens:
            yield t

    supervisor = ReActSupervisor(client=client)
    mgr = SessionManager()
    session = mgr.create_session(title="Plan Test")

    events = []
    with patch.object(client, "stream_chat", side_effect=mock_stream):
        async for ev in supervisor.stream_turn(session, "Plan warehouse setup", []):
            events.append(ev)

    from lang2sim.agent.react_loop import PlanCreated

    plan_events = [e for e in events if isinstance(e, PlanCreated)]
    assert len(plan_events) == 1
    assert len(plan_events[0].steps) == 3
    assert "Step 1: Procedural geometry synthesis" in plan_events[0].steps[0]


@pytest.mark.asyncio
async def test_react_supervisor_permission_flow():
    """Verify that mutating actions ask for user permission and can be approved or rejected."""
    client = NemotronClient(api_key="nvapi-test")

    async def mock_stream(messages, **kwargs):
        yield "Executing command: pytest -q"

    supervisor = ReActSupervisor(client=client)
    mgr = SessionManager()
    session = mgr.create_session(title="Permission Test")

    # Case A: User Rejects
    async def deny_permission(tool_name, cmd_str, kwargs, event_id):
        return False

    with patch.object(client, "stream_chat", side_effect=mock_stream):
        events_denied = []
        async for ev in supervisor.stream_turn(
            session, "run command: pytest -q", [], permission_checker=deny_permission
        ):
            events_denied.append(ev)

    from lang2sim.agent.react_loop import CommandExecuted, PermissionRequested

    perm_reqs = [e for e in events_denied if isinstance(e, PermissionRequested)]
    assert len(perm_reqs) == 1
    assert perm_reqs[0].tool_name == "run_command"
    assert "pytest -q" in perm_reqs[0].command

    cmd_execs = [e for e in events_denied if isinstance(e, CommandExecuted)]
    assert len(cmd_execs) == 1
    assert "declined by user" in cmd_execs[0].stderr.lower()

    # Case B: User Approves
    async def approve_permission(tool_name, cmd_str, kwargs, event_id):
        return True

    with patch.object(client, "stream_chat", side_effect=mock_stream):
        events_approved = []
        async for ev in supervisor.stream_turn(
            session, "run command: echo 'Approved'", [], permission_checker=approve_permission
        ):
            events_approved.append(ev)

    approved_execs = [e for e in events_approved if isinstance(e, CommandExecuted)]
    assert len(approved_execs) == 1
    assert approved_execs[0].exit_code == 0
    assert "Approved" in approved_execs[0].stdout


@pytest.mark.asyncio
async def test_command_card_widget_interaction():
    """Verify CommandCard interactive approval and status transitions."""
    from lang2sim.cli.widgets.command_card import CommandCard

    card = CommandCard(
        event_id="test-123",
        command="docker ps",
        tool_name="run_command",
        permission_required=True,
    )
    assert card.status == "awaiting_permission"

    # Set running
    card.set_running()
    assert card.status == "running"

    # Set result
    card.set_result(exit_code=0, duration_str="0.45s", stdout="CONTAINER ID   IMAGE", stderr="")
    assert card.status == "completed"
    assert card.exit_code == 0
    assert card.duration_str == "0.45s"


def test_plan_card_step_updates():
    """Verify PlanCard displays and transitions stage-wise execution steps."""
    from lang2sim.cli.widgets.plan_card import PlanCard

    steps = [
        "1. Spatial Layout Generation",
        "2. Compile Gazebo SDF 1.9",
        "3. Launch Gazebo Harmonic",
    ]
    plan = PlanCard(steps=steps)
    assert len(plan.steps) == 3
    assert plan.steps[0]["status"] == "running"
    assert plan.steps[1]["status"] == "pending"

    # Progress step 0 to completed, step 1 to running
    plan.update_step(0, "completed", "4 racks placed")
    assert plan.steps[0]["status"] == "completed"
    assert plan.steps[0]["detail"] == "4 racks placed"

    plan.update_step(1, "running", "Compiling world.sdf")
    assert plan.steps[1]["status"] == "running"


def test_raw_json_tool_extraction_and_cleaning():
    """Verify that naked/raw JSON tool calls from Nemotron are parsed and stripped from Markdown."""
    from lang2sim.agent.react_loop import ReActSupervisor

    supervisor = ReActSupervisor()
    raw_output = (
        "I will now arrange the warehouse.\n"
        '{"tool": "solve_spatial_layout", "arguments": {"dimensions": {"width": 14, "height": 10}, '
        '"objects": [{"type": "wall", "name": "outer_wall_bottom", "pose": [7, 0.1, 0], "size": [14, 0.2, 2.5]}]}}\n'
        "Placement completed."
    )

    tools, cleaned = supervisor._extract_tools_and_clean_text(raw_output)
    assert len(tools) == 1
    assert tools[0][0] == "solve_spatial_layout"
    assert tools[0][1]["dimensions"]["width"] == 14
    assert '{"tool":' not in cleaned
    assert "outer_wall_bottom" not in cleaned
    assert "I will now arrange the warehouse." in cleaned
    assert "Placement completed." in cleaned


def test_invoked_phrase_extraction_and_cleaning():
    """Verify that phrases like 'Invoked tool with {...}' are parsed as tool calls and stripped."""
    from lang2sim.agent.react_loop import ReActSupervisor

    supervisor = ReActSupervisor()
    raw_output = 'Invoked generate_zero_slam_map with {"sdf_path": "~/.lang2sim/projects/active_project/world.sdf"}'

    tools, cleaned = supervisor._extract_tools_and_clean_text(raw_output)
    assert len(tools) == 1
    assert tools[0][0] == "generate_zero_slam_map"
    assert tools[0][1]["sdf_path"] == "~/.lang2sim/projects/active_project/world.sdf"
    assert "Invoked generate_zero_slam_map" not in cleaned
    assert cleaned == ""
