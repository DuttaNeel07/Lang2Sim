"""
Headless Automated Verification Suite for Lang2Sim TUI
File: tests/test_agent_tui.py
"""

import pytest
from sim2lang.cli.app import Lang2SimApp
from sim2lang.cli.screens.landing_screen import LandingScreen
from sim2lang.cli.screens.workspace_screen import WorkspaceScreen
from sim2lang.cli.widgets.thought_box import ThoughtBox
from sim2lang.cli.widgets.action_pill import ActionPill
from sim2lang.cli.widgets.prompt_card import PromptCard
from sim2lang.cli.widgets.telemetry_sidebar import TelemetrySidebar
from sim2lang.agent.tools import TOOL_REGISTRY, execute_tool

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
        assert res.get("status") in ["success", "healthy", "repaired"] or "x" in res
