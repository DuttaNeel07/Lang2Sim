"""Comprehensive pytest test suite for Part 4:
  - FastMCP In-Process Dynamic Stress Controls
  - Gazebo Dynamic Mutation Bridge (Harmonic / Classic / DryRun)
  - Dual-Stream Visual Perception Observer
  - Hardware Contact Collision Counter & Telemetry Subscribers
  - Composite Navigation Quality Index (NQI) Benchmarking Engine

Can be run entirely offline on Windows/Linux without ROS 2 or Gazebo installed.
"""

from __future__ import annotations

import base64
import io
import json
import math
from pathlib import Path

import numpy as np
import pytest
from PIL import Image

from lang2sim.perturbation._compat import Pose2D
from lang2sim.perturbation.gazebo_bridge import (
    ClassicBridge,
    DryRunBridge,
    HarmonicBridge,
    get_bridge,
    reset_bridge_instance,
)
from lang2sim.perturbation.mcp_server import (
    call_tool_inprocess,
    mcp,
)
from lang2sim.telemetry.nqi_scorer import (
    NQIParams,
    build_report,
    compute_nqi,
    min_clearance,
    mission_success,
    optimal_length,
    path_efficiency,
    path_length,
    render_comparison_table,
    score_trajectory,
    trajectory_smoothness,
)
from lang2sim.telemetry.ros_subscribers import (
    CollisionCounter,
    ReplaySource,
    TelemetryCollector,
)
from lang2sim.telemetry.trajectory import Trajectory
from lang2sim.telemetry.vision import (
    capture_desktop_window,
    compute_optical_metrics,
    encode_jpeg_base64,
    generate_synthetic_frame,
    get_camera_sampler,
)


# ===========================================================================
# 1. NQI Mathematical Evaluation Tests
# ===========================================================================
class TestNQIEvaluator:
    def test_trajectory_creation_and_resample(self):
        t = np.array([0.0, 1.0, 2.0, 3.0])
        x = np.array([0.0, 1.0, 2.0, 3.0])
        y = np.zeros(4)
        traj = Trajectory(t=t, x=x, y=y)
        assert traj.n == 4
        assert traj.duration == 3.0
        assert math.isclose(path_length(traj), 3.0, abs_tol=1e-5)

        resampled = traj.resample(dt=0.5)
        assert resampled.n == 7
        assert math.isclose(resampled.duration, 3.0, abs_tol=1e-5)

    def test_trajectory_json_roundtrip(self, tmp_path: Path):
        t = np.array([0.0, 1.0, 2.0])
        x = np.array([0.0, 0.5, 1.0])
        y = np.array([0.0, 0.2, 0.0])
        orig = Trajectory(t=t, x=x, y=y, goal=(1.0, 0.0), collisions=2, label="test_run")
        out_file = tmp_path / "test_traj.json"
        orig.to_json(out_file)

        loaded = Trajectory.from_json(out_file)
        assert loaded.n == orig.n
        assert loaded.goal == (1.0, 0.0)
        assert loaded.collisions == 2
        assert loaded.label == "test_run"
        assert np.allclose(loaded.x, orig.x)

    def test_nominal_straight_line_nqi_perfect_score(self):
        # 10m straight line at 0.5m/s (20s duration)
        t = np.arange(0.0, 20.0 + 1e-9, 0.05)
        x = 0.5 * t
        y = np.zeros_like(t)
        traj = Trajectory(t=t, x=x, y=y, goal=(10.0, 0.0), label="nominal")

        scored = score_trajectory(traj, params=NQIParams())
        assert scored["E_p"] == 1.0
        assert scored["C_force"] == 0
        assert scored["S_mission"] == 1
        assert scored["S_tau"] < 0.1  # Zero acceleration jerk
        assert scored["nqi"] == 100.0  # Maxed out nominal index

    def test_oscillating_detour_degrades_nqi(self):
        # Slower detour trajectory
        t = np.arange(0.0, 30.0 + 1e-9, 0.05)
        x = 10.0 * (t / t[-1])
        y = 1.0 * np.sin(2 * np.pi * x / 5.0)  # Heavy oscillations
        traj = Trajectory(t=t, x=x, y=y, goal=(10.0, 0.0), label="detour")

        scored = score_trajectory(traj, params=NQIParams())
        assert scored["E_p"] < 0.90  # Path longer than straight Euclidean
        assert scored["S_tau"] > 0.5  # Significant jerk from sinusoidal turns
        assert scored["nqi"] < 95.0  # Noticeable degradation vs 100.0 nominal

    def test_collision_penalty_drops_score_by_fifty(self):
        t = np.arange(0.0, 20.0 + 1e-9, 0.05)
        x = 0.5 * t
        y = np.zeros_like(t)
        traj = Trajectory(t=t, x=x, y=y, goal=(10.0, 0.0), collisions=1)

        scored = score_trajectory(traj, params=NQIParams())
        assert scored["C_force"] == 1
        assert scored["S_mission"] == 0  # Mission failed due to collision
        assert math.isclose(scored["nqi"], 50.0, abs_tol=1e-1)  # 100 - 50 = 50

    def test_clearance_margin_calculation(self):
        t = np.array([0.0, 1.0, 2.0])
        x = np.array([0.0, 1.0, 2.0])
        y = np.zeros(3)
        traj = Trajectory(t=t, x=x, y=y)
        # Obstacle at (1.0, 0.5) with radius 0.1m, robot radius 0.22m
        # Center-to-center distance at t=1.0 is 0.5m.
        # Boundary clearance = 0.5 - 0.1 - 0.22 = 0.18m
        obstacles = [(1.0, 0.5, 0.1)]
        c_min = min_clearance(traj, obstacles=obstacles, r_robot=0.22)
        assert c_min is not None
        assert math.isclose(c_min, 0.18, abs_tol=1e-3)

    def test_benchmark_report_and_rich_table(self):
        t = np.arange(0.0, 10.0 + 1e-9, 0.1)
        nom_traj = Trajectory(t=t, x=t, y=np.zeros_like(t), goal=(10.0, 0.0), label="nominal")
        adv_traj = Trajectory(t=t, x=t, y=0.2 * np.sin(t), goal=(10.0, 0.0), collisions=0, label="adversarial")

        nom = score_trajectory(nom_traj)
        adv = score_trajectory(adv_traj)
        report = build_report(nom, adv)

        assert report["schema_version"] == "1.0"
        assert "nominal" in report["runs"]
        assert "adversarial" in report["runs"]
        assert "deltas" in report
        assert "nqi_pct" in report["deltas"]

        # Render Rich table without throwing exceptions
        table = render_comparison_table(report)
        assert table.title == "Lang2Sim Navigation Benchmark (NQI)"


# ===========================================================================
# 2. FastMCP & Mutation Bridge Tests
# ===========================================================================
class TestMutationBridge:
    @pytest.fixture(autouse=True)
    def clean_bridge(self):
        reset_bridge_instance()
        yield
        reset_bridge_instance()

    @pytest.mark.asyncio
    async def test_dryrun_bridge_harmonic_commands(self):
        bridge = DryRunBridge(world_name="test_world", simulated_target="harmonic")
        res = await bridge.spawn_entity("wooden_crate", Pose2D(x=3.5, y=2.0, yaw=0.0))
        assert res.ok is True
        assert "gz service -s /world/test_world/create" in res.command
        assert "model://wooden_crate" in res.command
        assert res.data["entity_name"] == "wooden_crate_1"

        light_res = await bridge.set_light(0.2)
        assert light_res.ok is True
        assert "light_config" in light_res.command
        assert "0.20" in light_res.command

        reset_res = await bridge.reset("world")
        assert reset_res.ok is True
        assert "WorldControl" in reset_res.command
        assert reset_res.latency_ms > 0

    @pytest.mark.asyncio
    async def test_dryrun_bridge_classic_commands(self):
        bridge = DryRunBridge(world_name="default", simulated_target="classic")
        res = await bridge.spawn_entity("barrel", Pose2D(x=1.0, y=1.0, yaw=1.57))
        assert res.ok is True
        assert "ros2 service call /spawn_entity" in res.command

        light_res = await bridge.set_light(0.5)
        assert light_res.ok is True
        assert "/set_light_properties" in light_res.command

    @pytest.mark.asyncio
    async def test_light_validation(self):
        bridge = DryRunBridge()
        invalid_res = await bridge.set_light(1.5)
        assert invalid_res.ok is False
        assert "between 0.0 and 1.0" in invalid_res.message

    @pytest.mark.asyncio
    async def test_fastmcp_inprocess_tool_calls(self):
        # Invoke all tools via in-process FastMCP call
        r1 = await call_tool_inprocess("spawn_dynamic_entity", {"model_name": "cargo_box", "pose": {"x": 2.0, "y": 3.0, "yaw": 0.0}})
        assert r1["ok"] is True
        assert r1["data"]["model_name"] == "cargo_box"

        r2 = await call_tool_inprocess("set_ambient_lighting", {"luminance": 0.15})
        assert r2["ok"] is True
        assert r2["data"]["luminance"] == 0.15

        r3 = await call_tool_inprocess("reset_simulation", {"mode": "world"})
        assert r3["ok"] is True
        assert "reset_ms" in r3["data"]

        r4 = await call_tool_inprocess("apply_chassis_perturbation", {"velocity_bias": {"x": -0.2, "y": 0.0, "yaw": 0.1}})
        assert r4["ok"] is True

        r5 = await call_tool_inprocess("list_spawned_entities", {})
        assert "cargo_box_1" in r5["result"]


# ===========================================================================
# 3. Dual-Stream Visual Perception Tests
# ===========================================================================
class TestVisionObserver:
    def test_optical_metrics_sharp_vs_blurred(self):
        # Generate sharp synthetic frame
        sharp_img = generate_synthetic_frame(width=320, height=240, motion_blur=0)
        blurred_img = generate_synthetic_frame(width=320, height=240, motion_blur=5)

        sharp_metrics = compute_optical_metrics(np.asarray(sharp_img))
        blurred_metrics = compute_optical_metrics(np.asarray(blurred_img))

        # Variance of Laplacian must be higher for sharp than blurred
        assert sharp_metrics["blur_laplacian_var"] > blurred_metrics["blur_laplacian_var"]

    def test_optical_metrics_fog_attenuation(self):
        clear_img = generate_synthetic_frame(width=320, height=240, fog_density=0.0)
        foggy_img = generate_synthetic_frame(width=320, height=240, fog_density=0.25)

        clear_metrics = compute_optical_metrics(np.asarray(clear_img))
        foggy_metrics = compute_optical_metrics(np.asarray(foggy_img))

        # Contrast must drop under volumetric fog
        assert clear_metrics["contrast_rms"] > foggy_metrics["contrast_rms"]

    def test_base64_jpeg_encoding_roundtrip(self):
        img = generate_synthetic_frame(width=200, height=150)
        b64_str = encode_jpeg_base64(img, quality=80)
        assert len(b64_str) > 0

        # Decode back to PIL image and verify dimensions
        data = base64.b64decode(b64_str.encode("ascii"))
        decoded = Image.open(io.BytesIO(data))
        assert decoded.width == 200
        assert decoded.height == 150

    def test_desktop_window_capture_fallback(self):
        # In headless or non-GUI testing, capture_desktop_window gracefully falls back
        res = capture_desktop_window("NonExistentTestWindow")
        assert res.ok is True
        assert res.width > 0
        assert res.height > 0
        assert len(res.jpeg_base64) > 0
        assert res.fallback in ("full_screen", "synthetic_desktop")

    def test_camera_sampler_fallback(self):
        sampler = get_camera_sampler("/camera/image_raw")
        res = sampler.sample()
        assert res.ok is True
        assert res.width == 640
        assert res.height == 480
        assert "brightness" in res.metrics


# ===========================================================================
# 4. Telemetry Subscribers & Collision Counter Tests
# ===========================================================================
class TestTelemetrySubscribers:
    def test_collision_counter_threshold(self):
        counter = CollisionCounter(threshold_n=5.0, cooldown_s=0.5)

        # Force below threshold: 4.5N -> No collision
        assert counter.process_contact([3.0, 3.0, 0.0], "chassis_link", timestamp=1.0) is False
        assert counter.collision_count == 0

        # Force above threshold: 7.0N -> Collision registered!
        assert counter.process_contact([5.0, 4.0, 3.0], "chassis_link", timestamp=1.1) is True
        assert counter.collision_count == 1

        # Sustained contact within cooldown window -> Not counted as second collision
        assert counter.process_contact([6.0, 5.0, 0.0], "chassis_link", timestamp=1.3) is False
        assert counter.collision_count == 1

        # Force drops below threshold
        assert counter.process_contact([0.0, 0.0, 0.0], "chassis_link", timestamp=1.5) is False

        # Next impact after cooldown elapsed -> Second collision!
        assert counter.process_contact([8.0, 0.0, 0.0], "chassis_link", timestamp=2.0) is True
        assert counter.collision_count == 2

    def test_collision_counter_ignores_ground_plane(self):
        counter = CollisionCounter()
        # High force from wheel hitting ground plane / floor must be ignored
        assert counter.process_contact([0.0, 0.0, 20.0], "ground_plane", timestamp=1.0) is False
        assert counter.process_contact([0.0, 0.0, 15.0], "warehouse_floor", timestamp=1.1) is False
        assert counter.collision_count == 0

    def test_telemetry_collector_run_recording(self):
        collector = TelemetryCollector()
        collector.start_run(label="unit_test_run", goal=(5.0, 5.0))

        # Push mock kinematic samples
        for i in range(10):
            collector.record_kinematics(t=float(i), x=float(i) * 0.5, y=0.0, yaw=0.0, v=0.5, w=0.0)

        traj = collector.stop_run()
        assert traj.n == 10
        assert traj.goal == (5.0, 5.0)
        assert traj.label == "unit_test_run"
        assert math.isclose(path_length(traj), 4.5, abs_tol=1e-5)

    def test_replay_source_snapshot(self):
        t = np.array([0.0, 1.0, 2.0])
        x = np.array([1.0, 2.0, 3.0])
        y = np.array([0.0, 0.0, 0.0])
        traj = Trajectory(t=t, x=x, y=y, collisions=1)
        source = ReplaySource(traj)

        snap = source.snapshot()
        assert snap["active"] is True
        assert snap["collisions"] == 1
        assert "x" in snap and "y" in snap

    def test_telemetry_collector_snapshot_fields_and_alias(self):
        collector = TelemetryCollector()
        try:
            snap = collector.get_snapshot()
            assert "backend" in snap
            assert "rtf" in snap
            assert "sim_time" in snap
            assert "active" in snap
            assert snap["samples_buffered"] == 0
        finally:
            collector.close()


class TestWorldRouterAndClient:
    """Test suite for FastAPI /world router and ControlClient mutation methods."""

    @pytest.mark.asyncio
    async def test_control_client_world_mutation_endpoints(self):
        import httpx
        from lang2sim.runtime.control_api.server import create_app
        from lang2sim.runtime.control_api.client import ControlClient

        app = create_app()
        transport = httpx.ASGITransport(app=app)
        async with ControlClient(transport=transport) as client:
            # 1. Health
            health = await client.world_health()
            assert health["ok"] is True
            assert "backend" in health

            # 2. Spawn
            spawn_res = await client.world_spawn({
                "model_name": "wooden_crate",
                "x": 2.5,
                "y": 1.0,
                "z": 0.5,
                "yaw": 0.0,
            })
            assert spawn_res["ok"] is True
            assert "spawn" in spawn_res["tool"]

            # 3. Entities
            entities_res = await client.world_entities()
            assert entities_res["count"] >= 1
            assert any("wooden_crate" in e for e in entities_res["entities"])

            # 4. Light
            light_res = await client.world_light({
                "diffuse": 0.3,
                "specular": 0.1,
                "intensity": 0.5,
            })
            assert light_res["ok"] is True

            # 5. Chassis
            chassis_res = await client.world_chassis({
                "force_x": 15.0,
                "duration_s": 0.2,
            })
            assert chassis_res["ok"] is True

            # 6. Reset
            reset_res = await client.world_reset({})
            assert reset_res["ok"] is True


class TestReportPathIntegration:
    """Test standardizing benchmark_report.json to core.config get_run_dir."""

    def test_default_report_path_resolves_to_run_dir(self, tmp_path, monkeypatch):
        monkeypatch.setenv("HOME", str(tmp_path))
        from lang2sim.telemetry.nqi_scorer import default_report_path
        path = default_report_path(project_name="test_proj", timestamp="run_042")
        assert path.name == "benchmark_report.json"
        assert "run_042" in str(path)

