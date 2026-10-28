"""Telemetry and scoring package for Lang2Sim."""

import sys

# Backward-compatibility alias
sys.modules.setdefault("sim2lang.telemetry", sys.modules[__name__])

from lang2sim.telemetry.nqi_scorer import (
    NQIParams,
    OccupancyGrid,
    build_report,
    compute_nqi,
    default_report_path,
    demo_trajectories,
    min_clearance,
    mission_success,
    optimal_length,
    path_efficiency,
    path_length,
    render_comparison_table,
    render_to_console,
    score_trajectory,
    trajectory_smoothness,
    write_report,
)
from lang2sim.telemetry.ros_subscribers import (
    CollisionCounter,
    ReplaySource,
    TelemetryCollector,
    TelemetrySample,
    get_telemetry_collector,
    query_robot_telemetry,
)
from lang2sim.telemetry.trajectory import Trajectory
from lang2sim.telemetry.vision import (
    CameraSampler,
    VisionResult,
    capture_desktop_window,
    capture_gazebo_window,
    capture_rviz_window,
    compute_optical_metrics,
    encode_jpeg_base64,
    generate_synthetic_frame,
    get_camera_sampler,
    read_robot_camera,
)

__all__ = [
    # Trajectory & Scoring
    "Trajectory",
    "NQIParams",
    "OccupancyGrid",
    "path_length",
    "optimal_length",
    "path_efficiency",
    "trajectory_smoothness",
    "min_clearance",
    "mission_success",
    "compute_nqi",
    "score_trajectory",
    "build_report",
    "write_report",
    "default_report_path",
    "render_comparison_table",
    "render_to_console",
    "demo_trajectories",
    # Vision
    "VisionResult",
    "compute_optical_metrics",
    "encode_jpeg_base64",
    "capture_desktop_window",
    "capture_gazebo_window",
    "capture_rviz_window",
    "read_robot_camera",
    "generate_synthetic_frame",
    "CameraSampler",
    "get_camera_sampler",
    # Subscribers & Telemetry
    "CollisionCounter",
    "TelemetrySample",
    "TelemetryCollector",
    "ReplaySource",
    "get_telemetry_collector",
    "query_robot_telemetry",
]
