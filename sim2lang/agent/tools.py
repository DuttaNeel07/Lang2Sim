"""
Typed Tool Registry & Function Schemas for Nemotron 3 ReAct Supervisor
File: sim2lang/agent/tools.py
"""

from typing import Dict, Any, Callable

# 14 Typed Tool Implementations (With baseline mockable handlers for Dev 2 testing)

def solve_spatial_layout(dimensions: Dict[str, float], objects: list) -> Dict[str, Any]:
    """Runs Shapely 2.0 geometry solver to calculate collision-free OBB bounds."""
    return {
        "status": "success",
        "room_dimensions": dimensions,
        "object_count": len(objects),
        "aisle_clearance_min": 2.4,
        "collisions_detected": 0,
    }

def compile_world_sdf(layout_spec: Dict[str, Any], target_simulator: str = "harmonic") -> Dict[str, Any]:
    """Compiles spatial layout into Gazebo SDF 1.6 / 1.9 world."""
    return {
        "status": "success",
        "target": target_simulator,
        "sdf_path": "~/.lang2sim/projects/active_project/world.sdf",
        "entity_count": layout_spec.get("object_count", 8),
    }

def generate_zero_slam_map(sdf_path: str = "") -> Dict[str, Any]:
    """Direct 3D geometric slicing to map.yaml and map.pgm in <3.5s."""
    return {
        "status": "success",
        "yaml_path": "~/.lang2sim/projects/active_project/map.yaml",
        "pgm_path": "~/.lang2sim/projects/active_project/map.pgm",
        "resolution": 0.05,
        "elapsed_seconds": 1.4,
    }

def launch_simulation_stack(world_path: str, robot_model: str = "turtlebot3_waffle") -> Dict[str, Any]:
    """Dispatches Docker X11 Gazebo GUI, RViz2, and Nav2 Simple Commander."""
    return {
        "status": "success",
        "pid": 9999,
        "display": ":0",
        "robot_model": robot_model,
        "gui_windows": ["Gazebo (Rendering: Hardware)", "RViz2 (Displays Active)"],
    }

def inspect_topic_health(topic_name: str) -> Dict[str, Any]:
    """Verifies topic publication rates and message validity."""
    return {
        "topic": topic_name,
        "status": "healthy",
        "rate_hz": 30.0,
        "active_publishers": 1,
    }

def spawn_dynamic_entity(model_name: str, x: float, y: float, z: float = 0.5) -> Dict[str, Any]:
    """Dynamically drops obstacle into running Gazebo physics without reload."""
    return {
        "status": "success",
        "entity_name": model_name,
        "spawn_pose": {"x": x, "y": y, "z": z},
        "ipc_latency_ms": 0.2,
    }

def set_ambient_lighting(luminance: float) -> Dict[str, Any]:
    """Adjusts ambient lighting dynamically (1.0 = nominal, 0.15 = night/dim)."""
    return {
        "status": "success",
        "target_luminance": max(0.0, min(1.0, luminance)),
        "viewport_dimmed": luminance < 0.5,
    }

def reset_simulation(mode: str = "all") -> Dict[str, Any]:
    """Sub-second world reset snapping robot back in <0.3s without restarting GUI."""
    return {
        "status": "success",
        "reset_mode": mode,
        "elapsed_seconds": 0.18,
    }

def apply_chassis_perturbation(velocity_bias: Dict[str, float]) -> Dict[str, Any]:
    """Applies traction loss or external disturbance velocity bias."""
    return {
        "status": "success",
        "applied_bias": velocity_bias,
    }

def capture_gazebo_window() -> Dict[str, Any]:
    """Captures native desktop Gazebo window via X11 screen grabbing."""
    return {
        "status": "success",
        "window": "Gazebo",
        "detected_state": "Running (RTF: 1.0x, Sim Time: 00:14.2)",
        "models_visible": 8,
    }

def capture_rviz_window() -> Dict[str, Any]:
    """Captures active RViz2 window (Displays, Costmaps, Planned Path)."""
    return {
        "status": "success",
        "window": "RViz2",
        "costmap_status": "Valid (No lethal obstacle in goal footprint)",
        "path_visible": True,
    }

def read_robot_camera(topic: str = "/camera/image_raw") -> Dict[str, Any]:
    """Samples forward-facing optical frame from robot camera."""
    return {
        "status": "success",
        "topic": topic,
        "resolution": "640x480",
        "fps": 30.0,
        "optical_blur_detected": False,
    }

def query_robot_telemetry() -> Dict[str, Any]:
    """Reads instantaneous kinematics, distance to goal, and contact forces."""
    return {
        "x": 3.85,
        "y": 2.10,
        "speed": 0.35,
        "distance_to_goal": 2.14,
        "bumper_force_newtons": 0.0,
        "collision_count": 0,
    }

def auto_repair_tf(parent_frame: str, child_frame: str) -> Dict[str, Any]:
    """Injects ephemeral static transform publisher if frames disconnect."""
    return {
        "status": "repaired",
        "injected_transform": f"{parent_frame} -> {child_frame}",
    }

TOOL_REGISTRY: Dict[str, Callable] = {
    "solve_spatial_layout": solve_spatial_layout,
    "compile_world_sdf": compile_world_sdf,
    "generate_zero_slam_map": generate_zero_slam_map,
    "launch_simulation_stack": launch_simulation_stack,
    "inspect_topic_health": inspect_topic_health,
    "spawn_dynamic_entity": spawn_dynamic_entity,
    "set_ambient_lighting": set_ambient_lighting,
    "reset_simulation": reset_simulation,
    "apply_chassis_perturbation": apply_chassis_perturbation,
    "capture_gazebo_window": capture_gazebo_window,
    "capture_rviz_window": capture_rviz_window,
    "read_robot_camera": read_robot_camera,
    "query_robot_telemetry": query_robot_telemetry,
    "auto_repair_tf": auto_repair_tf,
}

import inspect

def execute_tool(name: str, **kwargs) -> Dict[str, Any]:
    """Executes a tool from the registry safely, passing only accepted arguments."""
    if name not in TOOL_REGISTRY:
        return {"status": "error", "message": f"Unknown tool: {name}"}
    func = TOOL_REGISTRY[name]
    sig = inspect.signature(func)
    filtered = {k: v for k, v in kwargs.items() if k in sig.parameters}
    return func(**filtered)

