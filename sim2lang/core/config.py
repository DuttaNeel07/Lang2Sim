"""
Configuration and Path Resolution Manager for Sim2Lang.
File: sim2lang/core/config.py
"""

import os
from pathlib import Path
from datetime import datetime
from typing import Optional


class Config:
    """Central configuration for Sim2Lang runtime and compilers."""

    def __init__(self, workspace_root: Optional[Path] = None):
        self.workspace_root = (
            workspace_root or Path(__file__).resolve().parent.parent.parent
        )
        self.catalog_dir = self.workspace_root / "catalog"
        self.templates_dir = (
            self.workspace_root / "sim2lang" / "compiler" / "templates"
        )
        self.base_projects_dir = Path.home() / ".lang2sim" / "projects"

        # Docker container runtime configurations (Dev 3 synced)
        self.sim_image_tag: str = os.getenv(
            "LANG2SIM_IMAGE", "ghcr.io/lang2sim/lang2sim-sim:0.1.0"
        )
        self.control_api_port: int = int(os.getenv("CONTROL_API_PORT", "9090"))
        self.foxglove_port: int = int(os.getenv("FOXGLOVE_PORT", "8765"))
        self.novnc_port: int = int(os.getenv("NOVNC_PORT", "6080"))

        # Physical Robot Kinematics (TurtleBot3 Waffle standard)
        self.robot_circumscribed_radius: float = 0.22  # meters
        self.robot_safety_margin: float = 0.30        # meters
        self.min_aisle_corridor_width: float = 2.20   # meters
        self.min_wall_standoff: float = 0.50          # meters

        # Map Slicer Defaults
        self.map_resolution: float = 0.05             # meters per pixel
        self.lidar_min_height: float = 0.10           # meters
        self.lidar_max_height: float = 0.50           # meters

    def get_project_dir(self, project_name: str) -> Path:
        """Returns the isolated project directory."""
        pdir = self.base_projects_dir / project_name
        pdir.mkdir(parents=True, exist_ok=True)
        return pdir

    def create_run_dir(self, project_name: str) -> Path:
        """
        Creates an isolated timestamped run directory matching Dev 3 convention:
        ~/.lang2sim/projects/<project_name>/runs/<timestamp>/
        """
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        run_dir = self.get_project_dir(project_name) / "runs" / timestamp
        run_dir.mkdir(parents=True, exist_ok=True)
        return run_dir


# Global singleton instance
config = Config()
