"""
Runtime and Workspace Configuration Manager
File: sim2lang/core/config.py
"""

import os
from pathlib import Path
from typing import Optional
from datetime import datetime

def _env(*names: str, default: str) -> str:
    """First non-empty env var among `names` (L2S_* names are what the sim container sets)."""
    for n in names:
        v = os.getenv(n)
        if v:
            return v
    return default


# Root paths. Inside the sim container the entrypoint sets L2S_PROJECTS_DIR /
# L2S_CATALOG_DIR to the bind-mount targets, so the same code resolves to the
# host's ~/.lang2sim/projects on both sides.
DEFAULT_LANG2SIM_ROOT = Path(os.path.expanduser(_env("L2S_ROOT", default="~/.lang2sim")))
PROJECTS_DIR = Path(_env("L2S_PROJECTS_DIR", default=str(DEFAULT_LANG2SIM_ROOT / "projects")))
CATALOG_DIR = Path(_env("L2S_CATALOG_DIR", default=str(Path(__file__).resolve().parent.parent.parent / "catalog")))

# Container-side mount points (contract with docker/sim/Dockerfile).
CONTAINER_PROJECTS_DIR = "/lang2sim/projects"
CONTAINER_CATALOG_DIR = "/lang2sim/catalog"
CONTAINER_SRC_DIR = "/opt/lang2sim/src"

# Pinned Container Image and Network Defaults (Dev 3 synced).
# CI publishes :staging, :main, :sha-<short>, and :<semver> for v* tags; the
# `-vnc` suffix selects the noVNC variant.
DEFAULT_SIM_IMAGE = _env("L2S_IMAGE", "LANG2SIM_IMAGE", default="ghcr.io/duttaneel07/lang2sim-sim:staging")
DEFAULT_CONTROL_PORT = int(_env("L2S_API_PORT", "CONTROL_API_PORT", default="9090"))
DEFAULT_FOXGLOVE_PORT = int(_env("L2S_FOXGLOVE_PORT", "FOXGLOVE_PORT", default="8765"))
DEFAULT_NOVNC_PORT = int(_env("L2S_NOVNC_PORT", "NOVNC_PORT", default="6080"))

# Physical Robot Kinematics (TurtleBot3 Waffle standard)
ROBOT_CIRCUMSCRIBED_RADIUS = 0.22  # meters
ROBOT_SAFETY_MARGIN = 0.30        # meters
MIN_AISLE_CORRIDOR_WIDTH = 2.20   # meters
MIN_WALL_STANDOFF = 0.50          # meters
MAP_RESOLUTION = 0.05             # meters per pixel


def ensure_directories():
    """Ensure ~/.lang2sim and projects directory exist."""
    DEFAULT_LANG2SIM_ROOT.mkdir(parents=True, exist_ok=True)
    PROJECTS_DIR.mkdir(parents=True, exist_ok=True)


def get_projects_dir() -> Path:
    """Returns the base projects directory."""
    ensure_directories()
    return PROJECTS_DIR


def get_active_project_dir(project_name: str = "active_project") -> Path:
    """Returns an isolated project directory."""
    ensure_directories()
    p_dir = PROJECTS_DIR / project_name
    p_dir.mkdir(parents=True, exist_ok=True)
    return p_dir


def get_run_dir(project_name: str = "active_project", timestamp: Optional[str] = None) -> Path:
    """
    Returns an isolated timestamped run directory matching Dev 3 convention:
    ~/.lang2sim/projects/<project_name>/runs/<timestamp>/
    """
    p_dir = get_active_project_dir(project_name)
    runs_dir = p_dir / "runs"
    runs_dir.mkdir(parents=True, exist_ok=True)
    if timestamp is None:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir = runs_dir / timestamp
    run_dir.mkdir(parents=True, exist_ok=True)
    return run_dir


def get_catalog_dir() -> Path:
    """Returns the catalog directory."""
    return CATALOG_DIR


class Config:
    """Object-oriented wrapper around configuration settings."""

    def __init__(self, workspace_root: Optional[Path] = None):
        self.workspace_root = workspace_root or Path(__file__).resolve().parent.parent.parent
        self.catalog_dir = CATALOG_DIR
        self.templates_dir = self.workspace_root / "sim2lang" / "compiler" / "templates"
        self.base_projects_dir = PROJECTS_DIR

        self.sim_image_tag = DEFAULT_SIM_IMAGE
        self.control_api_port = DEFAULT_CONTROL_PORT
        self.foxglove_port = DEFAULT_FOXGLOVE_PORT
        self.novnc_port = DEFAULT_NOVNC_PORT

        self.robot_circumscribed_radius = ROBOT_CIRCUMSCRIBED_RADIUS
        self.robot_safety_margin = ROBOT_SAFETY_MARGIN
        self.min_aisle_corridor_width = MIN_AISLE_CORRIDOR_WIDTH
        self.min_wall_standoff = MIN_WALL_STANDOFF
        self.map_resolution = MAP_RESOLUTION

    def get_project_dir(self, project_name: str = "active_project") -> Path:
        return get_active_project_dir(project_name)

    def create_run_dir(self, project_name: str = "active_project", timestamp: Optional[str] = None) -> Path:
        return get_run_dir(project_name, timestamp)


# Global singleton instance
config = Config()
