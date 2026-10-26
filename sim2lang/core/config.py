"""
Runtime and Workspace Configuration Manager
File: sim2lang/core/config.py
"""

import os
from pathlib import Path
from typing import Optional
from datetime import datetime

DEFAULT_LANG2SIM_ROOT = Path(os.path.expanduser("~/.lang2sim"))
PROJECTS_DIR = DEFAULT_LANG2SIM_ROOT / "projects"
CATALOG_DIR = Path(__file__).resolve().parent.parent.parent / "catalog"

# Pinned Container Image and Network Defaults
DEFAULT_SIM_IMAGE = "ghcr.io/duttaneel07/lang2sim-sim:0.1.0"
DEFAULT_CONTROL_PORT = 9090
DEFAULT_FOXGLOVE_PORT = 8765
DEFAULT_NOVNC_PORT = 6080

def ensure_directories():
    DEFAULT_LANG2SIM_ROOT.mkdir(parents=True, exist_ok=True)
    PROJECTS_DIR.mkdir(parents=True, exist_ok=True)

def get_projects_dir() -> Path:
    ensure_directories()
    return PROJECTS_DIR

def get_active_project_dir(project_name: str = "active_project") -> Path:
    ensure_directories()
    p_dir = PROJECTS_DIR / project_name
    p_dir.mkdir(parents=True, exist_ok=True)
    return p_dir

def get_run_dir(project_name: str = "active_project", timestamp: Optional[str] = None) -> Path:
    p_dir = get_active_project_dir(project_name)
    runs_dir = p_dir / "runs"
    runs_dir.mkdir(parents=True, exist_ok=True)
    if timestamp is None:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir = runs_dir / timestamp
    run_dir.mkdir(parents=True, exist_ok=True)
    return run_dir

def get_catalog_dir() -> Path:
    return CATALOG_DIR
