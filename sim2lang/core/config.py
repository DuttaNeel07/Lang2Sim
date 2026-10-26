"""
Runtime and Workspace Configuration Manager
File: sim2lang/core/config.py
"""

import os
from pathlib import Path

DEFAULT_LANG2SIM_ROOT = Path(os.path.expanduser("~/.lang2sim"))
PROJECTS_DIR = DEFAULT_LANG2SIM_ROOT / "projects"
CATALOG_DIR = Path(__file__).resolve().parent.parent.parent / "catalog"

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

def get_catalog_dir() -> Path:
    return CATALOG_DIR
