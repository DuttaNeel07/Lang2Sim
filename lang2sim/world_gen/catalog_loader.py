"""
Catalog Loader and Semantic Anchor Resolver for Sim2Lang.
File: lang2sim/world_gen/catalog_loader.py
"""

import json
from pathlib import Path
from typing import Any, Dict, List, Optional

from lang2sim.core.config import get_catalog_dir
from lang2sim.core.models import ObjectSpec, Pose2D, Vector3D


class SceneCatalog:
    """Interface to inspect pre-built scenes, semantic anchors, and catalog models."""

    def __init__(self, catalog_dir: Optional[Path] = None):
        self.catalog_dir = catalog_dir or get_catalog_dir()
        self.catalog_file = self.catalog_dir / "scene_catalog.json"
        self._data: Dict[str, Any] = self._load()

    def _load(self) -> Dict[str, Any]:
        if not self.catalog_file.exists():
            raise FileNotFoundError(f"Catalog file not found: {self.catalog_file}")
        with open(self.catalog_file, encoding="utf-8") as f:
            return json.load(f)

    def list_scenes(self) -> List[str]:
        """List all pre-built scene IDs (e.g. 'aws_small_warehouse', 'robotics_lab')."""
        return list(self._data.get("scenes", {}).keys())

    def get_scene(self, scene_id: str) -> Dict[str, Any]:
        """Retrieve full metadata for a scene."""
        scenes = self._data.get("scenes", {})
        if scene_id not in scenes:
            raise KeyError(f"Scene '{scene_id}' not in catalog. Available: {self.list_scenes()}")
        return scenes[scene_id]

    def get_scene_dimensions(self, scene_id: str) -> Vector3D:
        """Returns the 3D bounding dimensions of a scene."""
        scene = self.get_scene(scene_id)
        d = scene["dimensions"]
        return Vector3D(x=float(d["x"]), y=float(d["y"]), z=float(d["z"]))

    def resolve_anchor(self, scene_id: str, anchor_name: str) -> Pose2D:
        """
        Resolves a semantic target anchor (e.g., 'rack_5', 'dock') into an exact Pose2D.
        Eliminates Nav2 'Goal in collision' aborts.
        """
        scene = self.get_scene(scene_id)
        anchors = scene.get("anchors", {})
        if anchor_name not in anchors:
            raise KeyError(
                f"Anchor '{anchor_name}' not found in scene '{scene_id}'. "
                f"Available anchors: {list(anchors.keys())}"
            )
        raw_anchor = anchors[anchor_name]
        return Pose2D.from_anchor(raw_anchor)

    def list_models(self) -> List[str]:
        """List all physical object blueprints in the catalog."""
        return list(self._data.get("models", {}).keys())

    def get_model_spec(
        self, model_id: str, object_id: str, pose: Optional[Pose2D] = None
    ) -> ObjectSpec:
        """
        Builds a typed ObjectSpec from catalog blueprints, ensuring exact physical dimensions and mass.
        """
        models = self._data.get("models", {})
        if model_id not in models:
            raise KeyError(
                f"Model '{model_id}' not found in catalog. Available: {self.list_models()}"
            )
        m = models[model_id]
        dims = m["dimensions"]
        return ObjectSpec(
            id=object_id,
            model_name=model_id,
            dimensions=Vector3D(x=float(dims["x"]), y=float(dims["y"]), z=float(dims["z"])),
            pose=pose or Pose2D(x=0.0, y=0.0, yaw=0.0),
            is_static=bool(m.get("is_static", True)),
        )


# Global singleton instance
catalog = SceneCatalog()
