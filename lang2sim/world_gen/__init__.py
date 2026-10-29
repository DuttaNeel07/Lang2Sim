"""
World generation and spatial geometry solver package for Sim2Lang.
"""

from lang2sim.world_gen.catalog_loader import SceneCatalog, catalog
from lang2sim.world_gen.map_slicer import ZeroSlamMapSlicer, slice_map_from_layout
from lang2sim.world_gen.shapely_solver import ShapelySpatialSolver, solve_spatial_layout

__all__ = [
    "ShapelySpatialSolver",
    "solve_spatial_layout",
    "ZeroSlamMapSlicer",
    "slice_map_from_layout",
    "SceneCatalog",
    "catalog",
]
