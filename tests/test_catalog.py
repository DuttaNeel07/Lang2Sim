"""
Unit Tests for Scene Catalog Loader and Pre-computed Maps.
File: tests/test_catalog.py
"""

import unittest

from lang2sim.world_gen.catalog_loader import catalog


class TestSceneCatalog(unittest.TestCase):
    """Test suite for the asset and scene catalog depot."""

    def test_list_scenes_and_models(self):
        """Verify scenes and models are indexed."""
        scenes = catalog.list_scenes()
        self.assertIn("aws_small_warehouse", scenes)
        self.assertIn("robotics_lab", scenes)

        models = catalog.list_models()
        self.assertIn("aws_warehouse_rack", models)
        self.assertIn("wooden_crate", models)
        self.assertIn("workbench", models)

    def test_get_scene_dimensions(self):
        """Verify 3D room dimensions."""
        dims = catalog.get_scene_dimensions("aws_small_warehouse")
        self.assertEqual(dims.x, 14.0)
        self.assertEqual(dims.y, 10.0)

    def test_resolve_semantic_anchor(self):
        """Verify anchor coordinates eliminate Nav2 goal in collision."""
        dock = catalog.resolve_anchor("aws_small_warehouse", "dock")
        self.assertEqual(dock.x, 0.0)
        self.assertEqual(dock.y, 0.0)

        rack_5 = catalog.resolve_anchor("aws_small_warehouse", "rack_5")
        self.assertEqual(rack_5.x, 7.5)
        self.assertEqual(rack_5.y, 3.0)

    def test_resolve_invalid_anchor_raises_error(self):
        """Verify non-existent anchor raises KeyError with available hints."""
        with self.assertRaises(KeyError):
            catalog.resolve_anchor("aws_small_warehouse", "non_existent_anchor")

    def test_get_model_spec(self):
        """Verify physical blueprint construction."""
        spec = catalog.get_model_spec("aws_warehouse_rack", "shelf_test")
        self.assertEqual(spec.id, "shelf_test")
        self.assertEqual(spec.dimensions.x, 2.8)
        self.assertEqual(spec.dimensions.y, 1.2)
        self.assertTrue(spec.is_static)

    def test_precomputed_map_files_exist(self):
        """Verify pre-computed map.yaml and map.pgm exist in catalog/maps/."""
        scene = catalog.get_scene("aws_small_warehouse")
        yaml_path = catalog.catalog_dir / "maps" / "aws_small_warehouse.yaml"
        pgm_path = catalog.catalog_dir / "maps" / "aws_small_warehouse.pgm"

        self.assertTrue(yaml_path.exists())
        self.assertTrue(pgm_path.exists())
        self.assertGreater(yaml_path.stat().st_size, 0)
        self.assertGreater(pgm_path.stat().st_size, 0)


if __name__ == "__main__":
    unittest.main()
