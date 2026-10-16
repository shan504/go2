"""Observed-grid preparation either succeeds completely or preserves the grid."""
from pathlib import Path
import sys
import unittest
from unittest.mock import patch
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'go2_3d'))
from geometry import ObservedGrid, line_cells


class GridTransactionTests(unittest.TestCase):
    def test_budget_failure_preserves_existing_grid_and_can_be_retried(self):
        grid = ObservedGrid(1.0, max_cells=3)
        grid.free.add((50, 50))
        original = grid.free.copy()
        points = np.array([[4., 0., 0.]])
        for operation in (grid.prepare_observation, grid.observe):
            with self.assertRaisesRegex(RuntimeError, 'Observed-space limit exceeded'):
                operation(points, [0., 0., 0.])
            self.assertEqual(grid.free, original)
        grid.max_cells = 5
        grid.observe(points, [0., 0., 0.])
        self.assertEqual(grid.free, original | {(0, 0), (1, 0), (2, 0), (3, 0)})

    def test_calculation_failure_after_a_ray_preserves_existing_grid(self):
        grid = ObservedGrid(1.0)
        grid.free.add((50, 50))
        original = grid.free.copy()
        points = np.array([[2., 0., 0.], [0., 3., 0.]])
        with patch('geometry.line_cells', side_effect=[
                line_cells((0, 0), (2, 0)), ValueError('invalid second ray')]):
            with self.assertRaisesRegex(ValueError, 'invalid second ray'):
                grid.observe(points, [0., 0., 0.])
        self.assertEqual(grid.free, original)

    def test_preparation_matches_existing_ray_geometry_without_mutation(self):
        grid = ObservedGrid(1.0)
        grid.free.update({(0, 0), (50, 50)})
        original = grid.free.copy()
        live_set = grid.free
        points = np.array([[4., 0., 0.], [0., 3., 0.], [-2., 0., 0.],
                           [0., -3., 0.], [2., 2., 1.]])
        expected = {(0, 0), (1, 0), (2, 0), (3, 0), (0, 1), (0, 2),
                    (-1, 0), (0, -1), (0, -2), (1, 1)}
        additions = grid.prepare_observation(points, [0., 0., 0.])
        self.assertEqual(additions, expected - original)
        self.assertEqual(grid.free, original)
        self.assertIs(grid.free, live_set)
        grid.observe(points, [0., 0., 0.])
        self.assertEqual(grid.free, original | expected)
        self.assertIs(grid.free, live_set)

    def test_duplicate_observations_only_budget_new_cells(self):
        grid = ObservedGrid(1.0, max_cells=3)
        points = np.array([[3., 0., 0.], [3., 0., 0.]])
        grid.observe(points, [0., 0., 0.])
        self.assertEqual(grid.free, {(0, 0), (1, 0), (2, 0)})
        self.assertEqual(grid.prepare_observation(points, [0., 0., 0.]), set())
        grid.observe(points, [0., 0., 0.])
        self.assertEqual(len(grid.free), 3)

    def test_nearest_obstacle_still_blocks_farther_returns(self):
        grid = ObservedGrid(1.0)
        points = np.array([[4., 0., 1.], [2., 0., 0.], [6., 0., -1.]])
        self.assertEqual(grid.prepare_observation(points, [0., 0., 0.]),
                         {(0, 0), (1, 0)})
        grid.observe(points, [0., 0., 0.])
        self.assertEqual(grid.free, {(0, 0), (1, 0)})


if __name__ == '__main__':
    unittest.main()
