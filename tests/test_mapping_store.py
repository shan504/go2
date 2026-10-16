import sys
import threading
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'go2_3d'))
from mapping_store import MappingStore, _aggregate


class MappingStoreTests(unittest.TestCase):
    def add(self, store, points):
        update = store.prepare(points)
        store.commit(update)
        return update

    def test_weighted_aggregation_of_unsorted_repeated_voxels(self):
        rng = np.random.default_rng(31)
        keys = rng.integers(-3, 4, size=(700, 3))
        points = (keys + rng.uniform(0.01, 0.99, size=keys.shape)) * 0.02
        weights = rng.integers(1, 9, size=len(keys), dtype=np.int64)
        sums = points * weights[:, None]
        expected = {}
        for key, total, weight in zip(map(tuple, keys), sums, weights):
            if key not in expected:
                expected[key] = (total.copy(), int(weight))
            else:
                old_total, old_weight = expected[key]
                expected[key] = (old_total + total, old_weight + int(weight))
        unique, totals, counts, centers = _aggregate(keys, sums, weights, 0.02)
        ordered_keys = sorted(expected)
        np.testing.assert_array_equal(unique, ordered_keys)
        np.testing.assert_array_equal(counts, [expected[key][1] for key in ordered_keys])
        np.testing.assert_allclose(totals, [expected[key][0] for key in ordered_keys], atol=1e-15)
        np.testing.assert_allclose(centers, [expected[key][0]/expected[key][1] for key in ordered_keys],
                                   atol=1e-15)

    def test_empty_aggregation_has_typed_three_column_outputs(self):
        keys, totals, counts, centers = _aggregate(
            np.empty((0, 3), dtype=np.int64), np.empty((0, 3)),
            np.empty(0, dtype=np.int64), 0.02)
        self.assertEqual(keys.shape, (0, 3))
        self.assertEqual(keys.dtype, np.int64)
        self.assertEqual(totals.shape, (0, 3))
        self.assertEqual(counts.shape, (0,))
        self.assertEqual(counts.dtype, np.int64)
        self.assertEqual(centers.shape, (0, 3))

    def test_negative_coordinates_and_tile_boundaries_have_one_owner(self):
        store = MappingStore(0.02, 0.15)
        coordinates = np.array([-6.01, -6.0, -0.01, 0.0, 5.99, 6.0, 6.01])
        points = np.column_stack((coordinates, np.full(len(coordinates), 0.01),
                                  np.full(len(coordinates), 0.01)))
        self.add(store, points)
        expected = len(np.unique(np.floor(points / 0.02).astype(np.int64), axis=0))
        self.assertEqual(store.count, expected)
        self.add(store, points)
        self.assertEqual(store.count, expected)
        self.assertEqual(set(store._tiles), {(-2, 0, 0), (-1, 0, 0), (0, 0, 0), (1, 0, 0)})
        np.testing.assert_allclose(np.sort(store.snapshot()[:, 0]),
                                   [-6.01, -6.0, -0.01, 0.0, 5.99, 6.005])

    def test_revisit_keeps_fixed_voxel_and_weighted_centroid(self):
        store = MappingStore(0.02, 0.15)
        self.add(store, [[0.001, 0.001, 0.001], [0.019, 0.019, 0.019]])
        self.add(store, [[6.01, 0.01, 0.01]])
        untouched = store._tiles[(1, 0, 0)]
        self.add(store, [[0.007, 0.007, 0.007]])
        self.assertEqual(store.count, 2)
        self.assertIs(store._tiles[(1, 0, 0)], untouched)
        np.testing.assert_allclose(store._tiles[(0, 0, 0)].fine, [[0.009, 0.009, 0.009]])
        np.testing.assert_allclose(store._tiles[(0, 0, 0)].weights, [3])

    def test_voxel_representative_cannot_drift_into_another_tile(self):
        store = MappingStore(0.02, 0.15)
        edge = np.nextafter(6.0, -np.inf)
        point = np.array([[edge, 0.01, 0.01]])
        for _ in range(8):
            self.add(store, np.repeat(point, 100, axis=0))
        self.assertEqual(store.count, 1)
        self.assertEqual(set(store._tiles), {(0, 0, 0)})
        self.assertLess(store.snapshot()[0, 0], 6.0)
        self.assertEqual(len(store.registration_snapshot()), 1)

    def test_snapshots_are_cached_readonly_and_stable_after_commit(self):
        store = MappingStore(0.06, 0.15)
        self.add(store, [[0.01, 0.01, 0.01]])
        old_fine, old_coarse = store.snapshot(), store.registration_snapshot()
        self.assertIs(store.snapshot(), old_fine)
        self.assertIs(store.registration_snapshot(), old_coarse)
        with self.assertRaises(ValueError):
            old_fine[0, 0] = 12
        self.add(store, [[6.01, 0.01, 0.01]])
        self.assertIsNot(store.snapshot(), old_fine)
        self.assertEqual(len(old_fine), 1)
        self.assertEqual(len(old_coarse), 1)
        self.assertEqual(len(store.snapshot()), 2)

    def test_nearby_excludes_far_tiles_and_filters_spherical_distance(self):
        store = MappingStore(0.02, 0.15)
        self.add(store, [[-0.3, 0.01, 0.01], [0.01, 0.01, 0.01],
                         [0.9, 0.9, 0.01], [1.1, 0.01, 0.01], [6000.01, 0.01, 0.01]])
        class TrackingTiles(dict):
            def get(self, key, default=None):
                visited.append(key)
                return super().get(key, default)
        visited = []
        store._tiles = TrackingTiles(store._tiles)
        nearby = store.nearby([0, 0, 0], 1.0)
        self.assertEqual(len(nearby), 2)
        self.assertNotIn((1000, 0, 0), visited)
        self.assertFalse(nearby.flags.writeable)
        self.assertLess(len(visited), 10)

    def test_concurrent_old_snapshot_cannot_replace_new_revision_cache(self):
        store = MappingStore(0.02, 0.15)
        self.add(store, [[0.01, 0.01, 0.01]])
        update = store.prepare([[6.01, 0.01, 0.01]])
        started, resume = threading.Event(), threading.Event()
        stack = np.vstack
        results = []
        def delayed_stack(arrays):
            if threading.current_thread().name == 'old-snapshot':
                started.set()
                if not resume.wait(timeout=3):
                    raise RuntimeError('snapshot test timed out')
            return stack(arrays)
        worker = threading.Thread(target=lambda: results.append(store.snapshot()),
                                  name='old-snapshot')
        with patch('mapping_store.np.vstack', side_effect=delayed_stack):
            worker.start()
            try:
                self.assertTrue(started.wait(timeout=3))
                store.commit(update)
                current = store.snapshot()
            finally:
                resume.set()
                worker.join(timeout=3)
        self.assertFalse(worker.is_alive())
        self.assertEqual(len(results[0]), 1)
        self.assertEqual(len(current), 2)
        self.assertIs(store.snapshot(), current)

    def test_failure_and_stale_update_do_not_partially_commit(self):
        store = MappingStore(0.02, 0.15)
        self.add(store, [[0.01, 0.01, 0.01]])
        old = store.snapshot()
        build = store._build_tile
        calls = 0
        def fail_second(*args):
            nonlocal calls
            calls += 1
            if calls == 2:
                raise RuntimeError('controlled tile failure')
            return build(*args)
        with patch.object(store, '_build_tile', side_effect=fail_second):
            with self.assertRaisesRegex(RuntimeError, 'controlled tile failure'):
                store.prepare([[0.03, 0.01, 0.01], [6.01, 0.01, 0.01]])
        self.assertIs(store.snapshot(), old)
        self.assertEqual(store.count, 1)
        self.assertEqual(store.revision, 1)
        first = store.prepare([[6.01, 0.01, 0.01]])
        stale = store.prepare([[12.01, 0.01, 0.01]])
        store.commit(first)
        current = store.snapshot()
        with self.assertRaisesRegex(ValueError, 'stale'):
            store.commit(stale)
        self.assertIs(store.snapshot(), current)
        self.assertEqual(store.count, 2)
        with self.assertRaisesRegex(ValueError, 'another store'):
            MappingStore(0.02, 0.15).commit(first)

    def test_grows_past_old_million_point_limit_without_coarsening_or_eviction(self):
        store = MappingStore(0.02, 0.15)
        index = np.arange(1_000_000)
        points = np.column_stack(((index % 1000 + 0.25) * 0.02,
                                  (index // 1000 + 0.25) * 0.02,
                                  np.full(len(index), 0.005)))
        self.add(store, points)
        self.assertEqual(store.count, 1_000_000)
        self.add(store, [[24.005, 0.005, 0.005]])
        self.assertEqual(store.count, 1_000_001)
        self.assertEqual(len(store.snapshot()), 1_000_001)
        self.assertEqual(store.map_voxel_size, 0.02)
        self.assertLess(len(store.registration_snapshot()), store.count)
        self.assertEqual(len(store.nearby([0.005, 0.005, 0.005], 0.2)), 1)
        self.assertEqual(len(store.nearby([24.005, 0.005, 0.005], 0.1)), 1)

    def test_rejects_invalid_input_and_accepts_empty_updates(self):
        for values in ((0, 0.15), (0.02, float('nan')), (0.07, 0.15)):
            with self.assertRaises(ValueError):
                MappingStore(*values)
        store = MappingStore(0.02, 0.15)
        self.add(store, [])
        self.assertEqual(store.snapshot().shape, (0, 3))
        for points in ([[float('nan'), 0, 0]], [[float('inf'), 0, 0]],
                       [[1e100, 0, 0]], [1, 2, 3]):
            with self.assertRaises(ValueError):
                store.prepare(points)
        self.assertEqual(store.count, 0)
        for center, radius in (([0, 0], 1), ([0, 0, 0], 0), ([0, 0, 0], float('inf'))):
            with self.assertRaises(ValueError):
                store.nearby(center, radius)


if __name__ == '__main__':
    unittest.main()
