"""Offline replay/journal contracts; controlled corrections are not SLAM proof."""
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np
import open3d as o3d
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'go2_3d'))
from geometry import pose_matrix, transform_points
from mapping_journal import MappingJournal, load_journal
from mapping_rebuild import rebuild
from kiss_replay import make_backend, replay, write_result
from map_library import fingerprints, validate_bundle
from navigation_grid import build_bundle
from test_3d import room


def pose(x=0., yaw=0.):
    return pose_matrix([x, 0., 0.], [0., 0., np.sin(yaw/2), np.cos(yaw/2)])


def points():
    rng = np.random.default_rng(720)
    return rng.uniform([1., -2., -.3], [3., 2., 1.], (400, 3))


def dataset(root):
    writer = MappingJournal(root, {'provenance': 'synthetic', 'recorded_backend': 'none'})
    for i, x in enumerate((0., 5., 7.)):
        writer.append((i+1)*1_000_000_000, points(), pose(x))
    return load_journal(root)


class CorrectionBackend:
    """Force a single known correction to test propagation/map consistency."""
    def __init__(self, fail=False):
        self.poses = []
        self.applied = False
        self.fail = fail
        self.flushed = False
        self.status = {'state': 'waiting'}

    def append(self, cloud, matrix):
        self.poses.append(matrix.copy())

    def detect_and_optimize(self):
        if len(self.poses) != 2 or self.applied:
            return None
        return SimpleNamespace(original_poses=tuple(self.poses),
                               poses=(self.poses[0], pose(4.)),
                               loop_edges=((0, 1),), status={'state': 'optimized'})

    def apply(self, proposal):
        if self.fail:
            raise RuntimeError('controlled apply failure')
        self.poses = list(proposal.poses)
        self.applied = True

    def flush(self):
        self.flushed = True


class JournalAndReplayTests(unittest.TestCase):
    def test_journal_removes_backend_frame_change_and_records_provenance(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)/'journal'
            writer = MappingJournal(root, {'provenance': 'synthetic', 'recorded_backend': 'kiss'})
            correction = pose(3., .2)
            writer.append(0, points(), pose())
            writer.append(1, points(), correction@pose(2.), correction)
            result = load_journal(root)
            np.testing.assert_allclose(result.observations[1].pose, pose(2.), atol=1e-12)
            self.assertEqual(result.metadata['provenance'], 'synthetic')
            self.assertTrue(result.metadata['frontend_feedback'])
            self.assertEqual(result.metadata['pose_convention'], 'backend_corrections_removed_gicp')
            before = result.sha256
            with self.assertRaises(FileExistsError):
                MappingJournal(root)
            with self.assertRaises(ValueError):
                writer.append(1, points(), pose(2.))
            self.assertEqual(load_journal(root).sha256, before)

    def test_invalid_point_pose_stamp_cannot_append_partial_frame(self):
        with tempfile.TemporaryDirectory() as tmp:
            writer = MappingJournal(Path(tmp)/'journal')
            for stamp, cloud, matrix in ((False, points(), pose()), (-1, points(), pose()),
                                          (1., points(), pose()), (1, np.full((3,3), np.nan), pose()),
                                          (1, points(), np.zeros((4,4))), (1, np.empty((0,3)), pose())):
                with self.subTest(stamp=stamp):
                    with self.assertRaises(ValueError):
                        writer.append(stamp, cloud, matrix)
            self.assertEqual(list(writer.frames_directory.iterdir()), [])
            self.assertEqual(writer.count, 0)

    def test_legacy_archive_requires_explicit_assumption_and_failed_archive_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)/'journal'
            dataset(root)
            (root/'session.json').unlink()
            with self.assertRaisesRegex(ValueError, 'explicit'):
                load_journal(root)
            result = load_journal(root, assume_uncorrected_gicp=True)
            self.assertEqual(result.metadata['provenance'], 'unknown')
            self.assertTrue(result.metadata['legacy_pose_convention_assumed_by_user'])
            (root/'FAILED.json').write_text('{}')
            with self.assertRaisesRegex(ValueError, 'failed'):
                load_journal(root, assume_uncorrected_gicp=True)

    def test_replay_applies_correction_once_to_future_input_and_rebuilds_every_scan(self):
        with tempfile.TemporaryDirectory() as tmp:
            data = dataset(Path(tmp)/'journal')
            originals = [o.pose.copy() for o in data.observations]
            backend = CorrectionBackend()
            result = replay(data, backend, (.06,.15,.1))
            report, events, observations, store, grid = result
            self.assertTrue(backend.flushed)
            self.assertEqual([o.pose[0,3] for o in observations], [0.,4.,6.])
            self.assertEqual([p[0,3] for p in backend.poses], [0.,4.,6.])
            reference, ref_grid = rebuild(data.observations, [pose(),pose(4.),pose(6.)], .06,.15,.1)
            np.testing.assert_allclose(store.snapshot(), reference.snapshot())
            self.assertEqual(grid.free, ref_grid.free)
            self.assertEqual(report['applied_optimizations'], 1)
            self.assertEqual(report['loop_triggered_map_rebuilds'], 1)
            self.assertFalse(report['real_world_loop_success_verified'])
            self.assertEqual(report['endpoint_measurement'], 'start_end_separation_not_ground_truth_error')
            self.assertEqual(sum(event['map_rebuilt'] for event in events), 1)
            for saved, original in zip(data.observations, originals):
                np.testing.assert_array_equal(saved.pose, original)
            out = Path(tmp)/'output'
            write_result(out, data, result, {})
            saved = np.asarray(o3d.io.read_point_cloud(str(out/'map.pcd')).points)
            np.testing.assert_allclose(saved, store.snapshot(), atol=5e-7)
            with np.load(out/'trajectory.npz') as archived:
                np.testing.assert_array_equal(archived['raw_poses'], originals)
                np.testing.assert_array_equal(archived['optimized_poses'], [o.pose for o in observations])
            with self.assertRaises(FileExistsError):
                write_result(out, data, result, {})

    def test_real_legacy_no_loop_does_not_claim_success(self):
        with tempfile.TemporaryDirectory() as tmp:
            data = dataset(Path(tmp)/'journal')
            backend, config = make_backend('lidar')
            result = replay(data, backend, (.06,.15,.1), same_place_confirmed=False)
            self.assertEqual(result[0]['applied_optimizations'], 0)
            self.assertIn('NOT_VERIFIED', result[0]['closure_evidence'])
            np.testing.assert_allclose([o.pose for o in result[2]], [o.pose for o in data.observations])
            self.assertTrue(result[0]['final_map_rebuilt'])

    def test_apply_failure_does_not_report_success(self):
        with tempfile.TemporaryDirectory() as tmp:
            data = dataset(Path(tmp)/'journal')
            with self.assertRaisesRegex(RuntimeError, 'apply failure'):
                replay(data, CorrectionBackend(fail=True))

    def test_offline_outputs_export_two_compatible_maps_and_reload_without_drift(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            for name, anchor in (('field_a', pose()), ('field_b', pose(4., .2))):
                writer = MappingJournal(root/(name+'_journal'), {'provenance': 'synthetic'})
                for index, x in enumerate((0., .5, 1.)):
                    original = pose(x)
                    local = transform_points(room(), np.linalg.inv(original))
                    writer.append(index, local, anchor@original)
                data = load_journal(writer.directory)
                backend, settings = make_backend('lidar')
                result = replay(data, backend)
                output = root/name
                report = write_result(output, data, result, settings, export_bundles=True)
                saved = validate_bundle(output/report['navigation_bundle'])
                source_fingerprints = fingerprints(saved)
                metadata = yaml.safe_load((saved/'metadata.yaml').read_text())
                np.testing.assert_allclose(metadata['map_start_pose'],
                    np.asarray(metadata['map_from_source_map'])@anchor, atol=1e-10)
                self.assertAlmostEqual(metadata['body_height_above_ground'], .3, delta=.02)
                for number in (1, 2):
                    loaded = root/(name+'_runtime_'+str(number))
                    build_bundle(output/'map_bundles', saved, loaded)
                    reloaded = yaml.safe_load((loaded/'metadata.yaml').read_text())
                    np.testing.assert_allclose(reloaded['map_start_pose'], metadata['map_start_pose'], atol=1e-10)
                    self.assertAlmostEqual(reloaded['body_height_above_ground'], metadata['body_height_above_ground'])
                self.assertEqual(fingerprints(saved), source_fingerprints)

    def test_forged_correction_or_native_odom_metadata_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)/'journal'
            dataset(root)
            meta = json.loads((root/'session.json').read_text())
            meta['pose_source'] = 'native_odom'
            (root/'session.json').write_text(json.dumps(meta))
            with self.assertRaisesRegex(ValueError, 'accepted GICP'):
                load_journal(root)
            meta['pose_source'] = 'gicp'
            (root/'session.json').write_text(json.dumps(meta))
            np.savez(root/'keyframes/1000000000.npz', stamp_ns=np.int64(1000000000),
                     points=points(), pose=pose(), mapped_pose=pose(5.), map_from_frontend=pose())
            with self.assertRaisesRegex(ValueError, 'inconsistent frame correction'):
                load_journal(root)


if __name__ == '__main__':
    unittest.main()
