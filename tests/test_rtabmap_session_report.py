"""Synthetic report checks; no camera recognition or robot success is implied."""
import json
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'go2_3d'))
from geometry import pose_matrix
from rtabmap_session_report import analyze, endpoint_error, load_session, seam_distances


def pose(x=0., yaw=0.):
    return pose_matrix([x, 0., 0.], [0., 0., np.sin(yaw/2), np.cos(yaw/2)])


def data():
    points = np.random.default_rng(923).uniform([-.2, -.2, .1], [.2, .2, 1.], (200, 3))
    frames = [(10, pose(), points), (20, pose(5.), points), (30, pose(3.), points)]
    raw = {stamp: matrix for stamp, matrix, _ in frames}
    graph = {'graph_revision': 3, 'optimized_poses': {'10': pose().tolist(), '30': pose().tolist()},
             'node_stamps': {'1': 10, '2': 30}, 'omitted_retained_anchors': [],
             'links': [{'from_id': 1, 'to_id': 2, 'type': 1}]}
    return frames, raw, graph


class SessionReportTests(unittest.TestCase):
    def test_three_metre_drift_and_seam_measurement_without_fitting(self):
        events = [dict(event='loop_candidate', highest_hypothesis_id=1, visual_inliers=32),
                  dict(event='graph_applied', graph_revision=3, map_rebuilt=True,
                       global_loop_constraints_added=[[1, 2]], optimization_iterations=4)]
        report = analyze(*data(), events=events, roi=[-1., -1., -1., 5., 1., 2.])
        self.assertAlmostEqual(report['raw_gicp_start_end']['translation_m'], 3.)
        self.assertAlmostEqual(report['optimized_start_end']['translation_m'], 0.)
        self.assertGreater(report['fixed_roi_seam']['raw_gicp']['median_m'], 2.)
        self.assertAlmostEqual(report['fixed_roi_seam']['optimized']['p95_m'], 0.)
        self.assertEqual(report['fixed_roi_seam']['optimized']['coverage_within_threshold'], [1., 1.])
        self.assertFalse(report['fixed_roi_seam']['optimized']['registration_fitted'])
        self.assertTrue(report['latest_graph_apply_recorded'])
        self.assertEqual(report['map_rebuild_events'], 1)
        self.assertEqual(len(report['recent_loop_and_map_events']), 2)
        self.assertEqual(report['global_loop_constraint_count'], 1)
        self.assertFalse(report['real_world_loop_success_verified'])

    def test_zero_endpoint_error_without_global_constraint_is_not_closure(self):
        frames, raw, graph = data()
        graph['links'][0]['type'] = 0
        report = analyze(frames, raw, graph, same_place_confirmed=True)
        self.assertEqual(report['optimized_start_end']['translation_m'], 0.)
        self.assertIn('NOT_VERIFIED', report['closure_evidence'])
        self.assertEqual(report['global_loop_constraint_count'], 0)
        self.assertFalse(report['real_world_loop_success_verified'])

    def test_missing_anchor_source_is_rejected(self):
        frames, raw, graph = data()
        del raw[30]
        with self.assertRaisesRegex(ValueError, 'exact source pose'):
            analyze(frames, raw, graph)

    def test_empty_roi_does_not_report_perfect_zero_seam(self):
        first = data()[0][0][2]
        report = seam_distances(first, first, [10., 10., 10., 11., 11., 11.])
        self.assertEqual(report['status'], 'insufficient_fixed_structure_points')
        self.assertIsNone(report['p95_m'])
        self.assertIsNone(report['coverage_within_threshold'])

    def test_unanchored_start_and_tail_are_identified(self):
        frames, raw, graph = data()
        graph['optimized_poses'] = {'20': pose(4.).tolist()}
        report = analyze(frames, raw, graph)
        self.assertFalse(report['selected_frames_within_graph_anchors'])

    def test_orientation_and_selection(self):
        self.assertAlmostEqual(endpoint_error(pose(), pose(3., np.pi/2))['rotation_deg'], 90.)
        with self.assertRaisesRegex(ValueError, 'ordered'):
            analyze(*data(), first_index=2, last_index=0)

    def test_persisted_session_load_and_conflicting_keyframe_rejection(self):
        frames, raw, graph = data()
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root/'keyframes').mkdir()
            (root/'graph_latest.json').write_text(json.dumps(graph))
            (root/'gicp_poses.jsonl').write_text('\n'.join(json.dumps(dict(stamp_ns=stamp, pose=matrix.tolist()))
                                                        for stamp, matrix in raw.items()))
            for stamp, matrix, points in frames:
                np.savez(root/'keyframes'/f'{stamp}.npz', stamp_ns=np.int64(stamp), points=points, pose=matrix)
            loaded = load_session(root)
            self.assertEqual(analyze(*loaded)['keyframe_count'], 3)
            np.savez(root/'keyframes'/'30.npz', stamp_ns=np.int64(30), points=frames[-1][2], pose=pose(9.))
            with self.assertRaisesRegex(ValueError, 'matching source pose'):
                load_session(root)


if __name__ == '__main__':
    unittest.main()
