"""Real geometry recapture checks; these are not robot/field acceptance."""
import math
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'go2_3d'))
from geometry import pose_matrix, rotation_angle, transform_points
from relocalization import RecoveryConfig, Relocalizer, _hypothesis_scores
from test_3d import room


def pose(x=0., y=0., yaw=0., z=0.):
    return pose_matrix([x, y, z], [0, 0, math.sin(yaw/2), math.cos(yaw/2)])


class RelocalizationTests(unittest.TestCase):
    def test_batched_seed_scores_equal_scalar_scores_for_every_hypothesis(self):
        source = room()[::14]
        for origin in (pose(), pose(100000, -200000, .63, 1.2)):
            world = transform_points(room(), origin)
            recovery = Relocalizer(world)
            hypotheses = recovery._hypotheses(origin @ pose(.8, -.3, .05))
            self.assertEqual(len(hypotheses), 245)
            native = _hypothesis_scores(world, source, hypotheses)
            scalar = _hypothesis_scores(world, source, hypotheses, native=False)
            np.testing.assert_allclose(native, scalar, atol=1e-10, rtol=1e-10)

    def test_recaptures_offset_after_three_fresh_moving_scans_in_arbitrary_map_frames(self):
        for frame in (pose(), pose(102, -37, .63, 1.2)):
            with self.subTest(frame=frame[:3, 3].tolist()):
                world = transform_points(room(), frame)
                original = world.copy()
                world.setflags(write=False)
                recovery = Relocalizer(world)
                trusted = frame @ pose(1.35, -.55, math.radians(8))
                for index in range(3):
                    odom = pose(.25+index*.08, -.15, .05+index*.01)
                    truth = frame @ odom
                    scan = transform_points(world, np.linalg.inv(truth))
                    before = scan.copy()
                    result = recovery.observe(scan, odom, trusted, 10+index*.5, 10.02+index*.5)
                    self.assertEqual(result.accepted, index == 2, result)
                    self.assertEqual(result.confirmations, index+1)
                    np.testing.assert_array_equal(scan, before)
                np.testing.assert_allclose(result.pose, truth, atol=.045)
                np.testing.assert_allclose(result.correction, frame, atol=.045)
                self.assertFalse(world.flags.writeable)
                np.testing.assert_array_equal(world, original)

    def test_duplicate_stale_and_too_fast_scans_cannot_supply_confirmation(self):
        world = room()
        recovery = Relocalizer(world)
        trusted = pose(.8, -.3, math.radians(5))
        first = recovery.observe(world, pose(), trusted, 10., 10.01)
        self.assertEqual(first.confirmations, 1, first)
        duplicate = recovery.observe(world, pose(), trusted, 10., 10.02)
        self.assertFalse(duplicate.accepted)
        self.assertEqual(duplicate.confirmations, 1)
        stale = recovery.observe(world, pose(), trusted, 10.5, 12.)
        self.assertFalse(stale.accepted)
        self.assertEqual(stale.confirmations, 0)
        restarted = recovery.observe(world, pose(), trusted, 12., 12.01)
        self.assertEqual(restarted.confirmations, 1)
        fast = recovery.observe(world, pose(), trusted, 12.05, 12.06)
        self.assertEqual(fast.confirmations, 1)
        self.assertFalse(fast.accepted)
        second = recovery.observe(world, pose(), trusted, 12.5, 12.51)
        self.assertEqual(second.confirmations, 2)
        self.assertTrue(recovery.observe(world, pose(), trusted, 13., 13.01).accepted)

    def test_unknown_or_planar_geometry_stays_unlocalized(self):
        rng = np.random.default_rng(14)
        floor = np.c_[rng.uniform(-8, 8, (3000, 2)), np.full(3000, -.3)]
        for world, scan in ((floor, floor), (room(), room()+[12, 8, 0])):
            with self.subTest(planar=world is floor):
                recovery = Relocalizer(world)
                for index in range(3):
                    result = recovery.observe(scan, pose(), pose(), 20+index*.5, 20.01+index*.5)
                    self.assertFalse(result.accepted)
                    self.assertEqual(result.confirmations, 0)

    def test_large_global_relocalization_is_never_attempted(self):
        world = room()
        recovery = Relocalizer(world)
        for index in range(3):
            # Identical attractive scene exists ten metres away from the
            # trusted prediction; the local recovery may not jump to it.
            result = recovery.observe(world, pose(10, 0), pose(), 30+index*.5, 30.01+index*.5)
            self.assertFalse(result.accepted)

    def test_identical_neighbouring_rows_cannot_confirm_a_false_jump(self):
        rng = np.random.default_rng(35)
        parts = [np.c_[rng.uniform(-.4, .4, 1000), rng.uniform(-4, 4, 1000),
                       np.full(1000, -.3)]]
        for x in (-.4, .4):
            parts.append(np.c_[np.full(900, x), rng.uniform(-4, 4, 900),
                               rng.uniform(-.3, 2.3, 900)])
        parts.append(np.c_[rng.uniform(-.4, .4, 600), np.full(600, 4),
                           rng.uniform(-.3, 2.3, 600)])
        tile = np.vstack(parts)
        world = np.vstack([tile+[x, 0, 0] for x in (-3, -1.5, 0, 1.5, 3)])
        recovery = Relocalizer(world)
        # Each tile is individually observable, and each can match perfectly.
        # Distinct positions within the 2m search budget are the ambiguity.
        for index in range(3):
            result = recovery.observe(tile, pose(), pose(), 5+index*.5, 5.01+index*.5)
            self.assertFalse(result.accepted)
            self.assertEqual(result.confirmations, 0)
            self.assertIn('ambiguous', result.reason)

    def test_inconsistent_motion_and_expired_confirmations_start_over(self):
        world = room()
        recovery = Relocalizer(world)
        trusted = pose(.8, -.3, .05)
        first = recovery.observe(world, pose(), trusted, 1, 1.01)
        self.assertEqual(first.confirmations, 1)
        # Both views fit the map, but their measured movement contradicts
        # unchanged odometry. They cannot confirm the same correction.
        displaced = transform_points(world, np.linalg.inv(pose(.7, 0)))
        changed = recovery.observe(displaced, pose(), trusted, 1.5, 1.51)
        self.assertEqual(changed.confirmations, 1)
        self.assertFalse(changed.accepted)
        self.assertIn('changed between scans', changed.reason)
        expired = recovery.observe(displaced, pose(), trusted, 5, 5.01)
        self.assertEqual(expired.confirmations, 1)
        self.assertFalse(expired.accepted)

    def test_reset_or_changed_trusted_correction_starts_a_new_session(self):
        world = room()
        recovery = Relocalizer(world)
        trusted = pose(.8, -.3, .05)
        self.assertEqual(recovery.observe(world, pose(), trusted, 1, 1.01).confirmations, 1)
        self.assertEqual(recovery.observe(world, pose(), trusted, 1.5, 1.51).confirmations, 2)
        changed = recovery.observe(world, pose(), pose(.7, -.2, .04), 2, 2.01)
        self.assertEqual(changed.confirmations, 1)
        recovery.reset()
        self.assertEqual(recovery.observe(world, pose(), trusted, 2.5, 2.51).confirmations, 1)

    def test_invalid_inputs_and_unbounded_configurations_are_rejected(self):
        for kwargs in (dict(confirmations=1), dict(candidate_count=1), dict(candidate_count=20),
                       dict(search_step=.001), dict(max_translation=float('inf')),
                       dict(confirmation_timeout=.01)):
            with self.assertRaises(ValueError):
                RecoveryConfig(**kwargs)
        world = room()
        recovery = Relocalizer(world)
        bad = world.copy()
        bad[0, 0] = float('nan')
        with self.assertRaises(ValueError):
            recovery.observe(bad, pose(), pose(), 1, 1)
        with self.assertRaises(ValueError):
            Relocalizer(bad)
        invalid_pose = pose()
        invalid_pose[0, 0] = 2
        with self.assertRaises(ValueError):
            recovery.observe(world, invalid_pose, pose(), 1, 1)
        with self.assertRaises(ValueError):
            recovery.observe(world, pose(), pose(), float('nan'), 1)


if __name__ == '__main__':
    unittest.main()
