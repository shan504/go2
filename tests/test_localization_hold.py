import sys
import unittest
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'go2_3d'))
from geometry import pose_matrix
from localization_hold import OdometryBudget


class HoldBudgetTests(unittest.TestCase):
    def test_unseeded_and_regressed_odometry_cannot_start_hold(self):
        budget = OdometryBudget()
        self.assertFalse(budget.observe(np.eye(4), 10))
        budget.reset(np.eye(4), 10)
        self.assertFalse(budget.observe(np.eye(4), 9))

    def test_round_trip_cannot_replenish_travel_or_rotation(self):
        budget = OdometryBudget()
        budget.reset(np.eye(4), 10)
        turn = np.deg2rad(40)/2
        pose = pose_matrix([0.6, 0, 0], [0, 0, np.sin(turn), np.cos(turn)])
        self.assertTrue(budget.observe(pose, 11))
        self.assertEqual(budget.exceeded(1.0, np.deg2rad(60)), '')
        budget.observe(np.eye(4), 12)
        self.assertAlmostEqual(budget.distance, 1.2)
        self.assertAlmostEqual(np.degrees(budget.angle), 80)
        self.assertEqual(budget.exceeded(1.0, np.deg2rad(90)), 'odometry_distance_limit')
        self.assertEqual(budget.exceeded(2.0, np.deg2rad(60)), 'odometry_angle_limit')

    def test_duplicate_tf_does_not_double_count_and_acceptance_resets(self):
        budget = OdometryBudget()
        budget.reset(np.eye(4), 10)
        pose = pose_matrix([0.3, 0, 0], [0, 0, 0, 1])
        budget.observe(pose, 11)
        budget.observe(pose, 11)
        self.assertAlmostEqual(budget.distance, 0.3)
        budget.reset(pose, 11)
        self.assertEqual(budget.distance, 0)
        self.assertEqual(budget.angle, 0)


if __name__ == '__main__':
    unittest.main()
