import json
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'go2_3d'))
from navigation_diagnostics import (ControlHistory, MAX_GRID_CELLS, MAX_STATUS_CHARS,
                                    yaw_from_quaternion)


class NavigationDiagnosticsTests(unittest.TestCase):
    def test_forward_zero_and_reversal_are_separate_from_actual_displacement(self):
        history = ControlHistory()
        for now, velocity in enumerate((0.3, 0.0, -0.2, 0.0, 0.3), 1):
            history.command('cmd_vel_nav', (velocity, 0.0, 0.1), now)
            history.command('cmd_vel', (velocity/2, 0.0, 0.05), now)
            history.odom('odom', (now % 2)*0.02, 0.0, 0.0, (0.0, 0.0, 0.0), now)
        result = history.snapshot(5)
        commands = result['commands']['cmd_vel_nav']
        self.assertEqual(commands['forward'], 2)
        self.assertEqual(commands['reverse'], 1)
        self.assertEqual(commands['vx_sign_changes'], 2)
        self.assertEqual(commands['yaw_sign_changes'], 0)
        self.assertAlmostEqual(commands['mean_abs'][0], 0.16)
        self.assertAlmostEqual(commands['mean_abs'][2], 0.1)
        self.assertEqual(commands['min'], [-0.2, 0.0, 0.1])
        self.assertEqual(commands['max'], [0.3, 0.0, 0.1])
        self.assertEqual(result['commands']['cmd_vel']['max'][0], 0.15)
        self.assertAlmostEqual(result['odometry']['net_displacement_m'], 0.0)
        self.assertAlmostEqual(result['odometry']['travelled_m'], 0.08)
        self.assertFalse(result['latest']['localization_valid']['available'])

    def test_yaw_reversals_ignore_zero_and_invalid_commands_in_mean_amplitude(self):
        history = ControlHistory()
        velocities = ((0.3, 0.1, 0.1), (0, 0, 0), (math.nan, 0.2, -0.2),
                      (-0.3, -0.1, -0.1), (0, 0, 0), (0.3, 0.1, 0.2))
        for now, velocity in enumerate(velocities):
            history.command('cmd_vel_nav', velocity, now)
        commands = history.snapshot(5)['commands']['cmd_vel_nav']
        self.assertEqual(commands['invalid_samples'], 1)
        self.assertEqual(commands['yaw_sign_changes'], 2)
        for actual, expected in zip(commands['mean_abs'], (0.18, 0.06, 0.08)):
            self.assertAlmostEqual(actual, expected)

    def test_history_is_time_bounded_and_high_rate_cap_is_visible(self):
        history = ControlHistory(window_seconds=20, max_samples=3)
        for now in range(6):
            history.command('cmd_vel', (now, 0, 0), now)
            history.odom('odom', now, 0, 0, (0, 0, 0), now)
        result = history.snapshot(5)
        self.assertEqual(len(history.commands['cmd_vel']), 3)
        self.assertEqual(len(history.odometry), 3)
        self.assertTrue(result['commands']['cmd_vel']['history_capped'])
        self.assertTrue(result['odometry']['history_capped'])
        result = history.snapshot(26)
        self.assertEqual(result['commands']['cmd_vel']['samples'], 0)
        self.assertEqual(result['odometry']['samples'], 0)
        self.assertFalse(result['commands']['cmd_vel']['fresh'])
        self.assertFalse(result['commands']['cmd_vel']['history_capped'])
        self.assertEqual(result['commands']['cmd_vel']['receipt_age_s'], 21)
        self.assertEqual(result['odometry']['reason'], 'odom receipt stale')

    def test_nonfinite_command_and_invalid_pose_never_become_json_nan(self):
        history = ControlHistory()
        history.command('cmd_vel', (math.nan, 0.0, 0.0), 1)
        history.odom('odom', math.inf, 0, 0, (math.nan, 0, 0), 1)
        result = history.snapshot(1)
        self.assertEqual(result['commands']['cmd_vel']['invalid_samples'], 1)
        self.assertIsNone(result['commands']['cmd_vel']['last'])
        self.assertIsNone(result['commands']['cmd_vel']['mean_abs'])
        self.assertEqual(result['commands']['cmd_vel']['yaw_sign_changes'], 0)
        self.assertEqual(result['odometry']['reason'], 'nonfinite odom pose')
        json.dumps(result, allow_nan=False)

    def test_yaw_wrap_and_odom_frame_changes_are_not_false_motion(self):
        history = ControlHistory()
        history.odom('odom', 0, 0, math.radians(179), (0, 0, 0), 1)
        history.odom('odom', 0, 0, math.radians(-179), (0, 0, 0), 2)
        self.assertAlmostEqual(history.snapshot(2)['odometry']['yaw_change_rad'], math.radians(2))
        history.odom('map', 100, 100, 0, (0, 0, 0), 3)
        result = history.snapshot(3)
        self.assertEqual(result['odometry']['reason'], 'odom frame is not odom')
        self.assertEqual(result['odometry']['samples'], 0)
        history.odom('odom', 0, 0, 0, (0, 0, 0), 4)
        self.assertIsNone(history.snapshot(4)['odometry']['net_displacement_m'])
        self.assertAlmostEqual(yaw_from_quaternion((0, 0, 2, 2)), math.pi/2)
        self.assertIsNone(yaw_from_quaternion((0, 0, 0, 0)))

    def test_status_drives_reporting_but_does_not_grant_motion(self):
        history = ControlHistory()
        self.assertFalse(history.executing)
        self.assertFalse(history.status('operator', 'route current=- queued=3 completed=0 paused=False started=True', 1))
        self.assertFalse(history.executing)
        self.assertFalse(history.status('operator', 'route current=1 queued=2 completed=0 paused=False started=True', 2))
        self.assertTrue(history.executing)
        failure = 'Nav2 goal completed: action status=6 (ABORTED); route current=- queued=3 completed=0 paused=True started=False'
        self.assertTrue(history.status('operator', failure, 3))
        self.assertFalse(history.executing)
        self.assertFalse(history.status('operator', failure, 3.1))
        history.status('operator', 'route current=1 queued=2 paused=False started=True', 4)
        self.assertTrue(history.status('operator', failure, 5))
        history.status('control', 'gate=forwarding Move requests; cmd=[0.3,0,0]', 5)
        history.status('localization_valid', 'True', 5)
        result = history.snapshot(5, 'navigation_failure')
        self.assertEqual(result['reason'], 'navigation_failure')
        self.assertEqual(result['latest']['localization_valid']['text'], 'True')
        self.assertTrue(result['latest']['control']['fresh'])
        self.assertFalse(history.snapshot(6)['latest']['localization_valid']['fresh'])
        history.status('control', 'x'*(MAX_STATUS_CHARS+10), 6)
        self.assertEqual(len(history.snapshot(6)['latest']['control']['text']), MAX_STATUS_CHARS)

    def test_small_rolling_grid_probes_raw_cells_and_rejects_large_or_wrong_frame(self):
        history = ControlHistory()
        history.odom('odom', 0.05, 0.05, 0, (0, 0, 0), 10, stamp=100)
        cells = [0]*40
        cells[4], cells[8], cells[12] = 100, -1, 45
        history.local_grid('odom', 20, 2, 0.1, (0, 0, 0), cells, 10, stamp=100)
        result = history.snapshot(10, ros_now=100)['local_costmap']
        self.assertTrue(result['available'])
        self.assertEqual([sample['value'] for sample in result['heading_probes']], [0, 100, -1, 45])
        self.assertEqual(result['unknown'], 1)
        self.assertEqual(result['occupied_or_inscribed'], 1)
        self.assertFalse(history.snapshot(14, ros_now=104)['local_costmap']['available'])
        history.local_grid('map', 20, 2, 0.1, (0, 0, 0), cells, 14)
        self.assertIn('not odom', history.snapshot(14)['local_costmap']['reason'])
        history.local_grid('odom', MAX_GRID_CELLS+1, 1, 0.1, (0, 0, 0), (), 14)
        self.assertIn('bounded cell limit', history.snapshot(14)['local_costmap']['reason'])
        self.assertNotIn('data', history.grid)

    def test_rotated_costmap_origin_and_sensor_stamp_age(self):
        history = ControlHistory()
        history.odom('odom', -0.05, 0.05, math.pi/2, (0, 0, 0), 10, stamp=100)
        history.local_grid('odom', 20, 2, 0.1, (0, 0, math.pi/2), [0]*40, 10, stamp=100)
        grid = history.snapshot(10, ros_now=100)['local_costmap']
        self.assertEqual([sample['cell'] for sample in grid['heading_probes']], [[0, 0], [4, 0], [8, 0], [12, 0]])
        self.assertEqual(history.snapshot(10, ros_now=101)['odometry']['reason'], 'odom source stamp stale/future')
        history.path('map', 12, (20.5, -10.5), 10, stamp=99.9)
        plan = history.snapshot(10)['plan']
        self.assertEqual(plan['points'], 12)
        self.assertEqual(plan['target'], (20.5, -10.5))
        self.assertFalse(history.snapshot(14)['plan']['fresh'])


if __name__ == '__main__':
    unittest.main()
