import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'go2_3d'))
from rtabmap_sync import AssociationGate


class AssociationTests(unittest.TestCase):
    def test_uncertainty_is_part_of_total_error_budget(self):
        gate = AssociationGate(.1, .04)
        gate.add_scan(1_000_000_000, 'scan')
        gate.add_camera(1_070_000_000, 'image')
        self.assertEqual(gate.ready(), [])
        self.assertEqual(gate.report()['skipped'], 1)

    def test_camera_before_or_after_scan_preserves_source_stamps(self):
        for offset in (-30_000_000, 30_000_000):
            gate = AssociationGate(.1, .02)
            gate.add_scan(1_000_000_000, ('odom', 'cloud'))
            gate.add_camera(1_000_000_000 + offset, ('rgb', 'info'))
            self.assertEqual(gate.ready(), [(1_000_000_000, ('odom', 'cloud'),
                1_000_000_000 + offset, ('rgb', 'info'))])

    def test_one_image_cannot_describe_two_poses(self):
        gate = AssociationGate(.1, .02)
        gate.add_camera(1_000_000_000, 'image')
        gate.add_scan(1_000_000_000, 'first')
        gate.add_scan(1_010_000_000, 'second')
        self.assertEqual(len(gate.ready()), 1)
        gate.expire_before(2_000_000_000)
        self.assertEqual(gate.report()['skipped'], 1)

    def test_clock_regression_and_backlog_are_explicit(self):
        gate = AssociationGate(.1, .02, max_pending=1)
        gate.add_scan(10, 'first')
        with self.assertRaisesRegex(ValueError, 'strictly increase'):
            gate.add_scan(10, 'duplicate')
        with self.assertRaisesRegex(ValueError, 'backlog'):
            gate.add_scan(20, 'second')
        gate.add_camera(20, 'image')
        with self.assertRaisesRegex(ValueError, 'strictly increase'):
            gate.add_camera(10, 'regressed')


if __name__ == '__main__':
    unittest.main()
