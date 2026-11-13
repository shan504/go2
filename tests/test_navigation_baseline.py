"""Pin the November 5 controller with the explicitly retained bounded retry.

These hashes are from d4244dcc72efacf4606938eaf645a38d6ac5e135. The test does
not require Git history or a network connection in the robot's build context.
An intentional later navigation change must revise this restoration contract.
"""
import hashlib
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET


class NavigationBaselineTest(unittest.TestCase):
    def test_controller_policy_and_preflight_match_november_5(self):
        root = Path(__file__).resolve().parents[1]
        expected = {
            'go2_3d/motion_profile.py': '11db4209ea51b5844ae0f2cb2cf751708afafc869abc9e222366e235657ac66c',
            'go2_3d/check_nav2_plugins.py': '5c99b86a03367f14002a11257c76c31981ad3b847e789492a30f443a94b7fb8b',
        }
        for path, digest in expected.items():
            with self.subTest(path=path):
                self.assertEqual(hashlib.sha256((root/path).read_bytes()).hexdigest(), digest)

    def test_retained_retry_wraps_the_whole_navigation_pipeline(self):
        root = Path(__file__).resolve().parents[1]
        tree = ET.parse(root/'go2_3d/navigate.xml')
        recovery = tree.find('./BehaviorTree/RecoveryNode')
        self.assertIsNotNone(recovery)
        self.assertEqual(recovery.get('number_of_retries'), '3')
        self.assertEqual([child.tag for child in recovery], ['PipelineSequence', 'Wait'])
        pipeline, wait = list(recovery)
        self.assertEqual([child.tag for child in pipeline], ['RateController', 'FollowPath'])
        self.assertEqual(pipeline[0].get('hz'), '1.0')
        self.assertEqual([child.tag for child in pipeline[0]], ['ComputePathToPose'])
        self.assertEqual(wait.get('wait_duration'), '2')
        self.assertEqual(wait.get('server_timeout'), '1000')
        self.assertEqual(pipeline[1].get('server_timeout'), '1000')
        self.assertFalse(list(tree.iter('Spin')) or list(tree.iter('BackUp')))
        self.assertFalse(any('Clear' in node.tag for node in tree.iter()))


if __name__ == '__main__':
    unittest.main()
