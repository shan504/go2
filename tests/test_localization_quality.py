"""Exercise the shared patrol quality settings with actual Open3D GICP."""
import sys
import unittest
from pathlib import Path
import numpy as np
import yaml
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'go2_3d'))
from registration import align, cloud
from test_3d import room


class LocalizationQualityTests(unittest.TestCase):
    def test_partial_overlap_can_pass_patrol_but_not_old_gate(self):
        settings = yaml.safe_load((Path(__file__).resolve().parents[1]/
                                   'go2_3d/gicp.yaml').read_text())['go2_gicp']['ros__parameters']
        rng = np.random.default_rng(104)
        target = cloud(room(), 0.10)
        visible = np.asarray(target.points)[rng.choice(len(target.points), 250, replace=False)]
        missing = rng.uniform([12, 12, 1], [18, 18, 3], (850, 3))
        source = cloud(np.vstack((visible, missing)), None)
        kwargs = dict(correspondence=settings['localization_correspondence'],
                      max_rmse=settings['localization_max_rmse'],
                      max_translation=settings['localization_max_translation'],
                      max_rotation=np.deg2rad(settings['localization_max_rotation_degrees']))
        result = align(source, target, np.eye(4),
                       min_fitness=settings['localization_min_fitness'], **kwargs)
        self.assertTrue(result.accepted, result)
        self.assertGreaterEqual(result.fitness, 0.20)
        self.assertLess(result.fitness, 0.30)
        np.testing.assert_allclose(result.transform, np.eye(4), atol=1e-5)
        old = align(source, target, np.eye(4), min_fitness=0.30, **kwargs)
        self.assertFalse(old.accepted)
        self.assertEqual(old.reason, 'overlap/residual rejected')
        # Enlarging the correspondence radius is not unconditional acceptance.
        absent = align(cloud(missing, None), target, np.eye(4),
                       min_fitness=settings['localization_min_fitness'], **kwargs)
        self.assertFalse(absent.accepted)


if __name__ == '__main__':
    unittest.main()
