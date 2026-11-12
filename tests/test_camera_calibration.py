"""Synthetic calibration/rectification tests; not calibration of a real Go2."""
import copy
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'go2_3d'))
from camera_calibration import (CalibrationError, CameraRectifier, corrected_stamp_ns,
                                load_calibration, validate_calibration, validate_input_topic)


def synthetic_calibration():
    """Synthetic pinhole fixture, explicitly not a deployable Go2 calibration."""
    return {
        'camera_name': 'synthetic_test_camera', 'image_width': 160, 'image_height': 120,
        'calibration_source': 'synthetic fixture only',
        'camera_matrix': {'rows': 3, 'cols': 3, 'data': [100., 0., 80., 0., 100., 60., 0., 0., 1.]},
        'distortion_model': 'plumb_bob',
        'distortion_coefficients': {'rows': 1, 'cols': 5, 'data': [0.] * 5},
        'rectification_matrix': {'rows': 3, 'cols': 3, 'data': [1., 0., 0., 0., 1., 0., 0., 0., 1.]},
        'projection_matrix': {'rows': 3, 'cols': 4, 'data': [100., 0., 80., 0., 0., 100., 60., 0., 0., 0., 1., 0.]},
        'extrinsics': {'parent_frame': 'base_link', 'child_frame': 'synthetic_camera_optical',
                       'translation': [.2, .01, .1], 'rotation_xyzw': [0., 0., 0., 1.],
                       'source': 'synthetic fixture, no robot measurement'},
        'timing': {'mode': 'measured_receipt_offset', 'stamp_offset_sec': -.06,
                   'uncertainty_sec': .01, 'source': 'synthetic time alignment fixture', 'clock': 'host_ros'},
    }


class CameraCalibrationTests(unittest.TestCase):
    def test_missing_calibration_cannot_enable_camera(self):
        for value in (None, {}, {'camera_name': 'x'}, []):
            with self.subTest(value=value), self.assertRaises(CalibrationError):
                validate_calibration(value)
        with self.assertRaises(CalibrationError):
            load_calibration('')

    def test_intrinsics_extrinsics_and_timing_are_all_required(self):
        original = synthetic_calibration()
        for key in original:
            data = copy.deepcopy(original)
            del data[key]
            with self.subTest(key=key), self.assertRaises(CalibrationError):
                validate_calibration(data)
        for section, keys in [('extrinsics', original['extrinsics']), ('timing', original['timing'])]:
            for key in keys:
                data = copy.deepcopy(original)
                del data[section][key]
                with self.subTest(section=section, key=key), self.assertRaises(CalibrationError):
                    validate_calibration(data)

    def test_yaml_load_preserves_measured_values(self):
        data = synthetic_calibration()
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'camera.yaml'
            path.write_text(yaml.safe_dump(data))
            calibration = load_calibration(path)
        self.assertEqual(calibration.translation, (.2, .01, .1))
        self.assertEqual(calibration.stamp_offset_sec, -.06)
        self.assertEqual(calibration.rectified_k, tuple(data['camera_matrix']['data']))

    def test_invalid_geometry_and_models_fail(self):
        edits = [
            ('image_width', 0), ('image_width', True), ('distortion_model', 'guess'),
            ('camera_matrix', {'rows': 3, 'cols': 3, 'data': [0.] * 9}),
            ('rectification_matrix', {'rows': 3, 'cols': 3, 'data': [1., 0., 0., 0., -1., 0., 0., 0., -1.]}),
            ('distortion_coefficients', {'rows': 1, 'cols': 3, 'data': [0.] * 3}),
            ('calibration_source', 'TODO'),
        ]
        for key, value in edits:
            data = synthetic_calibration()
            data[key] = value
            with self.subTest(key=key, value=value), self.assertRaises(CalibrationError):
                validate_calibration(data)
        for key, value in [('parent_frame', 'odom'), ('child_frame', 'map'), ('child_frame', 'gicp_odom'),
                           ('rotation_xyzw', [0., 0., 0., 0.]), ('translation', [0., float('nan'), 0.])]:
            data = synthetic_calibration()
            data['extrinsics'][key] = value
            with self.subTest(key=key), self.assertRaises(CalibrationError):
                validate_calibration(data)

    def test_unmeasured_or_excessive_timing_error_rejected(self):
        for key, value in [('stamp_offset_sec', .01), ('uncertainty_sec', 0.),
                           ('uncertainty_sec', .06), ('uncertainty_sec', float('nan')),
                           ('clock', 'camera_hardware'), ('source', 'unmeasured')]:
            data = synthetic_calibration()
            data['timing'][key] = value
            with self.subTest(key=key, value=value), self.assertRaises(CalibrationError):
                validate_calibration(data)

    def test_capture_mode_must_not_relabel_receipt_stamped_preview(self):
        data = synthetic_calibration()
        data['timing'].update(mode='capture_header', stamp_offset_sec=0.)
        calibration = validate_calibration(data)
        with self.assertRaises(CalibrationError):
            validate_input_topic(calibration, '/camera/image/compressed')
        validate_input_topic(calibration, '/calibrated_capture/image/compressed')

    def test_exposure_offset_and_stale_future_checks(self):
        calibration = validate_calibration(synthetic_calibration())
        stamp, age = corrected_stamp_ns(calibration, 2_000_000_000, 2_010_000_000)
        self.assertEqual(stamp, 1_940_000_000)
        self.assertAlmostEqual(age, .07)
        for stamp_ns, now in [(0, 2_000_000_000), (1_000_000_000, 3_000_000_000), (4_000_000_000, 3_000_000_000)]:
            with self.subTest(stamp=stamp_ns), self.assertRaises(CalibrationError):
                corrected_stamp_ns(calibration, stamp_ns, now)

    def test_actual_decode_rectification_and_quality(self):
        import cv2
        calibration = validate_calibration(synthetic_calibration())
        rectifier = CameraRectifier(calibration)
        rng = np.random.default_rng(123)
        pixels = rng.integers(0, 256, (120, 160, 3), dtype=np.uint8)
        okay, encoded = cv2.imencode('.png', pixels)
        self.assertTrue(okay)
        decoded, metrics = rectifier.decode_rectify(encoded.tobytes())
        np.testing.assert_array_equal(decoded, pixels)
        self.assertGreater(metrics['sharpness_laplacian_variance'], 0)
        self.assertGreater(metrics['feature_count'], 0)
        blank = cv2.imencode('.png', np.zeros_like(pixels))[1]
        _, blank_metrics = rectifier.decode_rectify(blank.tobytes())
        self.assertEqual(blank_metrics['feature_count'], 0)
        self.assertEqual(blank_metrics['sharpness_laplacian_variance'], 0)
        self.assertEqual(blank_metrics['saturated_fraction'], 1.)

    def test_wrong_resolution_and_malformed_image_rejected(self):
        import cv2
        rectifier = CameraRectifier(validate_calibration(synthetic_calibration()))
        wrong = cv2.imencode('.png', np.zeros((60, 80, 3), dtype=np.uint8))[1]
        for value in (wrong.tobytes(), b'not an image', b''):
            with self.subTest(length=len(value)), self.assertRaises(CalibrationError):
                rectifier.decode_rectify(value)

    def test_supported_distortion_models_have_real_opencv_rectification(self):
        import cv2
        image = cv2.imencode('.png', np.zeros((120, 160, 3), dtype=np.uint8))[1]
        for model, size in [('plumb_bob', 5), ('rational_polynomial', 8), ('equidistant', 4)]:
            data = synthetic_calibration()
            data.update(distortion_model=model, distortion_coefficients={'rows': 1, 'cols': size, 'data': [0.] * size})
            with self.subTest(model=model):
                rectifier = CameraRectifier(validate_calibration(data))
                pixels, _ = rectifier.decode_rectify(image.tobytes())
                self.assertEqual(pixels.shape, (120, 160, 3))


if __name__ == '__main__':
    unittest.main()
