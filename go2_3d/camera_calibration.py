"""Validated, measured calibration for the optional monocular loop camera.

The YAML contains standard ROS camera-calibration fields: image_width,
image_height, camera_name, camera_matrix {rows, cols, data}, distortion_model,
distortion_coefficients {rows, cols, data}, rectification_matrix and
projection_matrix. It additionally REQUIRES:

  calibration_source: description/path/date of the actual intrinsic calibration
  extrinsics:
    parent_frame: base_link
    child_frame: a_unique_camera_optical_frame
    translation: [measured x, measured y, measured z]  # metres
    rotation_xyzw: [measured x, measured y, measured z, measured w]
    source: description/path/date of the measured body-to-optical calibration
  timing:
    mode: measured_receipt_offset OR capture_header
    stamp_offset_sec: measured exposure time minus input header time
    uncertainty_sec: positive measured upper bound on time error
    source: description of the timing measurement/source
    clock: host_ros

No production calibration values are supplied. Receipt-mode offsets cannot be
positive. Capture-header mode requires zero offset and a genuinely capture-
stamped input, not the existing GetImageSample preview topic. The same clock
must be used by the shifted lidar/odometry stamps and the input image stamps.
Only monocular calibration with identity rectification rotation is supported;
this avoids silently changing the calibrated physical optical frame. Output
pixels are rectified using P's intrinsics, without changing image resolution.
"""
from dataclasses import dataclass
from pathlib import Path
import math
import re

import numpy as np
import yaml


class CalibrationError(ValueError):
    pass


def _finite(value, label):
    if isinstance(value, bool):
        raise CalibrationError(f'{label} must be numeric')
    try:
        result = float(value)
    except (ValueError, TypeError):
        raise CalibrationError(f'{label} must be numeric') from None
    if not math.isfinite(result):
        raise CalibrationError(f'{label} must be finite')
    return result


def _text(value, label):
    if not isinstance(value, str) or not value.strip() or value.strip().lower() in ('unknown', 'todo', 'replace_me', 'unmeasured'):
        raise CalibrationError(f'{label} requires the actual calibration/source description')
    return value.strip()


def _vector(value, size, label):
    if not isinstance(value, (list, tuple)) or len(value) != size:
        raise CalibrationError(f'{label} requires {size} measured values')
    return tuple(_finite(item, label) for item in value)


def _matrix(data, key, rows, cols):
    value = data.get(key)
    if not isinstance(value, dict) or value.get('rows') != rows or value.get('cols') != cols:
        raise CalibrationError(f'{key} requires ROS matrix shape {rows}x{cols}')
    return np.asarray(_vector(value.get('data'), rows * cols, key)).reshape(rows, cols)


@dataclass(frozen=True)
class CameraCalibration:
    width: int
    height: int
    name: str
    distortion_model: str
    k: tuple
    d: tuple
    r: tuple
    p: tuple
    optical_frame: str
    translation: tuple
    quaternion_xyzw: tuple
    timing_mode: str
    stamp_offset_sec: float
    uncertainty_sec: float
    calibration_source: str
    extrinsics_source: str
    timing_source: str

    @property
    def rectified_k(self):
        return tuple(np.asarray(self.p).reshape(3, 4)[:, :3].ravel())


def validate_calibration(data, max_sync_error_sec=0.05):
    """Reject missing/unusable calibration instead of inventing K, TF or time."""
    limit = _finite(max_sync_error_sec, 'max_sync_error_sec')
    if limit <= 0:
        raise CalibrationError('max_sync_error_sec must be positive')
    if not isinstance(data, dict):
        raise CalibrationError('calibration YAML must be a mapping')
    dimensions = []
    for key in ('image_width', 'image_height'):
        value = data.get(key)
        if type(value) is not int or not 1 <= value <= 16384:
            raise CalibrationError(f'{key} must be an actual positive image dimension')
        dimensions.append(value)
    width, height = dimensions
    k = _matrix(data, 'camera_matrix', 3, 3)
    r = _matrix(data, 'rectification_matrix', 3, 3)
    p = _matrix(data, 'projection_matrix', 3, 4)
    if not np.allclose(r, np.eye(3), atol=1e-8, rtol=0):
        raise CalibrationError('single-camera rectification_matrix must be identity; do not reuse a stereo rectification/extrinsic')
    for matrix, label in ((k, 'camera_matrix'), (p[:, :3], 'projection_matrix')):
        if matrix[0, 0] <= 0 or matrix[1, 1] <= 0 or not np.allclose(matrix[2], [0, 0, 1], atol=1e-8, rtol=0):
            raise CalibrationError(f'{label} has invalid focal lengths or homogeneous row')
        if abs(matrix[0, 1]) > 1e-8 or abs(matrix[1, 0]) > 1e-8:
            raise CalibrationError(f'{label} skew is unsupported by this rectifier')
    if not np.allclose(p[:, 3], 0, atol=1e-8, rtol=0):
        raise CalibrationError('projection_matrix must describe one monocular camera, without a stereo baseline')
    model = data.get('distortion_model')
    sizes = {'plumb_bob': (4, 5), 'rational_polynomial': (8, 12, 14), 'equidistant': (4,)}
    distortion = data.get('distortion_coefficients')
    if model not in sizes or not isinstance(distortion, dict):
        raise CalibrationError('supported measured distortion models: plumb_bob, rational_polynomial, equidistant')
    n = distortion.get('cols')
    if distortion.get('rows') != 1 or n not in sizes[model]:
        raise CalibrationError(f'invalid coefficient count for {model}')
    d = _vector(distortion.get('data'), n, 'distortion_coefficients')
    extrinsics = data.get('extrinsics')
    if not isinstance(extrinsics, dict) or extrinsics.get('parent_frame') != 'base_link':
        raise CalibrationError('measured extrinsics.parent_frame must be base_link')
    frame = extrinsics.get('child_frame')
    if not isinstance(frame, str) or not re.fullmatch(r'[A-Za-z][A-Za-z0-9_]*', frame) or frame in ('map', 'odom', 'gicp_odom', 'base_link', 'base_footprint'):
        raise CalibrationError('extrinsics.child_frame must be a unique optical frame without a leading slash')
    translation = _vector(extrinsics.get('translation'), 3, 'extrinsics.translation')
    q = _vector(extrinsics.get('rotation_xyzw'), 4, 'extrinsics.rotation_xyzw')
    norm = np.linalg.norm(q)
    if abs(norm - 1.) > 1e-3:
        raise CalibrationError('extrinsics.rotation_xyzw must be a measured unit quaternion')
    q = tuple(float(value / norm) for value in q)
    timing = data.get('timing')
    if not isinstance(timing, dict) or timing.get('clock') != 'host_ros':
        raise CalibrationError('timing.clock must explicitly be host_ros, shared with shifted lidar/odom')
    mode = timing.get('mode')
    if mode not in ('measured_receipt_offset', 'capture_header'):
        raise CalibrationError('timing.mode requires measured_receipt_offset or capture_header')
    offset = _finite(timing.get('stamp_offset_sec'), 'timing.stamp_offset_sec')
    uncertainty = _finite(timing.get('uncertainty_sec'), 'timing.uncertainty_sec')
    if not 0 < uncertainty <= limit:
        raise CalibrationError('measured timing.uncertainty_sec must be positive and no greater than max_sync_error_sec')
    if (mode == 'measured_receipt_offset' and offset > 0) or (mode == 'capture_header' and offset != 0):
        raise CalibrationError('receipt offset must be <=0; capture_header must already be in host ROS time with offset=0')
    return CameraCalibration(width, height, _text(data.get('camera_name'), 'camera_name'), model,
        tuple(k.ravel()), d, tuple(r.ravel()), tuple(p.ravel()), frame, translation, q,
        mode, offset, uncertainty, _text(data.get('calibration_source'), 'calibration_source'),
        _text(extrinsics.get('source'), 'extrinsics.source'), _text(timing.get('source'), 'timing.source'))


def load_calibration(path, max_sync_error_sec=0.05):
    if not path:
        raise CalibrationError('calibration_file is required; no camera calibration is bundled')
    try:
        data = yaml.safe_load(Path(path).read_text())
    except (OSError, yaml.YAMLError) as error:
        raise CalibrationError(f'cannot read calibration_file: {error}') from error
    return validate_calibration(data, max_sync_error_sec)


def validate_input_topic(calibration, topic):
    if calibration.timing_mode == 'capture_header' and topic == '/camera/image/compressed':
        raise CalibrationError('GetImageSample /camera/image/compressed is receipt-stamped, not capture-stamped; measure its offset or provide a capture-stamped topic')


def corrected_stamp_ns(calibration, input_ns, now_ns, max_age_sec=1.0, max_future_sec=0.05):
    if input_ns <= 0:
        raise CalibrationError('image has an unset/nonpositive timestamp')
    result = input_ns + round(calibration.stamp_offset_sec * 1e9)
    age = (now_ns - result) / 1e9
    if result <= 0 or age > max_age_sec or age < -max_future_sec:
        raise CalibrationError(f'image exposure estimate is stale/future: age={age:.6f}s')
    return result, age


class CameraRectifier:
    def __init__(self, calibration):
        import cv2
        self.cv2 = cv2
        self.calibration = calibration
        k = np.asarray(calibration.k).reshape(3, 3)
        d = np.asarray(calibration.d)
        r = np.asarray(calibration.r).reshape(3, 3)
        p = np.asarray(calibration.p).reshape(3, 4)[:, :3]
        size = (calibration.width, calibration.height)
        function = cv2.fisheye.initUndistortRectifyMap if calibration.distortion_model == 'equidistant' else cv2.initUndistortRectifyMap
        self.maps = function(k, d, r, p, size, cv2.CV_32FC1)
        self.features = cv2.ORB_create(nfeatures=500)

    def decode_rectify(self, payload):
        cv2 = self.cv2
        if not payload or len(payload) > 50 * 1024 * 1024:
            raise CalibrationError('empty or oversized compressed image')
        encoded = np.frombuffer(bytes(payload), dtype=np.uint8)
        pixels = cv2.imdecode(encoded, cv2.IMREAD_COLOR | cv2.IMREAD_IGNORE_ORIENTATION)
        if pixels is None:
            raise CalibrationError('compressed image cannot be decoded')
        expected = (self.calibration.height, self.calibration.width)
        if pixels.shape[:2] != expected:
            raise CalibrationError(f'image dimensions {pixels.shape[:2]} do not match calibration {expected}; recalibrate the actual stream')
        rectified = cv2.remap(pixels, *self.maps, interpolation=cv2.INTER_LINEAR)
        gray = cv2.cvtColor(rectified, cv2.COLOR_BGR2GRAY)
        metrics = {'sharpness_laplacian_variance': float(cv2.Laplacian(gray, cv2.CV_64F).var()),
                   'feature_count': len(self.features.detect(gray, None)),
                   'mean_brightness': float(gray.mean()),
                   'saturated_fraction': float(np.mean((gray <= 2) | (gray >= 253)))}
        return rectified, metrics
