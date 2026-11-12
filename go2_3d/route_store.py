"""Local, map-bound patrol route files. This module never sends motion commands."""
import hashlib
import json
import math
import os
from pathlib import Path
import re
import stat
import tempfile

import yaml


MAX_WAYPOINTS = 100
MAX_FILE_BYTES = 128 * 1024
_NAME = re.compile(r'[A-Za-z0-9][A-Za-z0-9_-]{0,63}\Z')
_FINGERPRINT = re.compile(r'[0-9a-f]{64}\Z')


def route_name(name):
    if not isinstance(name, str) or not _NAME.fullmatch(name):
        raise ValueError('Route name must be 1..64 ASCII letters/digits/underscore/hyphen, starting with a letter or digit')
    return name


def _number(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError('Waypoint coordinates must be finite numbers')
    try:
        value = float(value)
    except OverflowError as error:
        raise ValueError('Waypoint coordinates must be finite numbers') from error
    if not math.isfinite(value):
        raise ValueError('Waypoint coordinates must be finite numbers')
    return value


def _vector(value, length):
    if not isinstance(value, list) or len(value) != length:
        raise ValueError(f'Expected a list of {length} coordinates')
    return [_number(item) for item in value]


def _identity(value):
    if not isinstance(value, dict) or set(value) != {'frame_id', 'fingerprint', 'label'}:
        raise ValueError('Route map identity must contain frame_id, fingerprint and label')
    if value['frame_id'] != 'map':
        raise ValueError('Route frame_id must be map')
    if not isinstance(value['fingerprint'], str) or not _FINGERPRINT.fullmatch(value['fingerprint']):
        raise ValueError('Route map fingerprint must be a SHA-256 hex string')
    label = value['label']
    if not isinstance(label, str) or not label or len(label) > 256 or any(ord(c) < 32 for c in label):
        raise ValueError('Route map label must be a nonempty, single-line string of at most 256 characters')
    return dict(frame_id='map', fingerprint=value['fingerprint'], label=label)


def validate_route(route, expected_map=None):
    """Validate and copy the complete route before it can change any live queue."""
    if not isinstance(route, dict) or set(route) != {'schema_version', 'map', 'waypoints'}:
        raise ValueError('Route must contain schema_version, map and waypoints')
    if type(route['schema_version']) is not int or route['schema_version'] != 1:
        raise ValueError('Unsupported route schema_version; expected 1')
    identity = _identity(route['map'])
    if expected_map is not None:
        expected = _identity(expected_map)
        if identity['fingerprint'] != expected['fingerprint']:
            raise ValueError(f'Route map mismatch: saved {identity["label"]}; running {expected["label"]}. Select the original map version before loading this route')
    values = route['waypoints']
    if not isinstance(values, list) or not 1 <= len(values) <= MAX_WAYPOINTS:
        raise ValueError(f'Route needs 1..{MAX_WAYPOINTS} waypoints')
    waypoints = []
    for value in values:
        if not isinstance(value, dict) or set(value) != {'position', 'orientation'}:
            raise ValueError('Each waypoint must contain position and orientation')
        position = _vector(value['position'], 3)
        orientation = _vector(value['orientation'], 4)
        norm = math.hypot(*orientation)
        if not math.isclose(norm, 1.0, rel_tol=0.0, abs_tol=1e-3):
            raise ValueError('Waypoint orientation must be a unit quaternion')
        waypoints.append(dict(position=position, orientation=orientation))
    return dict(schema_version=1, map=identity, waypoints=waypoints)


def _canonical(value):
    """Exclude serialization differences such as 1 versus 1.0 from map identity."""
    if value is None or isinstance(value, (str, bool)):
        return value
    if isinstance(value, (int, float)):
        try:
            result = float(value)
        except OverflowError as error:
            raise ValueError('Map metadata contains nonfinite numbers') from error
        if not math.isfinite(result):
            raise ValueError('Map metadata contains nonfinite numbers')
        return result
    if isinstance(value, list):
        return [_canonical(item) for item in value]
    if isinstance(value, dict) and all(isinstance(key, str) for key in value):
        return {key: _canonical(item) for key, item in value.items()}
    raise ValueError('Map identity metadata must contain ordinary JSON values')


def _map_file(directory, name):
    name = Path(name)
    if name.is_absolute():
        raise ValueError('Map files must be relative to the runtime map directory')
    path = (directory / name).resolve(strict=True)
    if directory not in path.parents or not path.is_file():
        raise ValueError('Map file escapes the runtime map directory')
    return path


def _hash_file(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def map_identity(directory):
    """Fingerprint the prepared runtime map, once, rather than a mutable map name.

    PCD bytes, occupancy image and coordinate metadata bind the route to the
    same map geometry. Paths and creation timestamps are deliberately excluded
    so an unchanged archived/copied bundle can still reuse its route.
    """
    directory = Path(directory).resolve(strict=True)
    nav = yaml.safe_load(_map_file(directory, 'nav.yaml').read_text())
    metadata = yaml.safe_load(_map_file(directory, 'metadata.yaml').read_text())
    if not isinstance(nav, dict) or not isinstance(metadata, dict):
        raise ValueError('Map configuration and metadata must be mappings')
    if metadata.get('frame_id', 'map') != 'map':
        raise ValueError('Runtime map frame_id must be map')
    image = nav.get('image')
    if not isinstance(image, str) or not image:
        raise ValueError('Map configuration has no occupancy image')
    geometry = {key: metadata.get(key) for key in (
        'body_height_above_ground', 'map_start_pose', 'map_from_source_map')}
    ground = metadata.get('ground_model')
    if ground is not None and not isinstance(ground, dict):
        raise ValueError('Map ground_model must be a mapping')
    # Support counts/RANSAC diagnostics do not change the coordinate frame.
    geometry['ground_model'] = {'plane': ground.get('plane')} if ground is not None else None
    descriptor = dict(version=1, frame_id='map',
                      pcd=_hash_file(_map_file(directory, 'map.pcd')),
                      occupancy=_hash_file(_map_file(directory, image)),
                      grid={key: value for key, value in nav.items() if key != 'image'},
                      geometry=geometry)
    encoded = json.dumps(_canonical(descriptor), sort_keys=True, separators=(',', ':'),
                         allow_nan=False).encode('utf-8')
    parameters = metadata.get('parameters', {})
    label = parameters.get('map_name') if isinstance(parameters, dict) else None
    if not isinstance(label, str) or not label or len(label) > 256 or any(ord(c) < 32 for c in label):
        label = directory.name or 'map'
    return dict(frame_id='map', fingerprint=hashlib.sha256(encoded).hexdigest(), label=label)


def _unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f'Duplicate JSON key: {key}')
        result[key] = value
    return result


class RouteStore:
    def __init__(self, directory):
        self.directory = Path(directory).resolve()

    def _path(self, name):
        return self.directory / (route_name(name) + '.json')

    def save(self, name, route, overwrite=False):
        path = self._path(name)
        document = validate_route(route)
        encoded = (json.dumps(document, indent=2, ensure_ascii=False, allow_nan=False) + '\n').encode('utf-8')
        if len(encoded) > MAX_FILE_BYTES:
            raise ValueError('Route file exceeds size limit')
        self.directory.mkdir(parents=True, exist_ok=True)
        if path.is_symlink():
            raise ValueError('Refusing a symlink route file')
        temporary = None
        try:
            with tempfile.NamedTemporaryFile(prefix='.route-', suffix='.tmp', dir=self.directory,
                                             delete=False) as stream:
                temporary = Path(stream.name)
                stream.write(encoded)
                stream.flush()
                os.fsync(stream.fileno())
            if overwrite:
                os.replace(temporary, path)
            else:
                # Atomic no-clobber commit, including concurrent save attempts.
                os.link(temporary, path)
            directory_fd = os.open(self.directory, os.O_RDONLY | os.O_DIRECTORY)
            try:
                os.fsync(directory_fd)
            finally:
                os.close(directory_fd)
        finally:
            if temporary is not None:
                temporary.unlink(missing_ok=True)
        return path

    def load(self, name):
        path = self._path(name)
        try:
            fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
        except FileNotFoundError:
            raise
        except OSError as error:
            raise ValueError(f'Cannot read route file {name}: {error}') from error
        with os.fdopen(fd, 'rb') as stream:
            info = os.fstat(stream.fileno())
            if not stat.S_ISREG(info.st_mode) or info.st_size > MAX_FILE_BYTES:
                raise ValueError('Route file must be a regular file within the size limit')
            raw = stream.read(MAX_FILE_BYTES + 1)
        if len(raw) > MAX_FILE_BYTES:
            raise ValueError('Route file exceeds size limit')
        try:
            route = json.loads(raw, object_pairs_hook=_unique_object)
        except (UnicodeDecodeError, json.JSONDecodeError) as error:
            raise ValueError(f'Invalid JSON in route {name}: {error}') from error
        return validate_route(route)

    def list_routes(self):
        if not self.directory.exists():
            return []
        result = []
        for path in sorted(self.directory.glob('*.json')):
            route = self.load(path.stem)
            result.append(dict(name=path.stem, waypoints=len(route['waypoints']), map=route['map']['label']))
        return result
