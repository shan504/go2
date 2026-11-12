"""Append-only local-cloud journal for offline backend comparisons.

The pose is the accepted GICP pose with *applied backend frame corrections*
removed. GICP still saw its live local map: removing frame jumps does not undo
that feedback and does not turn this into an independent odometry benchmark.
No ROS, robot command, database pickle, or mutable saved-map dependency exists.
"""
from dataclasses import dataclass
import hashlib
import json
import os
from pathlib import Path
import uuid

import numpy as np

from map_start import validate_start_pose
from mapping_rebuild import MappingObservation


POSE_CONVENTIONS = ('uncorrected_gicp', 'backend_corrections_removed_gicp')
PROVENANCES = ('robot_recording', 'synthetic', 'unknown')


def _json(data):
    return json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False)


def _stamp(value):
    value = np.asarray(value)
    if value.shape != () or value.dtype.kind not in 'iu' or int(value) < 0:
        raise ValueError('stamp_ns must be a nonnegative integer scalar')
    stamp = int(value)
    if stamp > np.iinfo(np.int64).max:
        raise ValueError('stamp_ns exceeds signed 64-bit range')
    return stamp


class MappingJournal:
    """One session writer. Only a brand-new directory can be opened."""

    def __init__(self, directory, metadata=None):
        self.directory = Path(directory)
        self.metadata = {
            'schema_version': 1, 'coordinate_frame': 'gicp_frontend',
            'cloud_frame': 'base_link', 'pose_source': 'gicp',
            'pose_convention': 'backend_corrections_removed_gicp',
            'provenance': 'unknown', 'frontend_feedback': True,
            'interpretation': 'Frozen accepted GICP observations; removing backend '
                              'frame corrections does not remove frontend map feedback.',
            **dict(metadata or {}),
        }
        validate_metadata(self.metadata)
        encoded = _json(self.metadata)  # Validate before creating any files.
        self.directory.mkdir(parents=True, exist_ok=False)
        self.frames_directory = self.directory / 'keyframes'
        self.frames_directory.mkdir()
        with (self.directory / 'session.json').open('x') as stream:
            stream.write(encoded + '\n')
            stream.flush()
            os.fsync(stream.fileno())
        self.last_stamp = None
        self.count = 0

    def append(self, stamp_ns, points_body, mapped_pose, map_from_frontend=None):
        stamp = _stamp(stamp_ns)
        if self.last_stamp is not None and stamp <= self.last_stamp:
            raise ValueError('journal timestamps must be strictly increasing')
        mapped = validate_start_pose(mapped_pose)
        correction = validate_start_pose(np.eye(4) if map_from_frontend is None
                                         else map_from_frontend)
        pose = validate_start_pose(np.linalg.inv(correction) @ mapped)
        observation = MappingObservation(points_body, pose)
        if not len(observation.points_body):
            raise ValueError('journal point cloud must not be empty')
        path = self.frames_directory / f'{stamp}.npz'
        if path.exists():
            raise ValueError('journal timestamp already exists')
        temporary = path.with_name('.' + path.name + '.' + uuid.uuid4().hex)
        try:
            with temporary.open('xb') as stream:
                np.savez(stream, stamp_ns=np.int64(stamp), points=observation.points_body,
                         pose=observation.pose, mapped_pose=mapped,
                         map_from_frontend=correction)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, path)
        finally:
            temporary.unlink(missing_ok=True)
        self.last_stamp = stamp
        self.count += 1


def validate_metadata(metadata):
    if not isinstance(metadata, dict) or metadata.get('schema_version') != 1:
        raise ValueError('unsupported mapping journal schema')
    if metadata.get('pose_source') != 'gicp':
        raise ValueError('journal must contain accepted GICP poses, not native /odom')
    if metadata.get('pose_convention') not in POSE_CONVENTIONS:
        raise ValueError('unknown trajectory convention; backend-corrected map poses '
                         'cannot be treated as raw GICP observations')
    if metadata.get('provenance') not in PROVENANCES:
        raise ValueError('provenance must be robot_recording, synthetic, or unknown')
    if not metadata.get('coordinate_frame') or not metadata.get('cloud_frame'):
        raise ValueError('journal must name its pose and cloud coordinate frames')
    _json(metadata)


@dataclass(frozen=True)
class JournalDataset:
    observations: tuple
    stamps_ns: tuple
    metadata: dict
    sha256: str


def load_journal(directory, assume_uncorrected_gicp=False):
    """Read a completed session, never infer raw poses from corrected map poses.

    Legacy RTAB-Map keyframe archives lack this metadata. An explicit assumption
    can load them, recorded in the report, without asserting robot provenance.
    """
    directory = Path(directory).resolve(strict=True)
    if (directory/'FAILED.json').exists():
        raise ValueError('incomplete/failed journal cannot be replayed')
    metadata_file = directory / 'session.json'
    digest = hashlib.sha256()
    if metadata_file.is_file():
        data = metadata_file.read_bytes()
        metadata = json.loads(data, parse_constant=lambda value: (_ for _ in ()).throw(
            ValueError('nonfinite JSON value ' + value)))
        digest.update(b'session.json\0' + data)
    elif assume_uncorrected_gicp:
        metadata = dict(schema_version=1, coordinate_frame='gicp_odom',
                        cloud_frame='base_link', pose_source='gicp',
                        pose_convention='uncorrected_gicp', provenance='unknown',
                        legacy_pose_convention_assumed_by_user=True,
                        frontend_feedback='unknown')
        digest.update(_json(metadata).encode())
    else:
        raise ValueError('session.json is missing; an explicit '
                         '--assume-uncorrected-gicp is required for legacy archives')
    validate_metadata(metadata)
    frames = []
    for path in sorted((directory / 'keyframes').glob('*.npz')):
        if path.is_symlink() or directory not in path.resolve().parents:
            raise ValueError('keyframe must be a regular file inside the session')
        digest.update(path.name.encode() + b'\0')
        with path.open('rb') as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b''):
                digest.update(chunk)
        with np.load(path, allow_pickle=False) as archive:
            stamp = _stamp(archive['stamp_ns'])
            observation = MappingObservation(archive['points'], archive['pose'])
            if not len(observation.points_body):
                raise ValueError(f'empty point cloud: {path.name}')
            if 'mapped_pose' in archive and 'map_from_frontend' in archive:
                mapped = validate_start_pose(archive['mapped_pose'])
                correction = validate_start_pose(archive['map_from_frontend'])
                if not np.allclose(correction @ observation.pose, mapped,
                                   atol=1e-7, rtol=0):
                    raise ValueError(f'inconsistent frame correction: {path.name}')
        frames.append((stamp, observation))
    frames.sort(key=lambda item: item[0])
    if len(frames) < 2 or len({item[0] for item in frames}) != len(frames):
        raise ValueError('need at least two uniquely timestamped local keyframes')
    return JournalDataset(tuple(item[1] for item in frames),
                          tuple(item[0] for item in frames), metadata, digest.hexdigest())
