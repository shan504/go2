#!/usr/bin/env python3
"""Verify the isolated, source-built KISS-SLAM backend without ROS or motion.

The PyPI 0.0.2 wheel is NOT the IROS25 source revision. A version string alone
therefore cannot identify this integration. The bootstrap records the checked
Git revision and hashes of the installed Python/native artifacts.
"""
import argparse
import hashlib
import importlib.metadata as metadata
import json
from pathlib import Path
import subprocess
import sys


UPSTREAM_URL = 'https://github.com/PRBonn/kiss-slam.git'
UPSTREAM_COMMIT = 'db81ee31b59bc7a18af3fec8641d5867646ab57a'
UPSTREAM_TAG = 'IROS25'
CORE_VERSIONS = {
    'kiss-slam': '0.0.2', 'map_closures': '2.0.2', 'kiss-icp': '1.2.3',
    'numpy': '1.26.4', 'open3d': '0.19.0', 'PyYAML': '6.0.2',
    'pydantic': '2.11.7', 'pydantic-settings': '2.10.1',
}


def artifact_hashes():
    distribution = metadata.distribution('kiss-slam')
    artifacts = {}
    for item in distribution.files or []:
        name = str(item)
        if name.startswith('kiss_slam/') and (name.endswith('.py') or name.endswith('.so')):
            path = Path(distribution.locate_file(item))
            artifacts[name] = hashlib.sha256(path.read_bytes()).hexdigest()
    if not artifacts or not any(name.endswith('.so') for name in artifacts):
        raise RuntimeError('KISS-SLAM native extension is missing')
    return artifacts


def installed_versions():
    observed = {package: metadata.version(package) for package in CORE_VERSIONS}
    if observed != CORE_VERSIONS:
        raise RuntimeError(f'KISS backend dependency versions differ: {observed}; expected {CORE_VERSIONS}')
    return observed


def provenance_path():
    return Path(sys.prefix) / 'kiss-provenance.json'


def record_environment(source):
    """Called only by the bootstrap after building the clean, pinned source."""
    source = Path(source).resolve()
    commit = subprocess.check_output(['git', '-C', str(source), 'rev-parse', 'HEAD'], text=True).strip()
    if commit != UPSTREAM_COMMIT:
        raise RuntimeError(f'Unexpected upstream commit {commit}')
    subprocess.run(['git', '-C', str(source), 'diff', '--exit-code', UPSTREAM_COMMIT, '--'],
                   check=True, stdout=subprocess.DEVNULL)
    report = dict(schema=1, upstream_url=UPSTREAM_URL, upstream_tag=UPSTREAM_TAG,
                  upstream_commit=commit, python=sys.version.split()[0],
                  versions=installed_versions(), artifacts=artifact_hashes())
    target = provenance_path()
    temporary = target.with_suffix('.tmp')
    temporary.write_text(json.dumps(report, indent=2, sort_keys=True) + '\n')
    temporary.replace(target)
    return report


def verify_environment():
    path = provenance_path()
    if not path.is_file():
        raise RuntimeError('Missing KISS source provenance; run go2_3d/setup_kiss_env.sh in the feature worktree')
    report = json.loads(path.read_text())
    if report.get('schema') != 1 or report.get('upstream_commit') != UPSTREAM_COMMIT:
        raise RuntimeError('KISS-SLAM provenance does not match the pinned IROS25 commit')
    if report.get('versions') != installed_versions():
        raise RuntimeError('KISS backend dependency provenance changed after installation')
    if report.get('artifacts') != artifact_hashes():
        raise RuntimeError('Installed KISS-SLAM source/native artifacts changed after installation')
    return report


def native_smoke():
    """Exercise real MapClosures construction, g2o optimization and voxel map."""
    import numpy as np
    from kiss_slam.config import LoopCloserConfig
    from kiss_slam.config.config import PoseGraphOptimizerConfig
    from kiss_slam.loop_closer import LoopCloser
    from kiss_slam.pose_graph_optimizer import PoseGraphOptimizer
    from kiss_slam.voxel_map import VoxelMap

    LoopCloser(LoopCloserConfig())
    graph = PoseGraphOptimizer(PoseGraphOptimizerConfig())
    first = np.eye(4)
    initial = np.eye(4)
    initial[0, 3] = 1.2
    measured = np.eye(4)
    measured[0, 3] = 1.0
    graph.add_variable(0, first)
    graph.add_variable(1, initial)
    graph.fix_variable(0)
    # Upstream add_factor(source, target) stores target as vertex 0.
    graph.add_factor(1, 0, measured, np.eye(6))
    graph.optimize()
    estimate = np.asarray(graph.estimates()[1])
    if not np.allclose(estimate, measured, atol=1e-5):
        raise RuntimeError(f'g2o native smoke failed: {estimate}')
    voxels = VoxelMap(0.1)
    voxels.integrate_frame(np.array([[0., 0., 0.], [1., 0., 0.], [0., 1., 0.]]), np.eye(4))
    if voxels.num_voxels() != 3:
        raise RuntimeError('KISS voxel-map native smoke failed')
    return {'map_closures_constructed': True, 'g2o_two_pose_optimization': True,
            'voxel_map': True, 'field_loop_closure_tested': False}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--record-source', type=Path, help='Bootstrap only: record the just-built clean source')
    parser.add_argument('--smoke', action='store_true')
    args = parser.parse_args()
    report = record_environment(args.record_source) if args.record_source else verify_environment()
    result = {key: value for key, value in report.items() if key != 'artifacts'}
    if args.smoke:
        result['native_smoke'] = native_smoke()
    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == '__main__':
    main()
